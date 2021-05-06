// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use crate::velocity_profile::{
    generate_cosine_velocity_profile, generate_linear_velocity_profile, generate_s_curve_profile,
    VelocityProfileMapping,
};
use crate::PoseGenerator;
use nalgebra::Isometry3;
use s_curve::SCurveConstraints;
use std::time::Duration;

/// Select one of the predefined Velocity Profiles
#[derive(Debug)]
pub enum VelocityProfile {
    /// Linear scaling of the trajectory to fit the total duration of the trajectory
    Linear {
        /// Total duration of the trajectory
        total_duration: Duration,
    },
    /// Creates a smooth cosine velocity profile which is indefinitely often continuously differentiable.
    Cosine {
        /// Total duration of the trajectory
        total_duration: Duration,
    },
    /// creates an S-Curve velocity profile which is subject to jerk, acceleration and velocity constraints.
    SCurve {
        /// Estimation of the start pose
        start_pose: Isometry3<f64>,
        /// jerk, acceleration and velocity constraints to define the trajectory
        constraints: SCurveConstraints,
    },
}

/// Struct that describes the output of [`get_pose`](`CartesianTrajectory::get_pose`)
#[derive(Debug)]
pub struct CartesianTrajectoryOutput {
    /// calculated pose
    pub pose: Isometry3<f64>,
    /// is true when the trajectory is finished.
    pub finished: bool,
}
/// Represents a trajectory in cartesian space. It contains a function which
/// maps a start pose and time to the pose at that time and weather the trajectory is finished.
pub struct CartesianTrajectory {
    function: Box<dyn FnMut(&Isometry3<f64>, Duration) -> CartesianTrajectoryOutput>,
    total_duration: Duration,
}

impl CartesianTrajectory {
    /// Create a new CartesianTrajectory by specifying a pose generator and a velocity profile
    pub fn new(mut pose_generator: PoseGenerator, velocity_profile: VelocityProfile) -> Self {
        let velocity_profile_mapping = match velocity_profile {
            VelocityProfile::Linear { total_duration } => {
                generate_linear_velocity_profile(total_duration)
            }
            VelocityProfile::Cosine { total_duration } => {
                generate_cosine_velocity_profile(total_duration)
            }
            VelocityProfile::SCurve {
                start_pose,
                constraints,
            } => {
                let length = pose_generator.get_approximate_length(&start_pose, 10000);
                generate_s_curve_profile(length, constraints)
            }
        };
        Self::with_custom_velocity_profile_mapping(pose_generator, velocity_profile_mapping)
    }

    /// Create a new CartesianTrajectory by specifying a pose generator and a custom velocity profile
    pub fn with_custom_velocity_profile_mapping(
        mut pose_generator: PoseGenerator,
        mut velocity_profile_mapping: VelocityProfileMapping,
    ) -> Self {
        let total_duration = velocity_profile_mapping.get_total_duration();
        let cartesian_trajectory =
            move |start_pose: &Isometry3<f64>, time: Duration| -> CartesianTrajectoryOutput {
                let velocity_output = velocity_profile_mapping.get(time);
                let pose = pose_generator.get_pose(start_pose, velocity_output.progress);
                CartesianTrajectoryOutput {
                    pose,
                    finished: velocity_output.finished,
                }
            };
        CartesianTrajectory {
            function: Box::new(cartesian_trajectory),
            total_duration,
        }
    }
    /// Evaluates the cartesian trajectory at a certain point in time
    /// # Arguments
    /// * `start_pose` - the pose at the start time of the PoseGenerator
    /// * `time` - a duration which is between 0 and the total duration of the trajectory.
    pub fn get_pose(
        &mut self,
        start_pose: &Isometry3<f64>,
        time: Duration,
    ) -> CartesianTrajectoryOutput {
        (self.function)(start_pose, time)
    }

    /// Returns the total duration of the trajectory.
    pub fn get_total_duration(&self) -> Duration {
        self.total_duration
    }
}

impl CartesianTrajectory {}

#[cfg(test)]
mod tests {
    use crate::cartesian_trajectory::CartesianTrajectory;
    use crate::cartesian_trajectory::VelocityProfile::{Linear, SCurve};
    use crate::pose_generators::generate_absolute_motion;
    use nalgebra::Isometry3;
    use s_curve::SCurveConstraints;
    use std::time::Duration;

    #[test]
    fn can_create_cartesian_trajectory() {
        let start_pose = Isometry3::identity();
        let pose_generator = generate_absolute_motion(Isometry3::translation(1., 1., 0.));
        let mut trajectory = CartesianTrajectory::new(
            pose_generator,
            Linear {
                total_duration: Duration::from_secs(8),
            },
        );
        for i in 0..=100 {
            let time = Duration::from_secs_f64(i as f64 / 10.);
            let output = trajectory.get_pose(&start_pose, time);
            let progress = i as f64 / 80.;
            println!("{:?}", output);
            assert!(f64::abs(output.pose.translation.x - progress) < 1e-7);
            assert!(f64::abs(output.pose.translation.y - progress) < 1e-7);
            assert!(f64::abs(output.pose.translation.z) < 1e-7);
            if time.as_secs() < 8 {
                assert!(!output.finished)
            } else {
                assert!(output.finished)
            }
        }
    }
    #[test]
    fn can_create_cartesian_trajectory_with_scurve() {
        let start_pose = Isometry3::translation(2., 2., 0.);
        let pose_generator = generate_absolute_motion(Isometry3::translation(1., 1., 0.));
        let mut trajectory = CartesianTrajectory::new(
            pose_generator,
            SCurve {
                start_pose: start_pose.clone(),
                constraints: SCurveConstraints {
                    max_jerk: 5.,
                    max_acceleration: 5.,
                    max_velocity: 5.,
                },
            },
        );
        for &i in [0., 0.5, 1.0].iter() {
            let time = trajectory.total_duration.mul_f64(i);
            let output = trajectory.get_pose(&start_pose, time);
            println!("{:?}", output);
            assert!(f64::abs(output.pose.translation.x + i - 2.) < 1e-7);
            assert!(f64::abs(output.pose.translation.y + i - 2.) < 1e-7);
            if i < 1. {
                assert!(!output.finished);
            } else {
                assert!(output.finished);
            }
        }
    }
}
