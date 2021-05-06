// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use s_curve::{
    s_curve_generator, Derivative, SCurveConstraints, SCurveInput, SCurveStartConditions,
};
use std::f64::consts::PI;
use std::time::Duration;

/// output of the get function of a [`VelocityProfileMapping`](`VelocityProfileMapping`)
#[derive(Debug)]
pub struct VelocityProfileOutput {
    /// progress between 0 and 1
    pub progress: f64,
    /// should be true when the progress is >= 1
    pub finished: bool,
}
/// Custom Velocity Profile
pub struct VelocityProfileMapping {
    function: Box<dyn Fn(Duration) -> VelocityProfileOutput>,
    total_duration: Duration,
}
impl VelocityProfileMapping {
    /// evaluates the velocity profile function
    pub fn get(&mut self, time: Duration) -> VelocityProfileOutput {
        (self.function)(time)
    }
    /// creates a new VelocityProfileMapping by defining a velocity profile function.
    /// This function should map a duration monotonically to progress between 0 and 1.
    /// Further the total duration of the trajectory should be specified as it helps the user.
    pub fn new(
        function: Box<dyn Fn(Duration) -> VelocityProfileOutput>,
        total_duration: Duration,
    ) -> VelocityProfileMapping {
        VelocityProfileMapping {
            function,
            total_duration,
        }
    }
    /// Returns the total duration of the trajectory.
    pub fn get_total_duration(&self) -> Duration {
        self.total_duration
    }
}
/// Generates a simple Linear Velocity Profile. It linearly maps the duration to a progress.
pub fn generate_linear_velocity_profile(total_duration: Duration) -> VelocityProfileMapping {
    let velocity_profile = move |time: Duration| -> VelocityProfileOutput {
        VelocityProfileOutput {
            progress: time.as_secs_f64() / total_duration.as_secs_f64(),
            finished: time >= total_duration,
        }
    };
    VelocityProfileMapping::new(Box::new(velocity_profile), total_duration)
}
/// Generates a smooth cosine velocity profile which is indefinitely often continuously differentiable.
pub fn generate_cosine_velocity_profile(total_duration: Duration) -> VelocityProfileMapping {
    let velocity_profile = move |time: Duration| -> VelocityProfileOutput {
        let progress =
            (1. - f64::cos(PI * time.as_secs_f64() / total_duration.as_secs_f64())) / 2.0;
        VelocityProfileOutput {
            progress,
            finished: time >= total_duration,
        }
    };
    VelocityProfileMapping::new(Box::new(velocity_profile), total_duration)
}

/// creates an S-Curve velocity profile which is subject to jerk, acceleration and velocity constraints
/// # Arguments
/// * `total_length` - a rough estimation of the total length of the PoseGenerator.
/// Use [`get_approximate_length`](`crate::PoseGenerator::get_approximate_length`)
/// * `constraints` - jerk, acceleration and velocity constraints for the trajectory
pub fn generate_s_curve_profile(
    total_length: f64,
    constraints: SCurveConstraints,
) -> VelocityProfileMapping {
    let start_conditions = SCurveStartConditions {
        q0: 0.0,
        q1: total_length,
        v0: 0.0,
        v1: 0.0,
    };
    let input = SCurveInput {
        constraints,
        start_conditions,
    };
    let (params, s_curve) = s_curve_generator(&input, Derivative::Position);
    let total_duration = Duration::from_secs_f64(params.time_intervals.total_duration());
    let velocity_profile = move |time: Duration| -> VelocityProfileOutput {
        let progress = s_curve(time.as_secs_f64()) / total_length;
        VelocityProfileOutput {
            progress,
            finished: progress >= 1.,
        }
    };
    VelocityProfileMapping::new(Box::new(velocity_profile), total_duration)
}

#[cfg(test)]
mod tests {
    use crate::velocity_profile::{
        generate_cosine_velocity_profile, generate_linear_velocity_profile,
        generate_s_curve_profile,
    };
    use s_curve::SCurveConstraints;
    use std::f64::consts::PI;
    use std::time::Duration;

    #[test]
    fn test_linear_profile() {
        let mut profile = generate_linear_velocity_profile(Duration::from_secs_f64(10.));
        for i in 0..=100 {
            let time = Duration::from_secs_f64(i as f64 / 10.);
            let output = profile.get(time);
            println!("{:?}", output);
            assert!(f64::abs(output.progress - i as f64 / 100.) < 1e-7);
            if i != 100 {
                assert!(!output.finished);
            } else {
                assert!(output.finished);
            }
        }
    }
    #[test]
    fn test_cosine_profile() {
        let mut profile = generate_cosine_velocity_profile(Duration::from_secs_f64(10.));
        for i in 0..=100 {
            let time = Duration::from_secs_f64(i as f64 / 10.);
            let output = profile.get(time);
            println!("{:?}", output);
            assert!(f64::abs(output.progress - (1. - f64::cos(PI * i as f64 / 100.)) / 2.) < 1e-7);
            if i != 100 {
                assert!(!output.finished);
            } else {
                assert!(output.finished);
            }
        }
    }
    #[test]
    fn test_s_curve_profile() {
        let constraints = SCurveConstraints {
            max_jerk: 5.,
            max_acceleration: 5.,
            max_velocity: 5.,
        };
        let mut profile = generate_s_curve_profile(7.4, constraints);

        for &i in [0., 0.5, 1.0000000000000001].iter() {
            let time = profile.total_duration.mul_f64(i);
            let output = profile.get(time);
            println!("{:?}", output);
            assert!(f64::abs(output.progress - i) < 1e-7);
            if i < 1. {
                assert!(!output.finished);
            } else {
                assert!(output.finished);
            }
        }
    }
}
