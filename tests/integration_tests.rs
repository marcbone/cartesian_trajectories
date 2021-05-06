// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use cartesian_trajectories::s_curve::SCurveConstraints;
use cartesian_trajectories::CartesianTrajectory;
use cartesian_trajectories::VelocityProfile::SCurve;
use cartesian_trajectories::{
    generate_absolute_motion, OrientationGenerator, PoseGenerator, PositionGenerator,
    VelocityProfileMapping, VelocityProfileOutput,
};
use nalgebra::{Isometry3, UnitQuaternion};
use std::time::Duration;

#[test]
fn hello_world() {
    let start_pose = Isometry3::translation(2., 2., 0.);
    let pose_generator = generate_absolute_motion(Isometry3::translation(1., 1., 0.));
    let mut trajectory = CartesianTrajectory::new(
        pose_generator,
        SCurve {
            start_pose,
            constraints: SCurveConstraints {
                max_jerk: 5.,
                max_acceleration: 5.,
                max_velocity: 5.,
            },
        },
    );
    for &i in [0., 0.5, 1.0].iter() {
        let time = trajectory.get_total_duration().mul_f64(i);
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
#[test]
fn custom_trajectory() {
    let start_pose = Isometry3::translation(1., 1., 0.);
    let do_nothing_pose_generator = PoseGenerator::new(Box::new(|_, _| Isometry3::identity()));
    let position_generator = PositionGenerator::new(Box::new(|start_position, progress| {
        let mut position = start_position.into_owned();
        position.x += progress;
        position
    }));
    let orientation_generator = OrientationGenerator::new(Box::new(|_, progress| {
        UnitQuaternion::from_euler_angles(0., 0., progress)
    }));

    let do_something_pose_generator =
        PoseGenerator::from_parts(position_generator, orientation_generator);
    let combined_pose_generator = do_nothing_pose_generator * do_something_pose_generator;

    let velocity_profile = VelocityProfileMapping::new(
        Box::new(|progress| VelocityProfileOutput {
            progress: progress.as_secs_f64() / 10.,
            finished: { progress.as_secs_f64() >= 10. },
        }),
        Duration::from_secs(10),
    );

    let mut trajectory = CartesianTrajectory::with_custom_velocity_profile_mapping(
        combined_pose_generator,
        velocity_profile,
    );
    for &i in [0., 0.5, 1.0].iter() {
        let time = trajectory.get_total_duration().mul_f64(i);
        let output = trajectory.get_pose(&start_pose, time);
        println!("{:?}", output);
        assert!(f64::abs(output.pose.translation.x - i - 1.) < 1e-7);
        assert!(f64::abs(output.pose.rotation.euler_angles().2 - i) < 1e-7);
        if i < 1. {
            assert!(!output.finished);
        } else {
            assert!(output.finished);
        }
    }
}
