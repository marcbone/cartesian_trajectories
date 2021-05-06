// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
//! contains predefined PoseGenerators
use crate::{PoseGenerator, PositionGenerator};
use nalgebra::{Isometry3, Unit, Vector3};
/// Describes the reference frame for a relative motion.
pub enum RelativeMotionFrame {
    /// Performs the motion relative to the robots base coordinate system.
    /// Concretely the displacement is multiplied from the left to the start pose.
    Base,
    /// Performs the motion relative to the robots tool center point coordinate system.
    /// Concretely the displacement is multiplied from the right to the start pose.
    /// This means that if the tool is rotated the robot will translate in the corresponding direction
    /// of the tool.
    Tcp,
    /// Performs the motion relative to the Unit frame (Identity Matrix).
    /// Concretely that means that the start pose is completely ignored.
    /// This type of motion is good when you apply it to an already given Transformation
    Unit,
}
/// generates a motion from the start pose to the goal pose.
pub fn generate_absolute_motion(goal_pose: Isometry3<f64>) -> PoseGenerator {
    let pose_generator = move |initial_pose: &Isometry3<f64>, progress: f64| -> Isometry3<f64> {
        initial_pose.lerp_slerp(&goal_pose, progress)
    };
    PoseGenerator(Box::new(pose_generator))
}
/// generates a motion where to goal is specified relative to the start pose.
///
/// There are 3 different modes which define how the relative motion will look like:
///
/// * Base:
/// shifts the initial pose about `displacement` as seen by the world frame.
/// In other words, the displacement is multiplied on the left-hand side of the initial pose
///
/// * Tcp:
/// shifts the initial pose about `displacement` as seen by the end effector frame.
/// In other words, the displacement is multiplied on the right-hand side of the initial pose
///
/// * Unit:
/// shifts the initial pose about `displacement` as seen by the unit frame.
/// In other words, the initial pose is ignored, and we move from directly to the displacement.
/// This can be useful if you want to multiply a pose generator with another one.
pub fn generate_relative_motion(
    displacement: Isometry3<f64>,
    frame: RelativeMotionFrame,
) -> PoseGenerator {
    match frame {
        RelativeMotionFrame::Base => {
            let pose_generator =
                move |initial_pose: &Isometry3<f64>, progress: f64| -> Isometry3<f64> {
                    let goal = displacement * initial_pose;
                    initial_pose.lerp_slerp(&goal, progress)
                };
            PoseGenerator(Box::new(pose_generator))
        }
        RelativeMotionFrame::Tcp => {
            let pose_generator =
                move |initial_pose: &Isometry3<f64>, progress: f64| -> Isometry3<f64> {
                    let goal = initial_pose * displacement;
                    initial_pose.lerp_slerp(&goal, progress)
                };
            PoseGenerator(Box::new(pose_generator))
        }
        RelativeMotionFrame::Unit => {
            let pose_generator =
                move |_initial_pose: &Isometry3<f64>, progress: f64| -> Isometry3<f64> {
                    Isometry3::identity().lerp_slerp(&displacement, progress)
                };
            PoseGenerator(Box::new(pose_generator))
        }
    }
}
fn angle_between(a: &Vector3<f64>, b: &Vector3<f64>, normal: &Unit<Vector3<f64>>) -> f64 {
    f64::atan2(b.cross(a).dot(normal), a.dot(b))
}
/// generates a [`PositionGenerator`](`crate::PositionGenerator`) which represents a circle motion.
/// # Arguments
/// * `center` - the center of the circle
/// * `normal` - rotation axis of the circle
/// * `start` - start position of the circle
/// * `rotation_angle` - angle in radians
///
/// The corresponding Position generator will rotate from the `start` around the `center` with the `normal` as axis of rotation
/// a total amount of `rotation_angle` radians.
///
/// # Panics
/// Panics if start and center are too close together (1e-7 units).
///
/// # Note
/// The start position gets overwritten everytime you call the PositionGenerator with progress `0.`
/// That way it is not necessary that the start position is known at creation-time.
pub fn generate_circle_motion(
    center: Vector3<f64>,
    normal: Unit<Vector3<f64>>,
    start: Vector3<f64>,
    rotation_angle: f64,
) -> PositionGenerator {
    let center_direction = center - start;
    let mut radius: f64 = center_direction.norm();
    assert!(
        radius > 1e-7,
        "start and center are too close to each other"
    );
    let mut v1 = normal.cross(&center_direction).normalize();
    let mut v2 = normal.cross(&v1);
    let tmp_start = radius * v1;
    let mut start_angle = angle_between(&-center_direction, &tmp_start, &normal);
    let position_generator = move |start_position: &Vector3<f64>, progress: f64| -> Vector3<f64> {
        if progress == 0. {
            let center_direction = center - start_position;
            radius = center_direction.norm();
            assert!(
                radius > 1e-7,
                "start and center are too close to each other"
            );
            v1 = normal.cross(&center_direction).normalize();
            v2 = normal.cross(&v1);
            let tmp_start = radius * v1;
            start_angle = angle_between(&-center_direction, &tmp_start, &normal);
        }
        let desired_angle = start_angle + progress * rotation_angle;
        center + radius * (v1 * f64::cos(desired_angle) + v2 * f64::sin(desired_angle))
    };
    PositionGenerator(Box::new(position_generator))
}
#[cfg(test)]
mod tests {
    use crate::pose_generators::*;
    use crate::OrientationGenerator;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::PI;

    #[test]
    fn can_translate() {
        let start = Isometry3::translation(0., 0., 0.);
        let goal = Isometry3::translation(1., 1., 1.);
        let mut pose_generator = generate_absolute_motion(goal);
        for i in 0..=100 {
            let progress = i as f64 / 100.;
            let pose: Isometry3<f64> = pose_generator.get_pose(&start, progress);
            assert!(f64::abs(pose.translation.x - progress) < 1e-10);
            assert!(f64::abs(pose.translation.y - progress) < 1e-10);
            assert!(f64::abs(pose.translation.z - progress) < 1e-10);
        }
    }
    #[test]
    fn can_calculate_length() {
        let start = Isometry3::translation(2., 2., 0.);
        let goal = Isometry3::translation(1., 1., 0.);
        let mut pose_generator = generate_absolute_motion(goal);
        let length = pose_generator.get_approximate_length(&start, 10000);
        assert!(f64::abs(length - std::f64::consts::SQRT_2) < 1e-7);
    }
    #[test]
    fn can_rotate() {
        let start_roll = 0.1;
        let start_pitch = 0.2;
        let start_yaw = 0.3;
        let goal_roll = 0.2;
        let goal_pitch = 0.4;
        let goal_yaw = 0.4;

        let start = Isometry3::from_parts(
            Translation3::new(0., 0., 0.),
            UnitQuaternion::from_euler_angles(start_roll, start_pitch, start_yaw),
        );
        let goal = Isometry3::from_parts(
            Translation3::new(0., 0., 0.),
            UnitQuaternion::from_euler_angles(goal_roll, goal_pitch, goal_yaw),
        );
        let mut pose_generator = generate_absolute_motion(goal);

        let start_pose: Isometry3<f64> = pose_generator.get_pose(&start, 0.);
        let (roll, pitch, yaw) = start_pose.rotation.euler_angles();
        assert!(f64::abs(roll - start_roll) < 1e-10);
        assert!(f64::abs(pitch - start_pitch) < 1e-10);
        assert!(f64::abs(yaw - start_yaw) < 1e-10);

        let goal_pose: Isometry3<f64> = pose_generator.get_pose(&start, 1.);
        let (roll, pitch, yaw) = goal_pose.rotation.euler_angles();
        assert!(f64::abs(roll - goal_roll) < 1e-10);
        assert!(f64::abs(pitch - goal_pitch) < 1e-10);
        assert!(f64::abs(yaw - goal_yaw) < 1e-10);
    }
    #[test]
    fn can_do_relative_translation() {
        let start = Isometry3::translation(1., 1., 0.);
        let goal = Isometry3::translation(1., 0., 0.);
        let mut pose_generator = generate_relative_motion(goal, RelativeMotionFrame::Base);
        for i in 0..=100 {
            let progress = i as f64 / 100.;
            let pose: Isometry3<f64> = pose_generator.get_pose(&start, progress);
            println!("{:?}", pose.translation);
            assert!(f64::abs(pose.translation.x - progress - 1.) < 1e-10);
            assert!(f64::abs(pose.translation.y - 1.) < 1e-10);
            assert!(f64::abs(pose.translation.z) < 1e-10);
        }
    }

    #[test]
    fn can_do_relative_translation_in_tcp_frame() {
        let start = Isometry3::rotation(Vector3::new(0., 0., std::f64::consts::FRAC_PI_2));
        let goal = Isometry3::translation(1., 0., 0.);
        let mut pose_generator = generate_relative_motion(goal, RelativeMotionFrame::Tcp);
        for i in 0..=100 {
            let progress = i as f64 / 100.;
            let pose: Isometry3<f64> = pose_generator.get_pose(&start, progress);
            println!("{:?}", pose.translation);
            assert!(f64::abs(pose.translation.x) < 1e-10);
            assert!(f64::abs(pose.translation.y - progress) < 1e-10);
            assert!(f64::abs(pose.translation.z) < 1e-10);
        }
    }

    #[test]
    fn can_multiply() {
        let start = Isometry3::translation(1., -1., 0.5);
        let goal_1 = Isometry3::translation(1., 0., 0.);
        let goal_2 = Isometry3::translation(0., 1., 0.);
        let pose_generator_1 = generate_relative_motion(goal_1, RelativeMotionFrame::Unit);
        let pose_generator_2 = generate_relative_motion(goal_2, RelativeMotionFrame::Base);
        let mut pose_generator = pose_generator_1 * pose_generator_2;
        for i in 0..=100 {
            let progress = i as f64 / 100.;
            let pose: Isometry3<f64> = pose_generator.get_pose(&start, progress);
            println!("{:?}", pose.translation);
            assert!(f64::abs(pose.translation.x - progress - 1.) < 1e-10);
            assert!(f64::abs(pose.translation.y - progress + 1.) < 1e-10);
            assert!(f64::abs(pose.translation.z - 0.5) < 1e-10);
        }
    }
    #[test]
    fn can_combine_equal_generators() {
        let start = Isometry3::translation(1., 0., 0.5);
        let goal_1 = Isometry3::translation(0., 1., 0.);
        let goal_2 = Isometry3::translation(0., 1., 0.);
        let pose_generator_1 = generate_relative_motion(goal_1, RelativeMotionFrame::Base);
        let pose_generator_2 = generate_relative_motion(goal_2, RelativeMotionFrame::Base);
        let mut pose_generator: PoseGenerator = pose_generator_1.append(&start, pose_generator_2);
        let start_time = std::time::Instant::now();
        for i in 0..=100 {
            let progress = i as f64 / 100.;
            let pose: Isometry3<f64> = pose_generator.get_pose(&start, progress);
            println!("{:?}", pose.translation);
            assert!(f64::abs(pose.translation.x - 1.) < 1e-10);
            assert!(f64::abs(pose.translation.y - 2. * progress) < 1e-10);
            assert!(f64::abs(pose.translation.z - 0.5) < 1e-10);
        }
        println!("{:?}", std::time::Instant::now() - start_time);
    }
    #[test]
    fn can_combine_non_equal_generators() {
        let start = Isometry3::translation(1., 0., 0.5);
        let goal_1 = Isometry3::translation(0., 1., 0.);
        let goal_2 = Isometry3::translation(2., 0., 0.);
        let pose_generator_1 = generate_relative_motion(goal_1, RelativeMotionFrame::Base);
        let pose_generator_2 = generate_relative_motion(goal_2, RelativeMotionFrame::Base);
        let mut pose_generator: PoseGenerator = pose_generator_1.append(&start, pose_generator_2);
        let split = 1. / 3.;
        for i in 0..=100 {
            let progress = i as f64 / 100.;
            let pose: Isometry3<f64> = pose_generator.get_pose(&start, progress);
            println!("{:?}", pose.translation);
            if progress <= split {
                assert!(f64::abs(pose.translation.x - 1.) < 1e-10);
                assert!(f64::abs(pose.translation.y - 3. * progress) < 1e-10);
            } else {
                assert!(
                    f64::abs(pose.translation.x - 1. - 1. * 1. / split * (progress - split))
                        < 1e-10
                );
                assert!(f64::abs(pose.translation.y - 1.) < 1e-10);
            }
            assert!(f64::abs(pose.translation.z - 0.5) < 1e-10);
        }
    }
    #[test]
    fn can_do_circle() {
        let start = Isometry3::translation(1., 0., 0.);
        let circle_generator = generate_circle_motion(
            Vector3::zeros(),
            Vector3::z_axis(),
            start.translation.vector,
            PI,
        );
        let mut pose_generator = PoseGenerator::from_parts(
            circle_generator,
            OrientationGenerator::constant_orientation_generator(),
        );
        for i in 0..=100 {
            let progress = i as f64 / 100.;
            let pose: Isometry3<f64> = pose_generator.get_pose(&start, progress);
            println!("{:?}", pose.translation);
            assert!(f64::abs(pose.translation.x - f64::cos(PI * progress)) < 1e-10);
            assert!(f64::abs(pose.translation.y - f64::sin(PI * progress)) < 1e-10);
            assert!(f64::abs(pose.translation.z) < 1e-10);
        }
    }
}
