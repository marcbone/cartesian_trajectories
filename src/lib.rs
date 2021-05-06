// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#![deny(missing_docs)]
//! This library generate smooth trajectories in cartesian space in a functional style.
//!
//! # Overview
//! The idea is to specify a function that takes a start pose and a progress between 0 and 1 and spits out a pose.
//! This function is the [`PoseGenerator`](`PoseGenerator`). It is possible to directly define the function
//! or to combine a [`PositionGenerator`](`PositionGenerator`) and an [`OrientationGenerator`](`OrientationGenerator`).
//! Further, there are predefined PoseGenerators available. See [`pose_generators`](`crate::pose_generators`).
//!
//! You should make sure that your own PoseGenerators have a constant derivative, that means that all if you would sample
//! 1000 equidistant points, the length between neighboring points should be the same. Otherwise, some parts of
//! your trajectory will end up faster than other ones.
//!
//! PoseGenerators can be multiplied to generate a new PoseGenerator. Multiplication means that the
//! homogenous matrices of both pose generators will be multiplied together for a certain progress.
//!
//! Further, it is possible to combine PoseGenerators with the [`append`](`PoseGenerator::append`) method,
//! which will concatenate both PoseGenerators. Be aware that concatenating PoseGenerators can result in unsmooth
//! trajectories. For example, if you append a circle PoseGenerator to a linear PoseGenerator, there will be an infinite
//! jerk at the transition as the acceleration suddenly changes.
//!
//! With the final PoseGenerator it is now possible to add a VelocityProfile to it, which turns
//! the PoseGenerator into a [`CartesianTrajectory`](`crate::cartesian_trajectory::CartesianTrajectory`) which can be queried
//! with the [`get_pose`](`crate::cartesian_trajectory::CartesianTrajectory::get_pose`) method
//! with a start pose and a Duration.
//!
//! The following images shows how this library can be used to create a complex trajectory
//! ![](https://i.imgur.com/UkqynLa.png)
//!
use nalgebra::{Isometry3, UnitQuaternion, Vector3};

/// Struct that wraps a function that generates a pose
pub struct PoseGenerator(Box<dyn FnMut(&Isometry3<f64>, f64) -> Isometry3<f64>>);

/// Struct that wraps a function that generates a position
pub struct PositionGenerator(Box<dyn FnMut(&Vector3<f64>, f64) -> Vector3<f64>>);

/// Struct that wraps a function that generates an orientation
pub struct OrientationGenerator(Box<dyn FnMut(&UnitQuaternion<f64>, f64) -> UnitQuaternion<f64>>);

impl PositionGenerator {
    /// generates a new PositionGenerator by defining a function that takes
    /// an initial position and a progress between 0 and 1 and returns a position.
    pub fn new(
        position_generator_function: Box<dyn FnMut(&Vector3<f64>, f64) -> Vector3<f64>>,
    ) -> Self {
        PositionGenerator(position_generator_function)
    }
    /// Evaluates the function of the PositionGenerator
    /// # Arguments
    /// * `start_position` - the position at the start time of the PositionGenerator
    /// * `progress` - a progress from 0, indicating the start, and 1 , indicating the end of the
    /// trajectory.
    pub fn get_position(&mut self, start_position: &Vector3<f64>, progress: f64) -> Vector3<f64> {
        (self.0)(start_position, progress)
    }
    /// creates a position generator that does not move and therefore always returns the start position.
    pub fn constant_position_generator() -> Self {
        let position_generator = move |start_position: &Vector3<f64>,
                                       _progress: f64|
              -> Vector3<f64> { start_position.clone_owned() };
        PositionGenerator(Box::new(position_generator))
    }
}

impl OrientationGenerator {
    /// generates a new OrientationGenerator by defining a function that takes
    /// an initial orientation and a progress between 0 and 1 and returns an orientation.
    pub fn new(
        orientation_generator_function: Box<
            dyn FnMut(&UnitQuaternion<f64>, f64) -> UnitQuaternion<f64>,
        >,
    ) -> Self {
        OrientationGenerator(orientation_generator_function)
    }
    /// Evaluates the function of the OrientationGenerator
    /// # Arguments
    /// * `start_orientation` - the orientation at the start time of the OrientationGenerator
    /// * `progress` - a progress from 0, indicating the start, and 1 , indicating the end of the
    /// trajectory.
    pub fn get_orientation(
        &mut self,
        start_orientation: &UnitQuaternion<f64>,
        progress: f64,
    ) -> UnitQuaternion<f64> {
        (self.0)(start_orientation, progress)
    }
    /// creates an orientation generator that does not rotate and therefore always returns the start orientation.
    pub fn constant_orientation_generator() -> Self {
        let orientation_generator = move |start_orientation: &UnitQuaternion<f64>,
                                          _progress: f64|
              -> UnitQuaternion<f64> { *start_orientation };
        OrientationGenerator(Box::new(orientation_generator))
    }
}

impl PoseGenerator {
    /// generates a new PoseGenerator by defining a function that takes
    /// an initial position and a progress between 0 and 1 and returns a position.
    pub fn new(
        pose_generator_function: Box<dyn FnMut(&Isometry3<f64>, f64) -> Isometry3<f64>>,
    ) -> Self {
        PoseGenerator(pose_generator_function)
    }
    /// Generates a PoseGenerator from a position and an orientation generator.
    pub fn from_parts(
        mut position_generator: PositionGenerator,
        mut orientation_generator: OrientationGenerator,
    ) -> Self {
        let pose_generator =
            move |initial_pose: &Isometry3<f64>, progress: f64| -> Isometry3<f64> {
                let position =
                    position_generator.get_position(&initial_pose.translation.vector, progress);
                let orientation =
                    orientation_generator.get_orientation(&initial_pose.rotation, progress);
                Isometry3::from_parts(position.into(), orientation)
            };
        PoseGenerator(Box::new(pose_generator))
    }
    /// Evaluates the function of the PoseGenerator
    /// # Arguments
    /// * `start_pose` - the pose at the start time of the PoseGenerator
    /// * `progress` - a progress from 0, indicating the start, and 1 , indicating the end of the
    /// trajectory.
    pub fn get_pose(&mut self, start: &Isometry3<f64>, progress: f64) -> Isometry3<f64> {
        (self.0)(start, progress)
    }
    /// returns the approximate length of the trajectory in meter. The rotation is ignored.
    pub fn get_approximate_length(
        &mut self,
        start_pose: &Isometry3<f64>,
        sample_size: usize,
    ) -> f64 {
        (0..sample_size)
            .map(|x| {
                (
                    x as f64 / sample_size as f64,
                    (x as f64 + 1.) / sample_size as f64,
                )
            })
            .map(|(t1, t2)| {
                (self.get_pose(&start_pose, t1).translation.inverse()
                    * self.get_pose(&start_pose, t2).translation)
                    .vector
                    .norm()
            })
            .sum()
    }
    /// concatenates two pose generators.
    /// Be aware that concatenating PoseGenerators can result in unsmooth
    /// trajectories. For example, if you append a circle PoseGenerator to a linear PoseGenerator,
    /// there will be an infinite jerk at the transition as the acceleration suddenly changes.
    /// # Arguments
    /// * `start_pose` - a rough estimation of the start pose of the trajectory.
    /// * `other_generator` - the pose generator that should be appended
    pub fn append(
        mut self,
        start_pose: &Isometry3<f64>,
        mut other_generator: PoseGenerator,
    ) -> PoseGenerator {
        let length_self = self.get_approximate_length(start_pose, 10000);
        let end_pose_self = self.get_pose(start_pose, 1.);
        let length_other = other_generator.get_approximate_length(&end_pose_self, 10000);
        let length_split = length_self / (length_self + length_other);

        let pose_generator =
            move |initial_pose: &Isometry3<f64>, progress: f64| -> Isometry3<f64> {
                if progress < length_split {
                    self.get_pose(initial_pose, progress / length_split)
                } else {
                    other_generator.get_pose(
                        &end_pose_self,
                        (progress - length_split) / (1. - length_split),
                    )
                }
            };
        PoseGenerator(Box::new(pose_generator))
    }
}

impl std::ops::Mul for PoseGenerator {
    type Output = PoseGenerator;

    fn mul(mut self, mut rhs: Self) -> Self::Output {
        let pose_generator =
            move |initial_pose: &Isometry3<f64>, progress: f64| -> Isometry3<f64> {
                (self.0)(initial_pose, progress) * (rhs.0)(initial_pose, progress)
            };
        PoseGenerator(Box::new(pose_generator))
    }
}
mod cartesian_trajectory;
pub mod pose_generators;
mod velocity_profile;
pub use crate::cartesian_trajectory::{
    CartesianTrajectory, CartesianTrajectoryOutput, VelocityProfile,
};
pub use crate::pose_generators::{
    generate_absolute_motion, generate_circle_motion, generate_relative_motion, RelativeMotionFrame,
};
pub use crate::velocity_profile::{
    generate_cosine_velocity_profile, generate_linear_velocity_profile, generate_s_curve_profile,
    VelocityProfileMapping, VelocityProfileOutput,
};
pub use s_curve;
