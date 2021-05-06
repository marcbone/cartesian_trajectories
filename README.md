# cartesian_trajectories

cartesian_trajectories allows creating arbitrary cartesian trajectories and adding a velocity profile on top of it.



## Example
```rust
use cartesian_trajectories::generate_absolute_motion;
use cartesian_trajectories::s_curve::SCurveConstraints;
use cartesian_trajectories::CartesianTrajectory;
use cartesian_trajectories::VelocityProfile::SCurve;
use nalgebra::Isometry3;
use std::time::Duration;

fn main() {
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
```

This example will generate a trajectory that moves from (2,2,0) to (1,1,0) with an [SCurve](https://github.com/marcbone/s_curve#what-asfdis-an-s-curve) motion.
The trajectory can then be queried with `trajectory.get_pose()`, which takes the start pose and the current
time. The start pose is needed as the trajectory is not precalculated.


## Features
Predefined PoseGenerators for:
* Relative motions
* Absolute motions
* Circle motions

Further, it is possible to define custom position and orientation generators
and combine them to a pose generator. PoseGenerators can be combined by
multiplication, for example, to create a spiral movement from a circle and a relative motion.
It is also possible to concatenate multiple PoseGenerators.


A cartesian trajectory can then be created either by applying a custom velocity profile or using one of the following
predefined velocity profiles:
* Linear
* Cosine
* SCurve

## Licence
This library is copyrighted © 2021 Marco Boneberger


Licensed under the EUPL, Version 1.2 or – as soon they will be approved by the European Commission - subsequent versions of the EUPL (the "Licence");

You may not use this work except in compliance with the Licence.
You may obtain a copy of the Licence at:

[https://joinup.ec.europa.eu/software/page/eupl](https://joinup.ec.europa.eu/software/page/eupl)

Unless required by applicable law or agreed to in writing, software distributed under the Licence is distributed on an "AS IS" basis
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the Licence for the specific language governing permissions and limitations under the Licence.
