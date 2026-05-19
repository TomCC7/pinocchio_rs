// SPDX-License-Identifier: BSD-2-Clause
//! Pinocchio's `overview-SE3` example: compose two transforms, take inverse,
//! print log6.

use nalgebra::{Rotation3, Vector3};
use pinocchio_rs::SE3;

fn main() {
    let a = SE3::from_rotation_translation(
        Rotation3::from_euler_angles(0.1, 0.2, 0.3),
        Vector3::new(0.5, 0.0, 0.5),
    );
    let b = SE3::from_rotation_translation(
        Rotation3::from_euler_angles(-0.2, 0.0, 0.4),
        Vector3::new(0.0, 0.5, 0.0),
    );

    let c = a * b;
    println!("a * b translation = {:?}", c.translation());
    println!("(a * b).inverse() translation = {:?}", c.inverse().translation());
    println!("log6(a) = {:?}", a.log6());
}
