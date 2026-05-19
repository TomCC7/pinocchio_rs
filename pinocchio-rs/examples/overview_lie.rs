// SPDX-License-Identifier: BSD-2-Clause
//! Pinocchio's `overview-lie` example: integrate / difference round-trip on
//! the Panda model.

use pinocchio_rs::Model;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let model = Model::from_urdf(panda_urdf_path())?;
    let q0 = model.random_configuration(0);
    let q1 = model.random_configuration(1);

    let v = model.difference(&q0, &q1)?;
    let q_back = model.integrate(&q0, &v)?;

    println!("q0:      {:?}", q0.as_slice());
    println!("q1:      {:?}", q1.as_slice());
    println!("diff:    {:?}", v.as_slice());
    println!("q_back:  {:?}", q_back.as_slice());
    println!("max err: {:.3e}", (&q1 - &q_back).iter().copied().map(f64::abs).fold(0.0, f64::max));
    Ok(())
}

fn panda_urdf_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
        .join("panda")
        .join("panda.urdf")
}
