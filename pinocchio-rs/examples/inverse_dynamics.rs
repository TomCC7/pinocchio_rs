// SPDX-License-Identifier: BSD-2-Clause
//! Pinocchio's `inverse-dynamics` example. Builds the Panda model and runs
//! RNEA on a random configuration with a small reference acceleration.

use nalgebra::DVector;
use pinocchio_rs::{Data, Model};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let model = Model::from_urdf(panda_urdf_path())?;
    let mut data = Data::new(&model);

    let q = model.random_configuration(42);
    let v = DVector::zeros(model.nv());
    let a = DVector::from_element(model.nv(), 0.1);

    let tau = data.rnea(&model, &q, &v, &a)?;
    println!("inverse dynamics result for random q (seed 42):");
    for (i, t) in tau.iter().enumerate() {
        println!("  tau[{i}] = {t:>12.6}");
    }
    Ok(())
}

fn panda_urdf_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
        .join("panda")
        .join("panda.urdf")
}
