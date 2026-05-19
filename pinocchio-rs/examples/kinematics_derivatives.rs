// SPDX-License-Identifier: BSD-2-Clause
//! Pinocchio's `kinematics-derivatives` example.

use nalgebra::DVector;
use pinocchio_rs::{Data, Model, ReferenceFrame};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let model = Model::from_urdf(panda_urdf_path())?;
    let mut data = Data::new(&model);

    let q = model.random_configuration(7);
    let v = DVector::from_element(model.nv(), 0.05);
    let a = DVector::zeros(model.nv());

    data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)?;
    let joint = model.joint_id("panda_joint7")?;
    let (dq, dv) = data.joint_velocity_derivatives(&model, joint, ReferenceFrame::Local);

    println!("∂v/∂q shape = {:?}", dq.shape());
    println!("∂v/∂v shape = {:?}", dv.shape());
    println!("∂v/∂v ‖·‖∞ = {:.4}", dv.iter().copied().map(f64::abs).fold(0.0, f64::max));
    Ok(())
}

fn panda_urdf_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
        .join("panda")
        .join("panda.urdf")
}
