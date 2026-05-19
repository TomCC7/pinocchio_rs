// SPDX-License-Identifier: BSD-2-Clause
//! Mirror of Pinocchio's `overview-urdf` example: load the Panda URDF, run
//! forward kinematics at the neutral config, print every joint placement.

use pinocchio_rs::{Data, Model};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let model = Model::from_urdf(panda_urdf_path())?;
    let mut data = Data::new(&model);
    let q = model.neutral_configuration();
    data.forward_kinematics(&model, &q)?;

    println!("model: {:?}", model);
    for j in 0..model.n_joints() {
        let name = model.joint_name(j).unwrap_or_else(|_| "<unknown>".into());
        let t = data.joint_placement(j).translation();
        println!("  joint {j:>2} {name:<24} translation=[{:.4}, {:.4}, {:.4}]", t.x, t.y, t.z);
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
