// SPDX-License-Identifier: BSD-2-Clause
//! Mirror of Pinocchio's `overview-simple` example: build a 7R chain in
//! memory, run RNEA at the neutral configuration, print τ.

use nalgebra::DVector;
use pinocchio_rs::{Data, Model};

const XML: &str = include_str!("../tests/data/panda/panda.urdf");

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let model = Model::from_urdf_str(XML)?;
    let mut data = Data::new(&model);

    let q = model.neutral_configuration();
    let v = DVector::zeros(model.nv());
    let a = DVector::zeros(model.nv());

    let tau = data.rnea(&model, &q, &v, &a)?;
    println!("model: {:?}", model);
    println!("tau   = {:.4?}", tau.as_slice());
    Ok(())
}
