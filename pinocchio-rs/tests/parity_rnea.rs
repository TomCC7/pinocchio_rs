// SPDX-License-Identifier: BSD-2-Clause
#[path = "common/mod.rs"]
mod common;

use common::*;
use nalgebra::DVector;
use pinocchio_rs::{Data, Model};

#[derive(serde::Deserialize)]
struct Sample {
    q: Vec<f64>,
    v: Vec<f64>,
    a: Vec<f64>,
    tau: Vec<f64>,
}

#[test]
fn parity_rnea() {
    if maybe_skip_no_goldens("rnea_panda.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("rnea_panda.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();
    let mut data = Data::new(&model);

    for (s, sample) in env.samples.iter().enumerate() {
        let q = DVector::from_column_slice(&sample.q);
        let v = DVector::from_column_slice(&sample.v);
        let a = DVector::from_column_slice(&sample.a);
        let tau = data.rnea(&model, &q, &v, &a).unwrap();
        assert_vec_close(&tau, &sample.tau, TOL_DIRECT, "rnea.tau", s);
    }
}
