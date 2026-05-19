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
    tau: Vec<f64>,
    ddq: Vec<f64>,
}

#[test]
fn parity_aba() {
    if maybe_skip_no_goldens("aba_panda.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("aba_panda.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();
    let mut data = Data::new(&model);

    for (s, sample) in env.samples.iter().enumerate() {
        let q = DVector::from_column_slice(&sample.q);
        let v = DVector::from_column_slice(&sample.v);
        let tau = DVector::from_column_slice(&sample.tau);
        let ddq = data.aba(&model, &q, &v, &tau).unwrap();
        assert_vec_close(&ddq, &sample.ddq, TOL_DIRECT, "aba.ddq", s);
    }
}
