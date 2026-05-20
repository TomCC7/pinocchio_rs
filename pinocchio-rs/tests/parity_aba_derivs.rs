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
    dddq_dq: Vec<f64>,
    dddq_dv: Vec<f64>,
}

#[test]
fn parity_aba_derivatives() {
    if maybe_skip_no_goldens("aba_derivs_panda.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("aba_derivs_panda.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();
    let mut data = Data::new(&model);

    for (s, sample) in env.samples.iter().enumerate() {
        let q = DVector::from_column_slice(&sample.q);
        let v = DVector::from_column_slice(&sample.v);
        let tau = DVector::from_column_slice(&sample.tau);
        data.aba_derivatives(&model, &q, &v, &tau).unwrap();
        let (dddq_dq, dddq_dv) = data.aba_partials(&model);
        assert_mat_close_col_major(&dddq_dq, &sample.dddq_dq, TOL_DERIV, "aba.dddq_dq", s);
        assert_mat_close_col_major(&dddq_dv, &sample.dddq_dv, TOL_DERIV, "aba.dddq_dv", s);
    }
}
