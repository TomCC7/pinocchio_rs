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
    dtau_dq: Vec<f64>,
    dtau_dv: Vec<f64>,
}

#[test]
fn parity_rnea_derivatives() {
    if maybe_skip_no_goldens("rnea_derivs_panda.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("rnea_derivs_panda.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();
    let mut data = Data::new(&model);

    for (s, sample) in env.samples.iter().enumerate() {
        let q = DVector::from_column_slice(&sample.q);
        let v = DVector::from_column_slice(&sample.v);
        let a = DVector::from_column_slice(&sample.a);
        data.rnea_derivatives(&model, &q, &v, &a).unwrap();
        let (dtau_dq, dtau_dv) = data.rnea_partials(&model);
        assert_mat_close_col_major(&dtau_dq, &sample.dtau_dq, TOL_DERIV, "rnea.dtau_dq", s);
        assert_mat_close_col_major(&dtau_dv, &sample.dtau_dv, TOL_DERIV, "rnea.dtau_dv", s);
    }
}
