// SPDX-License-Identifier: BSD-2-Clause
#[path = "common/mod.rs"]
mod common;

use common::*;
use nalgebra::DVector;
use pinocchio_rs::Model;

#[derive(serde::Deserialize)]
struct Sample {
    q0: Vec<f64>,
    q1: Vec<f64>,
    v: Vec<f64>,
    q_back: Vec<f64>,
}

#[test]
fn parity_lie_ops() {
    if maybe_skip_no_goldens("lie_ops.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("lie_ops.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();

    for (s, sample) in env.samples.iter().enumerate() {
        let q0 = DVector::from_column_slice(&sample.q0);
        let q1 = DVector::from_column_slice(&sample.q1);
        let v_got = model.difference(&q0, &q1).unwrap();
        assert_vec_close(&v_got, &sample.v, TOL_DIRECT, "lie.diff(q0,q1)", s);
        let qb = model.integrate(&q0, &v_got).unwrap();
        assert_vec_close(&qb, &sample.q_back, TOL_DIRECT, "lie.integrate", s);
    }
}
