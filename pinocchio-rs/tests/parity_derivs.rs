// SPDX-License-Identifier: BSD-2-Clause
#[path = "common/mod.rs"]
mod common;

use common::*;
use nalgebra::DVector;
use pinocchio_rs::{Data, Model, ReferenceFrame};

#[derive(serde::Deserialize)]
struct Sample {
    q: Vec<f64>,
    v: Vec<f64>,
    a: Vec<f64>,
    joint_id: usize,
    rf: u8,
    dv_dq: Vec<f64>,
    dv_dv: Vec<f64>,
}

#[test]
fn parity_forward_kinematics_derivatives() {
    if maybe_skip_no_goldens("fk_derivs_panda.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("fk_derivs_panda.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();
    let mut data = Data::new(&model);

    for (s, sample) in env.samples.iter().enumerate() {
        let q = DVector::from_column_slice(&sample.q);
        let v = DVector::from_column_slice(&sample.v);
        let a = DVector::from_column_slice(&sample.a);
        data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)
            .unwrap();
        let rf = match sample.rf {
            0 => ReferenceFrame::World,
            1 => ReferenceFrame::Local,
            2 => ReferenceFrame::LocalWorldAligned,
            other => panic!("sample {s}: unknown rf={other}"),
        };
        let (dq, dv) = data.joint_velocity_derivatives(&model, sample.joint_id, rf);
        assert_mat_close_col_major(&dq, &sample.dv_dq, TOL_DERIV, "derivs.dv_dq", s);
        assert_mat_close_col_major(&dv, &sample.dv_dv, TOL_DERIV, "derivs.dv_dv", s);
    }
}
