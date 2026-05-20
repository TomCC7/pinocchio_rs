// SPDX-License-Identifier: BSD-2-Clause
#[path = "common/mod.rs"]
mod common;

use std::collections::HashSet;

use common::*;
use nalgebra::DVector;
use pinocchio_rs::{Data, Model, ReferenceFrame};

#[derive(serde::Deserialize)]
struct Sample {
    q: Vec<f64>,
    v: Vec<f64>,
    a: Vec<f64>,
    frame_id: usize,
    rf: u8,
    dv_dq: Vec<f64>,
    dv_dv: Vec<f64>,
    da_dq: Vec<f64>,
    da_dv: Vec<f64>,
    da_da: Vec<f64>,
}

#[test]
fn parity_frame_kinematics_derivatives() {
    if maybe_skip_no_goldens("frame_kinematics_derivs_panda.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("frame_kinematics_derivs_panda.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();
    let mut data = Data::new(&model);

    let mut seen_rfs: HashSet<u8> = HashSet::new();

    for (s, sample) in env.samples.iter().enumerate() {
        let q = DVector::from_column_slice(&sample.q);
        let v = DVector::from_column_slice(&sample.v);
        let a = DVector::from_column_slice(&sample.a);
        let rf = match sample.rf {
            0 => ReferenceFrame::World,
            1 => ReferenceFrame::Local,
            2 => ReferenceFrame::LocalWorldAligned,
            other => panic!("sample {s}: unknown rf={other}"),
        };
        seen_rfs.insert(sample.rf);

        data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)
            .unwrap();

        let (dv_dq, dv_dv) = data.frame_velocity_derivatives(&model, sample.frame_id, rf);
        assert_mat_close_col_major(&dv_dq, &sample.dv_dq, TOL_DERIV, "frame.dv_dq", s);
        assert_mat_close_col_major(&dv_dv, &sample.dv_dv, TOL_DERIV, "frame.dv_dv", s);

        let (da_dq, da_dv, da_da) =
            data.frame_acceleration_derivatives(&model, sample.frame_id, rf);
        assert_mat_close_col_major(&da_dq, &sample.da_dq, TOL_DERIV, "frame.da_dq", s);
        assert_mat_close_col_major(&da_dv, &sample.da_dv, TOL_DERIV, "frame.da_dv", s);
        assert_mat_close_col_major(&da_da, &sample.da_da, TOL_DERIV, "frame.da_da", s);
    }

    // Per spec: the 50 samples must cover all three ReferenceFrame variants.
    assert!(
        seen_rfs.contains(&0) && seen_rfs.contains(&1) && seen_rfs.contains(&2),
        "expected all three ReferenceFrame variants (0/1/2) to appear; saw {seen_rfs:?}",
    );
}
