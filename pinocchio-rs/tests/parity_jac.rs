// SPDX-License-Identifier: BSD-2-Clause
#[path = "common/mod.rs"]
mod common;

use common::*;
use nalgebra::DVector;
use pinocchio_rs::{Data, Model, ReferenceFrame};

#[derive(serde::Deserialize)]
struct Sample {
    q: Vec<f64>,
    frame_id: usize,
    rf: u8,
    #[serde(rename = "J")]
    j: Vec<f64>,
}

#[test]
fn parity_frame_jacobian_all_frames() {
    if maybe_skip_no_goldens("jac_panda.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("jac_panda.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();
    let mut data = Data::new(&model);

    let mut seen = [false; 3];
    for (s, sample) in env.samples.iter().enumerate() {
        let q = DVector::from_column_slice(&sample.q);
        let rf = match sample.rf {
            0 => ReferenceFrame::World,
            1 => ReferenceFrame::Local,
            2 => ReferenceFrame::LocalWorldAligned,
            other => panic!("sample {s}: unknown rf={other}"),
        };
        seen[sample.rf as usize] = true;
        let j = data.frame_jacobian(&model, &q, sample.frame_id, rf).unwrap();
        assert_mat_close_col_major(&j, &sample.j, TOL_DIRECT, "jac.J", s);
    }
    assert!(seen[0] && seen[1] && seen[2], "all three RF variants must appear: {seen:?}");
}
