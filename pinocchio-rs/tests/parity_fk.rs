// SPDX-License-Identifier: BSD-2-Clause
//! §14 — forward-kinematics parity.

#[path = "common/mod.rs"]
mod common;

use common::*;
use nalgebra::{DVector, Matrix4};
use pinocchio_rs::{Data, Model};

#[derive(serde::Deserialize)]
struct Sample {
    q: Vec<f64>,
    /// njoints * 16 column-major doubles.
    #[serde(rename = "oMi")]
    o_m_i: Vec<f64>,
}

#[test]
fn parity_forward_kinematics() {
    if maybe_skip_no_goldens("fk_panda.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("fk_panda.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();
    let mut data = Data::new(&model);
    let njoints = model.n_joints();

    for (s, sample) in env.samples.iter().enumerate() {
        let q = DVector::from_column_slice(&sample.q);
        data.forward_kinematics(&model, &q).unwrap();
        // Compare each joint's homogeneous matrix.
        for j in 0..njoints {
            let h = data.joint_placement(j).to_homogeneous();
            let want_slice = &sample.o_m_i[j * 16..(j + 1) * 16];
            let got_slice: [f64; 16] = h
                .as_slice()
                .try_into()
                .expect("Matrix4 has 16 doubles");
            for (k, (&a, &b)) in got_slice.iter().zip(want_slice).enumerate() {
                let d = (a - b).abs();
                assert!(
                    d <= TOL_DIRECT,
                    "fk sample {s} joint {j} entry {k}: got {a:.17e}, want {b:.17e}, |Δ|={d:.3e}",
                );
            }
            let _ = Matrix4::<f64>::identity(); // anchor the import
        }
    }
}
