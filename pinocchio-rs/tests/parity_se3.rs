// SPDX-License-Identifier: BSD-2-Clause
#[path = "common/mod.rs"]
mod common;

use common::*;
use nalgebra::{Matrix4, Vector6};
use pinocchio_rs::SE3;

#[derive(serde::Deserialize)]
struct Sample {
    m_in: Vec<f64>,
    inv: Vec<f64>,
    log6: Vec<f64>,
    jlog6: Vec<f64>,
}

#[test]
fn parity_se3_ops() {
    if maybe_skip_no_goldens("se3_ops.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("se3_ops.json");
    for (s, sample) in env.samples.iter().enumerate() {
        let m_in = Matrix4::from_column_slice(&sample.m_in);
        let t = SE3::from_homogeneous(m_in);
        // inverse
        let inv_got = t.inverse().to_homogeneous();
        let inv_got_slice: [f64; 16] = inv_got.as_slice().try_into().unwrap();
        for (k, (&a, &b)) in inv_got_slice.iter().zip(&sample.inv).enumerate() {
            let d = (a - b).abs();
            assert!(
                d <= TOL_DIRECT,
                "se3.inv sample {s} entry {k}: got {a:.17e}, want {b:.17e}, |Δ|={d:.3e}",
            );
        }
        // log6
        let lg = t.log6();
        for (k, (&a, &b)) in lg.iter().zip(&sample.log6).enumerate() {
            let d = (a - b).abs();
            assert!(
                d <= TOL_DIRECT,
                "se3.log6 sample {s} component {k}: got {a:.17e}, want {b:.17e}, |Δ|={d:.3e}",
            );
        }
        // jlog6
        let jl = t.jlog6();
        let jl_slice = jl.as_slice();
        for (k, (&a, &b)) in jl_slice.iter().zip(&sample.jlog6).enumerate() {
            let d = (a - b).abs();
            assert!(
                d <= TOL_DIRECT,
                "se3.jlog6 sample {s} entry {k}: got {a:.17e}, want {b:.17e}, |Δ|={d:.3e}",
            );
        }
        let _ = Vector6::<f64>::zeros(); // anchor import
    }
}
