// SPDX-License-Identifier: BSD-2-Clause
#[path = "common/mod.rs"]
mod common;

use common::*;
use nalgebra::DVector;
use pinocchio_rs::{Data, Model};

#[derive(serde::Deserialize)]
struct Sample {
    q: Vec<f64>,
    /// nv*nv doubles, column-major.
    #[serde(rename = "M")]
    m: Vec<f64>,
}

#[test]
fn parity_crba() {
    if maybe_skip_no_goldens("crba_panda.json").is_none() {
        return;
    }
    let env = load_envelope::<Sample>("crba_panda.json");
    let model = Model::from_urdf(panda_urdf_path()).unwrap();
    let mut data = Data::new(&model);

    for (s, sample) in env.samples.iter().enumerate() {
        let q = DVector::from_column_slice(&sample.q);
        data.crba(&model, &q).unwrap();
        let m = data.mass_matrix(&model);
        assert_mat_close_col_major(&m, &sample.m, TOL_DIRECT, "crba.M", s);
    }
}
