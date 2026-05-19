// SPDX-License-Identifier: BSD-2-Clause
//! Shared helpers for the §14 parity test suite.

use std::path::PathBuf;

use nalgebra::{DMatrix, DVector};

// ---------------- tolerances ----------------------------------------------

/// Allowed deviation for direct algorithm outputs (FK, RNEA, ABA, CRBA, Jac,
/// SE3, Lie). See design D6 / spec `numerical-parity-suite`.
pub const TOL_DIRECT: f64 = 1e-10;

/// Allowed deviation for analytical-derivative outputs (looser because the
/// computation chains more floating-point ops).
pub const TOL_DERIV: f64 = 1e-8;

// ---------------- paths ----------------------------------------------------

pub fn goldens_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("goldens")
}

pub fn panda_urdf_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
        .join("panda")
        .join("panda.urdf")
}

// ---------------- JSON shape ----------------------------------------------

#[derive(serde::Deserialize)]
pub struct Envelope<S> {
    pub robot: String,
    pub pinocchio_version: String,
    pub seed: u64,
    pub sample_count: usize,
    pub samples: Vec<S>,
}

pub fn load_envelope<S: for<'de> serde::Deserialize<'de>>(filename: &str) -> Envelope<S> {
    let path = goldens_dir().join(filename);
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
    let env: Envelope<S> = serde_json::from_str(&text)
        .unwrap_or_else(|e| panic!("failed to parse {}: {e}", path.display()));
    assert_eq!(env.samples.len(), env.sample_count, "{filename}: sample_count mismatch");
    env
}

// ---------------- comparison helpers --------------------------------------

pub fn assert_vec_close(
    got: &DVector<f64>,
    want: &[f64],
    tol: f64,
    label: &str,
    sample_index: usize,
) {
    assert_eq!(
        got.len(),
        want.len(),
        "{label} sample {sample_index}: length mismatch (got {}, want {})",
        got.len(),
        want.len(),
    );
    for (i, (&a, &b)) in got.iter().zip(want.iter()).enumerate() {
        let d = (a - b).abs();
        if d > tol {
            panic!(
                "{label} sample {sample_index} component {i}: got {a:.17e}, want {b:.17e}, |Δ|={d:.3e} > {tol:.3e}",
            );
        }
    }
}

pub fn assert_mat_close_col_major(
    got: &DMatrix<f64>,
    want: &[f64],
    tol: f64,
    label: &str,
    sample_index: usize,
) {
    let (r, c) = got.shape();
    assert_eq!(r * c, want.len(), "{label}: shape product mismatch");
    let buf = got.as_slice(); // column-major
    for (i, (&a, &b)) in buf.iter().zip(want.iter()).enumerate() {
        let d = (a - b).abs();
        if d > tol {
            let col = i / r;
            let row = i % r;
            panic!(
                "{label} sample {sample_index} [row {row}, col {col}]: got {a:.17e}, want {b:.17e}, |Δ|={d:.3e} > {tol:.3e}",
            );
        }
    }
}

pub fn maybe_skip_no_goldens(filename: &str) -> Option<PathBuf> {
    let p = goldens_dir().join(filename);
    if p.exists() {
        Some(p)
    } else {
        eprintln!("SKIP: {filename} not found — run `pixi run gen-goldens` first");
        None
    }
}
