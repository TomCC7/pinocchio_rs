// SPDX-License-Identifier: BSD-2-Clause
//! Shared helpers for in-module unit tests.

use std::path::PathBuf;

/// Path to the vendored Panda URDF, resolved at compile time.
pub(crate) fn panda_urdf_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
        .join("panda")
        .join("panda.urdf")
}
