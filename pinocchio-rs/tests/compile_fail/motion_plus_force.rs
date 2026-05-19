// SPDX-License-Identifier: BSD-2-Clause
//! UT.4d — `Motion + Force` must not compile.
//!
//! Mixing spatial velocities with spatial wrenches is a *unit error* in spatial
//! algebra; refusing it at compile time is one of the main reasons for the
//! newtype split (per design D4).

use pinocchio_rs::{Force, Motion};

fn main() {
    let m = Motion::zero();
    let f = Force::zero();
    let _bad = m + f;
}
