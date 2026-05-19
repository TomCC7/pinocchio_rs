// SPDX-License-Identifier: BSD-2-Clause
//! UT.4d / UT.8b — type-system claims defended with `trybuild`.

#[test]
fn ui() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/compile_fail/motion_plus_force.rs");
}
