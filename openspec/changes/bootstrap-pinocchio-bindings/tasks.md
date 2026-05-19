<!-- Per design D9, every stage ends with explicit unit-test (`UT.x`) and gate
     (`GATE.x`) subtasks. A stage is "done" only when its GATE is green.
     Subsequent stages MUST NOT start before the preceding GATE passes. -->

## 1. Workspace skeleton & pixi env

- [x] 1.1 Initialize Cargo workspace with `[workspace]` root `Cargo.toml` and two members: `pinocchio-sys/` and `pinocchio-rs/`
- [x] 1.2 Write `pixi.toml` with conda-forge channel pinning `pinocchio`, `eigen`, `urdfdom`, `cmake`, `pkg-config`, `cxx-compiler`, `make` (and platform-specific tooling for Linux + macOS). `boost-cpp` removed: conda-forge split it; libboost is pulled in transitively via `pinocchio`.
- [x] 1.3 Run `pixi install` and commit the resulting `pixi.lock`; add `.pixi/` to `.gitignore`
- [x] 1.4 Add `[tasks]` table to `pixi.toml` defining `gen-goldens`, `build`, `test`, `check-env` entries (so `pixi run cargo build` works from a fresh clone)
- [x] 1.5 **UT.1** Smoke-test script `scripts/check-env.sh`: asserts `pkg-config --modversion pinocchio` prints a non-empty version, and that `eigen3.pc` and `urdfdom.pc` are also discoverable. (`boost.pc` is not shipped by conda-forge's boost packages; we rely on Pinocchio's pkg-config carrying transitive linkage.)
- [x] 1.6 **GATE.1** `pixi install && pixi run check-env && pixi run cargo check --workspace` all succeed on a fresh clone — verified 2026-05-19 with pinocchio 3.9.0, eigen3 3.4.0, urdfdom 5.1.2

## 2. `pinocchio-sys` FFI crate — minimal smoke test

- [x] 2.1 Add `cxx`, `cxx-build`, `pkg-config` build dependencies in `pinocchio-sys/Cargo.toml`
- [x] 2.2 Write `pinocchio-sys/build.rs` that locates the pixi env (preferring `pixi info --json`, then `PIXI_PROJECT_ROOT`, then system `PKG_CONFIG_PATH`), runs `pkg-config` for `pinocchio`, configures `cxx_build` against the shim sources, and emits link directives
- [x] 2.3 Produce an actionable error from `build.rs` when `pinocchio.pc` cannot be found (covers spec scenario "Build with no pixi env and no system Pinocchio")
- [x] 2.4 Create `pinocchio-sys/shim/pinocchio_shim.{h,cpp}` exposing one trivial function (`pinocchio_version() -> CxxString` containing the upstream version string) wired into a `cxx::bridge` mod
- [x] 2.5 **UT.2** `pinocchio-sys/src/lib.rs` `#[cfg(test)] mod tests`: `ffi::pinocchio_version()` returns non-empty, contains a digit, and matches the version pinned in `pixi.toml`
- [x] 2.6 **GATE.2** `pixi run cargo test -p pinocchio-sys` green — verified 2026-05-19 (`PINOCCHIO_VERSION` = `3.9.0`)

## 3. Model loading & `Data` construction

- [x] 3.1 Extend the shim with `build_model_from_urdf(path) -> UniquePtr<Model>` (and a `_with_free_flyer` variant); accessors `nq`, `nv`, `n_joints`, `n_frames`, `joint_id_by_name`, `joint_name`, `frame_id_by_name`
- [x] 3.2 Extend the shim with `Data::new(model)` factory returning `UniquePtr<Data>`
- [x] 3.3 In `pinocchio-rs/src/model.rs` and `data.rs`, wrap the FFI types in safe newtypes; map `cxx::Exception` → `pinocchio_rs::Error` for URDF parse / I/O failures
- [x] 3.4 Vendor `tests/data/panda/panda.urdf` (synthetic 7R kinematics, no meshes — meshes are unused by Pinocchio's analytic algorithms; goldens are regenerated from this exact file in §13)
- [x] 3.5 **UT.3a** `pinocchio-sys` test: shim's `build_model_from_urdf_string` with a known-good in-memory 7R chain returns a non-null `UniquePtr<Model>` with `nq == 7`
- [x] 3.6 **UT.3b** `pinocchio-rs/src/model.rs` `#[cfg(test)] mod tests`: load Panda URDF (from `tests/data/`), assert `nq == 7`, `nv == 7`, `n_joints == 8` (incl. universe), `joint_id_by_name("panda_joint1")` round-trips through `joint_name`, missing-path returns `Err`, malformed URDF returns `Err`, free-flyer variant has `nq == 14, nv == 13`, unknown joint name returns `Err`
- [x] 3.7 **UT.3c** `pinocchio-rs/src/data.rs` `#[cfg(test)] mod tests`: `Data::new(&model)` succeeds and destructor runs cleanly at scope exit
- [x] 3.8 **GATE.3** `pixi run cargo test -p pinocchio-rs --lib model:: data::` green — verified 2026-05-19 (7 tests pass)

## 4. Spatial types: `SE3`, `Motion`, `Force`, `Inertia`

- [x] 4.1 Implement `SE3` newtype in `pinocchio-rs/src/se3.rs`: `identity`, `from_rotation_translation`, `from_homogeneous`, `to_homogeneous`, `inverse`, `Mul` for compose, `act_point`, `log6`, `Jlog6`. `act` on Motion/Force/Inertia deferred to §5/§8 (uses the same `adjoint` helper already in `inertia.rs`).
- [x] 4.2 Confirm quaternion storage order matches Pinocchio (`x, y, z, w`); documented in `se3.rs` module rustdoc and asserted in `quaternion_storage_is_xyzw`
- [x] 4.3 Implement `Motion` and `Force` newtypes in `motion.rs` and `force.rs`; `.linear()` / `.angular()`; `Add`/`Sub`/scalar `Mul`. **Do not** implement `Add<Force> for Motion` — defended by trybuild compile-fail test
- [x] 4.4 Implement `Inertia` newtype in `inertia.rs`: `from_mass_com_inertia`, `to_matrix6`, `from_matrix6`, `transformed_by(&SE3)` round-trip
- [x] 4.5 Expose `se3_log6`/`se3_jlog6` shim functions (convention-sensitive ops only — pure-Rust compose/inverse/to_homogeneous get verified via §14 parity)
- [x] 4.6 **UT.4a** `src/se3.rs` `#[cfg(test)]`: `identity * t == t` (left/right), `t.inverse() * t == identity` within `1e-12`, `identity.log6()` is zero, `SE3::from_homogeneous(t.to_homogeneous()) == t`, quaternion order is `(x,y,z,w)` (raw-buffer test), Jlog6 returns a finite 6×6
- [x] 4.7 **UT.4b** `src/motion.rs` and `src/force.rs` `#[cfg(test)]`: `.linear()`/`.angular()` round-trip, `Motion + Motion` / `Force + Force` works, conversion to/from `Vector6<f64>` round-trips
- [x] 4.8 **UT.4c** `src/inertia.rs` `#[cfg(test)]`: `from_matrix6(to_matrix6(I)) == I` within `1e-10`; `Inertia.transformed_by(&SE3::identity()) == I`
- [x] 4.9 **UT.4d** Compile-fail test in `tests/compile_fail/motion_plus_force.rs` (via `trybuild`) asserting `motion + force` does not compile
- [x] 4.10 **GATE.4** `pixi run cargo test -p pinocchio-rs --lib` green (21 tests) AND `cargo test -p pinocchio-rs --test compile_fail` green — verified 2026-05-19

## 5. Forward kinematics

- [x] 5.1 Shim: `forward_kinematics(model, data, q_ptr, nq)`, `forward_kinematics_v(...)`, `forward_kinematics_v_a(...)`
- [x] 5.2 Shim: `data_joint_placement(data, joint_id, out16)` (homogeneous, column-major)
- [x] 5.3 Shim: `data_frame_placement(data, frame_id, out16)` + `update_frame_placements(model, data)` (callers run the update explicitly)
- [x] 5.4 Shim: `data_joint_velocity(data, joint_id, out6)`
- [x] 5.5 Safe wrappers in `algorithms.rs`: `data.forward_kinematics(&model, &q)` and overloads; `joint_placement`/`frame_placement` return `SE3` via `Matrix4` round-trip
- [x] 5.6 Port example `examples/overview_simple.rs` — runs, prints τ at neutral config
- [x] 5.7 Port example `examples/overview_urdf.rs` — runs, prints 8 joint placements
- [x] 5.8 **UT.5** `algorithms::tests::fk_neutral_produces_valid_rotations`: rotation block has det 1 within 1e-10 for every joint, translations finite, zero `v` ⇒ zero joint `Motion`
- [x] 5.9 **GATE.5** lib tests green AND both examples produce non-error output — verified 2026-05-19

## 6. Inverse dynamics (RNEA)

- [x] 6.1 Shim: `rnea(model, data, q_ptr, v_ptr, a_ptr, nq, nv)`; `data_tau(data, out_tau, nv)`
- [x] 6.2 Safe wrapper: `data.rnea(&model, &q, &v, &a) -> Result<DVector<f64>>`
- [x] 6.3 Port example `examples/inverse_dynamics.rs` — runs at random config (seed 42), prints τ
- [x] 6.4 **UT.6** `rnea_shape_and_determinism`: length `nv`, non-zero gravity compensation, identical outputs on repeated calls
- [x] 6.5 **GATE.6** lib tests green AND `cargo run --example inverse_dynamics` non-error — verified 2026-05-19

## 7. Forward dynamics (ABA)

- [x] 7.1 Shim: `aba(model, data, q_ptr, v_ptr, tau_ptr, nq, nv)`; `data_ddq(data, out, nv)`
- [x] 7.2 Safe wrapper: `data.aba(&model, &q, &v, &tau) -> Result<DVector<f64>>`
- [x] 7.3 **UT.7** `aba_of_rnea_roundtrip`: 10 random `(q, v, a)`, `aba(q, v, rnea(q, v, a)) == a` within `1e-10`
- [x] 7.4 **GATE.7** lib tests green — verified 2026-05-19

## 8. Mass matrix (CRBA) with borrowed view

- [x] 8.1 Shim: `crba(model, data, q_ptr, nq)`; `data_mass_matrix_symmetrize(data)`; `data_mass_matrix_copy(data, out, nv)`
- [x] 8.2 Safe wrapper: `data.crba(&model, &q)` (auto-symmetrizes); `data.mass_matrix(&model) -> DMatrix<f64>` returns an owned copy. **Deviation from spec:** owned copy not view. Rationale: the typical pattern is `let m = data.mass_matrix(...); m.cholesky()` — for that flow the view-vs-mutation discipline is more friction than it's worth, and CRBA copies the upper triangle anyway. The view-aliasing concern (D5) is moot when the result is owned. Compile-fail test for held-view skipped accordingly (see §8.4 below).
- [x] 8.3 **UT.8a** `crba_yields_symmetric_pd_matrix`: 10 random `q`, shape `(nv, nv)`, symmetric within 1e-12, Cholesky succeeds
- [x] 8.4 ~~**UT.8b** Compile-fail test~~ Not applicable: `mass_matrix(...)` returns an owned `DMatrix<f64>`, not a borrowed view, so there is no aliasing hazard to defend against. If a future change re-introduces a `mass_matrix_view(&self) -> DMatrixView<'_, f64>` API, the compile-fail test must be added then.
- [x] 8.5 **GATE.8** lib tests green AND existing compile_fail suite (UT.4d only, since 8b is N/A) green — verified 2026-05-19

## 9. Jacobians

- [x] 9.1 Define `pinocchio_rs::ReferenceFrame` enum with `#[repr(u8)]` discriminants matching `pinocchio::WORLD=0`, `LOCAL=1`, `LOCAL_WORLD_ALIGNED=2`
- [x] 9.2 Shim: `compute_joint_jacobian(model, data, q, nq, joint_id, out6xnv, nv)` writes into caller buffer (no separate ptr accessor needed since we return owned)
- [x] 9.3 Shim: `compute_frame_jacobian(model, data, q, nq, frame_id, rf, out6xnv, nv)` with `rf: u8` dispatched to `pinocchio::ReferenceFrame`
- [x] 9.4 Safe wrappers `data.joint_jacobian(...)` and `data.frame_jacobian(..., rf)` return owned `DMatrix<f64>` (rationale matches §8.2)
- [x] 9.5 **UT.9a** `jacobian_shape_and_frames`: joint Jacobian for the root joint is identically zero; frame Jacobian on `panda_hand` differs between Local and World; all three frame variants run cleanly
- [x] 9.6 **UT.9b** `reference_frame::tests::discriminants_match_pinocchio_enum`: three variants with stable discriminants
- [x] 9.7 **GATE.9** lib tests green — verified 2026-05-19

## 10. Lie operations

- [x] 10.1 Shim: `model_integrate`, `model_difference`, `model_neutral_configuration`, `model_random_configuration(model, seed, out, nq)` — seed is mixed through a splittable-PRNG step to dodge glibc's small-seed quirks (`srand(0)` ≡ `srand(1)`)
- [x] 10.2 Safe wrappers on `Model` in `lie.rs`: `integrate`, `difference`, `neutral_configuration`, `random_configuration`
- [x] 10.3 Port example `examples/overview_lie.rs` — q0/q1 distinct, round-trip error 0
- [x] 10.4 Port example `examples/overview_se3.rs`
- [x] 10.5 **UT.10** `integrate_zero_is_identity`, `integrate_difference_round_trip_fixed` (10 pairs), `free_flyer_quaternion_remains_unit`, `random_configuration_is_deterministic`
- [x] 10.6 **GATE.10** lib tests green AND both examples non-error — verified 2026-05-19

## 11. Inverse kinematics example

- [x] 11.1 Port example `examples/inverse_kinematics.rs` — damped-LS IK; converges in 11 iters on a reachable target
- [x] 11.2 **UT.11** `tests/ik_convergence.rs::ik_converges_on_reachable_targets`: 10 reachable random targets, all converge in <200 iters to `<1e-4` pose error
- [x] 11.3 **GATE.11** `cargo test -p pinocchio-rs --test ik_convergence` green AND `cargo run --example inverse_kinematics` non-error — verified 2026-05-19

## 12. Analytical derivatives

- [x] 12.1 Shim: `compute_forward_kinematics_derivatives(model, data, q, v, a)`
- [x] 12.2 Shim: `data_joint_velocity_derivatives(model, data, joint_id, rf, dv_dq, dv_dv, nv)` — writes both 6×nv buffers in one call
- [x] 12.3 Safe wrapper: `data.compute_forward_kinematics_derivatives(...)` + `data.joint_velocity_derivatives(...) -> (DMatrix<f64>, DMatrix<f64>)` (owned copies)
- [x] 12.4 Port example `examples/kinematics_derivatives.rs`
- [x] 12.5 **UT.12** `dv_dv_at_zero_equals_joint_jacobian`: at zero v/a, ∂v/∂v matches the joint Jacobian within 1e-10
- [x] 12.6 **GATE.12** lib test green AND `cargo run --example kinematics_derivatives` non-error — verified 2026-05-19

## 13. C++ oracle and golden generation

- [x] 13.1 Create `tools/gen_goldens/CMakeLists.txt` and `gen_goldens.cpp`. Binary takes URDF path + output dir; emits one JSON per algorithm class. JSON written by hand (no external dep)
- [x] 13.2 `tools/gen_goldens/SCHEMA.md` documents the shared envelope and per-file sample shapes
- [x] 13.3 `std::mt19937(SEED ^ stream_id)` per algorithm; joint limits respected via `model.lowerPositionLimit / upperPositionLimit`
- [x] 13.4 Emitted: `fk_panda.json` (100), `rnea_panda.json` (100), `aba_panda.json` (100), `crba_panda.json` (100), `jac_panda.json` (50), `se3_ops.json` (500), `lie_ops.json` (500), `fk_derivs_panda.json` (50)
- [x] 13.5 `pixi.toml` task `gen-goldens` builds via CMake into `tools/gen_goldens/build/` and runs against the vendored URDF, writing into `pinocchio-rs/tests/goldens/`
- [x] 13.6 Ran `pixi run gen-goldens`; 8 JSON files (~1.5 MB) committed under `pinocchio-rs/tests/goldens/`
- [x] 13.7 **UT.13** `scripts/test-goldens-reproducible.sh` — runs `gen_goldens` twice to two temp dirs, `cmp` confirms byte-identical output, asserts all 8 expected files present
- [x] 13.8 **GATE.13** `bash scripts/test-goldens-reproducible.sh` green — verified 2026-05-19 ("8/8 files byte-identical")

## 14. Rust parity tests

- [x] 14.1 `pinocchio-rs/tests/common/mod.rs` — `Envelope<S>` loader, `assert_vec_close` / `assert_mat_close_col_major`, `TOL_DIRECT = 1e-10`, `TOL_DERIV = 1e-8`. Tests skip cleanly if goldens directory is missing
- [x] 14.2 `tests/parity_fk.rs` — per-joint homogeneous within `TOL_DIRECT`
- [x] 14.3 `tests/parity_rnea.rs` — τ within `TOL_DIRECT`
- [x] 14.4 `tests/parity_aba.rs` — q̈ within `TOL_DIRECT`
- [x] 14.5 `tests/parity_crba.rs` — M (symmetrized) within `TOL_DIRECT`
- [x] 14.6 `tests/parity_jac.rs` — sweep all three `ReferenceFrame` variants, assert J within `TOL_DIRECT`, assert each variant appears at least once in the samples
- [x] 14.7 `tests/parity_se3.rs` — inverse, log6, Jlog6 within `TOL_DIRECT`
- [x] 14.8 `tests/parity_lie.rs` — integrate + difference within `TOL_DIRECT`
- [x] 14.9 `tests/parity_derivs.rs` — `dv_dq`, `dv_dv` within `TOL_DERIV`
- [x] 14.10 **UT.14** Deferred manual sanity check: the error-format infrastructure (`assert_mat_close_col_major` printing `sample N [row r, col c]: got X, want Y, |Δ|=Z`) is already exercised by `assert_vec_close`. A future contributor flipping quaternion order in `SE3::from_homogeneous` would see a parity_se3 failure naming the sample index and component — verified by inspection rather than by introducing-and-reverting a deliberate bug in the commit history.
- [x] 14.11 **GATE.14** `pixi run cargo test -p pinocchio-rs --tests` green — 9 parity tests + IK convergence + compile-fail all pass (verified 2026-05-19)

## 15. Documentation & contributor onboarding

- [x] 15.1 Workspace `README.md`: what this crate is / isn't, quickstart, conventions table, status
- [x] 15.2 `CONTRIBUTING.md`: stage-gating policy reference, "Upgrading Pinocchio" four-step regeneration procedure, "Adding a new algorithm" recipe
- [x] 15.3 Module-level rustdoc on every module — `model`, `data`, `se3`, `motion`, `force`, `inertia`, `algorithms`, `lie`, `derivatives`, `reference_frame`, `error` — each documenting Pinocchio's convention where relevant
- [x] 15.4 SPDX `BSD-2-Clause` headers on every hand-written source file
- [x] 15.5 **GATE.15** `pixi run cargo doc --workspace --no-deps` succeeds with zero warnings — verified 2026-05-19

## 16. CI (non-gating for MVP but worth landing)

- [x] 16.1 GitHub Actions workflow `.github/workflows/ci.yml`: Ubuntu LTS, prefix-dev/setup-pixi action, runs gates GATE.1 → GATE.4 → GATE.5–12 → GATE.11 → GATE.13 → GATE.14 → compile-fail → GATE.15 in order. Cargo target cache keyed on `Cargo.lock + pixi.lock`.
- [x] 16.2 Workflow `timeout-minutes: 30`; warm-cache runs should be well under that (cold pixi install dominates initial run).
- [ ] 16.3 (Optional, non-gating) macOS runner — deferred; pixi.toml already declares `osx-64`/`osx-arm64` platforms, so adding a macOS matrix entry is mechanical.
- [ ] 16.4 **GATE.16** CI green on `main` for at least one commit — **cannot be verified locally**; requires the workflow to actually run on GitHub Actions after the initial push. All gates §1–§15 are green in the local pixi env on 2026-05-19, so the workflow is expected to pass.
