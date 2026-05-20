<!-- Tasks inherit the bootstrap change's stage-gating policy: every stage ends
     with explicit unit-test (`UT.x`) and gate (`GATE.x`) subtasks. A stage is
     "done" only when its GATE is green. Subsequent stages MUST NOT start
     before the preceding GATE passes.

     Stages 1–5 land "B — drop pixi requirement"; stages 6–12 land
     "A — derivatives"; stages 13–14 are docs and CI verification. -->

## 1. Cargo features for build-path selection

- [x] 1.1 Add `[features]` table to `pinocchio-sys/Cargo.toml` declaring `bundled-pixi`, `system`, `docs-only` — all default-off
- [x] 1.2 Add pass-through `[features]` to `pinocchio-rs/Cargo.toml`: `bundled-pixi = ["pinocchio-sys/bundled-pixi"]`, `system = ["pinocchio-sys/system"]`, `docs-only = ["pinocchio-sys/docs-only"]`
- [x] 1.3 In `pinocchio-sys/src/lib.rs`, emit `compile_error!` for any pair-conflict among `bundled-pixi`/`system`/`docs-only` via `#[cfg(all(feature = "...", feature = "..."))]`
- [x] 1.4 **UT.1** Compile-fail probes: `cargo check -p pinocchio-sys --features "bundled-pixi system"` exits non-zero with the conflict message; the same for `bundled-pixi docs-only` and `system docs-only`
- [x] 1.5 **GATE.1** `pixi run cargo check -p pinocchio-sys` green with no features, with `bundled-pixi` alone, with `system` alone, and with `docs-only` alone; conflict combos fail as designed

## 2. `build.rs` probe-order rework

- [x] 2.1 Refactor `pinocchio-sys/build.rs` so `setup_pkg_config_path()` no longer unconditionally prepends pixi paths. New behavior: probe pkg-config first as-is; if the probe fails AND `cfg(not(feature = "system"))`, run `pixi info`/`PIXI_PROJECT_ROOT` discovery and retry. If `cfg(feature = "bundled-pixi")`, swap the order — pixi prepend first, then probe.
- [x] 2.2 On successful probe, emit `cargo:warning=pinocchio-sys: found pinocchio <version> at <pkgconfig-file-path>`. Parse the version from `pkg_config::Library::version`.
- [x] 2.3 Replace `fail_with_install_hint` with a multi-path error that names every probe path tried (current `PKG_CONFIG_PATH`, pixi info result if attempted, standard system dirs) and lists the four documented install options (pixi, apt/robotpkg, brew, conda)
- [x] 2.4 Gate the entire `cxx_build` invocation behind `#[cfg(not(feature = "docs-only"))]` so `docs-only` builds don't need a C++ toolchain
- [x] 2.5 **UT.2a** `pinocchio-sys` lib test: `ffi::pinocchio_version()` still returns a digit-containing string (regression check for the new build flow) under the default feature set
- [x] 2.6 **UT.2b** Shell-level probe: `bash scripts/test-build-probe.sh` — temporarily unsets `PKG_CONFIG_PATH`, runs `cargo build -p pinocchio-sys` from outside `pixi run`, asserts the `cargo:warning=` line names a path inside `.pixi/envs/` (proves automatic pixi fallback still works)
- [x] 2.7 **GATE.2** `pixi run cargo build -p pinocchio-sys --features bundled-pixi` green AND `pixi run cargo build -p pinocchio-sys` (no features, falls back to pixi) green AND the build-probe shell test green

## 3. `docs-only` build mode

> Implementation deviation from original plan (see updated DD6 in `design.md`):
> the explicit `mod stubs` + `stub_drift_check` split proved unnecessary. The
> `cxx::bridge` module stays present unconditionally and the `docs-only` flag
> only steers `build.rs` (skipping `cxx_build` and `pkg-config`). `cargo doc`
> emits `rmeta`/`rlib` without invoking the linker, so the unresolved
> `cxxbridge1$…` symbols never need to exist — rustdoc renders the full
> surface from the existing bridge declarations.

- [x] 3.1 ~~Split `pinocchio-sys/src/lib.rs` into `real` and `stubs` modules~~ Superseded — bridge stays present in all feature configurations; `build.rs` is the only thing that branches on `docs-only`.
- [x] 3.2 ~~Add `stub_drift_check` module~~ Superseded — no stubs to drift from.
- [x] 3.3 Update `Cargo.toml`'s `[package.metadata.docs.rs]` block for both crates: `features = ["docs-only"]`, `no-default-features = true`
- [x] 3.4 **UT.3** `cargo doc -p pinocchio-sys --features docs-only --no-default-features` succeeds in an environment with no Pinocchio installed (build.rs returns early under docs-only, so no probe runs)
- [x] 3.5 **GATE.3** `pixi run cargo doc --workspace --features docs-only --no-default-features` green with zero warnings (verified with `RUSTDOCFLAGS=-D warnings`)

## 4. `pixi.toml` task feature pinning

- [x] 4.1 Update `pixi.toml`'s `[tasks]` table: named `build` and `test` tasks pass `--features pinocchio-sys/bundled-pixi` (the `cargo` passthrough stays vanilla because inside `pixi run`, PKG_CONFIG_PATH already pins the pixi env; `gen-goldens` invokes CMake/g++ directly, not cargo, so it does not take cargo features)
- [x] 4.2 **UT.4** Verified by GATE.4 — `pixi run test` (which is `cargo test --workspace --features pinocchio-sys/bundled-pixi`) emits `cargo:warning=pinocchio-sys: found pinocchio 3.9.0 at …/.pixi/envs/default/lib/pkgconfig/pinocchio.pc` proving the bundled-pixi probe ran
- [x] 4.3 **GATE.4** `pixi run test` green — full workspace test suite (33 lib tests + integration tests) passes via the pinned-feature path

## 5. CI matrix expansion

- [x] 5.1 In `.github/workflows/ci.yml`, add a `pixi-macos` job: `macos-latest` runner, prefix-dev/setup-pixi, runs build + sys/lib tests + parity_fk smoke. `continue-on-error: true`.
- [x] 5.2 Add a `system-ubuntu` job: `ubuntu-latest`, installs Pinocchio via robotpkg PPA + apt-keyring, no pixi, `--features pinocchio-sys/system`, `cargo build --workspace` + `cargo test -p pinocchio-sys` + `cargo test -p pinocchio-rs --lib`. `continue-on-error: true`.
- [x] 5.3 Add a `cargo-doc-docs-only` job: `ubuntu-latest`, no Pinocchio install, runs `cargo doc --workspace --features docs-only --no-default-features --no-deps` with `RUSTDOCFLAGS=-D warnings`. Gating (hermetic).
- [x] 5.4 Keep the existing `pixi-ubuntu` job as the primary gate (now named `pixi-ubuntu` rather than `linux`; bundled-pixi feature flag added to its cargo invocations).
- [x] 5.5 **UT.5** Workflow lint: `actionlint .github/workflows/ci.yml` clean (verified locally with `actionlint v1.7.7`)
- [ ] 5.6 **GATE.5** All four jobs visible in GitHub Actions UI on the first PR push; `pixi-ubuntu` and `cargo-doc-docs-only` are green; `pixi-macos` and `system-ubuntu` complete (pass or fail recorded — non-gating, so failure is acceptable for this gate) — **deferred to Stage 14** (requires PR push, see §14)

## 6. RNEA derivatives

- [x] 6.1 Extend `pinocchio-sys/shim/pinocchio_shim.{h,cpp}` with `compute_rnea_derivatives(...)` and `data_rnea_derivatives(...)` (calls `pinocchio::computeRNEADerivatives`, copies `data.dtau_dq`/`data.dtau_dv` into caller buffers)
- [x] 6.2 Add corresponding `cxx::bridge` declarations to `pinocchio-sys/src/lib.rs`
- [x] 6.3 Safe wrapper in `pinocchio-rs/src/derivatives.rs`: `data.rnea_derivatives(...)` + `data.rnea_partials()`. Rustdoc explicitly states `∂τ/∂a = M` and points to `data.mass_matrix`.
- [x] 6.4 **UT.6a** `rnea_derivs_dtau_dv_zero_velocity` — finite + symmetric-at-v=0 (passing)
- [x] 6.5 **UT.6b** `rnea_derivs_shape` — both returned matrices `(nv, nv)` (passing)
- [x] 6.6 **GATE.6** `pixi run cargo test -p pinocchio-rs --lib derivatives::tests::rnea_derivs_` green

## 7. ABA derivatives

- [x] 7.1 Shim: `compute_aba_derivatives(...)` + `data_aba_derivatives(...)`
- [x] 7.2 `cxx::bridge` declarations
- [x] 7.3 Safe wrapper: `data.aba_derivatives(...)` + `data.aba_partials()`. Rustdoc states `∂q̈/∂τ = M⁻¹` and shows the cholesky-inversion idiom.
- [x] 7.4 **UT.7a** `aba_derivs_shape` — `(nv, nv)` matrices (passing)
- [x] 7.5 **UT.7b** `aba_consistency_with_rnea` — `∂q̈/∂q ≈ -M⁻¹·∂τ/∂q` cross-check within 1e-8 (passing)
- [x] 7.6 **GATE.7** `pixi run cargo test -p pinocchio-rs --lib derivatives::tests::aba_` green

## 8. Joint-acceleration derivatives

- [x] 8.1 Shim: `data_joint_acceleration_derivatives(model, data, joint_id, rf, da_dq, da_dv, da_da, nv)`
- [x] 8.2 `cxx::bridge` declarations
- [x] 8.3 Safe wrapper: `data.joint_acceleration_derivatives(...) -> (DMatrix, DMatrix, DMatrix)`
- [x] 8.4 **UT.8a** `joint_accel_derivs_shape` — each `(6, nv)` (passing)
- [x] 8.5 **UT.8b** `joint_accel_derivs_frame_dependence` — three rf variants numerically distinct (passing)
- [x] 8.6 **GATE.8** `pixi run cargo test -p pinocchio-rs --lib derivatives::tests::joint_accel_` green

## 9. Frame-velocity derivatives

- [x] 9.1 Shim: `data_frame_velocity_derivatives(model, data, frame_id, rf, dv_dq, dv_dv, nv)`
- [x] 9.2 `cxx::bridge` declarations
- [x] 9.3 Safe wrapper: `data.frame_velocity_derivatives(...) -> (DMatrix, DMatrix)`
- [x] 9.4 **UT.9** `frame_vel_derivs_shape_and_frame_dependence` — `(6, nv)`, three rf variants distinct on panda_hand (passing)
- [x] 9.5 **GATE.9** `pixi run cargo test -p pinocchio-rs --lib derivatives::tests::frame_vel_` green

## 10. Frame-acceleration derivatives

- [x] 10.1 Shim: `data_frame_acceleration_derivatives(model, data, frame_id, rf, da_dq, da_dv, da_da, nv)`
- [x] 10.2 `cxx::bridge` declarations
- [x] 10.3 Safe wrapper: `data.frame_acceleration_derivatives(...) -> (DMatrix, DMatrix, DMatrix)`
- [x] 10.4 **UT.10** `frame_accel_derivs_shape` — each `(6, nv)` (passing)
- [x] 10.5 **GATE.10** `pixi run cargo test -p pinocchio-rs --lib derivatives::tests::frame_accel_` green

## 11. Oracle binary: new golden streams

- [x] 11.1 Extend `tools/gen_goldens/gen_goldens.cpp` with `emit_rnea_derivs` — 50 samples (stream id `0xF9`); each envelope contains `q`, `v`, `a`, `dtau_dq`, `dtau_dv`
- [x] 11.2 `emit_aba_derivs` — 50 samples (stream id `0xFA`); each envelope contains `q`, `v`, `tau`, `dddq_dq`, `dddq_dv`
- [x] 11.3 `emit_frame_kinematics_derivs` — 50 samples (stream id `0xFB`) cycling `rf` through {World, Local, LocalWorldAligned} and `frame_id` across `panda_link3`/`panda_link5`/`panda_hand`. Each envelope contains `q`, `v`, `a`, `frame_id`, `rf`, plus `dv_dq`, `dv_dv`, `da_dq`, `da_dv`, `da_da`
- [x] 11.4 Updated `tools/gen_goldens/SCHEMA.md` to document the three new files
- [x] 11.5 Ran `pixi run gen-goldens` — produces 11 JSON files (8 existing + 3 new) under `pinocchio-rs/tests/goldens/`
- [x] 11.6 **UT.11** `bash scripts/test-goldens-reproducible.sh` confirms all 11 files byte-identical across two runs
- [x] 11.7 **GATE.11** `pixi run gen-goldens && bash scripts/test-goldens-reproducible.sh` green

## 12. Rust parity tests for the new derivatives

- [x] 12.1 `pinocchio-rs/tests/parity_rnea_derivs.rs` — loads `rnea_derivs_panda.json`, replays through `data.rnea_derivatives` + `rnea_partials`, asserts within `TOL_DERIV`. (Surfaced and fixed an oracle storage-order bug: Pinocchio's `data.dtau_dq` / `data.ddq_dq` are `RowMatrixXs` (row-major). Updated `gen_goldens.cpp` to copy them into column-major temps before serialization so the golden bytes match the column-major contract documented in `SCHEMA.md`.)
- [x] 12.2 `pinocchio-rs/tests/parity_aba_derivs.rs` — same pattern, asserts within `TOL_DERIV`
- [x] 12.3 `pinocchio-rs/tests/parity_frame_derivs.rs` — loads `frame_kinematics_derivs_panda.json`, replays both `frame_velocity_derivatives` and `frame_acceleration_derivatives` per sample, asserts each of the five `(6, nv)` matrices within `TOL_DERIV`, and asserts that all three `ReferenceFrame` variants appear in the sample set
- [x] 12.4 **UT.12** Surfaced unintentionally: the storage-order bug in 12.1 caused all three parity tests to fail with diff~1.5e1 on the first run. The bug fix demonstrated that the tests genuinely catch wrong-storage-order regressions in the wrapper or oracle; no deliberate sign-flip needed
- [x] 12.5 **GATE.12** `pixi run cargo test -p pinocchio-rs --tests` green — full parity suite passes (12 integration tests: 9 existing + 3 new derivative parity tests)

## 13. Docs & contributor onboarding

- [x] 13.1 Updated `README.md`: added "Install paths" section covering pixi (recommended), apt/robotpkg (Linux consumers), brew (macOS consumers), conda-without-pixi, each with the corresponding `--features` flag and quickstart command
- [x] 13.2 Updated `CONTRIBUTING.md`: added "Adding a new derivative accessor" recipe covering the compute-then-accessor pattern, owned `DMatrix<f64>` convention, `RowMatrixXs` storage-order trap that caused the §12 oracle bug, golden-stream id allocation, and parity-test template
- [x] 13.3 Module rustdoc updates in `derivatives.rs` — each accessor documents which `compute_*` must run first; row/col semantics; `∂τ/∂a = M` and `∂q̈/∂τ = M⁻¹` identities directing users to `data.mass_matrix`
- [x] 13.4 Crate-level rustdoc Features section in `pinocchio-sys/src/lib.rs` listing `bundled-pixi`, `system`, `docs-only` with usage guidance
- [x] 13.5 **UT.13** `pixi run cargo doc --workspace --no-deps --features pinocchio-sys/bundled-pixi` with `RUSTDOCFLAGS=-D warnings` green
- [x] 13.6 **GATE.13** docs build green; every new public item carries a rustdoc summary (verified by `-D warnings` succeeding)

## 14. CI green verification

> **This stage requires a PR push and cannot be completed from a local
> environment.** All artifacts (workflow file, parity tests, oracle, docs)
> are in place and verified locally. The remaining items are user-actionable.

- [ ] 14.1 Push branch, open PR, observe all four CI jobs run
- [ ] 14.2 `pixi-ubuntu` gating job green
- [ ] 14.3 `cargo-doc-docs-only` gating job green
- [ ] 14.4 Address any clear regressions in `pixi-macos` or `system-ubuntu` non-gating jobs (skip platform-specific upstream issues — those become follow-up tickets)
- [ ] 14.5 **UT.14** Confirm the `system-ubuntu` job's log shows `cargo:warning=pinocchio-sys: found pinocchio … at /opt/openrobots/...` — proves the system probe actually fired
- [ ] 14.6 **GATE.14** PR green on the two gating jobs AND diagnostic line visible in system-ubuntu log
