## Why

The MVP ships exactly one analytical-derivative entry point (`computeForwardKinematicsDerivatives`) — yet the unique value of a Pinocchio binding for control and optimization is the *full* derivative suite (RNEA, ABA, frame-level kinematics). Without ∂τ/∂q and ∂q̈/∂q, the binding cannot meaningfully drive iLQR / DDP / MPC workloads, which is the audience this crate exists to serve. Separately, the only supported build path requires `pixi install`, which is a sharp adoption barrier for anyone integrating into an existing ROS, system-package, or pre-existing conda workflow, and it blocks publishing usable rustdoc to docs.rs. Both gaps need to close before a 0.2 release on crates.io.

## What Changes

**A — Round out the analytical-derivatives surface.**
- Bind `pinocchio::computeRNEADerivatives` and expose ∂τ/∂q, ∂τ/∂v, ∂τ/∂a (the last equals the mass matrix M — wrapper documents this and reuses the existing CRBA path rather than duplicating).
- Bind `pinocchio::computeABADerivatives` and expose ∂q̈/∂q, ∂q̈/∂v, ∂q̈/∂τ (= M⁻¹).
- Bind `getJointAccelerationDerivatives` for per-joint ∂a/∂q, ∂a/∂v in all three `ReferenceFrame` variants.
- Bind `getFrameVelocityDerivatives` and `getFrameAccelerationDerivatives` for per-frame partials, all three `ReferenceFrame` variants.
- Extend goldens with three new streams: `rnea_derivs_panda.json` (50 samples), `aba_derivs_panda.json` (50 samples), `frame_kinematics_derivs_panda.json` (50 samples × 3 frames).
- Parity tests for each, tolerance `TOL_DERIV = 1e-8`.

**B — Remove the hard pixi requirement (system-build mode).**
- `build.rs` accepts any `pkg-config`-discoverable Pinocchio: pixi env first (current behavior, unchanged for `pixi run cargo build`), then `PKG_CONFIG_PATH`, then standard system paths.
- Cargo features on `pinocchio-sys`: `system` (default, pkg-config only), `bundled-pixi` (preserves current pixi-env probing), `docs-only` (compiles cxx bridge to `unimplemented!()` stubs so docs.rs can render).
- `build.rs` emits a `cargo:warning=` line stating *which* Pinocchio was found and *where*, so misconfigured environments are immediately debuggable.
- CI matrix gains a system-install job on Ubuntu LTS (`apt install robotpkg-pinocchio` via robotpkg PPA) alongside the existing pixi job. macOS pixi job lands too, picking up the `osx-arm64` platform already declared in `pixi.toml`.
- README and `CONTRIBUTING.md` document three install paths: pixi (dev recommended), apt (Linux consumers), brew (macOS consumers).

**Non-goals (still deferred):** `f32` / `CppAD` / `CasADi` scalars, SDF/MJCF loaders, `GeometryModel` / collision, `constraintDynamics`, Windows support, centroidal-dynamics derivatives (`dccrba` — separate follow-up because the centroidal *forward* path isn't in MVP either).

## Capabilities

### New Capabilities
None — both A and B extend or modify capabilities introduced by the bootstrap change.

### Modified Capabilities
- `analytical-derivatives`: add RNEA, ABA, joint-acceleration, frame-velocity, and frame-acceleration derivative entry points with parity coverage.
- `pinocchio-bindings-build`: relax the hard pixi requirement — `build.rs` succeeds against any pkg-config-discoverable Pinocchio. Add cargo features `system` (default), `bundled-pixi`, `docs-only`. Extend the CI matrix.
- `numerical-parity-suite`: add three new golden streams (rnea_derivs, aba_derivs, frame_kinematics_derivs) at `TOL_DERIV = 1e-8`.

## Impact

**New code:**
- `pinocchio-sys/shim/pinocchio_shim.{h,cpp}`: ~6 new C++ entry points (RNEA derivs + accessors, ABA derivs + accessors, joint-accel getter, frame-velocity getter, frame-acceleration getter), ~150–200 LOC.
- `pinocchio-sys/src/lib.rs`: corresponding `cxx::bridge` declarations.
- `pinocchio-rs/src/derivatives.rs`: ~6–8 new safe-API functions, each returning owned `DMatrix<f64>` consistent with the §12 wrapper pattern.
- `tools/gen_goldens/gen_goldens.cpp`: emit three new JSON streams against the vendored Panda URDF.
- `pinocchio-rs/tests/parity_rnea_derivs.rs`, `parity_aba_derivs.rs`, `parity_frame_derivs.rs`.

**Modified code:**
- `pinocchio-sys/Cargo.toml`: declare features `system` (default), `bundled-pixi`, `docs-only`.
- `pinocchio-sys/build.rs`: reorder probe (pkg-config first, then pixi info, then error); emit "found at <path>" diagnostic via `cargo:warning=`; under `docs-only`, skip the cxx-build step entirely.
- `pinocchio-sys/src/lib.rs`: `#[cfg(feature = "docs-only")]` shim that replaces the cxx bridge with stub functions that `unimplemented!()` — only the type signatures need to compile for rustdoc.
- `pinocchio-rs/Cargo.toml`: pass-through features so downstream can pick the same.
- `.github/workflows/ci.yml`: add `system-ubuntu` job (robotpkg-pinocchio) and `pixi-macos` job; keep `pixi-ubuntu` as the gating job.
- `README.md`, `CONTRIBUTING.md`: document the three install paths and which feature flag matches.

**External dependencies:** no change to the *set*. `pixi-ubuntu` CI remains the primary signal; `system-ubuntu` is non-gating initially to catch silent breakage.

**Breaking changes:** none.
- Default feature switch (`bundled-pixi` → `system`) is observed only by people who build *outside* `pixi run`; inside a pixi env the discovery order still finds the pixi env first and behavior is identical.
- All MVP examples and tests continue to pass on the existing pixi setup with no Cargo.toml edits.

**Risk areas:**
- `docs-only` stub mode is non-trivial: every cxx-bridge item needs a `#[cfg]`-gated stub. If it grows to dominate `pinocchio-sys/src/lib.rs`, the fallback is to drop `docs-only` from this change and instead rely on a docs.rs custom build config that pre-installs pinocchio via apt. Captured as a design decision rather than a blocker.
- robotpkg's `pinocchio` apt package may lag upstream by a minor version. Acceptable for the system-build CI job (it's smoke coverage, not parity — pixi remains authoritative for parity tests).
