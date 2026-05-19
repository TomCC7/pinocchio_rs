## Why

Rust currently has solid kinematics tooling (`k`, `urdf-rs`) but no production-grade rigid-body dynamics with analytical derivatives — the RNEA / ABA / CRBA / derivative surface that control, trajectory optimization, and learning-based methods need. Pinocchio (`stack-of-tasks/pinocchio`) is the de-facto reference C++ implementation of those algorithms, with mature Python bindings via `eigenpy`. No Rust binding to it exists today (the `pinocchio` crate on crates.io is unrelated Solana tooling). This change creates one.

## What Changes

- Add a two-crate workspace: `pinocchio-sys` (FFI surface) and `pinocchio-rs` (safe wrapper, published name).
- Bind Pinocchio's core algorithms via a `cxx` bridge to a hand-written C++ shim that explicitly instantiates each algorithm for `f64`.
- Cover the canonical algorithm set: `forwardKinematics`, `rnea`, `aba`, `crba`, `computeJointJacobian`, `log6`/`Jlog6`, `integrate`/`difference`/`randomConfiguration`, plus at least one analytical-derivatives entry point (`computeForwardKinematicsDerivatives`).
- Load robot models from URDF via `pinocchio::urdf::buildModel`.
- Expose Pinocchio's spatial types (`SE3`, `Motion`, `Force`, `Inertia`) as Rust newtypes; configuration / tangent vectors via `nalgebra::DVector<f64>`.
- Use **pixi** to manage the Pinocchio + Eigen + urdfdom + Boost C++ stack from conda-forge. `build.rs` probes the pixi env via `pkg-config`.
- Ship 8 example programs ported from Pinocchio's own `examples/` directory: `overview-simple`, `overview-urdf`, `overview-SE3`, `overview-lie`, `inverse-kinematics`, `inverse-dynamics`, `kinematics-derivatives`, plus a model-loading smoke test.
- Ship a numerical-parity test suite that asserts every bound algorithm matches the corresponding C++ Pinocchio output to a documented tolerance, across many sampled inputs.
- Vendor one robot model (Panda URDF + meshes, ~5 MB) under `tests/data/panda/` for the examples and parity tests to run against.

**Non-goals** (deliberately deferred to follow-up changes):
- Scalars other than `f64` (no `f32`, `CppAD`, or `CasADi`).
- Model formats other than URDF.
- `GeometryModel` / collision / contact APIs.
- `constraintDynamics` and contact solvers.
- Re-implementing any Pinocchio algorithm in Rust.

## Capabilities

### New Capabilities
- `pinocchio-bindings-build`: pixi-managed C++ dependency stack, two-crate workspace, `cxx` bridge generation, build-script contract.
- `model-loading`: load a Pinocchio `Model` from a URDF file; construct a `Data` companion sized to the model.
- `spatial-types`: Rust newtypes for `SE3`, `Motion`, `Force`, `Inertia` with the operations Pinocchio supports on each (compose, inverse, act, log/exp).
- `kinematics`: `forwardKinematics` + access to per-joint and per-frame placements.
- `dynamics`: `rnea` (inverse dynamics), `aba` (forward dynamics), `crba` (mass matrix).
- `jacobians`: `computeJointJacobian` and per-frame Jacobian access in selectable reference frames.
- `lie-operations`: configuration-space `integrate`, `difference`, `randomConfiguration`, and `neutral` — correctly handling free-flyer / spherical joint manifolds.
- `analytical-derivatives`: at least one derivative entry point (`computeForwardKinematicsDerivatives`) exposed end-to-end.
- `numerical-parity-suite`: golden-value generation from a C++ oracle plus Rust integration tests that assert every algorithm matches the oracle within tolerance.

### Modified Capabilities
None — this is the bootstrap change; no specs exist yet to modify.

## Impact

- **New code**: `pinocchio-sys/` (FFI crate with `build.rs`, `cxx::bridge`, C++ shim), `pinocchio-rs/` (safe wrapper crate, examples, tests), `pixi.toml` + `pixi.lock` at workspace root, `tests/data/panda/` model assets, `tools/gen_goldens/` C++ oracle binary.
- **External dependencies (managed by pixi from conda-forge)**: `pinocchio` (3.x), `eigen`, `urdfdom`, `boost-cpp`, `cmake`, `pkg-config`, a C++ compiler.
- **Rust dependencies**: `cxx`, `cxx-build`, `pkg-config`, `nalgebra`.
- **Build prerequisite**: contributors run `pixi install` once; `cargo build` afterwards Just Works because `build.rs` probes the pixi env's `pkg-config` path.
- **License**: BSD-2-Clause to match upstream Pinocchio.
- **Platforms**: Linux is gating; macOS is best-effort (pixi supports it but mesh viewer / Boost edge cases may bite); Windows is non-goal for the MVP.
