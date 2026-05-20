# Contributing to pinocchio_rs

## Development setup

```bash
pixi install
pixi run cargo test --workspace
```

The pixi env vendors the entire native toolchain (Pinocchio, Eigen, urdfdom,
the C++ compiler, and the matching Rust toolchain). There is no `cmake`
invocation from `build.rs`; all native deps come from `pkg-config`.

## Stage-gating policy

Per design D9, every implementation stage in
`openspec/changes/bootstrap-pinocchio-bindings/tasks.md` ends with:

* **UT.N** — in-module `#[cfg(test)] mod tests` asserting module-local
  invariants (identity laws, shapes, round-trips, determinism).
* **GATE.N** — an explicit `cargo test -p <crate> --lib <module>::` command
  that must be green before the next stage's work may start.

Parity tests in `pinocchio-rs/tests/parity_*` are the *integration* layer —
they exist to catch convention bugs (quaternion order, frame conventions,
mass matrix layout) that a wrapper-only test could not see.

## Upgrading Pinocchio

When `pinocchio` is bumped on conda-forge:

1. **Bump `pixi.toml`** — update the version constraint (e.g. `>=3.10,<4`).
2. **`pixi install`** — re-resolves and writes a new `pixi.lock`.
3. **`pixi run gen-goldens`** — regenerates every JSON file under
   `pinocchio-rs/tests/goldens/` using the new Pinocchio version.
4. **`pixi run cargo test --workspace`** — confirms the new goldens still
   round-trip through the Rust wrappers within the documented tolerances.
   Review the JSON diff — small numerical drift is normal; structural drift
   indicates an API change you need to follow up on.

Commit `pixi.toml`, `pixi.lock`, and every changed JSON in a *single PR* so
reviewers can see the upgrade in one place.

## Adding a new algorithm

1. Add the C++ entry in `pinocchio-sys/shim/pinocchio_shim.{h,cpp}`. Pass and
   return through flat `double*` buffers — no Eigen types across the FFI.
2. Add the corresponding `unsafe fn` to the `cxx::bridge` mod in
   `pinocchio-sys/src/lib.rs`.
3. Implement the safe wrapper as a method on `Model` or `Data` in the most
   topical module under `pinocchio-rs/src/`.
4. Write `UT.N` tests in the same module — assert shape, determinism, and at
   least one algorithm-specific invariant (round-trip, identity, …).
5. Extend `tools/gen_goldens/gen_goldens.cpp` with a new emitter, regenerate
   goldens, and add a `tests/parity_<algo>.rs` integration test.

## Adding a new derivative accessor

Pinocchio's analytical derivatives almost always follow a **compute then
accessor** pattern:

1. A `compute_*_derivatives(model, data, q, v, ...)` algorithm fills cached
   fields in `Data` (`data.dtau_dq`, `data.ddq_dv`, …).
2. A `get_*_derivatives(...)` accessor reads those fields into caller-provided
   Eigen matrices (typically `6×nv` or `nv×nv`, column-major).

Mirror the same shape in the binding:

1. **Shim header** (`pinocchio-sys/shim/pinocchio_shim.h`) — add `void
   compute_<algo>_derivatives(...)` if the algorithm needs its own compute pass,
   plus one or more accessors that take `double*` output buffers.
2. **Shim implementation** — call the underlying `pinocchio::compute*` /
   `pinocchio::get*` functions and write the results into the buffers using
   `Eigen::Map<MatrixXd>` (which is column-major). **Beware Pinocchio's
   `RowMatrixXs` fields** (`data.dtau_dq`, `data.dtau_dv`, `data.ddq_dq`,
   `data.ddq_dv`, `data.Minv` are row-major in `data.hpp`). When copying into
   a column-major `Eigen::Map`, Eigen does the transposition automatically;
   when serializing for goldens, copy through a column-major temporary first
   (`Eigen::MatrixXd col_major = data.ddq_dq;`) — otherwise the JSON contains
   the transpose. The fix for the original `aba_derivs` parity bug was
   exactly this: copy into `Eigen::MatrixXd` before `write_eigen`.
3. **cxx::bridge** declaration in `pinocchio-sys/src/lib.rs`.
4. **Safe wrapper** in `pinocchio-rs/src/derivatives.rs`. Convention (DD1):
   return **owned** `nalgebra::DMatrix<f64>` matrices, not borrowed views —
   typical control / optimization consumers read each derivative once into a
   per-step tensor, and views complicate the borrow checker. Match the
   compute-then-accessor pair-shape: a `compute_*` method that calls into the
   Pinocchio algorithm and returns `Result<()>`, plus zero-or-more `*_partials`
   / `*_derivatives` accessors that return owned matrices.
5. **Document Pinocchio identities** that the wrapper deliberately does NOT
   expose. RNEA's `∂τ/∂a = M` and ABA's `∂q̈/∂τ = M⁻¹` are computed by
   `computeRNEADerivatives` / `computeABADerivatives` but we direct callers to
   `data.mass_matrix(&model)` and its Cholesky inverse, respectively — see
   DD2 in `openspec/changes/derivatives-and-system-build/design.md`.
6. **Golden stream** — extend `tools/gen_goldens/gen_goldens.cpp` with an
   `emit_<algo>_derivs` function. Pick a fresh stream id by XOR-ing the seed
   with a one-byte tag that does not collide with existing streams (see
   `0xF1`..`0xFB` for the current allocations) and document it in
   `tools/gen_goldens/SCHEMA.md`. Add the filename to the `expected_files`
   array in `scripts/test-goldens-reproducible.sh`.
7. **Parity test** — `pinocchio-rs/tests/parity_<algo>_derivs.rs`, loading
   the golden envelope and replaying every sample through the safe wrapper at
   `TOL_DERIV = 1e-8`. Use the existing `assert_mat_close_col_major` helper in
   `tests/common/mod.rs`.

## Conventions to keep

* Vectors / matrices cross the FFI in **column-major** flat buffers.
* Owned `DVector` / `DMatrix` over borrowed views — design D5 was relaxed for
  the algorithm wrappers (see `tasks.md` §8.2 for the rationale).
* C++ exceptions are mapped to `pinocchio_rs::Error` via `cxx::Exception`.
* Every public item gets rustdoc; modules document the upstream Pinocchio
  convention they wrap.

## License

All contributions are licensed BSD-2-Clause, matching Pinocchio. Source files
must carry an `SPDX-License-Identifier: BSD-2-Clause` line at the top.
