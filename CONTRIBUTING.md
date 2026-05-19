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
