## Context

Pinocchio is a **header-only, heavily-templated** C++ library: every public algorithm takes `Eigen::MatrixBase<...>` references and is parameterized on a scalar type. There is no stable `libpinocchio.so` ABI for the generic surface — concrete instantiations are produced at the call site. That single fact dominates every design decision below.

Pinocchio also has a fairly heavy native dependency tree (`Eigen`, `urdfdom`, `Boost`, `console_bridge`, `tinyxml2`, plus optional `hpp-fcl`, `coal`, etc.). Telling contributors to "install Pinocchio yourself" is realistic for ROS users but hostile to Rust newcomers. The native-dep story is the second-largest design problem after the binding itself.

The third constraint is *correctness verification*: robotics conventions (frame ordering, quaternion order, `log6` axis layout) are notoriously easy to mis-translate, and a binding that compiles and runs but silently uses the wrong convention is worse than one that fails to build. We need numerical-parity tests as a first-class deliverable, not an afterthought.

## Goals / Non-Goals

**Goals:**
- A `cargo build && cargo test` flow that produces a working binding to Pinocchio's core algorithms for `f64`-scalar URDF models, given only that the contributor has run `pixi install` once.
- 8 ported examples + a parity test suite that pins every bound algorithm to the C++ ground truth.
- An architecture that can grow to cover `GeometryModel`, `constraintDynamics`, and additional scalar types without restructuring the foundation.

**Non-Goals:**
- Re-implementing any Pinocchio algorithm in Rust.
- `f32` / `CppAD` / `CasADi` scalars.
- SDF / MJCF loaders.
- Collision, geometry, contact dynamics.
- Windows support (best-effort if it falls out of pixi cross-platform behavior; non-gating).
- Tracking Pinocchio master — we pin to a specific 3.x tag.

## Decisions

### D1 — FFI: `cxx` bridge over a hand-written C++ shim

**Decision.** Use [`cxx`](https://cxx.rs/) for the Rust↔C++ boundary. The C++ side is a hand-written `pinocchio_shim.{h,cpp}` that explicitly instantiates each algorithm we expose for `f64` and presents non-templated free functions to cxx.

**Rationale.** Because Pinocchio is templated header-only, any FFI tool needs concrete instantiations on the C++ side — that's a fixed cost regardless of tool. Given that, the tool choice is about the Rust-side ergonomics:
- `cxx` generates `UniquePtr<Model>` / `UniquePtr<Data>` with sensible destructor semantics, automatic C++ exception → `Result` conversion, and `Pin<&mut T>` for mutable access — all of which we'd otherwise hand-write.
- `bindgen` + C-shim is the PhysX-rs / Bullet-sys pattern and is fine, but forces a larger hand-written safe-wrapper layer.
- `autocxx` and `crubit` are both less mature for templated-Eigen libraries.

**Alternatives considered.** `bindgen` + C-shim (rejected: more hand-written unsafe surface). `autocxx` (rejected: template handling not robust for Eigen). `crubit` (rejected: MVP-stage, not production-ready).

### D2 — Native deps: pixi + conda-forge

**Decision.** Manage Pinocchio + Eigen + urdfdom + Boost + tooling via [pixi](https://pixi.sh) from conda-forge. `pixi.toml` at the workspace root pins exact versions; `pixi.lock` is checked in. `build.rs` reads `PIXI_PROJECT_ROOT` (or runs `pixi info` if unset) to locate the env and prepends `<env>/lib/pkgconfig` to `PKG_CONFIG_PATH` before probing for `pinocchio.pc`.

```
pinocchio_rs/
├── pixi.toml
├── pixi.lock
├── .pixi/                              # gitignored; created by `pixi install`
│   └── envs/default/
│       ├── include/pinocchio/...
│       ├── lib/pkgconfig/pinocchio.pc
│       └── lib/libpinocchio_*.{so,dylib}
└── pinocchio-sys/build.rs              # reads .pixi env, calls pkg_config
```

**Rationale.** Pinocchio is on conda-forge (`conda install -c conda-forge pinocchio`) and so is every transitive dep. Pixi gives reproducible, lockfile-backed, cross-platform installs with one command and no global state. It avoids:
- Vendoring Pinocchio + Eigen + urdfdom + Boost as submodules (~hundreds of MB, slow first build).
- Asking users to `apt install robotpkg-pinocchio` / `brew install pinocchio` / build-from-source (high friction, no version control).
- Maintaining a CMake invocation inside `build.rs` (cross-compilation pain, ccache integration pain).

`build.rs` becomes a pkg-config probe — ~30 lines, well-trodden territory.

**Alternatives considered.** Git-submodule vendor + cmake-from-build.rs (rejected: slow, complex, fragile). System pkg-config without pixi (rejected: forces every contributor to figure out their own install). `conda` directly (rejected: pixi is a strict superset; project-local envs are nicer).

### D3 — Crate layout: `pinocchio-sys` + `pinocchio-rs`

**Decision.** Cargo workspace with two members:
- `pinocchio-sys/` — unsafe FFI. Contains `build.rs`, the `cxx::bridge` mod, and the C++ shim sources. Published as `pinocchio-sys`.
- `pinocchio-rs/` — safe Rust API. Depends on `pinocchio-sys`. Re-exports nothing unsafe. Published as `pinocchio-rs`. (Local dir name is `pinocchio-rs` to match the published name; the bare `pinocchio` crate is squatted on crates.io.)

```
pinocchio_rs/
├── Cargo.toml                # [workspace]
├── pixi.toml
├── pinocchio-sys/
│   ├── Cargo.toml
│   ├── build.rs              # cxx_build + pkg_config
│   ├── shim/
│   │   ├── pinocchio_shim.h
│   │   └── pinocchio_shim.cpp
│   └── src/lib.rs            # cxx::bridge mod ffi { ... }
├── pinocchio-rs/
│   ├── Cargo.toml
│   ├── src/
│   │   ├── lib.rs
│   │   ├── model.rs
│   │   ├── data.rs
│   │   ├── se3.rs            # SE3 newtype
│   │   ├── motion.rs
│   │   ├── force.rs
│   │   ├── inertia.rs
│   │   ├── algorithms.rs     # fwd_kinematics, rnea, aba, crba, jacobian
│   │   ├── lie.rs            # integrate, difference, randomConfiguration
│   │   ├── derivatives.rs
│   │   └── urdf.rs
│   ├── tests/
│   │   ├── data/panda/...
│   │   ├── goldens/*.json
│   │   └── parity_*.rs
│   └── examples/
│       ├── overview_simple.rs
│       ├── overview_urdf.rs
│       ├── overview_se3.rs
│       ├── overview_lie.rs
│       ├── inverse_kinematics.rs
│       ├── inverse_dynamics.rs
│       └── kinematics_derivatives.rs
└── tools/
    └── gen_goldens/          # C++ binary that emits golden JSONs
        ├── CMakeLists.txt
        └── gen_goldens.cpp
```

### D4 — Safe-API style: newtypes for spatial types

**Decision.** `SE3`, `Motion`, `Force`, `Inertia` are Rust newtypes. Configuration / tangent / acceleration vectors and Jacobians cross the boundary as `nalgebra::DVector<f64>` / `nalgebra::DMatrix<f64>`.

**Rationale.** Pinocchio's own Python binding does this — `pin.SE3`, `pin.Motion`, `pin.Force` are distinct classes — and so do the C++ types they wrap. Newtypes prevent the classic "I added a `Motion` to a `Force` and the types didn't catch it" bug, and they're the natural place to encode Pinocchio-specific conventions (e.g., `log6` ordering, quaternion (x,y,z,w) order). The cost is bounded — roughly 150 LoC per type.

`nalgebra` is the de-facto Rust linear algebra crate; using it for configuration vectors avoids inventing another container and gives users free access to nalgebra's broader ecosystem.

### D5 — Algorithm output model: mixed (copy small, borrow large)

**Decision.** Accessors on `Data` follow a size-based convention:
- **Owned copy** for small results: `tau` (nv ≈ 7–50 floats), `SE3` poses (16 floats), individual `Motion`/`Force` (6 floats each).
- **Borrowed view** for large results: mass matrix `M` (nv²), Jacobians (6×nv per frame), derivative tensors.

```rust
// owned
let tau: DVector<f64> = data.tau();
let oMi: SE3 = data.joint_placement(joint_id);

// borrowed (lifetime tied to &Data; blocks the next mutation)
let M: DMatrixView<'_, f64> = data.mass_matrix();
```

**Rationale.** Eliminates both the "every call costs an nv² copy" tax (would bite humanoids in 1kHz control loops) and the "every accessor needs lifetime gymnastics" tax (would bite tutorial code). The line at "small vs large" is approximately 200 floats; below that, copies are free in cache; above, copies are noticeable. Matches what `eigenpy` does conceptually for the Python side.

**Alternatives considered.** Borrow everything (rejected: too noisy for small results). Copy everything (rejected: real perf cost on mass matrix). Caller-owned out-buffers (rejected: un-Rust-y).

### D6 — Numerical-parity testing: C++ oracle → static golden JSONs

**Decision.** Build a separate C++ binary `tools/gen_goldens/gen_goldens.cpp` that links to Pinocchio directly (via the same pixi env) and emits JSON files containing input/output pairs for every algorithm under test. Goldens are checked into `pinocchio-rs/tests/goldens/`. Rust integration tests load goldens, run the Rust binding on the inputs, and assert outputs match within tolerance.

```
tools/gen_goldens/gen_goldens.cpp
        │ links libpinocchio directly
        ▼
generates → pinocchio-rs/tests/goldens/
            ├── fk_panda_100samples.json    {q[100][nq], oMi[100][njoints][16]}
            ├── rnea_panda_100samples.json  {q,v,a → tau}
            ├── aba_panda_100samples.json   {q,v,tau → ddq}
            ├── crba_panda_100samples.json  {q → M[nv][nv]}
            ├── jac_panda_50samples.json    {q,frame_id,refframe → J[6][nv]}
            ├── se3_ops_500samples.json     {SE3 inputs → compose/inverse/log6}
            ├── lie_ops_500samples.json     {q,v → integrate; q0,q1 → difference}
            └── fk_derivs_50samples.json    {q,v → ∂v/∂q, ∂v/∂v_in}

pinocchio-rs/tests/parity_*.rs (load JSON, run Rust, assert)
```

Tolerance default: `1e-10` for direct algorithm outputs; `1e-8` for derivatives (looser because of accumulated floating-point error in the derivative computations themselves).

**Sample sizes per algorithm:** 100 for the main dynamics algorithms (fk/rnea/aba/crba), 50 for Jacobians and derivatives, 500 for cheap pure-math operations (SE3, Lie). Seeded RNG (`std::mt19937` seed = 42) so results are bit-reproducible.

**Rationale.** Static goldens:
- Keep Rust tests fast and Rust-only (no C++ link from test crates).
- Make regressions visible as git diffs on the JSON.
- Survive Pinocchio version bumps cleanly (regenerate goldens, see exactly what changed).

The C++ oracle uses the same pinned Pinocchio that the binding wraps, so we're verifying *binding faithfulness*, not Pinocchio correctness — that's the right scope.

**Alternatives considered.** C++ oracle linked into Rust tests (rejected: doubles build complexity and test runtime). Python pinocchio bindings as oracle (rejected: requires Python in CI and locally; less hermetic than a single-source binary). Generate goldens at test time (rejected: non-deterministic timing, masks regressions).

### D7 — Pinocchio version pin

**Decision.** Pin `pixi.toml` to a specific Pinocchio 3.x release from conda-forge (latest stable at workspace creation time). Document the upgrade procedure in `CONTRIBUTING.md`: bump `pixi.toml`, run `pixi install`, regenerate goldens, run parity tests, commit all in one PR.

### D8 — Lie-group ergonomics

**Decision.** The safe API never exposes `q + v` as a meaningful operation. Configuration update goes through `model.integrate(&q, &v_dt) -> DVector<f64>` and `model.difference(&q0, &q1) -> DVector<f64>`. Users who try to add `DVector` to `DVector` to update a configuration get a vector that's numerically wrong on free-flyer / spherical components — *that's a nalgebra-level op we can't prevent* — but every example, every doc, and every public method uses the manifold operations exclusively. The `overview-lie` example exists partly to drill this in.

### D9 — Stage-gating policy: each module ships with its own unit tests

**Decision.** Every implementation stage in `tasks.md` ends with two mandatory subtasks before the next stage may begin:

1. **In-module unit tests** — `#[cfg(test)] mod tests { ... }` inside the module's own source file (`src/se3.rs`, `src/algorithms.rs`, etc.). These tests assert *module-local* invariants only: shape, identity laws, round-trips, type-system claims — anything that does not require a golden JSON. They run as `cargo test --lib`.
2. **Gate** — the stage is "done" only when `cargo test -p <crate> --lib <module-path>::` is green for the module(s) introduced in that stage. Subsequent stages MUST NOT start before the gate is green.

**Two-layer split:**
- `pinocchio-sys` carries per-shim-function smoke tests in `src/lib.rs` `#[cfg(test)]` (does the function link, does it not crash, does it produce non-garbage output for trivial inputs). Run with `cargo test -p pinocchio-sys`.
- `pinocchio-rs` carries module-level unit tests in each module file (identity laws, round-trips, etc.). Run with `cargo test -p pinocchio-rs --lib`.
- Parity tests (the golden-JSON comparison suite from §14) live in `pinocchio-rs/tests/parity_*.rs` and run with `cargo test -p pinocchio-rs --test 'parity_*'`. They are the final integration layer — independent of stage gates, not a substitute for them.

**Rationale.** The point of gating is to catch regressions where they happen. A convention bug introduced in §4 (e.g., quaternion order in `SE3`) would otherwise only surface at §14 parity tests — by which point five subsequent stages have built on top of it. Local module tests catch it at §4. The cost is small: each module already has natural identity laws to assert (`t.inverse() * t == identity`, `log(identity) == 0`, `integrate(q, 0) == q`, etc.), and writing them takes minutes per module.

"Independently" here means: stage N's unit tests pass running only stage N's shim functions and safe wrappers, without requiring stages N+1..16. Stage N may freely depend on stages 1..N−1 — that's the normal build order.

**Alternatives considered.** Parity-tests-only (rejected: catches regressions too late, no per-module signal). Doctests-only (rejected: doctests are documentation-driven, awkward to enforce invariants like compile-fail). Property tests via `proptest` (rejected for MVP: nice-to-have, but not load-bearing; can be added later without restructuring).

## Risks / Trade-offs

**[Pixi env activation]** — `cargo build` outside `pixi shell` may fail to locate `pinocchio.pc` if `build.rs` can't find the pixi env. **Mitigation:** `build.rs` calls `pixi info --json` to discover the env path; documents the `pixi run cargo build` invocation as canonical; falls back to a regular `pkg-config` probe so system-installed Pinocchio also works.

**[Compile-time blow-up]** — every algorithm in the shim is a fresh template instantiation. **Mitigation:** the shim's surface is bounded by the 8 example targets + the parity-test surface. Adding new algorithms is intentional, not transparent.

**[Convention drift]** — Pinocchio's `log6` returns `(linear, angular)`, quaternion order is `(x,y,z,w)`, frame Jacobians have a `ReferenceFrame` enum (`LOCAL`, `LOCAL_WORLD_ALIGNED`, `WORLD`). Wrong convention compiles, runs, silently breaks downstream. **Mitigation:** parity tests are the only line of defense; this is precisely why D6 elevates them to a deliverable rather than a nice-to-have.

**[`Data` borrow ergonomics]** — `Pin<&mut Data>` for algorithm calls is unusual Rust syntax for a robotics audience. **Mitigation:** the safe wrapper hides the `Pin` in method signatures: `data.rnea(&model, &q, &v, &a)` is what users write.

**[Mesh-loading dep creep]** — Panda URDF references STL meshes; mesh parsing pulls in `console_bridge` and friends. **Mitigation:** for MVP, the binding does *not* parse meshes; URDF visual/collision elements are parsed but ignored. Meshes ship for future use.

**[Cross-platform pixi]** — pixi works on Linux & macOS for Pinocchio; Windows conda-forge `pinocchio` package coverage is incomplete. **Mitigation:** Linux is the only gating platform for MVP. macOS is exercised opportunistically; Windows is non-goal.

**[Crates.io name shadowing]** — `pinocchio` on crates.io is unrelated Solana tooling; new users may land there first. **Mitigation:** publish as `pinocchio-rs` with a clear README pointing at the C++ Pinocchio project upstream.

## Open Questions

- **Which exact Pinocchio 3.x tag to pin?** Resolved in tasks.md when `pixi.toml` is authored — use the latest stable available on conda-forge at that moment.
- **License headers in shim files?** Default to BSD-2-Clause to match upstream; revisit if `pinocchio-rs/Cargo.toml` adopts a dual license for the Rust side.
- **Should `cargo build` (outside pixi) attempt a `pixi install` automatically?** Probably not — silent network ops in build scripts are user-hostile. Document and let the contributor invoke explicitly.
