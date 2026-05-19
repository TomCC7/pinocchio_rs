# pinocchio_rs

Safe Rust bindings to [Pinocchio](https://github.com/stack-of-tasks/pinocchio),
the C++ rigid body dynamics library used widely in robotics research for
high-performance kinematic and dynamic algorithms.

## What this is

* Two-crate Cargo workspace:
  * `pinocchio-sys` — hand-written `cxx::bridge` FFI shim, links Pinocchio via
    `pkg-config`.
  * `pinocchio-rs` — safe wrappers (`Model`, `Data`, `SE3`, `Motion`, `Force`,
    `Inertia`, `ReferenceFrame`) plus algorithms exposed as methods on `Data`.
* `f64` only; URDF only; fixed-base or free-flyer root joint.
* Mirrors of Pinocchio's seven canonical "basic" examples — runnable as
  `cargo run --example <name>`:
  `overview_simple`, `overview_urdf`, `overview_se3`, `overview_lie`,
  `inverse_dynamics`, `inverse_kinematics`, `kinematics_derivatives`.

## What this is not

* Not a port — the algorithms run in Pinocchio's C++ code, not in Rust.
* Not a physics engine — no contact, no collision, no integration.
* Not zero-dependency — needs `pinocchio`, `eigen`, `urdfdom`, and a C++17
  compiler. We get all of them from one `pixi install`.

## Quickstart

```bash
pixi install                                  # one-shot env (Pinocchio + Eigen + …)
pixi run cargo build --workspace              # compile both crates
pixi run cargo test --workspace               # all 33+ lib tests + parity suite
pixi run cargo run --example overview_urdf    # FK on the vendored Panda URDF
```

The pixi env is the supported build path. Other environments work iff
`pkg-config --modversion pinocchio` returns a 3.x version.

## Status

MVP. All algorithm gates pass numerical-parity tests against the upstream
Pinocchio C++ implementation. See `openspec/changes/bootstrap-pinocchio-bindings/`
for the full change spec and `openspec/changes/bootstrap-pinocchio-bindings/tasks.md`
for the per-stage gate history.

## Conventions

These match Pinocchio and are pinned by tests in `pinocchio-rs/src/`:

| concept                | choice                          | tested in                       |
|------------------------|---------------------------------|---------------------------------|
| Quaternion order       | `(x, y, z, w)`                  | `se3::tests::quaternion_storage_is_xyzw` |
| `Motion` / `Force` order | `[linear:3, angular:3]`       | `motion::tests`, `force::tests` |
| `ReferenceFrame` discr. | `World=0, Local=1, LocalWorldAligned=2` | `reference_frame::tests::discriminants_match_pinocchio_enum` |
| `log6` ordering        | matches `pinocchio::log6`       | `parity_se3`                   |
| Mass matrix            | symmetric (lower triangle filled by wrapper) | `parity_crba`        |

## License

BSD-2-Clause, matching Pinocchio.
