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
pixi run build                                # compile both crates (--features pinocchio-sys/bundled-pixi)
pixi run test                                 # 40+ lib tests + 12 parity tests
pixi run cargo run --example overview_urdf    # FK on the vendored Panda URDF
```

## Install paths

`pinocchio-sys/build.rs` discovers Pinocchio via `pkg-config`. It does not
call `cmake` and does not vendor Pinocchio. Pick whichever install style
matches your environment:

| Path | When to use | How |
|------|-------------|-----|
| **pixi** (recommended for development of this crate) | local dev, contributors, CI | `pixi install` then `pixi run build` |
| **apt + robotpkg PPA** | Linux consumers integrating into ROS / system-wide stacks | follow [robotpkg's install guide](http://robotpkg.openrobots.org/install.html), then `apt install robotpkg-pinocchio libeigen3-dev`, then `cargo build --features pinocchio-sys/system` with `PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig` |
| **homebrew** | macOS consumers | `brew install pinocchio eigen` then plain `cargo build` |
| **conda-forge without pixi** | existing conda workflows | `conda install -c conda-forge pinocchio eigen urdfdom` then `cargo build` in the activated env |

On a successful build, `pinocchio-sys` emits
`cargo:warning=pinocchio-sys: found pinocchio <ver> at <pkgconfig-file>` so
the chosen Pinocchio is visible in build output. If you have both a system
and a pixi-managed install and want the pixi one to win, pass
`--features pinocchio-sys/bundled-pixi`. To require the system install and
fail fast otherwise, pass `--features pinocchio-sys/system`.

For docs.rs only, `--features pinocchio-sys/docs-only --no-default-features`
skips the native build entirely so the API can render without Pinocchio
installed.

## Status

MVP + first follow-up (`derivatives-and-system-build`). All algorithm gates
pass numerical-parity tests against the upstream Pinocchio C++ implementation.
The follow-up adds:

* Full analytical-derivatives surface — RNEA, ABA, joint-acceleration, frame-velocity,
  frame-acceleration partials (all three `ReferenceFrame` variants).
* System-install build path (apt/robotpkg, brew, conda-without-pixi) via the
  `pinocchio-sys/system` Cargo feature, alongside the pixi-managed default.
* `pinocchio-sys/docs-only` for hermetic docs.rs rendering.

See `openspec/changes/derivatives-and-system-build/` for the follow-up's
change spec, and `openspec/changes/bootstrap-pinocchio-bindings/` for the MVP.

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
