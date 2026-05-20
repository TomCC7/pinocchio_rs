## MODIFIED Requirements

### Requirement: Build script locates the pixi environment

The `pinocchio-sys` crate's `build.rs` SHALL locate a Pinocchio installation via `pkg-config` and configure the C++ build against it. The probe order SHALL be:

1. Probe `pkg-config` for `pinocchio` using `PKG_CONFIG_PATH` as the user (or invoking task) has set it.
2. If step 1 fails AND the `system` Cargo feature is NOT enabled, attempt to locate a pixi-managed environment (`pixi info --json`, falling back to `PIXI_PROJECT_ROOT`). If a pixi env is found, prepend its `lib/pkgconfig` directory to `PKG_CONFIG_PATH` and retry the probe.
3. If both fail, exit with an actionable error message that lists *all* discovery paths tried (pkg-config search dirs, pixi info result, system defaults) and the documented install options (`pixi install`, system package, brew, conda).

When the `bundled-pixi` Cargo feature is enabled, the pixi prepend (step 2) SHALL run *before* step 1 instead of after — forcing the pixi env to win even when a system Pinocchio is also discoverable.

When the `docs-only` Cargo feature is enabled, `build.rs` SHALL skip the `cxx_build` invocation entirely and exit successfully without probing.

On a successful probe, `build.rs` SHALL emit exactly one `cargo:warning=pinocchio-sys: found pinocchio <version> at <pkgconfig-path>` line so the chosen install is visible in build output.

#### Scenario: Build inside pixi env

- **WHEN** a contributor runs `pixi run cargo build` after `pixi install` (the pixi task passes `--features pinocchio-sys/bundled-pixi`)
- **THEN** the build succeeds, uses the pixi env's Pinocchio, and the `cargo:warning=` line names a path under `.pixi/envs/`

#### Scenario: Build outside pixi env, system Pinocchio installed

- **WHEN** a contributor runs `cargo build` directly (no pixi run) in an environment where `pkg-config --exists pinocchio` already succeeds (e.g., system apt install, brew install, or a pre-configured `PKG_CONFIG_PATH`)
- **THEN** the build succeeds using the system Pinocchio, no pixi probe is performed (system probe succeeds at step 1), and the `cargo:warning=` line names the system path

#### Scenario: Build outside pixi env when pixi env exists

- **WHEN** a contributor runs `cargo build` directly (no pixi run, no `--features bundled-pixi`) in a workspace that has been `pixi install`ed, with no system Pinocchio
- **THEN** step 1 fails, step 2 discovers the pixi env via `pixi info`, the retried probe succeeds, and the build proceeds

#### Scenario: Build with no Pinocchio anywhere

- **WHEN** a contributor runs `cargo build` without `pixi install`, without `--features bundled-pixi`, and without any system Pinocchio
- **THEN** `build.rs` exits with an error message that names every probe path tried and lists the three documented install options (pixi, apt/robotpkg, brew)

#### Scenario: `system` feature forbids pixi fallback

- **WHEN** a contributor runs `cargo build --features pinocchio-sys/system` and no system Pinocchio is discoverable
- **THEN** `build.rs` exits with an error after step 1 without attempting step 2, regardless of whether a pixi env exists

#### Scenario: `docs-only` feature skips the native build

- **WHEN** a contributor runs `cargo doc --features pinocchio-sys/docs-only --no-default-features` in an environment with no Pinocchio installed
- **THEN** `build.rs` returns success without invoking `cxx_build` or `pkg-config`, and `cargo doc` renders the full rustdoc surface

## ADDED Requirements

### Requirement: Cargo features for build-path selection

The `pinocchio-sys` crate SHALL declare exactly three optional Cargo features that steer build-time Pinocchio discovery, and `pinocchio-rs` SHALL re-export them transitively:

- `bundled-pixi` — pixi env is tried before system pkg-config. Set by `pixi.toml`'s cargo tasks.
- `system` — pixi auto-discovery is disabled; only `PKG_CONFIG_PATH` as the user set it is consulted. Used by the system-install CI lane.
- `docs-only` — `build.rs` skips native compilation; `pinocchio-sys/src/lib.rs` substitutes stub function bodies. Used by the docs.rs build configuration.

No two of `bundled-pixi`, `system`, `docs-only` SHALL be activated simultaneously; `build.rs` SHALL emit a compile error if a conflicting combination is detected.

#### Scenario: Features are documented in crate rustdoc

- **WHEN** a user opens `pinocchio-sys`'s crate-level rustdoc on docs.rs
- **THEN** a Features section lists each of `bundled-pixi`, `system`, `docs-only` with a one-paragraph description of when to use it

#### Scenario: Conflicting features fail the build

- **WHEN** a contributor runs `cargo build --features pinocchio-sys/bundled-pixi,pinocchio-sys/system`
- **THEN** the build fails with a `compile_error!` message naming the conflicting features

### Requirement: `pixi.toml` tasks pin the `bundled-pixi` feature

The `cargo build`, `cargo test`, and `gen-goldens` tasks declared in `pixi.toml` SHALL invoke cargo with `--features pinocchio-sys/bundled-pixi` so the pixi-supported flow is deterministic and never silently falls back to a system Pinocchio.

#### Scenario: Pixi task pins the feature

- **WHEN** an engineer inspects `pixi.toml`
- **THEN** the `[tasks]` table's `cargo build`, `cargo test`, and `gen-goldens` entries each contain `--features pinocchio-sys/bundled-pixi`

### Requirement: CI covers pixi and system install paths

The repository's GitHub Actions workflow SHALL include at minimum these three jobs, each running on every push to `main` and every pull request:

- `pixi-ubuntu` — Ubuntu LTS, `pixi install`, full gate sequence including parity tests. Gating.
- `pixi-macos` — macOS arm64, `pixi install`, build + lib tests + at least one parity test. Non-gating initially.
- `system-ubuntu` — Ubuntu LTS, install Pinocchio via `apt install robotpkg-pinocchio` (robotpkg PPA), no pixi, `--features pinocchio-sys/system`, build + lib tests only. Non-gating initially.

#### Scenario: System-install path is exercised in CI

- **WHEN** a PR introduces a regression in the non-pixi probe (e.g., reintroduces a hard pixi dependency)
- **THEN** the `system-ubuntu` job fails, surfacing the regression even though it does not block merge

#### Scenario: macOS path is exercised in CI

- **WHEN** a PR introduces a macOS-only build regression
- **THEN** the `pixi-macos` job fails, surfacing the regression even though it does not block merge
