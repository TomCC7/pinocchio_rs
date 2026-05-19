## ADDED Requirements

### Requirement: Pixi-managed native dependency stack

The workspace SHALL declare its full native dependency stack (`pinocchio`, `eigen`, `urdfdom`, `boost-cpp`, `cmake`, `pkg-config`, a C++ compiler) in a `pixi.toml` file at the workspace root, with versions pinned via a checked-in `pixi.lock`. Contributors SHALL be able to provision the full native stack with a single `pixi install` command on any supported platform.

#### Scenario: Fresh contributor checkout

- **WHEN** a contributor clones the repository on a supported platform and runs `pixi install`
- **THEN** the command succeeds without further configuration, and `.pixi/envs/default/` contains a working `pinocchio.pc` plus the libraries it references

#### Scenario: Reproducible installs

- **WHEN** two contributors run `pixi install` against the same `pixi.lock`
- **THEN** both end up with byte-identical resolved package versions for every dependency

### Requirement: Build script locates the pixi environment

The `pinocchio-sys` crate's `build.rs` SHALL locate the pixi-managed environment (preferring `pixi info --json`, falling back to `PIXI_PROJECT_ROOT` and then to system `PKG_CONFIG_PATH`) and use `pkg-config` to discover Pinocchio's include paths and link flags. The build script MUST fail with an actionable error message if `pinocchio.pc` cannot be found.

#### Scenario: Build inside pixi env

- **WHEN** a contributor runs `pixi run cargo build` after `pixi install`
- **THEN** the build succeeds without manually setting `PKG_CONFIG_PATH`

#### Scenario: Build outside pixi env when pixi env exists

- **WHEN** a contributor runs `cargo build` directly in a workspace that has been `pixi install`ed
- **THEN** `build.rs` discovers the env via `pixi info` and the build succeeds

#### Scenario: Build with no pixi env and no system Pinocchio

- **WHEN** a contributor runs `cargo build` without `pixi install` and without a system Pinocchio
- **THEN** `build.rs` exits with an error message that explicitly instructs the user to run `pixi install`

### Requirement: Two-crate workspace layout

The repository SHALL be structured as a Cargo workspace with exactly two published members: `pinocchio-sys` (FFI surface) and `pinocchio-rs` (safe wrapper). `pinocchio-rs` SHALL not expose any `unsafe` items in its public API surface.

#### Scenario: Public API safety

- **WHEN** a downstream crate depends only on `pinocchio-rs` and calls any item exported from `pinocchio_rs::*`
- **THEN** the downstream code compiles without requiring `unsafe` blocks
