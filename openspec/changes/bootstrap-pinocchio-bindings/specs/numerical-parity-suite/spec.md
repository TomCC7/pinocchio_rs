## ADDED Requirements

### Requirement: C++ oracle binary generates golden values

The repository SHALL include a C++ binary at `tools/gen_goldens/gen_goldens.cpp` (built via the same pixi env) that links directly to Pinocchio and emits JSON files under `pinocchio-rs/tests/goldens/`. The binary SHALL be invokable via `pixi run gen-goldens` (a task defined in `pixi.toml`), and its outputs SHALL be deterministic given a fixed RNG seed (default seed = 42).

#### Scenario: Goldens are reproducible

- **WHEN** a contributor runs `pixi run gen-goldens` twice on the same machine without modifying the source
- **THEN** the two runs produce byte-identical JSON files

#### Scenario: Goldens cover every parity-tested algorithm

- **WHEN** the binary runs to completion
- **THEN** at minimum the following JSON files are produced: `fk_panda.json`, `rnea_panda.json`, `aba_panda.json`, `crba_panda.json`, `jac_panda.json`, `se3_ops.json`, `lie_ops.json`, `fk_derivs_panda.json`

### Requirement: Rust parity tests load goldens and assert tolerance

`pinocchio-rs/tests/` SHALL contain integration tests (one per algorithm family) that load the corresponding golden JSON, replay each input through the Rust binding, and assert that outputs match the golden values within a documented tolerance. Tests SHALL be pure Rust — no C++ linking from the test crates themselves.

#### Scenario: Parity test fails on detectable regression

- **WHEN** a contributor modifies the safe wrapper to (e.g.) swap quaternion order
- **THEN** at least one parity test fails with an output-mismatch message that identifies the algorithm, the sample index, and the component that diverged

#### Scenario: Parity tests run in pure Rust

- **WHEN** a contributor runs `cargo test -p pinocchio-rs` inside the pixi env
- **THEN** all parity tests execute without any additional C++ build step (beyond what `pinocchio-sys`'s `build.rs` already does)

### Requirement: Documented tolerance per algorithm class

The parity-test crate SHALL define tolerance constants in one location, with values: `1e-10` for direct algorithm outputs (FK, RNEA, ABA, CRBA, Jacobians, SE3, Lie); `1e-8` for analytical-derivative outputs.

#### Scenario: Tolerances are centrally defined

- **WHEN** an engineer inspects the parity-test source
- **THEN** every `assert_relative_eq!` / `assert_abs_diff_eq!` invocation references a named tolerance constant rather than a hard-coded literal

### Requirement: Documented golden regeneration procedure

The repository SHALL include `CONTRIBUTING.md` (or equivalent) documentation explaining the golden-regeneration procedure for the case where Pinocchio is upgraded: bump `pixi.toml`, run `pixi install`, run `pixi run gen-goldens`, run `cargo test`, review JSON diffs, commit all in one PR.

#### Scenario: Procedure is discoverable

- **WHEN** a contributor reads `CONTRIBUTING.md`
- **THEN** they find a section titled "Upgrading Pinocchio" with the four-step procedure above
