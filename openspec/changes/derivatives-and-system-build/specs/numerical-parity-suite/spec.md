## MODIFIED Requirements

### Requirement: C++ oracle binary generates golden values

The repository SHALL include a C++ binary at `tools/gen_goldens/gen_goldens.cpp` (built via the same pixi env) that links directly to Pinocchio and emits JSON files under `pinocchio-rs/tests/goldens/`. The binary SHALL be invokable via `pixi run gen-goldens` (a task defined in `pixi.toml`), and its outputs SHALL be deterministic given a fixed RNG seed (default seed = 42).

#### Scenario: Goldens are reproducible

- **WHEN** a contributor runs `pixi run gen-goldens` twice on the same machine without modifying the source
- **THEN** the two runs produce byte-identical JSON files

#### Scenario: Goldens cover every parity-tested algorithm

- **WHEN** the binary runs to completion
- **THEN** at minimum the following JSON files are produced: `fk_panda.json`, `rnea_panda.json`, `aba_panda.json`, `crba_panda.json`, `jac_panda.json`, `se3_ops.json`, `lie_ops.json`, `fk_derivs_panda.json`, `rnea_derivs_panda.json`, `aba_derivs_panda.json`, `frame_kinematics_derivs_panda.json`

## ADDED Requirements

### Requirement: Goldens for RNEA derivatives

The oracle binary SHALL emit a golden file `rnea_derivs_panda.json` containing 50 samples. Each sample SHALL include:

- inputs: `q` (length nq), `v` (length nv), `a` (length nv), all sampled within joint limits using `std::mt19937(seed ^ stream_id)` with a stable stream id for this algorithm;
- outputs: `dtau_dq` and `dtau_dv` as `(nv, nv)` column-major flat arrays.

#### Scenario: RNEA-derivs golden file is well-formed

- **WHEN** a parity test loads `rnea_derivs_panda.json`
- **THEN** the file deserializes into 50 envelopes, each containing `q` of length `nq`, `v` and `a` of length `nv`, and two `nv*nv` flat arrays labeled `dtau_dq` and `dtau_dv`

### Requirement: Goldens for ABA derivatives

The oracle binary SHALL emit a golden file `aba_derivs_panda.json` containing 50 samples. Each sample SHALL include inputs `q` (length nq), `v` (length nv), `tau` (length nv), and outputs `dddq_dq` and `dddq_dv` as `(nv, nv)` column-major flat arrays.

#### Scenario: ABA-derivs golden file is well-formed

- **WHEN** a parity test loads `aba_derivs_panda.json`
- **THEN** the file deserializes into 50 envelopes, each containing `q` of length `nq`, `v` and `tau` of length `nv`, and two `nv*nv` flat arrays labeled `dddq_dq` and `dddq_dv`

### Requirement: Goldens for frame kinematics derivatives

The oracle binary SHALL emit a golden file `frame_kinematics_derivs_panda.json` containing 50 samples. Each sample SHALL include inputs `q`, `v`, `a`, plus a frame id and `ReferenceFrame` selector, and outputs covering both frame-velocity derivatives (`dv_dq`, `dv_dv`) and frame-acceleration derivatives (`da_dq`, `da_dv`, `da_da`), each as `(6, nv)` column-major flat arrays. All three `ReferenceFrame` variants SHALL appear at least once across the 50 samples.

#### Scenario: Frame-derivs golden file covers all reference frames

- **WHEN** a parity test loads `frame_kinematics_derivs_panda.json`
- **THEN** the 50 envelopes collectively cover all three `ReferenceFrame` discriminants (World, Local, LocalWorldAligned), and every envelope contains the five `(6, nv)` matrices for the frame derivatives

### Requirement: Rust parity test for RNEA derivatives

`pinocchio-rs/tests/parity_rnea_derivs.rs` SHALL load `rnea_derivs_panda.json`, replay each sample through `data.rnea_derivatives(...)` followed by the accessor, and assert that the returned `dtau_dq` and `dtau_dv` match the golden within `TOL_DERIV = 1e-8` per component.

#### Scenario: RNEA-derivs parity test runs in pure Rust

- **WHEN** a contributor runs `pixi run cargo test -p pinocchio-rs --test parity_rnea_derivs`
- **THEN** the test loads goldens, replays through the binding, and reports a pass with no C++ linking from the test crate itself

### Requirement: Rust parity test for ABA derivatives

`pinocchio-rs/tests/parity_aba_derivs.rs` SHALL load `aba_derivs_panda.json`, replay each sample through `data.aba_derivatives(...)` followed by the accessor, and assert that the returned `dddq_dq` and `dddq_dv` match the golden within `TOL_DERIV = 1e-8` per component.

#### Scenario: ABA-derivs parity test runs in pure Rust

- **WHEN** a contributor runs `pixi run cargo test -p pinocchio-rs --test parity_aba_derivs`
- **THEN** the test loads goldens, replays through the binding, and reports a pass

### Requirement: Rust parity test for frame kinematics derivatives

`pinocchio-rs/tests/parity_frame_derivs.rs` SHALL load `frame_kinematics_derivs_panda.json` and, for each sample, replay through `data.frame_velocity_derivatives(frame_id, rf)` and `data.frame_acceleration_derivatives(frame_id, rf)`. The five returned matrices SHALL match their golden counterparts within `TOL_DERIV = 1e-8` per component, and the test SHALL assert that all three `ReferenceFrame` variants appear in the sample set.

#### Scenario: Frame-derivs parity test covers all three reference frames

- **WHEN** a contributor runs `pixi run cargo test -p pinocchio-rs --test parity_frame_derivs`
- **THEN** the test passes and the test's own logic asserts that every `ReferenceFrame` variant occurred at least once across the 50 samples
