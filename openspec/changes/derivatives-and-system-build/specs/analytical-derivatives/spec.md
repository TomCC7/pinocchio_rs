## ADDED Requirements

### Requirement: RNEA derivatives

The crate SHALL expose `data.rnea_derivatives(&model, &q, &v, &a)` invoking Pinocchio's `computeRNEADerivatives` algorithm. After the call, accessors SHALL return `∂τ/∂q` and `∂τ/∂v` as owned `nalgebra::DMatrix<f64>` values of shape `(nv, nv)` each. The third partial `∂τ/∂a` SHALL NOT be exposed as a separate accessor — it equals the mass matrix `M` already returned by `data.mass_matrix(&model)`, and the wrapper's rustdoc SHALL state this equivalence.

#### Scenario: RNEA derivatives produce nv x nv matrices

- **WHEN** the user calls `data.rnea_derivatives(&model, &q, &v, &a)` and then `data.rnea_partials()`
- **THEN** the returned `(dtau_dq, dtau_dv)` pair contains two `(model.nv(), model.nv())` matrices

#### Scenario: RNEA derivative dtau_da is documented as the mass matrix

- **WHEN** a contributor reads the rustdoc for `data.rnea_derivatives` and `data.rnea_partials`
- **THEN** the documentation explicitly notes that `∂τ/∂a` is the mass matrix and directs the user to `data.mass_matrix(&model)` for it

### Requirement: ABA derivatives

The crate SHALL expose `data.aba_derivatives(&model, &q, &v, &tau)` invoking Pinocchio's `computeABADerivatives` algorithm. After the call, accessors SHALL return `∂q̈/∂q` and `∂q̈/∂v` as owned `(nv, nv)` matrices. The third partial `∂q̈/∂τ` (= M⁻¹) SHALL NOT be exposed — callers compute it via Cholesky-inversion of `data.mass_matrix(&model)` and the rustdoc SHALL state this.

#### Scenario: ABA derivatives produce nv x nv matrices

- **WHEN** the user calls `data.aba_derivatives(&model, &q, &v, &tau)` and then `data.aba_partials()`
- **THEN** the returned `(dddq_dq, dddq_dv)` pair contains two `(model.nv(), model.nv())` matrices

### Requirement: Joint-acceleration derivatives

The crate SHALL expose `data.joint_acceleration_derivatives(joint_id, ReferenceFrame)` returning `(∂a/∂q, ∂a/∂v, ∂a/∂a)` as a triple of owned `(6, nv)` matrices for any of the three `ReferenceFrame` variants. The accessor SHALL require that `data.compute_forward_kinematics_derivatives(...)` has been called first (this is the same precondition as the existing joint-velocity-derivatives accessor); the rustdoc SHALL document this precondition.

#### Scenario: Joint-acceleration derivatives in all three frames

- **WHEN** the user calls `data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)` and then `data.joint_acceleration_derivatives(joint_id, rf)` for each `rf` ∈ {World, Local, LocalWorldAligned}
- **THEN** every call returns three `(6, model.nv())` matrices, and the three calls yield numerically distinct results (modulo trivial cases like the root joint)

### Requirement: Frame-velocity derivatives

The crate SHALL expose `data.frame_velocity_derivatives(frame_id, ReferenceFrame)` returning `(∂v_frame/∂q, ∂v_frame/∂v)` as a pair of owned `(6, nv)` matrices. The accessor SHALL require `compute_forward_kinematics_derivatives(...)` to have been called first.

#### Scenario: Frame-velocity derivatives shape and frame dependence

- **WHEN** the user calls `data.frame_velocity_derivatives(frame_id, rf)` for any valid `frame_id` and any `rf` ∈ {World, Local, LocalWorldAligned}
- **THEN** the returned matrices are each `(6, model.nv())` and the three reference-frame variants yield numerically distinct results on the Panda `panda_hand` frame

### Requirement: Frame-acceleration derivatives

The crate SHALL expose `data.frame_acceleration_derivatives(frame_id, ReferenceFrame)` returning `(∂a_frame/∂q, ∂a_frame/∂v, ∂a_frame/∂a)` as a triple of owned `(6, nv)` matrices. The accessor SHALL require `compute_forward_kinematics_derivatives(...)` to have been called first.

#### Scenario: Frame-acceleration derivatives shape

- **WHEN** the user calls `data.frame_acceleration_derivatives(frame_id, rf)` after `compute_forward_kinematics_derivatives`
- **THEN** the returned triple contains three `(6, model.nv())` matrices

### Requirement: Numerical parity for added derivative algorithms

Every output of every newly added derivative accessor (RNEA derivatives, ABA derivatives, joint-acceleration derivatives, frame-velocity derivatives, frame-acceleration derivatives) SHALL match the corresponding C++ Pinocchio output to within an absolute tolerance of `1e-8` per component — the same `TOL_DERIV` constant defined by the numerical-parity-suite capability.

#### Scenario: Parity over 50 sampled inputs per derivative family

- **WHEN** each derivative-specific parity test loads its golden JSON (50 samples each for RNEA-derivs, ABA-derivs, frame-kinematics-derivs) and replays through the Rust binding
- **THEN** every returned partial-derivative matrix matches its golden within `1e-8` in every component, for every sample, across all applicable reference-frame variants
