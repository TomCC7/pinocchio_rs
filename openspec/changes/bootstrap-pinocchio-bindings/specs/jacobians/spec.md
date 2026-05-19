## ADDED Requirements

### Requirement: Joint Jacobian computation

The crate SHALL provide `data.compute_joint_jacobian(&model, &q, joint_id)` that populates the joint Jacobian for the given joint, and an accessor `data.joint_jacobian(joint_id) -> DMatrixView<'_, f64>` returning a borrowed `6 × nv` view.

#### Scenario: Joint Jacobian shape

- **WHEN** the user calls `data.compute_joint_jacobian(&model, &q, joint_id)` and reads `data.joint_jacobian(joint_id)`
- **THEN** the returned view has shape `(6, model.nv())`

### Requirement: Frame Jacobian with selectable reference frame

The crate SHALL expose a `ReferenceFrame` enum with variants `Local`, `LocalWorldAligned`, and `World` matching Pinocchio's C++ enum values. `data.compute_frame_jacobian(&model, &q, frame_id, ReferenceFrame)` SHALL populate the frame Jacobian for the requested frame in the requested reference frame.

#### Scenario: Reference frame variants are exhaustive

- **WHEN** a user matches on `ReferenceFrame`
- **THEN** all three variants (`Local`, `LocalWorldAligned`, `World`) are covered

#### Scenario: Different reference frames produce different Jacobians

- **WHEN** the user computes the frame Jacobian for the same `(q, frame_id)` in `Local` and `World` reference frames
- **THEN** the resulting matrices differ in general (proving the enum is plumbed correctly through to Pinocchio)

### Requirement: Numerical parity with C++ Pinocchio for Jacobians

For every Jacobian produced by `compute_joint_jacobian` or `compute_frame_jacobian`, the result SHALL match the corresponding C++ Pinocchio output to within an absolute tolerance of `1e-10` per component.

#### Scenario: Parity over 50 sampled (q, frame_id, reference_frame)

- **WHEN** the parity test loads 50 (q, frame_id, reference_frame, expected_J) tuples from the C++-generated golden JSON and runs the Rust binding on each
- **THEN** every returned Jacobian matches the golden value within `1e-10` in every component
