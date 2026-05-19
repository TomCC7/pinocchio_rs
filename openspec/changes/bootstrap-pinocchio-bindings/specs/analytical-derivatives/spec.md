## ADDED Requirements

### Requirement: Forward kinematics derivatives

The crate SHALL expose `data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)` invoking Pinocchio's `computeForwardKinematicsDerivatives` algorithm, plus accessors that retrieve the resulting partial-derivative matrices (specifically the joint-velocity partials `∂v_joint/∂q` and `∂v_joint/∂v` in the `Local` reference frame for a given joint).

#### Scenario: Derivatives produce 6 x nv matrices

- **WHEN** the user calls `data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)` followed by `data.joint_velocity_derivatives(joint_id, ReferenceFrame::Local)`
- **THEN** the returned `(∂v/∂q, ∂v/∂v)` pair contains two `(6, model.nv())` matrices

### Requirement: Numerical parity with C++ Pinocchio for derivatives

For every output of the exposed derivative accessor, the result SHALL match the corresponding C++ Pinocchio output to within an absolute tolerance of `1e-8` per component (looser than direct-algorithm tolerance because derivative computations accumulate more floating-point error).

#### Scenario: Parity over 50 sampled (q, v) derivatives

- **WHEN** the parity test loads 50 (q, v, expected_dv_dq, expected_dv_dv) tuples from the C++-generated golden JSON and runs the Rust binding on each
- **THEN** every returned partial-derivative matrix matches its golden within `1e-8` in every component
