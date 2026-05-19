## ADDED Requirements

### Requirement: Forward kinematics

The crate SHALL provide a method `data.forward_kinematics(&model, &q)` (and overloads taking optional `v` and `a`) that invokes Pinocchio's `forwardKinematics` algorithm and populates `Data` with joint placements (`oMi[]`), velocities, and accelerations.

#### Scenario: Forward kinematics on neutral configuration

- **WHEN** the user calls `data.forward_kinematics(&model, &model.neutral_configuration())` on the Panda model
- **THEN** the call returns without error and subsequent reads of `data.joint_placement(i)` for every joint `i` return the corresponding `SE3` for the neutral pose

#### Scenario: Forward kinematics with velocity

- **WHEN** the user calls `data.forward_kinematics_v(&model, &q, &v)` with non-zero `v`
- **THEN** subsequent reads of `data.joint_velocity(i)` return non-zero `Motion` values consistent with `v`

### Requirement: Access joint and frame placements

The `Data` type SHALL expose owned-copy accessors `joint_placement(joint_id) -> SE3` and `frame_placement(&model, frame_id) -> SE3` after `forward_kinematics` has been called.

#### Scenario: Frame placement requires updateFramePlacements

- **WHEN** the user reads `data.frame_placement(&model, frame_id)` after `forward_kinematics`
- **THEN** the call internally invokes `pinocchio::updateFramePlacements` if needed and returns the correct `SE3` for the requested frame

### Requirement: Numerical parity with C++ Pinocchio for forward kinematics

For every joint placement returned by `data.joint_placement(i)` after `forward_kinematics(&model, &q)`, the result SHALL match the corresponding output of `pinocchio::forwardKinematics(model, data, q)` in C++ to within an absolute tolerance of `1e-10` in each component of the homogeneous matrix.

#### Scenario: Parity over 100 sampled configurations

- **WHEN** the parity test loads 100 (q, expected_oMi) pairs from the C++-generated golden JSON and runs the Rust binding on each
- **THEN** every per-joint placement equals the golden value within `1e-10`
