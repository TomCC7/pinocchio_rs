## ADDED Requirements

### Requirement: Configuration manifold integrate

The crate SHALL provide `model.integrate(&q, &v_dt) -> DVector<f64>` that advances a configuration along a tangent direction using Pinocchio's manifold integration, correctly handling SO(3) / SE(3) sub-manifolds within `q` for free-flyer and spherical joints.

#### Scenario: Integrate zero tangent is identity

- **WHEN** the user calls `model.integrate(&q, &zero_vec)` for any valid `q`
- **THEN** the returned configuration equals `q` to machine precision

#### Scenario: Integrate preserves quaternion normalization

- **WHEN** the user calls `model.integrate(&q, &v_dt)` on a model with a free-flyer base
- **THEN** the SE(3) quaternion components of the result are unit-normalized within `1e-12`

### Requirement: Configuration manifold difference

The crate SHALL provide `model.difference(&q0, &q1) -> DVector<f64>` that returns the tangent vector that, when integrated from `q0`, produces `q1`. `model.integrate(&q0, &model.difference(&q0, &q1))` SHALL recover `q1` exactly (within numerical tolerance).

#### Scenario: Integrate-difference round-trip

- **WHEN** the user computes `v = model.difference(&q0, &q1)` and then `q1_back = model.integrate(&q0, &v)`
- **THEN** `q1_back` equals `q1` within `1e-10` in every Euclidean component and within `1e-10` orientation error on each SO(3) sub-manifold

### Requirement: Neutral and random configuration

The crate SHALL provide `model.neutral_configuration() -> DVector<f64>` returning the canonical neutral configuration, and `model.random_configuration() -> DVector<f64>` returning a uniformly-sampled valid configuration that respects joint limits and manifold constraints.

#### Scenario: Neutral configuration is valid

- **WHEN** the user calls `model.neutral_configuration()` on a free-flyer model
- **THEN** the SE(3) sub-vector has a unit quaternion and zero translation

#### Scenario: Random configuration is valid

- **WHEN** the user calls `model.random_configuration()` 100 times on the Panda model
- **THEN** each result is a length-`nq` vector whose joint angles are within Panda's URDF-declared limits

### Requirement: Numerical parity with C++ Pinocchio for Lie operations

For every output of `integrate`, `difference`, `neutral_configuration`, and (with a fixed RNG seed) `random_configuration`, the result SHALL match the corresponding C++ Pinocchio output to within an absolute tolerance of `1e-10`.

#### Scenario: Parity over 500 sampled Lie operations

- **WHEN** the parity test loads 500 (q, v, expected_q_next) tuples for `integrate` and 500 (q0, q1, expected_v) tuples for `difference` from C++-generated golden JSON
- **THEN** every Rust binding output matches its golden within `1e-10`
