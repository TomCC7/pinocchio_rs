## ADDED Requirements

### Requirement: SE3 newtype with Pinocchio semantics

The crate SHALL provide an `SE3` newtype representing an element of SE(3), with constructors (`identity`, `from_rotation_translation`, `from_homogeneous`), operations (`inverse`, `act` on `Motion`/`Force`/point, `compose` via `*`), and manifold operations (`log6` returning a `Motion`-shaped 6-vector, `Jlog6` returning a 6×6 Jacobian). Quaternion components inside SE3 SHALL be stored in `(x, y, z, w)` order to match Pinocchio.

#### Scenario: Identity is left and right neutral

- **WHEN** the user composes any `SE3` `t` with `SE3::identity()` on either side
- **THEN** the result equals `t` to machine precision

#### Scenario: Inverse round-trip

- **WHEN** the user computes `t.inverse() * t` for any `SE3` `t`
- **THEN** the result equals `SE3::identity()` to numerical tolerance (1e-12)

#### Scenario: Log of identity is zero

- **WHEN** the user calls `SE3::identity().log6()`
- **THEN** the returned 6-vector is zero to machine precision

#### Scenario: Quaternion order matches Pinocchio

- **WHEN** the user constructs an `SE3` from a known rotation and reads its quaternion components
- **THEN** the components are laid out as `(x, y, z, w)` matching `pinocchio::SE3::rotation()` consumed via Eigen quaternion

### Requirement: Motion and Force spatial vector types

The crate SHALL provide `Motion` and `Force` newtypes representing elements of `se(3)` (the Lie algebra) and its dual (spatial forces) respectively. Each SHALL expose `.linear()` and `.angular()` views, `+`/`-`/scalar multiplication, and conversion to/from `nalgebra::Vector6<f64>`. `Motion` and `Force` MUST be distinct types: `Motion + Force` SHALL be a compile-time type error.

#### Scenario: Motion plus Force does not compile

- **WHEN** a user writes `motion + force` in Rust source
- **THEN** the code fails to compile because no `Add<Force> for Motion` impl exists

#### Scenario: Motion linear/angular accessors

- **WHEN** the user constructs a `Motion` from `(linear, angular)` 3-vectors and reads `.linear()` and `.angular()`
- **THEN** the returned 3-vectors equal the inputs to machine precision

### Requirement: Inertia type

The crate SHALL provide an `Inertia` newtype with constructors from `(mass, com, inertia_tensor)`, conversion to/from a 6×6 spatial inertia matrix, and standard spatial operations (`SE3.act(&Inertia)` for frame transport).

#### Scenario: Round-trip through 6x6 spatial inertia

- **WHEN** the user constructs an `Inertia` from physical parameters, converts to a 6×6 matrix, and reconstructs an `Inertia` from that matrix
- **THEN** the reconstructed inertia is equal to the original to numerical tolerance (1e-10)
