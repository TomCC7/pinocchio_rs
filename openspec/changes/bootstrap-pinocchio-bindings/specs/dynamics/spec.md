## ADDED Requirements

### Requirement: Inverse dynamics (RNEA)

The crate SHALL provide `data.rnea(&model, &q, &v, &a) -> DVector<f64>` that invokes Pinocchio's Recursive Newton-Euler Algorithm and returns the generalized joint torques `tau` corresponding to the requested motion. The result MUST be an owned copy (not a borrow).

#### Scenario: RNEA on zero motion

- **WHEN** the user calls `data.rnea(&model, &q, &zero_vec, &zero_vec)` on a fixed-base manipulator with default gravity
- **THEN** the returned `tau` equals the joint torques required to hold pose `q` against gravity, matching `pinocchio::rnea` to within `1e-10` in every component

#### Scenario: Parity over 100 sampled (q, v, a)

- **WHEN** the parity test loads 100 (q, v, a, expected_tau) tuples from the C++-generated golden JSON and runs the Rust binding on each
- **THEN** every returned `tau` equals the golden value within `1e-10`

### Requirement: Forward dynamics (ABA)

The crate SHALL provide `data.aba(&model, &q, &v, &tau) -> DVector<f64>` that invokes Pinocchio's Articulated Body Algorithm and returns the generalized joint accelerations `ddq` corresponding to the requested torques. The result MUST be an owned copy.

#### Scenario: ABA inverse of RNEA

- **WHEN** the user computes `tau = data.rnea(&model, &q, &v, &a)` and then `ddq = data.aba(&model, &q, &v, &tau)`
- **THEN** `ddq` equals `a` to within `1e-10` in every component

#### Scenario: Parity over 100 sampled (q, v, tau)

- **WHEN** the parity test loads 100 (q, v, tau, expected_ddq) tuples from the C++-generated golden JSON and runs the Rust binding on each
- **THEN** every returned `ddq` equals the golden value within `1e-10`

### Requirement: Mass matrix (CRBA)

The crate SHALL provide an accessor pattern where `data.crba(&model, &q)` populates the internal mass matrix and `data.mass_matrix() -> DMatrixView<'_, f64>` returns a borrowed `nv × nv` view. The returned view's lifetime MUST be tied to the borrow of `Data` to prevent reading after a subsequent mutation.

#### Scenario: Mass matrix is symmetric positive definite

- **WHEN** the user calls `data.crba(&model, &q)` and reads `data.mass_matrix()`
- **THEN** the matrix is symmetric to within `1e-12` and all eigenvalues are strictly positive

#### Scenario: Mass matrix borrow blocks mutation

- **WHEN** the user holds a `DMatrixView` from `data.mass_matrix()` and attempts to call `data.rnea(...)` before dropping the view
- **THEN** the code fails to compile because the immutable borrow conflicts with the mutable borrow required by `rnea`

#### Scenario: Parity over 100 sampled q

- **WHEN** the parity test loads 100 (q, expected_M) pairs from the C++-generated golden JSON and runs the Rust binding on each
- **THEN** every returned `M` matches the golden value within `1e-10` in every component
