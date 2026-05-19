## ADDED Requirements

### Requirement: Load a Model from a URDF file

The crate SHALL provide a function that constructs a Pinocchio `Model` from a URDF file on disk, returning a `Result` that surfaces parse and I/O errors. Free-floating bases SHALL be supported via an explicit constructor variant.

#### Scenario: Load a valid URDF

- **WHEN** the user calls the URDF loader with a path to `tests/data/panda/panda.urdf`
- **THEN** the call returns `Ok(Model)` and the returned `Model`'s `nq()` and `nv()` reflect the URDF's joint counts

#### Scenario: Load a URDF with a free-floating base

- **WHEN** the user calls the URDF loader's free-flyer variant on a humanoid URDF
- **THEN** the returned `Model`'s `nq() == 7 + n_joint_q` and `nv() == 6 + n_joint_v`, reflecting the SE(3) base + joints

#### Scenario: Load a missing URDF

- **WHEN** the user calls the URDF loader with a path that does not exist
- **THEN** the call returns `Err` with a message identifying the missing path

#### Scenario: Load a malformed URDF

- **WHEN** the user calls the URDF loader with a file that fails Pinocchio's URDF parse
- **THEN** the call returns `Err` with a message originating from the underlying urdfdom parser, propagated through `cxx`'s `Result`

### Requirement: Construct Data sized to a Model

The crate SHALL provide a `Data` companion type constructed from a borrowed `Model`. A given `Data` instance MUST remain associated with its source `Model` for its lifetime, and the safe API SHALL prevent calling algorithms with a mismatched `(Model, Data)` pair.

#### Scenario: Construct Data from Model

- **WHEN** the user calls `Data::new(&model)`
- **THEN** the returned `Data` has internal Eigen storage sized to `(nq, nv, n_joints)` of the given model

### Requirement: Expose model topology

The `Model` type SHALL expose the queries needed to drive algorithms and tests: `nq()`, `nv()`, `n_joints()`, `n_frames()`, joint name → joint id lookup, and frame name → frame id lookup.

#### Scenario: Lookup a joint by name

- **WHEN** the user calls `model.joint_id("panda_joint1")` on the loaded Panda model
- **THEN** the call returns `Some(joint_id)` and `model.joint_name(joint_id) == "panda_joint1"`

#### Scenario: Lookup an unknown joint

- **WHEN** the user calls `model.joint_id("does_not_exist")`
- **THEN** the call returns `None`
