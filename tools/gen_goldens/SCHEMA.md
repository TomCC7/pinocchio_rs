# Golden file schema

Each algorithm family produces one JSON file under `pinocchio-rs/tests/goldens/`.
The shared envelope is:

```json
{
  "robot": "panda",
  "pinocchio_version": "3.9.0",
  "seed": 42,
  "samples": [ /* per-algorithm sample object */, ... ]
}
```

All vectors and matrices are emitted as **flat arrays of `f64`** in **column-major**
order (matching nalgebra's default layout and Pinocchio's Eigen Matrix storage).

## Per-file sample shapes

| File                                  | Sample keys                                                                                                 |
|---------------------------------------|-------------------------------------------------------------------------------------------------------------|
| `fk_panda.json`                       | `q (nq)`, `oMi` (njoints × 16) — homogeneous, col-major                                                     |
| `rnea_panda.json`                     | `q (nq)`, `v (nv)`, `a (nv)`, `tau (nv)`                                                                    |
| `aba_panda.json`                      | `q (nq)`, `v (nv)`, `tau (nv)`, `ddq (nv)`                                                                  |
| `crba_panda.json`                     | `q (nq)`, `M (nv*nv)` — col-major, symmetrized                                                              |
| `jac_panda.json`                      | `q (nq)`, `frame_id`, `rf` (0/1/2), `J` (6*nv) col-major                                                    |
| `se3_ops.json`                        | `m_in` (16), `inv` (16), `log6` (6), `jlog6` (36)                                                           |
| `lie_ops.json`                        | `q0 (nq)`, `q1 (nq)`, `v` (nv), `q_back` (nq)                                                               |
| `fk_derivs_panda.json`                | `q`, `v`, `a`, `joint_id`, `rf`, `dv_dq` (6*nv), `dv_dv` (6*nv)                                             |
| `rnea_derivs_panda.json`              | `q`, `v`, `a`, `dtau_dq` (nv*nv), `dtau_dv` (nv*nv)                                                         |
| `aba_derivs_panda.json`               | `q`, `v`, `tau`, `dddq_dq` (nv*nv), `dddq_dv` (nv*nv)                                                       |
| `frame_kinematics_derivs_panda.json`  | `q`, `v`, `a`, `frame_id`, `rf`, `dv_dq` (6*nv), `dv_dv` (6*nv), `da_dq` (6*nv), `da_dv` (6*nv), `da_da` (6*nv) |

Sample counts: 100 for FK/RNEA/ABA/CRBA; 50 for Jacobians and derivatives
(including the three new derivative families); 500 for `se3_ops` and
`lie_ops` (pure-math suites are cheap).

`frame_kinematics_derivs_panda.json` cycles `rf` through {0, 1, 2} (World,
Local, LocalWorldAligned) across its 50 samples and rotates `frame_id`
across `panda_link3`, `panda_link5`, `panda_hand` so the file collectively
covers all three reference frames and at least three frames.
