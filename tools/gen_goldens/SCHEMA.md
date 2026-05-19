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

| File                      | Sample keys                                                |
|---------------------------|------------------------------------------------------------|
| `fk_panda.json`           | `q (nq)`, `oMi` (njoints × 16) — homogeneous, col-major    |
| `rnea_panda.json`         | `q (nq)`, `v (nv)`, `a (nv)`, `tau (nv)`                   |
| `aba_panda.json`          | `q (nq)`, `v (nv)`, `tau (nv)`, `ddq (nv)`                 |
| `crba_panda.json`         | `q (nq)`, `M (nv*nv)` — col-major, symmetrized             |
| `jac_panda.json`          | `q (nq)`, `frame_id`, `rf` (0/1/2), `J` (6*nv) col-major  |
| `se3_ops.json`            | `m_in` (16), `inv` (16), `log6` (6), `jlog6` (36)         |
| `lie_ops.json`            | `q0 (nq)`, `q1 (nq)`, `v` (nv), `q_back` (nq)             |
| `fk_derivs_panda.json`    | `q`, `v`, `a`, `joint_id`, `rf`, `dv_dq` (6*nv), `dv_dv` (6*nv) |

Sample counts: 100 for FK/RNEA/ABA/CRBA; 50 for Jacobians and derivatives;
500 for `se3_ops` and `lie_ops` (pure-math suites are cheap).
