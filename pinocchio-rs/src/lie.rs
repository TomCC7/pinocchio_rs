// SPDX-License-Identifier: BSD-2-Clause
//! Lie-group operations on the configuration manifold.
//!
//! For fixed-base robots whose joints are all revolute/prismatic, the
//! configuration space is just Rⁿ and these operations reduce to vector
//! addition/subtraction. They become nontrivial in the presence of free-flyer
//! or spherical joints, where `q` lives on SE(3) × Rⁿ or S³ × Rⁿ.

use nalgebra::DVector;

use crate::{Error, Model, Result};

impl Model {
    /// `q_out = integrate(q, v)`. `q.len() == nq`, `v.len() == nv`, output is
    /// length `nq`.
    pub fn integrate(&self, q: &DVector<f64>, v: &DVector<f64>) -> Result<DVector<f64>> {
        check_len("q", q.len(), self.nq())?;
        check_len("v", v.len(), self.nv())?;
        let mut out = vec![0.0; self.nq()];
        unsafe {
            pinocchio_sys::ffi::model_integrate(
                self.raw(),
                q.as_ptr(),
                self.nq(),
                v.as_ptr(),
                self.nv(),
                out.as_mut_ptr(),
                self.nq(),
            );
        }
        Ok(DVector::from_vec(out))
    }

    /// `v = difference(q0, q1)`. Both inputs length `nq`, output length `nv`.
    pub fn difference(&self, q0: &DVector<f64>, q1: &DVector<f64>) -> Result<DVector<f64>> {
        check_len("q0", q0.len(), self.nq())?;
        check_len("q1", q1.len(), self.nq())?;
        let mut out = vec![0.0; self.nv()];
        unsafe {
            pinocchio_sys::ffi::model_difference(
                self.raw(),
                q0.as_ptr(),
                q1.as_ptr(),
                self.nq(),
                out.as_mut_ptr(),
                self.nv(),
            );
        }
        Ok(DVector::from_vec(out))
    }

    /// Random configuration respecting per-joint bounds. The PRNG is seeded
    /// by `seed` so two calls with the same seed produce identical output.
    pub fn random_configuration(&self, seed: u64) -> DVector<f64> {
        let mut out = vec![0.0; self.nq()];
        unsafe {
            pinocchio_sys::ffi::model_random_configuration(
                self.raw(),
                seed,
                out.as_mut_ptr(),
                self.nq(),
            );
        }
        DVector::from_vec(out)
    }
}

fn check_len(name: &str, got: usize, expected: usize) -> Result<()> {
    if got != expected {
        Err(Error::OutOfRange(format!(
            "`{name}` has length {got}, expected {expected}",
        )))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::panda_urdf_path;
    use approx::assert_relative_eq;

    fn panda_fixed() -> Model {
        Model::from_urdf(panda_urdf_path()).unwrap()
    }

    fn panda_free_flyer() -> Model {
        Model::from_urdf_with_free_flyer(panda_urdf_path()).unwrap()
    }

    /// UT.10 — Lie operations on a fixed-base 7R robot.
    #[test]
    fn integrate_zero_is_identity() {
        let m = panda_fixed();
        let q = m.neutral_configuration();
        let zero = DVector::zeros(m.nv());
        let q_out = m.integrate(&q, &zero).unwrap();
        assert_relative_eq!(q, q_out, epsilon = 1e-12);
    }

    #[test]
    fn integrate_difference_round_trip_fixed() {
        let m = panda_fixed();
        for seed in 0..10u64 {
            let q0 = m.random_configuration(seed);
            let q1 = m.random_configuration(seed.wrapping_mul(31) + 7);
            let v = m.difference(&q0, &q1).unwrap();
            let q_back = m.integrate(&q0, &v).unwrap();
            assert_relative_eq!(q1, q_back, epsilon = 1e-10);
        }
    }

    #[test]
    fn random_configuration_is_deterministic() {
        let m = panda_fixed();
        let a = m.random_configuration(42);
        let b = m.random_configuration(42);
        assert_relative_eq!(a, b, epsilon = 0.0);
    }

    /// UT.10 — free-flyer quaternion stays unit-normalized after integrate.
    #[test]
    fn free_flyer_quaternion_remains_unit() {
        let m = panda_free_flyer();
        let q = m.neutral_configuration();
        // Small random twist on the free-flyer's 6-DoF + 7 arm joints.
        let mut v = vec![0.0; m.nv()];
        for (i, x) in v.iter_mut().enumerate() {
            *x = 0.01 * ((i as f64) + 1.0).sin();
        }
        let q_out = m.integrate(&q, &DVector::from_vec(v)).unwrap();
        // The free-flyer quaternion lives at positions [3..7] (after the
        // 3-vec translation) in pinocchio's storage order.
        let qw = q_out[3];
        let qx = q_out[4];
        let qy = q_out[5];
        let qz = q_out[6];
        let norm = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();
        assert_relative_eq!(norm, 1.0, epsilon = 1e-12);
    }
}
