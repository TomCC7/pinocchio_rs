// SPDX-License-Identifier: BSD-2-Clause
//! Analytical derivatives of forward kinematics.
//!
//! See `pinocchio::computeForwardKinematicsDerivatives`.

use nalgebra::{DMatrix, DVector};

use crate::{Data, Error, Model, ReferenceFrame, Result};

impl Data {
    /// Pre-pass that fills the per-joint derivative caches read by
    /// [`Self::joint_velocity_derivatives`].
    pub fn compute_forward_kinematics_derivatives(
        &mut self,
        model: &Model,
        q: &DVector<f64>,
        v: &DVector<f64>,
        a: &DVector<f64>,
    ) -> Result<()> {
        check_len("q", q.len(), model.nq())?;
        check_len("v", v.len(), model.nv())?;
        check_len("a", a.len(), model.nv())?;
        unsafe {
            pinocchio_sys::ffi::compute_forward_kinematics_derivatives(
                model.raw(),
                self.inner.pin_mut(),
                q.as_ptr(),
                model.nq(),
                v.as_ptr(),
                model.nv(),
                a.as_ptr(),
                model.nv(),
            );
        }
        Ok(())
    }

    /// Read `∂v/∂q` and `∂v/∂v` for a single joint, in the requested frame.
    ///
    /// Returns `(dv_dq, dv_dv)`, each a 6×nv matrix.
    pub fn joint_velocity_derivatives(
        &mut self,
        model: &Model,
        joint_id: usize,
        rf: ReferenceFrame,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        let nv = model.nv();
        let mut dq = vec![0.0; 6 * nv];
        let mut dv = vec![0.0; 6 * nv];
        unsafe {
            pinocchio_sys::ffi::data_joint_velocity_derivatives(
                model.raw(),
                self.inner.pin_mut(),
                joint_id,
                rf.to_u8(),
                dq.as_mut_ptr(),
                dv.as_mut_ptr(),
                nv,
            );
        }
        (
            DMatrix::from_vec(6, nv, dq),
            DMatrix::from_vec(6, nv, dv),
        )
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
    use crate::Data;
    use approx::assert_relative_eq;

    /// UT.12 — for zero velocity and zero acceleration, ∂v/∂v equals the joint
    /// Jacobian (a known identity). Catches sign/transpose bugs.
    #[test]
    fn dv_dv_at_zero_equals_joint_jacobian() {
        let model = Model::from_urdf(panda_urdf_path()).unwrap();
        let mut data = Data::new(&model);
        let q = model.neutral_configuration();
        let zero = DVector::zeros(model.nv());

        // Reference: joint Jacobian for the end-effector joint in the Local frame.
        let joint_id = model.joint_id("panda_joint7").unwrap();
        let j_ref = data.joint_jacobian(&model, &q, joint_id).unwrap();

        // Compute the derivatives at zero v, zero a.
        data.compute_forward_kinematics_derivatives(&model, &q, &zero, &zero)
            .unwrap();
        let (_dv_dq, dv_dv) = data.joint_velocity_derivatives(&model, joint_id, ReferenceFrame::Local);

        assert_eq!(dv_dv.shape(), (6, model.nv()));
        assert_relative_eq!(dv_dv, j_ref, epsilon = 1e-10);
    }
}
