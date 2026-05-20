// SPDX-License-Identifier: BSD-2-Clause
//! Analytical derivatives of dynamics and kinematics.
//!
//! Wraps Pinocchio's `computeForwardKinematicsDerivatives`,
//! `computeRNEADerivatives`, `computeABADerivatives`, and the family of
//! `get*Derivatives` accessors. Every accessor returns owned
//! `nalgebra::DMatrix<f64>` values (design DD1): the typical control /
//! optimization consumer reads each derivative once into its own per-step
//! tensor, so the allocation cost is not on the inner loop.
//!
//! # Pinocchio identities exposed implicitly
//!
//! Two third partials are NOT exposed by this module because they equal
//! quantities already reachable via [`Data::mass_matrix`]:
//!
//! - **RNEA**: `∂τ/∂a = M(q)` — get it via [`Data::mass_matrix`] after
//!   [`Data::crba`].
//! - **ABA**: `∂q̈/∂τ = M(q)⁻¹` — get it by Cholesky-inverting
//!   [`Data::mass_matrix`] (see the `aba_derivatives` rustdoc for the
//!   recommended idiom).
//!
//! Returning the same matrix twice through two different APIs would invite
//! the "which is the source of truth?" trap when the user mutates `Data`
//! between calls.

use nalgebra::{DMatrix, DVector};

use crate::{Data, Error, Model, ReferenceFrame, Result};

impl Data {
    // ============================================================
    // Forward-kinematics derivatives (shared precondition for the
    // joint-acceleration, frame-velocity, and frame-acceleration
    // accessors below).
    // ============================================================

    /// Pre-pass that fills the per-joint derivative caches read by
    /// [`Self::joint_velocity_derivatives`],
    /// [`Self::joint_acceleration_derivatives`],
    /// [`Self::frame_velocity_derivatives`], and
    /// [`Self::frame_acceleration_derivatives`].
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

    /// Read `∂a/∂q`, `∂a/∂v`, `∂a/∂a` for a single joint, in the requested
    /// frame.
    ///
    /// Returns a triple of 6×nv matrices. Requires
    /// [`Self::compute_forward_kinematics_derivatives`] to have run first.
    pub fn joint_acceleration_derivatives(
        &mut self,
        model: &Model,
        joint_id: usize,
        rf: ReferenceFrame,
    ) -> (DMatrix<f64>, DMatrix<f64>, DMatrix<f64>) {
        let nv = model.nv();
        let mut dq = vec![0.0; 6 * nv];
        let mut dv = vec![0.0; 6 * nv];
        let mut da = vec![0.0; 6 * nv];
        unsafe {
            pinocchio_sys::ffi::data_joint_acceleration_derivatives(
                model.raw(),
                self.inner.pin_mut(),
                joint_id,
                rf.to_u8(),
                dq.as_mut_ptr(),
                dv.as_mut_ptr(),
                da.as_mut_ptr(),
                nv,
            );
        }
        (
            DMatrix::from_vec(6, nv, dq),
            DMatrix::from_vec(6, nv, dv),
            DMatrix::from_vec(6, nv, da),
        )
    }

    /// Read `∂v_frame/∂q` and `∂v_frame/∂v` for a single frame, in the
    /// requested reference frame.
    ///
    /// Returns `(dv_dq, dv_dv)`, each a 6×nv matrix. Requires
    /// [`Self::compute_forward_kinematics_derivatives`] to have run first.
    pub fn frame_velocity_derivatives(
        &mut self,
        model: &Model,
        frame_id: usize,
        rf: ReferenceFrame,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        let nv = model.nv();
        let mut dq = vec![0.0; 6 * nv];
        let mut dv = vec![0.0; 6 * nv];
        unsafe {
            pinocchio_sys::ffi::data_frame_velocity_derivatives(
                model.raw(),
                self.inner.pin_mut(),
                frame_id,
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

    /// Read `∂a_frame/∂q`, `∂a_frame/∂v`, `∂a_frame/∂a` for a single frame,
    /// in the requested reference frame.
    ///
    /// Returns a triple of 6×nv matrices. Requires
    /// [`Self::compute_forward_kinematics_derivatives`] to have run first.
    pub fn frame_acceleration_derivatives(
        &mut self,
        model: &Model,
        frame_id: usize,
        rf: ReferenceFrame,
    ) -> (DMatrix<f64>, DMatrix<f64>, DMatrix<f64>) {
        let nv = model.nv();
        let mut dq = vec![0.0; 6 * nv];
        let mut dv = vec![0.0; 6 * nv];
        let mut da = vec![0.0; 6 * nv];
        unsafe {
            pinocchio_sys::ffi::data_frame_acceleration_derivatives(
                model.raw(),
                self.inner.pin_mut(),
                frame_id,
                rf.to_u8(),
                dq.as_mut_ptr(),
                dv.as_mut_ptr(),
                da.as_mut_ptr(),
                nv,
            );
        }
        (
            DMatrix::from_vec(6, nv, dq),
            DMatrix::from_vec(6, nv, dv),
            DMatrix::from_vec(6, nv, da),
        )
    }

    // ============================================================
    // RNEA derivatives (inverse-dynamics partials)
    // ============================================================

    /// Compute `∂τ/∂q` and `∂τ/∂v` for the inverse-dynamics map
    /// `τ = RNEA(model, q, v, a)`. Read with [`Self::rnea_partials`].
    ///
    /// The third partial `∂τ/∂a = M(q)` is **not** returned — it equals the
    /// joint-space mass matrix already reachable via
    /// [`Self::mass_matrix`] (after [`Self::crba`]). The shim's call to
    /// `computeRNEADerivatives` also leaves the upper triangle of `data.M`
    /// populated as a side effect; if you need M afterwards, run
    /// `crba(...)` to refresh and symmetrize it.
    pub fn rnea_derivatives(
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
            pinocchio_sys::ffi::compute_rnea_derivatives(
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

    /// Return `(∂τ/∂q, ∂τ/∂v)` as owned `(nv, nv)` matrices.
    ///
    /// Must be called after [`Self::rnea_derivatives`].
    pub fn rnea_partials(&self, model: &Model) -> (DMatrix<f64>, DMatrix<f64>) {
        let nv = model.nv();
        let mut dq = vec![0.0; nv * nv];
        let mut dv = vec![0.0; nv * nv];
        unsafe {
            pinocchio_sys::ffi::data_rnea_derivatives(
                self._raw(),
                dq.as_mut_ptr(),
                dv.as_mut_ptr(),
                nv,
            );
        }
        (
            DMatrix::from_vec(nv, nv, dq),
            DMatrix::from_vec(nv, nv, dv),
        )
    }

    // ============================================================
    // ABA derivatives (forward-dynamics partials)
    // ============================================================

    /// Compute `∂q̈/∂q` and `∂q̈/∂v` for the forward-dynamics map
    /// `q̈ = ABA(model, q, v, τ)`. Read with [`Self::aba_partials`].
    ///
    /// The third partial `∂q̈/∂τ = M(q)⁻¹` is **not** returned — invert the
    /// mass matrix yourself, e.g.:
    ///
    /// ```ignore
    /// data.crba(&model, &q)?;
    /// let m_inv = data.mass_matrix(&model).cholesky().expect("M is SPD").inverse();
    /// ```
    pub fn aba_derivatives(
        &mut self,
        model: &Model,
        q: &DVector<f64>,
        v: &DVector<f64>,
        tau: &DVector<f64>,
    ) -> Result<()> {
        check_len("q", q.len(), model.nq())?;
        check_len("v", v.len(), model.nv())?;
        check_len("tau", tau.len(), model.nv())?;
        unsafe {
            pinocchio_sys::ffi::compute_aba_derivatives(
                model.raw(),
                self.inner.pin_mut(),
                q.as_ptr(),
                model.nq(),
                v.as_ptr(),
                model.nv(),
                tau.as_ptr(),
                model.nv(),
            );
        }
        Ok(())
    }

    /// Return `(∂q̈/∂q, ∂q̈/∂v)` as owned `(nv, nv)` matrices.
    ///
    /// Must be called after [`Self::aba_derivatives`].
    pub fn aba_partials(&self, model: &Model) -> (DMatrix<f64>, DMatrix<f64>) {
        let nv = model.nv();
        let mut dq = vec![0.0; nv * nv];
        let mut dv = vec![0.0; nv * nv];
        unsafe {
            pinocchio_sys::ffi::data_aba_derivatives(
                self._raw(),
                dq.as_mut_ptr(),
                dv.as_mut_ptr(),
                nv,
            );
        }
        (
            DMatrix::from_vec(nv, nv, dq),
            DMatrix::from_vec(nv, nv, dv),
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

    fn panda() -> (Model, Data) {
        let model = Model::from_urdf(panda_urdf_path()).unwrap();
        let data = Data::new(&model);
        (model, data)
    }

    /// UT.12 — for zero velocity and zero acceleration, ∂v/∂v equals the joint
    /// Jacobian (a known identity). Catches sign/transpose bugs.
    #[test]
    fn dv_dv_at_zero_equals_joint_jacobian() {
        let (model, mut data) = panda();
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

    // ============================================================
    // §6 — RNEA derivatives
    // ============================================================

    /// UT.6a — at `v = 0`, dtau_dv is finite and (within numerical noise)
    /// equal to its own transpose. Pure smoke check that the FFI returns
    /// real numbers, not garbage. (At `v = 0` the velocity-dependent
    /// coupling terms vanish.)
    #[test]
    fn rnea_derivs_dtau_dv_zero_velocity() {
        let (model, mut data) = panda();
        let nv = model.nv();
        let q = model.neutral_configuration();
        let zero = DVector::zeros(nv);

        data.rnea_derivatives(&model, &q, &zero, &zero).unwrap();
        let (dtau_dq, dtau_dv) = data.rnea_partials(&model);

        assert_eq!(dtau_dq.shape(), (nv, nv));
        assert_eq!(dtau_dv.shape(), (nv, nv));
        for x in dtau_dv.iter() {
            assert!(x.is_finite(), "dtau_dv contains non-finite {x}");
        }
        for x in dtau_dq.iter() {
            assert!(x.is_finite(), "dtau_dq contains non-finite {x}");
        }
        // Symmetry check on dtau_dv at v = 0 (Coriolis coupling vanishes).
        for i in 0..nv {
            for j in 0..nv {
                assert_relative_eq!(dtau_dv[(i, j)], dtau_dv[(j, i)], epsilon = 1e-10);
            }
        }
    }

    /// UT.6b — shape and consistency under non-trivial inputs.
    #[test]
    fn rnea_derivs_shape() {
        let (model, mut data) = panda();
        let nv = model.nv();
        let q = model.random_configuration(7);
        let v = small_random_vec(nv, 11);
        let a = small_random_vec(nv, 13);

        data.rnea_derivatives(&model, &q, &v, &a).unwrap();
        let (dq, dv) = data.rnea_partials(&model);
        assert_eq!(dq.shape(), (nv, nv));
        assert_eq!(dv.shape(), (nv, nv));
        for x in dq.iter().chain(dv.iter()) {
            assert!(x.is_finite());
        }
    }

    /// Length-mismatch must surface as an error (not panic).
    #[test]
    fn rnea_derivs_wrong_lengths_errors() {
        let (model, mut data) = panda();
        let q = DVector::zeros(model.nq() + 1);
        let v = DVector::zeros(model.nv());
        let a = DVector::zeros(model.nv());
        let err = data.rnea_derivatives(&model, &q, &v, &a).unwrap_err();
        assert!(matches!(err, Error::OutOfRange(_)));
    }

    // ============================================================
    // §7 — ABA derivatives
    // ============================================================

    /// UT.7a — shape sanity.
    #[test]
    fn aba_derivs_shape() {
        let (model, mut data) = panda();
        let nv = model.nv();
        let q = model.random_configuration(17);
        let v = small_random_vec(nv, 19);
        let tau = small_random_vec(nv, 23);

        data.aba_derivatives(&model, &q, &v, &tau).unwrap();
        let (dq, dv) = data.aba_partials(&model);
        assert_eq!(dq.shape(), (nv, nv));
        assert_eq!(dv.shape(), (nv, nv));
        for x in dq.iter().chain(dv.iter()) {
            assert!(x.is_finite());
        }
    }

    /// UT.7b — known identity cross-check between the two algorithms.
    ///
    /// For τ = RNEA(q, v, a) and q̈ = ABA(q, v, τ), Pinocchio guarantees
    /// `∂q̈/∂q = -M⁻¹ · ∂τ/∂q` at the same (q, v). Strong proof that both
    /// derivative wrappers agree on signs and shapes.
    #[test]
    fn aba_consistency_with_rnea() {
        let (model, mut data) = panda();
        let nv = model.nv();
        let q = model.random_configuration(29);
        let v = small_random_vec(nv, 31);
        let a = small_random_vec(nv, 37);

        let tau = data.rnea(&model, &q, &v, &a).unwrap();

        // M⁻¹ from CRBA + Cholesky.
        data.crba(&model, &q).unwrap();
        let m = data.mass_matrix(&model);
        let m_inv = m.clone().cholesky().expect("M is SPD").inverse();

        // RNEA partials at (q, v, a).
        data.rnea_derivatives(&model, &q, &v, &a).unwrap();
        let (dtau_dq, _dtau_dv) = data.rnea_partials(&model);

        // ABA partials at (q, v, τ).
        data.aba_derivatives(&model, &q, &v, &tau).unwrap();
        let (dddq_dq, _dddq_dv) = data.aba_partials(&model);

        let predicted = -&m_inv * &dtau_dq;
        let diff_max = (&predicted - &dddq_dq)
            .iter()
            .copied()
            .map(f64::abs)
            .fold(0.0, f64::max);
        assert!(
            diff_max < 1e-8,
            "ABA-vs-RNEA-derivs identity violated (max |Δ| = {diff_max:.3e})",
        );
    }

    // ============================================================
    // §8 — Joint-acceleration derivatives
    // ============================================================

    /// UT.8a — shape.
    #[test]
    fn joint_accel_derivs_shape() {
        let (model, mut data) = panda();
        let nv = model.nv();
        let q = model.random_configuration(41);
        let v = small_random_vec(nv, 43);
        let a = small_random_vec(nv, 47);
        data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)
            .unwrap();
        let (dq, dv, da) =
            data.joint_acceleration_derivatives(&model, 4, ReferenceFrame::Local);
        assert_eq!(dq.shape(), (6, nv));
        assert_eq!(dv.shape(), (6, nv));
        assert_eq!(da.shape(), (6, nv));
    }

    /// UT.8b — the three reference-frame variants must yield distinct results
    /// on a non-root joint.
    #[test]
    fn joint_accel_derivs_frame_dependence() {
        let (model, mut data) = panda();
        let nv = model.nv();
        let q = model.random_configuration(53);
        let v = small_random_vec(nv, 59);
        let a = small_random_vec(nv, 61);
        data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)
            .unwrap();
        // Use joint id 4 (a mid-chain joint, distinct from root).
        let (_, _, da_world) =
            data.joint_acceleration_derivatives(&model, 4, ReferenceFrame::World);
        let (_, _, da_local) =
            data.joint_acceleration_derivatives(&model, 4, ReferenceFrame::Local);
        let (_, _, da_lwa) = data.joint_acceleration_derivatives(
            &model,
            4,
            ReferenceFrame::LocalWorldAligned,
        );
        let diff_wl = (&da_world - &da_local).iter().map(|x| x.abs()).fold(0.0, f64::max);
        let diff_ll = (&da_local - &da_lwa).iter().map(|x| x.abs()).fold(0.0, f64::max);
        assert!(diff_wl > 1e-6, "World and Local ∂a/∂a must differ at a non-root joint");
        assert!(diff_ll > 1e-6, "Local and LocalWorldAligned ∂a/∂a must differ at a non-root joint");
    }

    // ============================================================
    // §9 — Frame-velocity derivatives
    // ============================================================

    /// UT.9 — shape + frame dependence on `panda_hand`.
    #[test]
    fn frame_vel_derivs_shape_and_frame_dependence() {
        let (model, mut data) = panda();
        let nv = model.nv();
        let q = model.random_configuration(67);
        let v = small_random_vec(nv, 71);
        let a = small_random_vec(nv, 73);
        data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)
            .unwrap();
        let frame_id = model.frame_id("panda_hand").unwrap();
        let (dq_w, _) = data.frame_velocity_derivatives(&model, frame_id, ReferenceFrame::World);
        let (dq_l, _) = data.frame_velocity_derivatives(&model, frame_id, ReferenceFrame::Local);
        let (dq_lwa, _) =
            data.frame_velocity_derivatives(&model, frame_id, ReferenceFrame::LocalWorldAligned);
        assert_eq!(dq_w.shape(), (6, nv));
        assert_eq!(dq_l.shape(), (6, nv));
        assert_eq!(dq_lwa.shape(), (6, nv));
        let diff_wl = (&dq_w - &dq_l).iter().map(|x| x.abs()).fold(0.0, f64::max);
        let diff_ll = (&dq_l - &dq_lwa).iter().map(|x| x.abs()).fold(0.0, f64::max);
        assert!(diff_wl > 1e-6, "World and Local ∂v_frame/∂q must differ on panda_hand");
        assert!(diff_ll > 1e-6, "Local and LocalWorldAligned ∂v_frame/∂q must differ on panda_hand");
    }

    // ============================================================
    // §10 — Frame-acceleration derivatives
    // ============================================================

    /// UT.10 — shape.
    #[test]
    fn frame_accel_derivs_shape() {
        let (model, mut data) = panda();
        let nv = model.nv();
        let q = model.random_configuration(79);
        let v = small_random_vec(nv, 83);
        let a = small_random_vec(nv, 89);
        data.compute_forward_kinematics_derivatives(&model, &q, &v, &a)
            .unwrap();
        let frame_id = model.frame_id("panda_hand").unwrap();
        let (dq, dv, da) =
            data.frame_acceleration_derivatives(&model, frame_id, ReferenceFrame::Local);
        assert_eq!(dq.shape(), (6, nv));
        assert_eq!(dv.shape(), (6, nv));
        assert_eq!(da.shape(), (6, nv));
        for x in dq.iter().chain(dv.iter()).chain(da.iter()) {
            assert!(x.is_finite());
        }
    }

    // ============================================================
    // helpers
    // ============================================================

    fn small_random_vec(n: usize, seed: u64) -> DVector<f64> {
        // Deterministic LCG — keeps tests free of dev-only RNG crates.
        let mut state = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
        let mut v = vec![0.0f64; n];
        for x in &mut v {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let bits = (state >> 11) as f64 / (1u64 << 53) as f64;
            *x = (bits - 0.5) * 0.2;
        }
        DVector::from_vec(v)
    }
}
