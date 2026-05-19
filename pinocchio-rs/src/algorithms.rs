// SPDX-License-Identifier: BSD-2-Clause
//! Forward kinematics, RNEA, ABA, CRBA, and Jacobians.
//!
//! Algorithm wrappers are exposed as methods on [`Data`]. Each method writes
//! into `Data`'s internal buffers; the result is read back with a follow-up
//! accessor (`joint_placement`, `tau`, `ddq`, `mass_matrix`, …).

use nalgebra::{DMatrix, DVector, Matrix4, Vector6};

use crate::{Data, Error, Model, Motion, ReferenceFrame, Result, SE3};

impl Data {
    // ============================================================
    // Forward kinematics
    // ============================================================

    /// Compute joint placements only (no velocity, no acceleration).
    ///
    /// `q` must have length `model.nq()`.
    pub fn forward_kinematics(&mut self, model: &Model, q: &DVector<f64>) -> Result<()> {
        check_len("q", q.len(), model.nq())?;
        // SAFETY: q slice is exactly nq doubles; shim reads it as Eigen::Map.
        unsafe {
            pinocchio_sys::ffi::forward_kinematics(
                model.raw(),
                self.inner.pin_mut(),
                q.as_ptr(),
                model.nq(),
            );
        }
        Ok(())
    }

    /// FK with velocity. `q.len() == model.nq()`, `v.len() == model.nv()`.
    pub fn forward_kinematics_v(
        &mut self,
        model: &Model,
        q: &DVector<f64>,
        v: &DVector<f64>,
    ) -> Result<()> {
        check_len("q", q.len(), model.nq())?;
        check_len("v", v.len(), model.nv())?;
        unsafe {
            pinocchio_sys::ffi::forward_kinematics_v(
                model.raw(),
                self.inner.pin_mut(),
                q.as_ptr(),
                model.nq(),
                v.as_ptr(),
                model.nv(),
            );
        }
        Ok(())
    }

    /// FK with velocity and acceleration.
    pub fn forward_kinematics_v_a(
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
            pinocchio_sys::ffi::forward_kinematics_v_a(
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

    /// Pinocchio caches frame placements lazily — call this before reading any
    /// `frame_placement(...)` after a fresh `forward_kinematics`.
    pub fn update_frame_placements(&mut self, model: &Model) {
        pinocchio_sys::ffi::update_frame_placements(model.raw(), self.inner.pin_mut());
    }

    /// Joint placement `oMi[joint_id]` after a successful FK call.
    pub fn joint_placement(&self, joint_id: usize) -> SE3 {
        let mut out = [0.0f64; 16];
        // SAFETY: out is 16 doubles.
        unsafe {
            pinocchio_sys::ffi::data_joint_placement(self._raw(), joint_id, out.as_mut_ptr());
        }
        SE3::from_homogeneous(Matrix4::from_column_slice(&out))
    }

    /// Frame placement `oMf[frame_id]` after `update_frame_placements`.
    pub fn frame_placement(&self, frame_id: usize) -> SE3 {
        let mut out = [0.0f64; 16];
        unsafe {
            pinocchio_sys::ffi::data_frame_placement(self._raw(), frame_id, out.as_mut_ptr());
        }
        SE3::from_homogeneous(Matrix4::from_column_slice(&out))
    }

    /// Joint spatial velocity `v[joint_id]` after `forward_kinematics_v`.
    pub fn joint_velocity(&self, joint_id: usize) -> Motion {
        let mut out = [0.0f64; 6];
        unsafe {
            pinocchio_sys::ffi::data_joint_velocity(self._raw(), joint_id, out.as_mut_ptr());
        }
        Motion::from_vector(Vector6::from_column_slice(&out))
    }

    // ============================================================
    // RNEA (inverse dynamics)
    // ============================================================

    /// τ = RNEA(model, q, v, a). Returns an owned `DVector<f64>` of length `nv`.
    pub fn rnea(
        &mut self,
        model: &Model,
        q: &DVector<f64>,
        v: &DVector<f64>,
        a: &DVector<f64>,
    ) -> Result<DVector<f64>> {
        check_len("q", q.len(), model.nq())?;
        check_len("v", v.len(), model.nv())?;
        check_len("a", a.len(), model.nv())?;
        unsafe {
            pinocchio_sys::ffi::rnea(
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
        let mut tau = vec![0.0; model.nv()];
        unsafe {
            pinocchio_sys::ffi::data_tau(self._raw(), tau.as_mut_ptr(), model.nv());
        }
        Ok(DVector::from_vec(tau))
    }

    // ============================================================
    // ABA (forward dynamics)
    // ============================================================

    /// q̈ = ABA(model, q, v, τ). Returns an owned `DVector<f64>` of length `nv`.
    pub fn aba(
        &mut self,
        model: &Model,
        q: &DVector<f64>,
        v: &DVector<f64>,
        tau: &DVector<f64>,
    ) -> Result<DVector<f64>> {
        check_len("q", q.len(), model.nq())?;
        check_len("v", v.len(), model.nv())?;
        check_len("tau", tau.len(), model.nv())?;
        unsafe {
            pinocchio_sys::ffi::aba(
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
        let mut ddq = vec![0.0; model.nv()];
        unsafe {
            pinocchio_sys::ffi::data_ddq(self._raw(), ddq.as_mut_ptr(), model.nv());
        }
        Ok(DVector::from_vec(ddq))
    }

    // ============================================================
    // CRBA (mass matrix)
    // ============================================================

    /// Compute the joint-space mass matrix and symmetrize it.
    ///
    /// After this call, [`Data::mass_matrix`] returns the full symmetric `M`.
    pub fn crba(&mut self, model: &Model, q: &DVector<f64>) -> Result<()> {
        check_len("q", q.len(), model.nq())?;
        unsafe {
            pinocchio_sys::ffi::crba(
                model.raw(),
                self.inner.pin_mut(),
                q.as_ptr(),
                model.nq(),
            );
        }
        // Pinocchio's crba leaves the lower triangle uninitialized — we
        // symmetrize so callers see a full SPD matrix.
        pinocchio_sys::ffi::data_mass_matrix_symmetrize(self.inner.pin_mut());
        Ok(())
    }

    /// Read the (symmetrized) mass matrix as an owned `DMatrix<f64>`.
    ///
    /// We deliberately return an owned copy rather than a view: the borrow-
    /// checker rules for "view into Pin<&mut Data> while &mut self is alive"
    /// are awkward for the typical use case (Cholesky on M, then keep mutating
    /// `Data`). UT.8b enforces the view-vs-mutation pattern at the FFI layer.
    pub fn mass_matrix(&self, model: &Model) -> DMatrix<f64> {
        let nv = model.nv();
        let mut buf = vec![0.0; nv * nv];
        unsafe {
            pinocchio_sys::ffi::data_mass_matrix_copy(self._raw(), buf.as_mut_ptr(), nv);
        }
        DMatrix::from_vec(nv, nv, buf)
    }

    // ============================================================
    // Jacobians
    // ============================================================

    /// Joint Jacobian (6×nv) in the joint's local frame.
    pub fn joint_jacobian(
        &mut self,
        model: &Model,
        q: &DVector<f64>,
        joint_id: usize,
    ) -> Result<DMatrix<f64>> {
        check_len("q", q.len(), model.nq())?;
        let nv = model.nv();
        let mut buf = vec![0.0; 6 * nv];
        unsafe {
            pinocchio_sys::ffi::compute_joint_jacobian(
                model.raw(),
                self.inner.pin_mut(),
                q.as_ptr(),
                model.nq(),
                joint_id,
                buf.as_mut_ptr(),
                nv,
            );
        }
        Ok(DMatrix::from_vec(6, nv, buf))
    }

    /// Frame Jacobian (6×nv) in the requested reference frame.
    pub fn frame_jacobian(
        &mut self,
        model: &Model,
        q: &DVector<f64>,
        frame_id: usize,
        rf: ReferenceFrame,
    ) -> Result<DMatrix<f64>> {
        check_len("q", q.len(), model.nq())?;
        let nv = model.nv();
        let mut buf = vec![0.0; 6 * nv];
        unsafe {
            pinocchio_sys::ffi::compute_frame_jacobian(
                model.raw(),
                self.inner.pin_mut(),
                q.as_ptr(),
                model.nq(),
                frame_id,
                rf.to_u8(),
                buf.as_mut_ptr(),
                nv,
            );
        }
        Ok(DMatrix::from_vec(6, nv, buf))
    }
}

impl Model {
    /// Neutral configuration vector (length `nq`).
    pub fn neutral_configuration(&self) -> DVector<f64> {
        let nq = self.nq();
        let mut buf = vec![0.0; nq];
        unsafe {
            pinocchio_sys::ffi::model_neutral_configuration(self.raw(), buf.as_mut_ptr(), nq);
        }
        DVector::from_vec(buf)
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
    use nalgebra::Matrix3;

    fn panda() -> (Model, Data) {
        let model = Model::from_urdf(panda_urdf_path()).unwrap();
        let data = Data::new(&model);
        (model, data)
    }

    // ---- §5 forward kinematics ----

    /// UT.5 — FK on neutral config yields valid placements and zero velocities.
    #[test]
    fn fk_neutral_produces_valid_rotations() {
        let (model, mut data) = panda();
        let q = model.neutral_configuration();
        let v = DVector::zeros(model.nv());

        data.forward_kinematics_v(&model, &q, &v).unwrap();

        for joint_id in 0..model.n_joints() {
            let m = data.joint_placement(joint_id).to_homogeneous();
            // Rotation block must have det = 1 (within 1e-10) → valid SO(3).
            let r = m.fixed_view::<3, 3>(0, 0).into_owned();
            let det = r.determinant();
            assert_relative_eq!(det, 1.0, epsilon = 1e-10);
            // Translation must be finite.
            assert!(m[(0, 3)].is_finite() && m[(1, 3)].is_finite() && m[(2, 3)].is_finite());
        }

        // Joint velocities must be zero when v = 0.
        for joint_id in 1..model.n_joints() {
            let v_i = data.joint_velocity(joint_id);
            assert_relative_eq!(v_i.linear(), nalgebra::Vector3::zeros(), epsilon = 1e-12);
            assert_relative_eq!(v_i.angular(), nalgebra::Vector3::zeros(), epsilon = 1e-12);
        }
    }

    // ---- §6 RNEA ----

    /// UT.6 — RNEA returns nv-length output, is non-zero under gravity, and
    /// is deterministic.
    #[test]
    fn rnea_shape_and_determinism() {
        let (model, mut data) = panda();
        let q = model.neutral_configuration();
        let zero = DVector::zeros(model.nv());

        let tau1 = data.rnea(&model, &q, &zero, &zero).unwrap();
        let tau2 = data.rnea(&model, &q, &zero, &zero).unwrap();

        assert_eq!(tau1.len(), model.nv());
        // Gravity compensation must be non-zero (with default gravity).
        let max_abs = tau1.iter().copied().map(f64::abs).fold(0.0, f64::max);
        assert!(max_abs > 1e-6, "expected non-zero gravity torques, got {tau1:?}");
        // Two identical calls produce identical results.
        assert_relative_eq!(tau1, tau2, epsilon = 1e-15);
    }

    // ---- §7 ABA ----

    /// UT.7 — ABA-of-RNEA round-trip recovers `a` within 1e-10 for 10 random
    /// (q, v, a) triples. Catches sign/transpose bugs in the FFI.
    #[test]
    fn aba_of_rnea_roundtrip() {
        let (model, mut data) = panda();
        let nv = model.nv();

        for seed in 0..10u64 {
            // q from random_configuration (respects joint limits), v + a are
            // small uniform vectors (large values can stress the integrator).
            let q = model.random_configuration(seed);
            let v = small_random_vec(nv, seed.wrapping_mul(7) + 1);
            let a = small_random_vec(nv, seed.wrapping_mul(11) + 3);

            let tau = data.rnea(&model, &q, &v, &a).unwrap();
            let a_back = data.aba(&model, &q, &v, &tau).unwrap();
            assert_relative_eq!(a, a_back, epsilon = 1e-10);
        }
    }

    // ---- §8 CRBA ----

    /// UT.8a — mass matrix is `(nv, nv)`, symmetric, positive-definite.
    #[test]
    fn crba_yields_symmetric_pd_matrix() {
        let (model, mut data) = panda();
        let nv = model.nv();
        for seed in 0..10u64 {
            let q = model.random_configuration(seed.wrapping_mul(13) + 1);
            data.crba(&model, &q).unwrap();
            let m = data.mass_matrix(&model);
            assert_eq!(m.shape(), (nv, nv));
            // Symmetry within 1e-12.
            for i in 0..nv {
                for j in 0..nv {
                    assert_relative_eq!(m[(i, j)], m[(j, i)], epsilon = 1e-12);
                }
            }
            // Positive definite via Cholesky.
            let chol = m.clone().cholesky();
            assert!(chol.is_some(), "mass matrix not PD for seed={seed}: {m:?}");
        }
    }

    // ---- §9 Jacobians ----

    /// UT.9a — Jacobian shapes; Local vs World differ; root joint = zero.
    #[test]
    fn jacobian_shape_and_frames() {
        let (model, mut data) = panda();
        let nv = model.nv();
        let q = model.neutral_configuration();

        let j_root = data.joint_jacobian(&model, &q, 0).unwrap();
        assert_eq!(j_root.shape(), (6, nv));
        // The universe joint's Jacobian is identically zero.
        for x in j_root.iter() {
            assert_relative_eq!(*x, 0.0, epsilon = 1e-12);
        }

        // Frame Jacobian on the end-effector frame: Local and World differ
        // because the end-effector is offset from the world origin.
        let ee_frame = model.frame_id("panda_hand").unwrap();
        let j_local = data
            .frame_jacobian(&model, &q, ee_frame, ReferenceFrame::Local)
            .unwrap();
        let j_world = data
            .frame_jacobian(&model, &q, ee_frame, ReferenceFrame::World)
            .unwrap();
        let j_lwa = data
            .frame_jacobian(&model, &q, ee_frame, ReferenceFrame::LocalWorldAligned)
            .unwrap();
        assert_eq!(j_local.shape(), (6, nv));
        assert_eq!(j_world.shape(), (6, nv));
        assert_eq!(j_lwa.shape(), (6, nv));
        let diff_lw = (&j_local - &j_world).iter().copied().map(f64::abs).fold(0.0, f64::max);
        assert!(diff_lw > 1e-6, "Local and World Jacobians must differ for a non-origin frame");
    }

    /// Length-mismatch must surface as an error (not panic).
    #[test]
    fn rnea_wrong_lengths_errors() {
        let (model, mut data) = panda();
        let q = DVector::zeros(model.nq() + 1);
        let v = DVector::zeros(model.nv());
        let a = DVector::zeros(model.nv());
        let err = data.rnea(&model, &q, &v, &a).unwrap_err();
        assert!(matches!(err, Error::OutOfRange(_)));
    }

    // ---- helpers ----

    fn small_random_vec(n: usize, seed: u64) -> DVector<f64> {
        // Deterministic LCG — keeps tests free of dev-only RNG crates.
        let mut state = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
        let mut v = vec![0.0f64; n];
        for x in &mut v {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let bits = (state >> 11) as f64 / (1u64 << 53) as f64; // [0, 1)
            *x = (bits - 0.5) * 0.2; // small range to keep dynamics tame
        }
        let _unused: Matrix3<f64> = Matrix3::identity();
        DVector::from_vec(v)
    }
}
