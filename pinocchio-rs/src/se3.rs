// SPDX-License-Identifier: BSD-2-Clause
//! `SE3` — a rigid body transform in 3D.
//!
//! ### Conventions
//!
//! - Internally stored as a [`nalgebra::Isometry3<f64>`]; the underlying
//!   quaternion uses storage order `(i, j, k, w)` = `(x, y, z, w)`, matching
//!   Pinocchio's convention. UT.4a asserts this directly against the raw buffer.
//! - Convention-sensitive ops (`log6`, `Jlog6`) call through to Pinocchio's
//!   C++ implementation to guarantee an exact match.
//! - Convention-insensitive ops (`compose`, `inverse`, `to_homogeneous`) are
//!   implemented in pure Rust via nalgebra; §14 parity tests validate them
//!   against the C++ reference.

use nalgebra::{Isometry3, Matrix4, Matrix6, Quaternion, Translation3, UnitQuaternion, Vector3, Vector6};
use pinocchio_sys::ffi;

/// A rigid-body transform in SE(3).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SE3(pub(crate) Isometry3<f64>);

impl SE3 {
    /// The identity transform.
    pub fn identity() -> Self {
        Self(Isometry3::identity())
    }

    /// Build from a rotation matrix and a translation vector.
    pub fn from_rotation_translation(rotation: nalgebra::Rotation3<f64>, translation: Vector3<f64>) -> Self {
        let q = UnitQuaternion::from_rotation_matrix(&rotation);
        Self(Isometry3::from_parts(Translation3::from(translation), q))
    }

    /// Build from a unit quaternion (rotation) and a translation vector.
    pub fn from_unit_quaternion_translation(q: UnitQuaternion<f64>, translation: Vector3<f64>) -> Self {
        Self(Isometry3::from_parts(Translation3::from(translation), q))
    }

    /// Build from a 4×4 homogeneous matrix. The bottom row is ignored
    /// (asserted to be `[0, 0, 0, 1]` in debug).
    pub fn from_homogeneous(m: Matrix4<f64>) -> Self {
        debug_assert!(
            (m[(3, 0)].abs() < 1e-10)
                && (m[(3, 1)].abs() < 1e-10)
                && (m[(3, 2)].abs() < 1e-10)
                && ((m[(3, 3)] - 1.0).abs() < 1e-10),
            "SE3::from_homogeneous: bottom row must be [0,0,0,1], got {:?}",
            m.row(3),
        );
        let r = m.fixed_view::<3, 3>(0, 0).into_owned();
        let t = Vector3::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]);
        let rotation = nalgebra::Rotation3::from_matrix_unchecked(r);
        Self::from_rotation_translation(rotation, t)
    }

    /// 4×4 homogeneous matrix representation.
    pub fn to_homogeneous(&self) -> Matrix4<f64> {
        self.0.to_homogeneous()
    }

    /// `M^{-1}`.
    pub fn inverse(&self) -> Self {
        Self(self.0.inverse())
    }

    /// Apply to a 3D point: `M * p`.
    pub fn act_point(&self, p: Vector3<f64>) -> Vector3<f64> {
        self.0 * p
    }

    /// Rotation as a unit quaternion (storage order `(x, y, z, w)`).
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        self.0.rotation
    }

    /// Translation vector.
    pub fn translation(&self) -> Vector3<f64> {
        self.0.translation.vector
    }

    /// `log6(M)` — the SE(3) logarithm, returned as a 6-vector
    /// `(linear:3, angular:3)` matching Pinocchio's convention.
    pub fn log6(&self) -> Vector6<f64> {
        let m = self.to_homogeneous();
        let mut out = [0.0f64; 6];
        // SAFETY: cxx-generated function; both buffers have the documented
        // sizes (16 and 6 doubles respectively).
        unsafe {
            ffi::se3_log6(m.as_ptr(), out.as_mut_ptr());
        }
        Vector6::from_column_slice(&out)
    }

    /// `Jlog6(M)` — the 6×6 Jacobian of `log6` at `M`, returned as a 6×6 matrix.
    pub fn jlog6(&self) -> Matrix6<f64> {
        let m = self.to_homogeneous();
        let mut out = [0.0f64; 36];
        // SAFETY: cxx-generated function; buffers sized 16 and 36 doubles.
        unsafe {
            ffi::se3_jlog6(m.as_ptr(), out.as_mut_ptr());
        }
        Matrix6::from_column_slice(&out)
    }

    /// Raw quaternion coefficients in storage order `(x, y, z, w)`.
    ///
    /// Exposed primarily for parity tests and convention verification — most
    /// callers should use [`Self::rotation`] instead.
    pub fn quaternion_xyzw(&self) -> [f64; 4] {
        let q: &Quaternion<f64> = self.0.rotation.quaternion();
        // nalgebra::Quaternion's coords field is a Vector4 stored as (i, j, k, w).
        [q.coords.x, q.coords.y, q.coords.z, q.coords.w]
    }
}

impl std::ops::Mul for SE3 {
    type Output = SE3;
    fn mul(self, rhs: SE3) -> SE3 {
        SE3(self.0 * rhs.0)
    }
}

impl std::ops::Mul<&SE3> for &SE3 {
    type Output = SE3;
    fn mul(self, rhs: &SE3) -> SE3 {
        SE3(self.0 * rhs.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Rotation3;

    fn nontrivial() -> SE3 {
        let rot = Rotation3::from_euler_angles(0.3, -0.7, 1.1);
        SE3::from_rotation_translation(rot, Vector3::new(0.1, -0.2, 0.3))
    }

    /// UT.4a — identity laws and round-trips.
    #[test]
    fn identity_is_left_and_right_identity() {
        let t = nontrivial();
        let id = SE3::identity();
        assert_eq!(id * t, t, "identity * t == t");
        assert_eq!(t * id, t, "t * identity == t");
    }

    #[test]
    fn inverse_compose_yields_identity() {
        let t = nontrivial();
        let r = t.inverse() * t;
        let id = SE3::identity().to_homogeneous();
        assert_relative_eq!(r.to_homogeneous(), id, epsilon = 1e-12);
    }

    #[test]
    fn identity_log6_is_zero() {
        let v = SE3::identity().log6();
        assert_relative_eq!(v, Vector6::zeros(), epsilon = 1e-12);
    }

    #[test]
    fn from_homogeneous_round_trip() {
        let t = nontrivial();
        let r = SE3::from_homogeneous(t.to_homogeneous());
        assert_relative_eq!(r.to_homogeneous(), t.to_homogeneous(), epsilon = 1e-12);
    }

    #[test]
    fn quaternion_storage_is_xyzw() {
        // Build a rotation whose components are individually distinguishable
        // so we can pin down the storage order. Use a yaw of 0.3 rad about Z.
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3);
        let t = SE3::from_unit_quaternion_translation(q, Vector3::zeros());
        let [x, y, z, w] = t.quaternion_xyzw();
        // For yaw-only: x = y = 0, z = sin(angle/2), w = cos(angle/2).
        assert_relative_eq!(x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(z, (0.3_f64 / 2.0).sin(), epsilon = 1e-12);
        assert_relative_eq!(w, (0.3_f64 / 2.0).cos(), epsilon = 1e-12);
    }

    #[test]
    fn jlog6_is_6x6_finite() {
        let j = nontrivial().jlog6();
        assert_eq!(j.shape(), (6, 6));
        assert!(j.iter().all(|x| x.is_finite()), "Jlog6 contains non-finite entries");
    }
}
