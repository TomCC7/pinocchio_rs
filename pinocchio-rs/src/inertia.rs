// SPDX-License-Identifier: BSD-2-Clause
//! `Inertia` — a rigid body's spatial inertia tensor.
//!
//! Stored as the 6×6 spatial-inertia matrix used by Pinocchio. The conversion
//! from `(mass, com, I_C)` follows the standard parallel-axis identity:
//!
//! ```text
//!         [ m·I₃       -m·[c]× ]
//!   I_O = [                    ]
//!         [ m·[c]×    I_C + m·[c]×ᵀ[c]× ]
//! ```
//!
//! where `[c]×` is the skew-symmetric cross-product matrix of the center-of-
//! mass offset `c`, and `I_C` is the 3×3 inertia about the COM.

use nalgebra::{Matrix3, Matrix6, Vector3};

use crate::se3::SE3;

/// A spatial inertia tensor (6×6).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertia(pub(crate) Matrix6<f64>);

impl Inertia {
    /// Build from mass, center-of-mass offset, and the 3×3 rotational
    /// inertia tensor expressed at the COM.
    pub fn from_mass_com_inertia(mass: f64, com: Vector3<f64>, inertia_com: Matrix3<f64>) -> Self {
        let cx = skew(com);
        let mut m6 = Matrix6::zeros();
        // top-left: m * I₃
        m6.fixed_view_mut::<3, 3>(0, 0).copy_from(&(Matrix3::identity() * mass));
        // top-right: -m * [c]×
        m6.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-cx * mass));
        // bottom-left: m * [c]×
        m6.fixed_view_mut::<3, 3>(3, 0).copy_from(&(cx * mass));
        // bottom-right: I_C + m * [c]×ᵀ * [c]× = I_C - m * [c]× * [c]×
        let cx_cx = cx * cx;
        m6.fixed_view_mut::<3, 3>(3, 3).copy_from(&(inertia_com - cx_cx * mass));
        Self(m6)
    }

    /// 6×6 matrix view.
    pub fn to_matrix6(&self) -> Matrix6<f64> {
        self.0
    }

    /// Build directly from a 6×6 matrix (caller asserts spatial-inertia structure).
    pub fn from_matrix6(m: Matrix6<f64>) -> Self {
        Self(m)
    }

    /// Apply an SE(3) transform: `I' = X · I · Xᵀ` in spatial form.
    ///
    /// This implementation delegates to the standard adjoint formula. Pinocchio
    /// parity is validated in §14.
    pub fn transformed_by(&self, x: &SE3) -> Self {
        let adj = adjoint(x);
        Self(adj * self.0 * adj.transpose())
    }
}

fn skew(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y,
        v.z, 0.0, -v.x,
        -v.y, v.x, 0.0,
    )
}

/// 6×6 adjoint matrix of an SE(3) transform, in `[linear; angular]` block layout
/// matching Pinocchio's spatial-vector convention.
fn adjoint(x: &SE3) -> Matrix6<f64> {
    let r = x.rotation().to_rotation_matrix();
    let r_mat: Matrix3<f64> = r.into_inner();
    let t = x.translation();
    let t_skew = skew(t);
    let mut a = Matrix6::zeros();
    a.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_mat);
    a.fixed_view_mut::<3, 3>(0, 3).copy_from(&(t_skew * r_mat));
    a.fixed_view_mut::<3, 3>(3, 3).copy_from(&r_mat);
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// UT.4c — round-trip and SE3-identity invariance.
    #[test]
    fn from_to_matrix6_round_trip() {
        let i = Inertia::from_mass_com_inertia(
            2.5,
            Vector3::new(0.1, -0.05, 0.2),
            Matrix3::from_diagonal(&Vector3::new(0.01, 0.02, 0.015)),
        );
        let back = Inertia::from_matrix6(i.to_matrix6());
        assert_relative_eq!(back.to_matrix6(), i.to_matrix6(), epsilon = 1e-10);
    }

    #[test]
    fn identity_act_is_identity() {
        let i = Inertia::from_mass_com_inertia(
            1.2,
            Vector3::new(0.3, 0.0, 0.0),
            Matrix3::identity() * 0.005,
        );
        let result = i.transformed_by(&SE3::identity());
        assert_relative_eq!(result.to_matrix6(), i.to_matrix6(), epsilon = 1e-12);
    }
}
