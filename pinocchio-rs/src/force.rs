// SPDX-License-Identifier: BSD-2-Clause
//! `Force` — a spatial wrench (linear force + torque) in SE(3).
//!
//! Stored as `[linear:3, angular:3]` matching Pinocchio's `pinocchio::Force`
//! convention. `Force` and [`Motion`](crate::Motion) are deliberately distinct
//! types; mixing them is rejected at compile time (see the trybuild test).

use nalgebra::{Vector3, Vector6};

/// A spatial wrench (linear force + angular torque).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Force(pub(crate) Vector6<f64>);

impl Force {
    /// Zero wrench.
    pub fn zero() -> Self {
        Self(Vector6::zeros())
    }

    /// Build from separate linear-force and torque 3-vectors.
    pub fn new(linear: Vector3<f64>, angular: Vector3<f64>) -> Self {
        let mut v = Vector6::zeros();
        v.fixed_view_mut::<3, 1>(0, 0).copy_from(&linear);
        v.fixed_view_mut::<3, 1>(3, 0).copy_from(&angular);
        Self(v)
    }

    /// Build from a 6-vector stored as `[linear:3, angular:3]`.
    pub fn from_vector(v: Vector6<f64>) -> Self {
        Self(v)
    }

    /// 6-vector view stored as `[linear:3, angular:3]`.
    pub fn as_vector(&self) -> Vector6<f64> {
        self.0
    }

    /// Linear-force part (first three components).
    pub fn linear(&self) -> Vector3<f64> {
        Vector3::new(self.0[0], self.0[1], self.0[2])
    }

    /// Torque part (last three components).
    pub fn angular(&self) -> Vector3<f64> {
        Vector3::new(self.0[3], self.0[4], self.0[5])
    }
}

impl std::ops::Add for Force {
    type Output = Force;
    fn add(self, rhs: Force) -> Force {
        Force(self.0 + rhs.0)
    }
}

impl std::ops::Sub for Force {
    type Output = Force;
    fn sub(self, rhs: Force) -> Force {
        Force(self.0 - rhs.0)
    }
}

impl std::ops::Mul<f64> for Force {
    type Output = Force;
    fn mul(self, rhs: f64) -> Force {
        Force(self.0 * rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// UT.4b — round-trip and arithmetic.
    #[test]
    fn linear_angular_round_trip() {
        let lin = Vector3::new(7.0, 8.0, 9.0);
        let ang = Vector3::new(-1.0, -2.0, -3.0);
        let f = Force::new(lin, ang);
        assert_relative_eq!(f.linear(), lin);
        assert_relative_eq!(f.angular(), ang);
    }

    #[test]
    fn vector6_round_trip() {
        let v = Vector6::from_column_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let f = Force::from_vector(v);
        assert_relative_eq!(f.as_vector(), v);
    }

    #[test]
    fn force_plus_force_works() {
        let a = Force::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        let b = Force::new(Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let c = a + b;
        assert_relative_eq!(c.linear(), Vector3::new(1.0, 1.0, 0.0));
        assert_relative_eq!(c.angular(), Vector3::new(1.0, 1.0, 0.0));
    }
}
