// SPDX-License-Identifier: BSD-2-Clause
//! `ReferenceFrame` — the frame in which a spatial quantity is expressed.
//!
//! Mirrors `pinocchio::ReferenceFrame`. Discriminant values are pinned so the
//! C++ shim can take a `u8` and round-trip through `pinocchio::ReferenceFrame`
//! without ambiguity.

/// Frame in which Pinocchio expresses a spatial quantity (Jacobian column,
/// velocity, acceleration, …).
///
/// | variant            | discriminant | Pinocchio constant       |
/// |--------------------|--------------|--------------------------|
/// | `World`            | 0            | `pinocchio::WORLD`               |
/// | `Local`            | 1            | `pinocchio::LOCAL`               |
/// | `LocalWorldAligned`| 2            | `pinocchio::LOCAL_WORLD_ALIGNED` |
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReferenceFrame {
    World = 0,
    Local = 1,
    LocalWorldAligned = 2,
}

impl ReferenceFrame {
    pub(crate) fn to_u8(self) -> u8 {
        self as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// UT.9b — three variants, stable discriminants.
    #[test]
    fn discriminants_match_pinocchio_enum() {
        assert_eq!(ReferenceFrame::World.to_u8(), 0);
        assert_eq!(ReferenceFrame::Local.to_u8(), 1);
        assert_eq!(ReferenceFrame::LocalWorldAligned.to_u8(), 2);
    }
}
