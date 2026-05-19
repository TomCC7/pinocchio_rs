// SPDX-License-Identifier: BSD-2-Clause
//! Safe wrapper around `pinocchio::Model`.
//!
//! A `Model` owns Pinocchio's serialized kinematic + inertial tree. It is
//! immutable after construction — all dynamics state lives in [`Data`](crate::Data).
//!
//! ### Conventions
//!
//! - `nq` is the configuration-space dimension; `nv` is the tangent-space
//!   dimension. They agree for fixed-base robots whose joints are all
//!   revolute/prismatic, and differ on free-flyer / spherical joints.
//! - The implicit *universe* joint occupies index `0`; user joints start at
//!   index `1`. `njoints` includes the universe joint.

use cxx::UniquePtr;
use pinocchio_sys::ffi;

use crate::error::{Error, Result};

/// An immutable Pinocchio kinematic + inertial model.
pub struct Model {
    pub(crate) inner: UniquePtr<ffi::Model>,
}

impl Model {
    /// Load a URDF from disk into a fixed-base [`Model`].
    pub fn from_urdf(path: impl AsRef<std::path::Path>) -> Result<Self> {
        let p = path.as_ref().to_str().ok_or_else(|| {
            Error::UrdfParse(format!("path is not valid UTF-8: {:?}", path.as_ref()))
        })?;
        let inner = ffi::build_model_from_urdf(p).map_err(map_urdf_err)?;
        Ok(Self { inner })
    }

    /// Load a URDF from disk and prepend a free-flyer (6-DoF) root joint.
    pub fn from_urdf_with_free_flyer(path: impl AsRef<std::path::Path>) -> Result<Self> {
        let p = path.as_ref().to_str().ok_or_else(|| {
            Error::UrdfParse(format!("path is not valid UTF-8: {:?}", path.as_ref()))
        })?;
        let inner = ffi::build_model_from_urdf_with_free_flyer(p).map_err(map_urdf_err)?;
        Ok(Self { inner })
    }

    /// Load a URDF whose XML is already in memory (handy for tests).
    pub fn from_urdf_str(xml: &str) -> Result<Self> {
        let inner = ffi::build_model_from_urdf_string(xml).map_err(map_urdf_err)?;
        Ok(Self { inner })
    }

    /// Variant of [`Self::from_urdf_str`] with a free-flyer root joint.
    pub fn from_urdf_str_with_free_flyer(xml: &str) -> Result<Self> {
        let inner = ffi::build_model_from_urdf_string_with_free_flyer(xml).map_err(map_urdf_err)?;
        Ok(Self { inner })
    }

    pub(crate) fn raw(&self) -> &ffi::Model {
        self.inner.as_ref().expect("Model UniquePtr is never null after construction")
    }

    /// Configuration-space dimension.
    pub fn nq(&self) -> usize {
        ffi::model_nq(self.raw())
    }

    /// Tangent-space dimension.
    pub fn nv(&self) -> usize {
        ffi::model_nv(self.raw())
    }

    /// Number of joints (including the universe joint at index 0).
    pub fn n_joints(&self) -> usize {
        ffi::model_njoints(self.raw())
    }

    /// Number of frames (joint frames + URDF link frames + user-added frames).
    pub fn n_frames(&self) -> usize {
        ffi::model_nframes(self.raw())
    }

    /// Look up a joint's index by URDF name.
    pub fn joint_id(&self, name: &str) -> Result<usize> {
        let id = ffi::model_joint_id_by_name(self.raw(), name);
        if id >= self.n_joints() {
            Err(Error::UnknownName(format!("joint '{name}'")))
        } else {
            Ok(id)
        }
    }

    /// Look up a frame's index by URDF name.
    pub fn frame_id(&self, name: &str) -> Result<usize> {
        let id = ffi::model_frame_id_by_name(self.raw(), name);
        if id >= self.n_frames() {
            Err(Error::UnknownName(format!("frame '{name}'")))
        } else {
            Ok(id)
        }
    }

    /// Look up a joint's URDF name by index.
    pub fn joint_name(&self, joint_id: usize) -> Result<String> {
        ffi::model_joint_name(self.raw(), joint_id).map_err(|e| {
            Error::OutOfRange(format!("joint_id {joint_id}: {}", e.what()))
        })
    }

    /// Does the model contain a joint with this name?
    pub fn has_joint(&self, name: &str) -> bool {
        ffi::model_has_joint(self.raw(), name)
    }

    /// Does the model contain a frame with this name?
    pub fn has_frame(&self, name: &str) -> bool {
        ffi::model_has_frame(self.raw(), name)
    }
}

impl std::fmt::Debug for Model {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Model")
            .field("nq", &self.nq())
            .field("nv", &self.nv())
            .field("njoints", &self.n_joints())
            .field("nframes", &self.n_frames())
            .finish()
    }
}

fn map_urdf_err(e: cxx::Exception) -> Error {
    Error::UrdfParse(e.what().to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::test_support::panda_urdf_path;

    /// UT.3b — Panda URDF round-trip.
    #[test]
    fn panda_loads_with_expected_dimensions() {
        let model = Model::from_urdf(panda_urdf_path()).expect("panda urdf loads");
        assert_eq!(model.nq(), 7, "fixed-base 7R Panda → nq = 7");
        assert_eq!(model.nv(), 7, "fixed-base 7R Panda → nv = 7");
        // universe (0) + 7 revolute joints
        assert_eq!(model.n_joints(), 8);
    }

    #[test]
    fn joint_name_round_trip() {
        let model = Model::from_urdf(panda_urdf_path()).unwrap();
        let id = model.joint_id("panda_joint1").expect("joint exists");
        let name = model.joint_name(id).expect("id is valid");
        assert_eq!(name, "panda_joint1");
    }

    #[test]
    fn missing_path_returns_err() {
        let err = Model::from_urdf("/nonexistent/path/that/does/not/exist.urdf")
            .expect_err("missing path must fail");
        assert!(
            matches!(err, Error::UrdfParse(_)),
            "expected UrdfParse, got {err:?}",
        );
    }

    #[test]
    fn malformed_urdf_returns_err() {
        let err = Model::from_urdf_str("<this is not urdf>")
            .expect_err("malformed URDF must fail");
        assert!(matches!(err, Error::UrdfParse(_)), "expected UrdfParse, got {err:?}");
    }

    #[test]
    fn free_flyer_dimensions() {
        // Free-flyer prepends a 6-DoF root: nq += 7 (quat+translation), nv += 6.
        let model = Model::from_urdf_with_free_flyer(panda_urdf_path()).unwrap();
        assert_eq!(model.nq(), 7 + 7);
        assert_eq!(model.nv(), 7 + 6);
    }

    #[test]
    fn unknown_joint_name_is_err() {
        let model = Model::from_urdf(panda_urdf_path()).unwrap();
        let err = model.joint_id("not_a_real_joint").unwrap_err();
        assert!(matches!(err, Error::UnknownName(_)));
    }
}
