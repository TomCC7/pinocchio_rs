// SPDX-License-Identifier: BSD-2-Clause
//! Safe wrapper around `pinocchio::Data`.
//!
//! `Data` is the mutable scratch space that Pinocchio's algorithms read from
//! and write into. Each [`Model`] corresponds to one or more `Data` instances
//! — typically one per parallel worker thread.

use cxx::UniquePtr;
use pinocchio_sys::ffi;

use crate::model::Model;

/// Pinocchio's per-`Model` mutable scratch.
pub struct Data {
    // Read by §5+ algorithm wrappers; for §3 the destructor at scope exit is
    // the only consumer, so the dead-code lint would otherwise warn.
    #[allow(dead_code)]
    pub(crate) inner: UniquePtr<ffi::Data>,
}

impl Data {
    /// Allocate a fresh `Data` sized for `model`.
    pub fn new(model: &Model) -> Self {
        let inner = ffi::data_new(model.raw());
        Self { inner }
    }

    pub(crate) fn _raw(&self) -> &ffi::Data {
        self.inner.as_ref().expect("Data UniquePtr is never null after construction")
    }
}

impl std::fmt::Debug for Data {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::panda_urdf_path;

    /// UT.3c — Data construction succeeds for the Panda model.
    #[test]
    fn data_new_for_panda() {
        let model = Model::from_urdf(panda_urdf_path()).unwrap();
        let _data = Data::new(&model);
        // No public accessor yet — §5 onward will exercise the buffers via the
        // FK / RNEA / CRBA wrappers. The point of this gate is just to confirm
        // that allocation through the FFI doesn't crash and the destructor
        // runs cleanly at scope exit (the test ends without leaking).
    }
}
