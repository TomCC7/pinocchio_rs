// SPDX-License-Identifier: BSD-2-Clause
//! Safe Rust bindings to the Pinocchio C++ rigid body dynamics library.
//!
//! See the workspace `README.md` for an overview and the per-module rustdoc for
//! the Pinocchio conventions each module wraps.

pub mod algorithms;
pub mod data;
pub mod derivatives;
pub mod error;
pub mod force;
pub mod inertia;
pub mod lie;
pub mod model;
pub mod motion;
pub mod reference_frame;
pub mod se3;

pub use data::Data;
pub use error::{Error, Result};
pub use force::Force;
pub use inertia::Inertia;
pub use model::Model;
pub use motion::Motion;
pub use reference_frame::ReferenceFrame;
pub use se3::SE3;

#[cfg(test)]
pub(crate) mod test_support;
