// SPDX-License-Identifier: BSD-2-Clause
//! Error type used across the safe wrappers.
//!
//! Every fallible operation maps C++ exceptions raised by Pinocchio into a
//! single `Error` enum — keeping the public surface narrow and matching
//! Rust's idiomatic `Result<T, Error>` style.

use std::fmt;

/// Errors produced by `pinocchio-rs`. Wraps the underlying C++ exception
/// message when one is available.
#[derive(Debug)]
pub enum Error {
    /// URDF parsing or file I/O failure (e.g., file not found, malformed XML,
    /// unsupported joint type).
    UrdfParse(String),
    /// Looked up a joint or frame name that the model does not contain.
    UnknownName(String),
    /// An index passed to an accessor was out of range for the model/data.
    OutOfRange(String),
    /// Any other C++ exception not classified above.
    Cxx(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UrdfParse(msg) => write!(f, "URDF parse error: {msg}"),
            Self::UnknownName(name) => write!(f, "unknown name: {name}"),
            Self::OutOfRange(msg) => write!(f, "out of range: {msg}"),
            Self::Cxx(msg) => write!(f, "C++ exception: {msg}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<cxx::Exception> for Error {
    fn from(e: cxx::Exception) -> Self {
        // We don't try to introspect message text further — Pinocchio + urdfdom
        // produce a wide variety of strings. Callers that need to discriminate
        // can match on `Error::UrdfParse(msg)` etc. when we have explicit
        // wrappers that classify before constructing the variant.
        Self::Cxx(e.what().to_owned())
    }
}

/// Convenience alias.
pub type Result<T> = core::result::Result<T, Error>;
