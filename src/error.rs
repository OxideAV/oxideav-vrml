//! Crate-local error type.
//!
//! The syntax layer ([`crate::syntax`]) and the AST never depend on the
//! framework error vocabulary, so the lexer / parser / writer stay
//! reusable by sibling crates (e.g. an X3D ClassicVRML reader) with no
//! extra dependency. The [`Mesh3DDecoder`](oxideav_mesh3d::Mesh3DDecoder)
//! / [`Mesh3DEncoder`](oxideav_mesh3d::Mesh3DEncoder) adaptors map this
//! type onto [`oxideav_mesh3d::Error`] at the trait boundary via
//! [`Error::into_mesh3d`].

use std::fmt;

/// Errors produced while reading or writing VRML.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// Lexical or syntactic violation of the ISO/IEC 14772-1 Annex A
    /// grammar. `line` / `column` are 1-based and point at the
    /// offending token.
    Syntax {
        /// 1-based line of the offending token.
        line: usize,
        /// 1-based column (in characters) of the offending token.
        column: usize,
        /// Human-readable description of the violation.
        message: String,
    },
    /// A configured [`ParseLimits`](crate::syntax::ParseLimits) /
    /// conversion cap was exceeded — the canonical "DoS protection
    /// fired" rejection for hostile input.
    LimitExceeded(String),
    /// Structurally invalid input that is not a grammar violation
    /// (bad header, broken gzip stream, undefined `USE` name, …).
    InvalidData(String),
    /// Well-formed input using a construct this crate does not handle
    /// (e.g. a VRML 1.0 `#VRML V1.0 ascii` file).
    Unsupported(String),
}

impl Error {
    /// Build an [`Error::Syntax`].
    pub fn syntax(line: usize, column: usize, message: impl Into<String>) -> Self {
        Self::Syntax {
            line,
            column,
            message: message.into(),
        }
    }

    /// Build an [`Error::InvalidData`].
    pub fn invalid(message: impl Into<String>) -> Self {
        Self::InvalidData(message.into())
    }

    /// Build an [`Error::LimitExceeded`].
    pub fn limit(message: impl Into<String>) -> Self {
        Self::LimitExceeded(message.into())
    }

    /// Build an [`Error::Unsupported`].
    pub fn unsupported(message: impl Into<String>) -> Self {
        Self::Unsupported(message.into())
    }

    /// Map onto the [`oxideav_mesh3d::Error`] vocabulary used by the
    /// decoder / encoder traits.
    pub fn into_mesh3d(self) -> oxideav_mesh3d::Error {
        match self {
            Self::Unsupported(m) => oxideav_mesh3d::Error::unsupported(format!("VRML: {m}")),
            #[cfg(feature = "registry")]
            Self::LimitExceeded(m) => {
                oxideav_mesh3d::Error::ResourceExhausted(format!("VRML: limit exceeded: {m}"))
            }
            other => oxideav_mesh3d::Error::invalid(format!("VRML: {other}")),
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Syntax {
                line,
                column,
                message,
            } => write!(f, "syntax error at {line}:{column}: {message}"),
            Self::LimitExceeded(m) => write!(f, "limit exceeded: {m}"),
            Self::InvalidData(m) => write!(f, "invalid data: {m}"),
            Self::Unsupported(m) => write!(f, "unsupported: {m}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<Error> for oxideav_mesh3d::Error {
    fn from(e: Error) -> Self {
        e.into_mesh3d()
    }
}

/// Convenience alias used throughout the crate.
pub type Result<T> = std::result::Result<T, Error>;
