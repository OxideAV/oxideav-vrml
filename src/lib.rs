//! # oxideav-vrml
//!
//! Pure-Rust VRML97 (ISO/IEC 14772-1:1997) reader and writer.
//!
//! The crate is layered:
//!
//! * [`syntax`] + [`ast`] — a format-generic lexer, typed parser,
//!   writer and PROTO expander for the UTF-8 grammar of Annex A,
//!   producing a node tree that preserves unknown nodes, `DEF` / `USE`
//!   sharing, `PROTO` / `EXTERNPROTO` declarations, `IS` mappings and
//!   `ROUTE`s. The same layer also accepts the X3D ClassicVRML dialect
//!   so a sibling X3D crate can reuse it. It has no dependency on the
//!   3-D scene model.
//!
//! The crate is a clean-room implementation written from the ISO/IEC
//! 14772-1 text published by the Web3D Consortium.

#![deny(missing_docs)]
#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

pub mod ast;
pub mod error;
pub mod syntax;

pub use error::{Error, Result};
