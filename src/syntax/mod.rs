//! Format-generic syntax layer: tokeniser, parser, writer and PROTO
//! expansion for the ISO/IEC 14772-1 UTF-8 encoding (and, with
//! [`Dialect::X3dClassic`], the X3D ClassicVRML encoding).
//!
//! Nothing here knows about [`oxideav_mesh3d`]; the layer produces and
//! consumes the [`crate::ast`] tree only, so other crates can reuse it
//! for their own scene models.
//!
//! ```
//! use oxideav_vrml::syntax::{parse, write_document};
//!
//! let doc = parse("#VRML V2.0 utf8\nShape { geometry Box { size 1 2 3 } }").unwrap();
//! let text = write_document(&doc);
//! assert!(text.contains("size 1 2 3"));
//! ```

pub mod catalog;
pub mod expand;
pub mod lexer;
pub mod parser;
pub mod writer;

pub use catalog::{vrml97_node, FieldSchema, NodeCatalog, NodeSchema, Vrml97Catalog};
pub use expand::{expand_protos, expand_protos_with, ExpandLimits};
pub use lexer::{Lexer, Token, TokenKind};
pub use parser::{parse, parse_field_value, parse_with, Dialect, ParseLimits, ParseOptions};
pub use writer::{write_document, write_document_with, WriteOptions};
