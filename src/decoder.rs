//! [`VrmlDecoder`] — bytes → [`Scene3D`].
//!
//! Pipeline: optional gzip inflate (`.wrz` / `.wrl.gz`, detected by
//! the RFC 1952 magic) → text (UTF-8, with a Latin-1 fallback for the
//! many legacy files that carry 8-bit bytes in strings/comments) →
//! [`parse`](crate::syntax::parse_with) → header check →
//! [`expand_protos`](crate::syntax::expand_protos_with) →
//! [`document_to_scene`](crate::convert::document_to_scene()).

use std::borrow::Cow;
use std::sync::Arc;

use oxideav_mesh3d::{Mesh3DDecoder, Scene3D};

use crate::ast::Document;
use crate::convert::{document_to_scene, ConvertOptions, UrlResolver};
use crate::error::{Error, Result};
use crate::syntax::{expand_protos_with, parse_with, ExpandLimits, ParseLimits, ParseOptions};

/// Cap on the inflated size of a gzip-compressed file
/// (decompression-bomb guard).
pub const MAX_INFLATED: u64 = 512 * 1024 * 1024;

/// `true` if `bytes` start with the gzip magic (`1F 8B`).
pub fn is_gzip(bytes: &[u8]) -> bool {
    bytes.starts_with(&[0x1F, 0x8B])
}

/// `true` if `bytes` look like a VRML97 file (plain or gzip-compressed).
pub fn probe(bytes: &[u8]) -> bool {
    if is_gzip(bytes) {
        return true;
    }
    let b = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF]).unwrap_or(bytes);
    b.starts_with(b"#VRML V2.0")
}

/// Inflate gzip input (capped at [`MAX_INFLATED`]); other input is
/// returned unchanged.
pub fn maybe_inflate(bytes: &[u8]) -> Result<Cow<'_, [u8]>> {
    if !is_gzip(bytes) {
        return Ok(Cow::Borrowed(bytes));
    }
    match compcol::vec::decompress_to_vec_capped::<compcol::gzip::Gzip>(bytes, MAX_INFLATED) {
        Ok(v) => Ok(Cow::Owned(v)),
        Err(compcol::Error::OutputLimitExceeded) => Err(Error::limit(format!(
            "gzip stream inflates past {MAX_INFLATED} bytes"
        ))),
        Err(e) => Err(Error::invalid(format!("gzip inflate failed: {e}"))),
    }
}

/// Bytes → text: UTF-8, else Latin-1 (every byte maps to U+00xx).
pub fn decode_text(bytes: &[u8]) -> Cow<'_, str> {
    match std::str::from_utf8(bytes) {
        Ok(s) => Cow::Borrowed(s),
        Err(_) => Cow::Owned(bytes.iter().map(|&b| b as char).collect()),
    }
}

/// Inflate, decode and parse a VRML97 file (no PROTO expansion), with
/// default limits. Rejects non-VRML97 headers.
pub fn read_document(bytes: &[u8]) -> Result<Document> {
    read_document_with(bytes, &ParseLimits::default())
}

/// [`read_document`] with explicit parse limits.
pub fn read_document_with(bytes: &[u8], limits: &ParseLimits) -> Result<Document> {
    let raw = maybe_inflate(bytes)?;
    let text = decode_text(&raw);
    let opts = ParseOptions {
        limits: *limits,
        ..ParseOptions::default()
    };
    let doc = parse_with(&text, &opts)?;
    check_header(&doc)?;
    Ok(doc)
}

fn check_header(doc: &Document) -> Result<()> {
    let h = &doc.header;
    if h.format != "VRML" {
        return Err(Error::invalid(format!(
            "not a VRML file (header `#{} {} {}`)",
            h.format, h.version, h.encoding
        )));
    }
    if h.version != "V2.0" {
        return Err(Error::unsupported(format!(
            "VRML version {} (only VRML97 / V2.0 is supported)",
            h.version
        )));
    }
    if h.encoding != "utf8" {
        return Err(Error::unsupported(format!(
            "VRML encoding `{}` (only utf8 is defined by ISO/IEC 14772-1)",
            h.encoding
        )));
    }
    Ok(())
}

/// VRML97 decoder.
#[derive(Debug, Clone, Default)]
pub struct VrmlDecoder {
    options: ConvertOptions,
    parse_limits: ParseLimits,
    expand_limits: ExpandLimits,
}

impl VrmlDecoder {
    /// Decoder with default options.
    pub fn new() -> Self {
        Self::default()
    }

    /// Replace the scene-conversion options.
    pub fn with_options(mut self, options: ConvertOptions) -> Self {
        self.options = options;
        self
    }

    /// Follow `Inline` URLs through `resolver`.
    pub fn with_resolver(mut self, resolver: Arc<dyn UrlResolver>) -> Self {
        self.options.resolver = Some(resolver);
        self
    }

    /// Replace the parser caps.
    pub fn with_parse_limits(mut self, limits: ParseLimits) -> Self {
        self.parse_limits = limits;
        self
    }

    /// Replace the PROTO-expansion caps.
    pub fn with_expand_limits(mut self, limits: ExpandLimits) -> Self {
        self.expand_limits = limits;
        self
    }

    /// Decode with the crate-local error type.
    pub fn decode_scene(&self, bytes: &[u8]) -> Result<Scene3D> {
        let doc = read_document_with(bytes, &self.parse_limits)?;
        let doc = expand_protos_with(&doc, &self.expand_limits)?;
        document_to_scene(&doc, &self.options)
    }
}

impl Mesh3DDecoder for VrmlDecoder {
    fn decode(&mut self, bytes: &[u8]) -> oxideav_mesh3d::Result<Scene3D> {
        self.decode_scene(bytes).map_err(Error::into_mesh3d)
    }
}
