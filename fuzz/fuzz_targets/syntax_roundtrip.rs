//! Parse arbitrary text with the generic syntax layer (both dialects);
//! whatever parses must survive `write → parse` unchanged, and PROTO
//! expansion must stay bounded.

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_vrml::syntax::{
    expand_protos_with, parse_with, write_document_with, Dialect, ExpandLimits, ParseLimits,
    ParseOptions, WriteOptions,
};

fuzz_target!(|data: &[u8]| {
    let Ok(text) = std::str::from_utf8(data) else {
        return;
    };
    for dialect in [Dialect::Vrml97, Dialect::X3dClassic] {
        let opts = ParseOptions {
            dialect,
            limits: ParseLimits {
                max_depth: 32,
                max_nodes: 20_000,
                max_protos: 256,
                max_field_elements: 1 << 20,
                max_image_pixels: 1 << 16,
            },
            require_header: false,
            ..ParseOptions::default()
        };
        let Ok(doc) = parse_with(text, &opts) else {
            continue;
        };
        let wopts = WriteOptions {
            dialect,
            ..WriteOptions::default()
        };
        let written = write_document_with(&doc, &wopts);
        let again = parse_with(&written, &opts).expect("writer output must re-parse");
        let rewritten = write_document_with(&again, &wopts);
        assert_eq!(written, rewritten, "write/parse is not a fixed point");
        let _ = expand_protos_with(
            &doc,
            &ExpandLimits {
                max_nodes: 50_000,
                max_depth: 64,
            },
        );
    }
});
