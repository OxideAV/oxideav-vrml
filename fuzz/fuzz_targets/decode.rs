//! Decode an arbitrary byte string as a VRML97 file (plain or gzip)
//! through the full pipeline — parse, PROTO expansion, Scene3D
//! conversion — and, when that succeeds, re-encode the scene and decode
//! it again. Nothing may panic, loop or allocate without bound.

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_vrml::syntax::{ExpandLimits, ParseLimits};
use oxideav_vrml::{ConvertOptions, Tessellation, VrmlDecoder, VrmlEncoder};

fuzz_target!(|data: &[u8]| {
    let decoder = VrmlDecoder::new()
        .with_parse_limits(ParseLimits {
            max_depth: 32,
            max_nodes: 20_000,
            max_protos: 256,
            max_field_elements: 1 << 20,
            max_image_pixels: 1 << 16,
        })
        .with_expand_limits(ExpandLimits {
            max_nodes: 50_000,
            max_depth: 64,
        })
        .with_options(ConvertOptions {
            tessellation: Tessellation {
                slices: 8,
                stacks: 4,
            },
            max_scene_nodes: 20_000,
            max_generated: 200_000,
            all_switch_choices: true,
            all_lod_levels: true,
            ..ConvertOptions::default()
        });
    let Ok(scene) = decoder.decode_scene(data) else {
        return;
    };
    let Ok(bytes) = VrmlEncoder::new().encode_scene(&scene) else {
        return;
    };
    // The encoder's own output must always be readable again.
    decoder
        .decode_scene(&bytes)
        .expect("re-decoding encoder output failed");
});
