//! Hostile-input hardening and fuzz regressions.

use oxideav_vrml::syntax::{parse_with, write_document, ParseOptions};
use oxideav_vrml::{ConvertOptions, Error, VrmlDecoder};

fn roundtrip_fixed_point(text: &str) {
    let opts = ParseOptions {
        require_header: false,
        ..ParseOptions::default()
    };
    let doc = parse_with(text, &opts).unwrap();
    let w1 = write_document(&doc);
    let w2 = write_document(&parse_with(&w1, &opts).unwrap());
    assert_eq!(w1, w2);
}

#[test]
fn fuzz_header_comment_split() {
    // The header comment starts after the third word, not at the first
    // textual occurrence of the encoding word.
    roundtrip_fixed_point("#Vaius 0.25oon 0 1 0 children\nShape { }");
}

#[test]
fn fuzz_inferred_float_stays_float() {
    // An unknown node's MFFloat written as integers would re-infer as
    // MFInt32.
    roundtrip_fixed_point("#VRML V2.0 utf8\nFoo { l [ -1. 0 0 1 1 0 2 1 0 3 0 ] }");
}

#[test]
fn exponential_use_instancing_is_capped() {
    // Each level USEs the previous one twice: 2^40 scene nodes.
    let mut s =
        String::from("#VRML V2.0 utf8\nDEF L0 Group { children Shape { geometry Box {} } }\n");
    for i in 1..40 {
        s.push_str(&format!(
            "DEF L{i} Group {{ children [ USE L{p} USE L{p} ] }}\n",
            p = i - 1
        ));
    }
    let r = VrmlDecoder::new().decode_scene(s.as_bytes());
    assert!(matches!(r, Err(Error::LimitExceeded(_))), "{r:?}");
}

#[test]
fn deep_nesting_is_rejected() {
    let mut s = String::from("#VRML V2.0 utf8\n");
    for _ in 0..100_000 {
        s.push_str("Transform{children ");
    }
    assert!(matches!(
        VrmlDecoder::new().decode_scene(s.as_bytes()),
        Err(Error::LimitExceeded(_))
    ));
}

#[test]
fn extrusion_product_is_capped() {
    let mut s = String::from("#VRML V2.0 utf8\nShape { geometry Extrusion { spine [");
    for i in 0..400 {
        s.push_str(&format!("0 {i} 0, "));
    }
    s.push_str("] crossSection [");
    for i in 0..400 {
        s.push_str(&format!("{i} 0, "));
    }
    s.push_str("] } }");
    let opts = ConvertOptions {
        max_generated: 10_000,
        ..ConvertOptions::default()
    };
    let scene = VrmlDecoder::new()
        .with_options(opts)
        .decode_scene(s.as_bytes())
        .unwrap();
    assert!(scene.meshes.is_empty());
}

#[test]
fn gzip_bomb_and_garbage() {
    // Truncated / corrupt gzip.
    assert!(VrmlDecoder::new()
        .decode_scene(&[0x1F, 0x8B, 8, 0, 0, 0, 0, 0, 0, 3, 1, 2, 3])
        .is_err());
    // Cycles through USE inside the node's own subtree terminate.
    let s = b"#VRML V2.0 utf8\nDEF G Group { children [ Shape { geometry Box {} } USE G ] }";
    let scene = VrmlDecoder::new().decode_scene(s).unwrap();
    assert_eq!(scene.nodes.len(), 1);
}

#[test]
fn hostile_indices_and_values() {
    let s = b"#VRML V2.0 utf8
Shape { geometry IndexedFaceSet {
  coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
  coordIndex [ 0 1 2 -1 0 1 99999999 -1 -5 2 1 0 ]
  normalIndex [ 7 7 7 7 ] normal Normal { vector [ 0 0 0 ] }
  colorIndex [ -3 ] color Color { color [ ] }
  texCoordIndex [ 100 ] texCoord TextureCoordinate { point [ 0 0 ] }
  creaseAngle 1e30
} }
Shape { geometry ElevationGrid { xDimension 2147483647 zDimension 2147483647 height [ 1 ] } }
Shape { geometry Sphere { radius -1 } }
Shape { geometry Box { size 0 0 0 } }
Transform { scale 0 0 0 rotation 0 0 0 0 children Shape { geometry Cone { height 1e39 } } }
DEF T TimeSensor { cycleInterval -1 }
DEF P PositionInterpolator { key [ 1 0 0.5 0.5 ] keyValue [ 0 0 0 ] }
ROUTE T.fraction_changed TO P.set_fraction
";
    let scene = VrmlDecoder::new().decode_scene(s).unwrap();
    assert!(scene.validate().is_ok(), "{:?}", scene.validate());
}
