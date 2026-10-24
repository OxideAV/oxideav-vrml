//! Hostile-input hardening and fuzz regressions.

use oxideav_vrml::ast::{BodyElement, Document, FieldData, FieldType, FieldValue, Node, Statement};
use oxideav_vrml::syntax::{
    Dialect, ParseOptions, WriteOptions, parse_with, write_document, write_document_with,
};
use oxideav_vrml::{ConvertOptions, Error, VrmlDecoder, VrmlEncoder};

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

/// `write → parse` must reproduce the tree exactly and be a textual
/// fixed point, in both dialects. Returns the VRML97 text.
fn roundtrip_exact(text: &str) -> String {
    let mut out = String::new();
    for dialect in [Dialect::Vrml97, Dialect::X3dClassic] {
        let opts = ParseOptions {
            dialect,
            require_header: false,
            ..ParseOptions::default()
        };
        let wopts = WriteOptions {
            dialect,
            ..WriteOptions::default()
        };
        let doc = parse_with(text, &opts).unwrap();
        let w1 = write_document_with(&doc, &wopts);
        let again = parse_with(&w1, &opts).unwrap_or_else(|e| panic!("{e}\n{w1}"));
        assert_eq!(doc, again, "{w1}");
        assert_eq!(w1, write_document_with(&again, &wopts));
        if dialect == Dialect::Vrml97 {
            out = w1;
        }
    }
    out
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
fn fuzz_field_assigned_before_its_declaration() {
    // Minimised crash (scheduled Fuzz, syntax_roundtrip, 2026-10-18..24):
    // NULs and commas are whitespace, so this is
    // `; { J 2 eventIn MFVec2d J }`. The parser typed `J 2` by shape
    // (nothing declared `J` yet) and the writer used to emit the
    // declaration first, so the re-parse read `2` as an MFVec2d and
    // failed on the closing brace.
    let text = std::str::from_utf8(b";{J\x002\x00eventIn,MFVec2d,J}").unwrap();
    let opts = ParseOptions {
        dialect: Dialect::X3dClassic,
        require_header: false,
        ..ParseOptions::default()
    };
    let wopts = WriteOptions {
        dialect: Dialect::X3dClassic,
        ..WriteOptions::default()
    };
    let doc = parse_with(text, &opts).unwrap();
    let w1 = write_document_with(&doc, &wopts);
    let again = parse_with(&w1, &opts).unwrap_or_else(|e| panic!("{e}\n{w1}"));
    assert_eq!(doc, again);
    assert_eq!(w1, write_document_with(&again, &wopts));
    assert_eq!(
        doc.nodes[0].body_order,
        [BodyElement::Field, BodyElement::Interface]
    );
}

#[test]
fn body_order_is_preserved() {
    // Annex A lets fields, Script-style declarations and nested
    // ROUTE / PROTO statements interleave freely, and the typed parser
    // reads a field according to what precedes it; the writer must
    // keep the interleaving.
    let w = roundtrip_exact(
        "Script { J 2 eventIn MFVec2f J J 1 2 ROUTE X.a TO Y.b field SFInt32 k 3 k 7 }",
    );
    let lines: Vec<&str> = w.lines().map(str::trim).collect();
    assert_eq!(
        &lines[1..9],
        [
            "Script {",
            "J 2",
            "eventIn MFVec2f J",
            "J 1 2",
            "ROUTE X.a TO Y.b",
            "field SFInt32 k 3",
            "k 7",
            "}",
        ]
    );
    // §4.8.4: a PROTO types only the instantiations after it. `a P`
    // is an unknown node (MFInt32 by shape), `b P` a typed instance.
    let w = roundtrip_exact(
        "Foo { a P { v [ 1 2, 3 4 ] } PROTO P [ field MFVec2f v 0 0 ] { Box {} } \
         b P { v [ 1 2, 3 4 ] } }",
    );
    assert!(w.contains("v [ 1, 2, 3, 4 ]"), "{w}");
    assert!(w.contains("v [ 1 2, 3 4 ]"), "{w}");
    // A programmatically built node has no recorded order and is
    // written declarations → fields → statements.
    let opts = ParseOptions {
        require_header: false,
        ..ParseOptions::default()
    };
    let mut doc = parse_with("Script { url \"x\" eventIn SFBool go }", &opts).unwrap();
    doc.nodes[0].body_order.clear();
    let w = write_document(&doc);
    assert_eq!(
        w,
        "#VRML V2.0 utf8\nScript {\n  eventIn SFBool go\n  url \"x\"\n}\n"
    );
    roundtrip_fixed_point(&w);
}

#[test]
fn fuzz_nonfinite_environment_field_redecodes() {
    // Local `decode` fuzz find (2026-10-24): `1e999` reads as +inf, the
    // Fog is preserved as JSON extras, serde_json renders the infinite
    // component as `null`, the encoder dropped it and wrote `color `
    // with no value. Non-finite components are now exported as 0 (the
    // writer's convention) and a short SFColor falls back to the
    // schema default instead of being written incomplete.
    let src = b"#VRML V2.0 utf8\nFog { color 0.5 0.5 1e999 visibilityRange 100 }\n\
                Background { skyColor [ 0 0 1e999, 1 1 1 ] skyAngle 1e999 }";
    let scene = VrmlDecoder::new().decode_scene(src).unwrap();
    let bytes = VrmlEncoder::new().encode_scene(&scene).unwrap();
    let text = String::from_utf8(bytes.clone()).unwrap();
    assert!(text.contains("color 0.5 0.5 0"), "{text}");
    assert!(text.contains("skyColor [ 0 0 0, 1 1 1 ]"), "{text}");
    assert!(text.contains("skyAngle 0"), "{text}");
    VrmlDecoder::new()
        .decode_scene(&bytes)
        .unwrap_or_else(|e| panic!("{e}\n{text}"));
}

#[test]
fn writer_completes_malformed_single_values() {
    // Programmatically built values with the wrong component count
    // must still produce parseable text: short / empty SF values are
    // padded (0, FALSE, ""), overlong ones truncated.
    let mut doc = Document::default();
    let mut n = Node::new("Foo");
    let bad = |ty: FieldType, data: FieldData| {
        let mut v = FieldValue::empty(ty);
        v.data = data;
        v
    };
    n.set_field("c", bad(FieldType::SFColor, FieldData::Floats(vec![0.5])));
    n.set_field(
        "v",
        bad(FieldType::SFVec3f, FieldData::Floats(vec![1.0; 6])),
    );
    n.set_field("d", bad(FieldType::SFVec3d, FieldData::Doubles(vec![])));
    n.set_field("i", bad(FieldType::SFInt32, FieldData::Int32s(vec![])));
    n.set_field("b", bad(FieldType::SFBool, FieldData::Bools(vec![])));
    n.set_field("s", bad(FieldType::SFString, FieldData::Strings(vec![])));
    n.set_field(
        "r",
        bad(FieldType::SFRotation, FieldData::Floats(vec![0.0, 1.0])),
    );
    let id = doc.add_node(n);
    doc.statements.push(Statement::Node(id));
    let text = write_document(&doc);
    let opts = ParseOptions {
        require_header: false,
        ..ParseOptions::default()
    };
    let again = parse_with(&text, &opts).unwrap_or_else(|e| panic!("{e}\n{text}"));
    let n = &again.nodes[0];
    assert_eq!(n.fields.len(), 7, "{text}");
    // `Foo` is not a catalogued node, so the re-parse infers the field
    // types (and floats then gain a `.0`); from there on the text is a
    // fixed point.
    let w2 = write_document(&again);
    assert_eq!(write_document(&parse_with(&w2, &opts).unwrap()), w2);
    for (name, expect) in [
        ("c", "c 0.5 0 0"),
        ("v", "v 1 1 1"),
        ("d", "d 0 0 0"),
        ("i", "i 0"),
        ("b", "b FALSE"),
        ("s", "s \"\""),
        ("r", "r 0 1 0 0"),
    ] {
        assert!(text.contains(expect), "{name}: {text}");
    }
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
    assert!(
        VrmlDecoder::new()
            .decode_scene(&[0x1F, 0x8B, 8, 0, 0, 0, 0, 0, 0, 3, 1, 2, 3])
            .is_err()
    );
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
