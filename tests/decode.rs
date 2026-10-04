//! Decoder tests over the hand-written fixtures in `tests/fixtures/`.

use std::sync::Arc;

use oxideav_mesh3d::{
    AnimationProperty, AnimationValues, Camera, ImageData, Light, Mesh, Scene3D, Topology,
    Transform, WrapMode,
};
use oxideav_vrml::{ConvertOptions, UrlResolver, VrmlDecoder};

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(format!(
        "{}/tests/fixtures/{name}",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap()
}

fn decode(name: &str) -> Scene3D {
    let s = VrmlDecoder::new().decode_scene(&fixture(name)).unwrap();
    assert!(s.validate().is_ok(), "{name}: {:?}", s.validate());
    s
}

fn named<'a>(s: &'a Scene3D, name: &str) -> &'a oxideav_mesh3d::Node {
    s.nodes
        .iter()
        .find(|n| n.name.as_deref() == Some(name))
        .unwrap_or_else(|| panic!("no node {name}"))
}

fn mesh_named<'a>(s: &'a Scene3D, name: &str) -> &'a Mesh {
    s.meshes
        .iter()
        .find(|m| m.name.as_deref() == Some(name))
        .unwrap_or_else(|| panic!("no mesh {name}"))
}

fn mesh_of<'a>(s: &'a Scene3D, n: &oxideav_mesh3d::Node) -> &'a Mesh {
    &s.meshes[n.mesh.expect("node has a mesh").0 as usize]
}

#[test]
fn primitives_and_def_use() {
    let s = decode("primitives.wrl");
    // BALL is USE'd → both instances share one mesh.
    let ball = named(&s, "BALL");
    let ball_mesh = ball.mesh.unwrap();
    let users = s.nodes.iter().filter(|n| n.mesh == Some(ball_mesh)).count();
    assert_eq!(users, 2, "USE BALL should instance the same mesh");
    match ball.transform {
        Transform::Trs {
            scale, rotation, ..
        } => {
            assert_eq!(scale, [2.0; 3]);
            assert!((rotation[1] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-5);
        }
        _ => panic!("plain Transform should be TRS"),
    }
    let bb = s.meshes[ball_mesh.0 as usize].bounding_box().unwrap();
    assert!((bb.max[1] - 0.5).abs() < 1e-5);
    // Shared appearance → one material, Phong originals preserved.
    let red: Vec<_> = s
        .materials
        .iter()
        .filter(|m| m.name.as_deref() == Some("RedMat"))
        .collect();
    assert_eq!(red.len(), 1);
    assert_eq!(&red[0].base_color[..3], &[1.0, 0.0, 0.0]);
    assert_eq!(red[0].metallic, 0.0);
    assert_eq!(red[0].extras["vrml:material"]["shininess"], 0.5);
    // Shapes without a Material are unlit (§4.14.2).
    assert!(s.materials.iter().any(|m| m.ext.unlit));
    assert!(s.extras.contains_key("vrml:worldInfo"));
    // Box 1×2×3 has volume 6.
    let boxes: Vec<_> = s
        .meshes
        .iter()
        .filter(|m| (m.volume() - 6.0).abs() < 1e-3)
        .collect();
    assert_eq!(boxes.len(), 1);
}

#[test]
fn indexed_face_set_hints() {
    let s = decode("indexed.wrl");
    // creaseAngle 0.5 on a cube: every corner split → 24 vertices,
    // all normals axis-aligned.
    let cube = &mesh_named(&s, "CREASE").primitives[0];
    assert_eq!(cube.positions.len(), 24);
    assert_eq!(cube.triangle_count(), 12);
    assert!(cube.signed_volume() > 7.99, "{}", cube.signed_volume());
    for n in cube.normals.as_ref().unwrap() {
        assert_eq!(n.iter().filter(|c| c.abs() > 0.999).count(), 1, "{n:?}");
    }
    // creaseAngle > π on the same cube → smooth: 8 shared vertices…
    // except per-face colours split them again (6 faces × 4 corners).
    let others: Vec<&oxideav_mesh3d::Node> = s
        .nodes
        .iter()
        .filter(|n| n.mesh.is_some() && n.name.is_none())
        .collect();
    let smooth = others
        .iter()
        .map(|n| &mesh_of(&s, n).primitives[0])
        .find(|p| p.triangle_count() == 12 && !p.colors.is_empty())
        .unwrap();
    assert_eq!(smooth.positions.len(), 24);
    let n0 = smooth.normals.as_ref().unwrap()[0];
    let k = 1.0 / 3f32.sqrt();
    assert!(n0.iter().all(|c| (c.abs() - k).abs() < 1e-5), "{n0:?}");
    // Concave hexagon: convex FALSE, ccw FALSE, solid FALSE.
    let concave = others
        .iter()
        .map(|n| (n, &mesh_of(&s, n).primitives[0]))
        .find(|(_, p)| p.triangle_count() == 4)
        .unwrap();
    let p = concave.1;
    assert!((p.surface_area() - 3.0).abs() < 1e-5, "ear clipping area");
    let mat = &s.materials[p.material.unwrap().0 as usize];
    assert!(mat.double_sided);
    assert_eq!(p.extras["vrml:solid"], false);
    // Per-face explicit normal (0 0 1) with ccw FALSE: winding reversed
    // so the geometric normal agrees with the supplied +Z normal.
    let tri = p.triangle_indices()[0];
    let [a, b, c] = tri.map(|i| p.positions[i as usize]);
    let gz = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    assert!(gz > 0.0);
    assert!(p
        .normals
        .as_ref()
        .unwrap()
        .iter()
        .all(|n| *n == [0.0, 0.0, 1.0]));
    assert_eq!(p.colors[0].len(), p.positions.len());
}

#[test]
fn line_and_point_sets() {
    let s = decode("lines_points.wrl");
    let prims: Vec<_> = s.meshes.iter().flat_map(|m| &m.primitives).collect();
    let lines = prims
        .iter()
        .find(|p| p.topology == Topology::Lines)
        .unwrap();
    // Polylines 0-1-2-3-0 (4 segments) and 0-2 (1 segment).
    assert_eq!(lines.indices.as_ref().unwrap().len(), 10);
    assert!(!lines.colors.is_empty());
    let m = &s.materials[lines.material.unwrap().0 as usize];
    assert!(m.ext.unlit);
    let points = prims
        .iter()
        .find(|p| p.topology == Topology::Points)
        .unwrap();
    assert_eq!(points.positions.len(), 3);
    assert_eq!(points.colors[0][2], [0.0, 0.0, 1.0, 1.0]);
}

#[test]
fn elevation_grid_and_extrusion() {
    let s = decode("grid_extrusion.wrl");
    let prims: Vec<_> = s.meshes.iter().flat_map(|m| &m.primitives).collect();
    let grid = prims
        .iter()
        .find(|p| p.extras["vrml:geometry"] == "ElevationGrid")
        .unwrap();
    assert_eq!(grid.triangle_count(), 3 * 2 * 2);
    let bb = grid.bounding_box().unwrap();
    assert_eq!(bb.max[0], 1.5);
    assert_eq!(bb.max[2], 4.0);
    assert!(grid.normals.as_ref().unwrap().iter().all(|n| n[1] > 0.0));
    let ext: Vec<_> = prims
        .iter()
        .filter(|p| p.extras["vrml:geometry"] == "Extrusion")
        .collect();
    assert_eq!(ext.len(), 2);
    // Revolution: 8 segments × 2 spine intervals × 2 + 2 caps × 6.
    let rev = ext[0];
    assert_eq!(rev.triangle_count(), 8 * 2 * 2 + 2 * 6);
    assert!(rev.signed_volume() > 0.0, "outward facing");
    let rb = rev.bounding_box().unwrap();
    assert!((rb.max[1] - 2.0).abs() < 1e-5);
    // Waist scaled by 0.5 at y = 1.
    let waist = rev
        .positions
        .iter()
        .filter(|p| (p[1] - 1.0).abs() < 1e-5)
        .map(|p| (p[0] * p[0] + p[2] * p[2]).sqrt())
        .fold(0.0f32, f32::max);
    assert!((waist - 0.5).abs() < 1e-3, "{waist}");
    // Bent / twisted spine without beginCap: 3 × 4 quads + end cap.
    assert_eq!(ext[1].triangle_count(), 3 * 4 * 2 + 2);
}

#[test]
fn protos_expand_with_is_mapping() {
    let s = decode("protos.wrl");
    let a = named(&s, "A");
    match a.transform {
        Transform::Trs { translation, .. } => assert_eq!(translation, [0.0, 2.0, 0.0]),
        _ => panic!(),
    }
    let mat = &s.materials[mesh_of(&s, a).primitives[0].material.unwrap().0 as usize];
    assert_eq!(&mat.base_color[..3], &[1.0, 0.0, 0.0]);
    // Pair → two ColoredBoxes coloured blue via a nested IS chain.
    let blue = s
        .materials
        .iter()
        .filter(|m| m.base_color[..3] == [0.0, 0.0, 1.0])
        .count();
    assert!(blue >= 1);
    let small = s
        .meshes
        .iter()
        .filter(|m| (m.volume() - 0.125).abs() < 1e-4)
        .count();
    assert_eq!(small, 1, "boxSize default overridden on one box");
    // Unresolved EXTERNPROTO instance is preserved as a placeholder.
    let remote = s
        .nodes
        .iter()
        .find(|n| {
            n.extras
                .get("vrml:node")
                .is_some_and(|v| v["type"] == "Remote")
        })
        .unwrap();
    assert_eq!(remote.extras["vrml:node"]["fields"]["weight"], 2.0);
}

#[test]
fn animations_from_routes() {
    let s = decode("animation.wrl");
    let clock = s
        .animations
        .iter()
        .find(|a| a.name.as_deref() == Some("CLOCK"))
        .unwrap();
    let mover = s
        .nodes
        .iter()
        .position(|n| n.name.as_deref() == Some("MOVER"))
        .unwrap();
    let spinner = s
        .nodes
        .iter()
        .position(|n| n.name.as_deref() == Some("SPINNER"))
        .unwrap();
    let ch = |node: usize, prop| {
        clock
            .channels
            .iter()
            .find(|c| c.target.node.0 as usize == node && c.target.property == prop)
    };
    let tr = ch(mover, AnimationProperty::Translation).unwrap();
    assert_eq!(tr.sampler.keyframes, [0.0, 2.0, 4.0]);
    let rot = ch(spinner, AnimationProperty::Rotation).unwrap();
    match &rot.sampler.values {
        AnimationValues::Quat(q) => assert!((q[1][1] - 1.0).abs() < 1e-4),
        _ => panic!(),
    }
    // CoordinateInterpolator → morph targets + one-hot weights.
    let morph = ch(spinner, AnimationProperty::MorphWeights).unwrap();
    assert_eq!(morph.sampler.keyframes, [0.0, 4.0]);
    let prim = &mesh_of(&s, &s.nodes[spinner]).primitives[0];
    assert_eq!(prim.targets.len(), 2);
    let morphed = prim.morphed(&[0.0, 1.0]);
    let bb = morphed.bounding_box().unwrap();
    assert_eq!(bb.max, [2.0, 2.0, 0.0]);
    // ScalarInterpolator has no Scene3D mapping: kept in extras.
    assert!(s.extras["vrml:behaviour"].get("FADE").is_some());
    assert!(s.extras["vrml:routes"].as_array().unwrap().len() >= 8);
    // Behaviour inside the PROTO body (T → P → B).
    let inner = s
        .animations
        .iter()
        .find(|a| a.name.as_deref() == Some("T"))
        .expect("PROTO-internal TimeSensor animation");
    assert_eq!(inner.channels.len(), 1);
    // Route to the instance's `set_where` is forwarded through IS.
    let bounce = s
        .nodes
        .iter()
        .position(|n| n.name.as_deref() == Some("BOUNCE"))
        .unwrap();
    assert!(ch(bounce, AnimationProperty::Translation).is_some());
}

#[test]
fn environment_cameras_and_lights() {
    let s = decode("environment.wrl");
    let entry = named(&s, "Entry");
    let cam = s.cameras[entry.camera.unwrap().0 as usize];
    match cam {
        Camera::Perspective {
            yfov, znear, zfar, ..
        } => {
            assert_eq!(yfov, 0.9);
            assert_eq!(znear, 0.25);
            assert_eq!(zfar, Some(500.0));
        }
        _ => panic!(),
    }
    match entry.transform {
        Transform::Trs { translation, .. } => assert_eq!(translation, [0.0, 1.6, 10.0]),
        _ => panic!(),
    }
    let sun = named(&s, "Sun");
    assert!(matches!(
        s.lights[sun.light.unwrap().0 as usize],
        Light::Directional { intensity, .. } if intensity == 0.8
    ));
    // The node's −Z axis points along the light direction (0 −1 0).
    let world = s.world_node_transforms();
    let idx = s
        .nodes
        .iter()
        .position(|n| n.name.as_deref() == Some("Sun"))
        .unwrap();
    let m = world[idx].unwrap();
    assert!((-m[1][2] - -1.0).abs() < 1e-5, "{m:?}");
    let lamp = named(&s, "Lamp");
    assert!(matches!(
        s.lights[lamp.light.unwrap().0 as usize],
        Light::Point { range: Some(r), .. } if r == 20.0
    ));
    assert_eq!(lamp.extras["vrml:light"]["attenuation"][1], 0.1f32 as f64);
    let spot = named(&s, "Spot");
    match s.lights[spot.light.unwrap().0 as usize] {
        Light::Spot {
            intensity,
            outer_cone_angle,
            inner_cone_angle,
            ..
        } => {
            assert_eq!(intensity, 0.0, "on FALSE");
            assert_eq!(outer_cone_angle, 0.6);
            assert!((inner_cone_angle - 0.3).abs() < 1e-6);
        }
        _ => panic!(),
    }
    for k in [
        "vrml:worldInfo",
        "vrml:navigationInfo",
        "vrml:background",
        "vrml:fog",
    ] {
        assert!(s.extras.contains_key(k), "{k}");
    }
}

#[test]
fn textures_and_texture_transform() {
    let s = decode("textures.wrl");
    assert_eq!(s.textures.len(), 3);
    match &s.textures[0].image {
        ImageData::External { uri, .. } => assert_eq!(uri, "brick.png"),
        _ => panic!(),
    }
    assert_eq!(s.textures[0].sampler.wrap_s, WrapMode::ClampToEdge);
    assert_eq!(s.textures[0].sampler.wrap_t, WrapMode::Repeat);
    // PixelTexture → embedded PNG.
    match &s.textures[1].image {
        ImageData::Source(src) => assert_eq!(src.mime(), Some("image/png")),
        _ => panic!(),
    }
    // RGB texture replaces the diffuse colour (Table 4.6) …
    let checker_mat = s
        .materials
        .iter()
        .find(|m| m.base_color_texture.is_some_and(|t| t.texture.0 == 1))
        .unwrap();
    assert_eq!(&checker_mat.base_color[..3], &[1.0; 3]);
    // … an intensity texture modulates it.
    let gray = s
        .materials
        .iter()
        .find(|m| m.base_color_texture.is_some_and(|t| t.texture.0 == 2))
        .unwrap();
    assert_eq!(&gray.base_color[..3], &[1.0, 0.0, 0.0]);
    // Box UVs with TextureTransform scale 2 / translation 0.5 baked in:
    // (s, t) = (0, 0) → ((0 + 0.5) · 2, 0) → glTF (1, 1).
    let boxp = s
        .meshes
        .iter()
        .flat_map(|m| &m.primitives)
        .find(|p| p.extras["vrml:geometry"] == "Box")
        .unwrap();
    assert!(boxp.uvs[0].contains(&[1.0, 1.0]));
    assert!(boxp.uvs[0].contains(&[3.0, -1.0]));
    // Default IFS mapping (no texCoord): S along X, T along Y.
    let quad = s
        .meshes
        .iter()
        .flat_map(|m| &m.primitives)
        .find(|p| p.material == Some(oxideav_mesh3d::MaterialId(1)))
        .unwrap();
    let top_right = quad
        .positions
        .iter()
        .position(|q| *q == [1.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(quad.uvs[0][top_right], [1.0, 0.0]);
}

struct MapResolver;

impl UrlResolver for MapResolver {
    fn resolve(&self, url: &str) -> Option<Vec<u8>> {
        (url == "child.wrl").then(|| fixture("child.wrl"))
    }
}

#[test]
fn grouping_nodes() {
    let s = decode("grouping.wrl");
    let with = |k: &str| s.nodes.iter().find(|n| n.extras.contains_key(k)).unwrap();
    assert_eq!(
        with("vrml:anchor").extras["vrml:anchor"]["url"][0],
        "other.wrl#View"
    );
    assert!(with("vrml:billboard").mesh.is_some());
    assert_eq!(
        with("vrml:collision").extras["vrml:collision"]["collide"],
        false
    );
    // Switch whichChoice 1 → only the sphere is converted.
    let sw = with("vrml:switch");
    assert_eq!(sw.children.len(), 1);
    let sphere = mesh_of(&s, &s.nodes[sw.children[0].0 as usize]);
    assert!(sphere.primitives[0].extras["vrml:geometry"] == "Sphere");
    // LOD → finest level only.
    assert_eq!(with("vrml:lod").children.len(), 1);
    // Unresolved Inline kept as a reference.
    let inl = with("vrml:inline");
    assert_eq!(inl.extras["vrml:inline"]["url"][0], "child.wrl");
    assert!(inl.children.is_empty());
    // center / scaleOrientation → matrix with originals in extras.
    let tf = with("vrml:transform");
    assert!(matches!(tf.transform, Transform::Matrix(_)));
    // Unknown node kept, its `children` still converted.
    let fancy = with("vrml:node");
    assert_eq!(fancy.extras["vrml:node"]["type"], "FancyNode");
    assert!(fancy.mesh.is_some());
    // Text geometry is recorded as unsupported.
    assert!(s.extras.contains_key("vrml:unsupportedGeometry"));

    // All choices / levels on request; Inline followed by a resolver.
    let opts = ConvertOptions {
        all_switch_choices: true,
        all_lod_levels: true,
        resolver: Some(Arc::new(MapResolver)),
        ..ConvertOptions::default()
    };
    let s2 = VrmlDecoder::new()
        .with_options(opts)
        .decode_scene(&fixture("grouping.wrl"))
        .unwrap();
    assert!(s2.validate().is_ok());
    let with2 = |k: &str| s2.nodes.iter().find(|n| n.extras.contains_key(k)).unwrap();
    assert_eq!(with2("vrml:switch").children.len(), 2);
    assert_eq!(with2("vrml:lod").children.len(), 2);
    let inl = with2("vrml:inline");
    assert_eq!(inl.children.len(), 1);
    let child = &s2.nodes[inl.children[0].0 as usize];
    assert!(
        matches!(child.transform, Transform::Trs { translation, .. } if translation == [0.0, 1.0, 0.0])
    );
}

#[test]
fn gzip_input() {
    let raw = fixture("primitives.wrl");
    let gz = compcol::vec::compress_to_vec::<compcol::gzip::Gzip>(&raw).unwrap();
    let a = VrmlDecoder::new().decode_scene(&raw).unwrap();
    let b = VrmlDecoder::new().decode_scene(&gz).unwrap();
    assert_eq!(a.nodes.len(), b.nodes.len());
    assert_eq!(a.meshes.len(), b.meshes.len());
}

#[test]
fn rejects_other_headers() {
    let d = VrmlDecoder::new();
    assert!(matches!(
        d.decode_scene(b"#VRML V1.0 ascii\nSeparator {}"),
        Err(oxideav_vrml::Error::Unsupported(_))
    ));
    assert!(d.decode_scene(b"solid x\nendsolid").is_err());
    assert!(d.decode_scene(b"").is_err());
    // Latin-1 bytes in a comment / string are tolerated.
    let s = d
        .decode_scene(b"#VRML V2.0 utf8\nWorldInfo { title \"caf\xe9\" }")
        .unwrap();
    assert_eq!(
        s.extras["vrml:worldInfo"][0]["fields"]["title"],
        "caf\u{e9}"
    );
}

struct LibResolver;

impl UrlResolver for LibResolver {
    fn resolve(&self, url: &str) -> Option<Vec<u8>> {
        (url == "lib.wrl").then(|| fixture("lib.wrl"))
    }
}

#[test]
fn externproto_through_resolver() {
    let s = VrmlDecoder::new()
        .with_resolver(Arc::new(LibResolver))
        .decode_scene(&fixture("protos.wrl"))
        .unwrap();
    assert!(s.validate().is_ok());
    assert!(!s.nodes.iter().any(|n| n.extras.contains_key("vrml:node")));
    // Remote { weight 2 } → Sphere radius 2.
    let big = s.meshes.iter().any(|m| {
        m.bounding_box()
            .is_some_and(|b| (b.max[1] - 2.0).abs() < 1e-5 && (b.min[1] + 2.0).abs() < 1e-5)
    });
    assert!(big);
}
