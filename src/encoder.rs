//! [`VrmlEncoder`] — [`Scene3D`] → VRML97 text.
//!
//! The scene is first lowered to a [`Document`] ([`scene_to_document`])
//! and then serialised by the generic [writer](crate::syntax::writer):
//!
//! * scene nodes → `Transform` (or `Group` for identity transforms,
//!   `Anchor` / `Billboard` / `Collision` / `Switch` / `LOD` / `Inline`
//!   when the decoder's `vrml:*` extras say so); a camera-only node
//!   becomes a `Viewpoint`, a light-only node a `PointLight` /
//!   `SpotLight`; directional lights are hoisted to the root with
//!   their world-space direction (VRML scopes them to their siblings);
//! * each mesh primitive → one `Shape` (`IndexedFaceSet` with explicit
//!   normals / texture coordinates / colours, `IndexedLineSet`,
//!   `PointSet`); meshes and materials referenced more than once are
//!   written once with `DEF` and re-used with `USE`;
//! * materials → `Appearance` / `Material` (the decoder's original
//!   Phong values are restored from `vrml:material` extras when
//!   present), textures → `ImageTexture` (URI), `PixelTexture`
//!   (embedded PNG) or an `ImageTexture` with a `data:` URI (other
//!   embedded images);
//! * animations → one `TimeSensor` per animation driving
//!   `PositionInterpolator` / `OrientationInterpolator` /
//!   `CoordinateInterpolator` nodes through `ROUTE`s (step
//!   interpolation becomes duplicated keys, cubic splines keep their
//!   key values).

use std::collections::{HashMap, HashSet};

use oxideav_mesh3d::{
    AlphaMode, AnimationProperty, AnimationValues, Camera, ImageData, Interpolation, Light,
    Material, Mesh3DEncoder, NodeId as SceneNodeId, Primitive, Scene3D, Topology, Transform,
    WrapMode,
};
use serde_json::Value;

use crate::ast::{
    Document, FieldData, FieldType, FieldValue, Header, Image, Node, NodeId, Route, Scalar,
    Statement,
};
use crate::convert::image::{data_uri, png_to_image};
use crate::convert::math::{quat_rotate, quat_to_axis_angle};
use crate::convert::roughness_to_shininess;
use crate::error::{Error, Result};
use crate::syntax::catalog::vrml97_node;
use crate::syntax::{write_document_with, WriteOptions};

/// How embedded (non-URI) textures are written.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TextureEmbedding {
    /// PNG payloads become `PixelTexture` (standard VRML97); other
    /// formats an `ImageTexture` with a `data:` URI.
    #[default]
    PixelTexture,
    /// Every embedded payload becomes an `ImageTexture` with a base64
    /// `data:` URI (compact, but not every VRML97 browser resolves
    /// `data:` URLs).
    DataUri,
    /// Embedded textures are dropped (only URI textures are written).
    Skip,
}

/// Encoder options.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EncodeOptions {
    /// Embedded-texture policy.
    pub textures: TextureEmbedding,
    /// gzip-compress the output (`.wrz`).
    pub gzip: bool,
    /// Header comment (after `#VRML V2.0 utf8`).
    pub header_comment: String,
    /// Writer formatting.
    pub write: WriteOptions,
}

impl Default for EncodeOptions {
    fn default() -> Self {
        Self {
            textures: TextureEmbedding::default(),
            gzip: false,
            header_comment: "oxideav-vrml".into(),
            write: WriteOptions::default(),
        }
    }
}

/// VRML97 encoder.
#[derive(Debug, Clone, Default)]
pub struct VrmlEncoder {
    options: EncodeOptions,
}

impl VrmlEncoder {
    /// Encoder with default options (plain text, PixelTexture embedding).
    pub fn new() -> Self {
        Self::default()
    }

    /// Encoder with explicit options.
    pub fn with_options(options: EncodeOptions) -> Self {
        Self { options }
    }

    /// Encoder producing gzip-compressed output (`.wrz`).
    pub fn gzip() -> Self {
        Self::with_options(EncodeOptions {
            gzip: true,
            ..EncodeOptions::default()
        })
    }

    /// Encode with the crate-local error type.
    pub fn encode_scene(&self, scene: &Scene3D) -> Result<Vec<u8>> {
        let doc = scene_to_document(scene, &self.options)?;
        let text = write_document_with(&doc, &self.options.write);
        if self.options.gzip {
            compcol::vec::compress_to_vec::<compcol::gzip::Gzip>(text.as_bytes())
                .map_err(|e| Error::invalid(format!("gzip deflate failed: {e}")))
        } else {
            Ok(text.into_bytes())
        }
    }
}

impl Mesh3DEncoder for VrmlEncoder {
    fn encode(&mut self, scene: &Scene3D) -> oxideav_mesh3d::Result<Vec<u8>> {
        self.encode_scene(scene).map_err(Error::into_mesh3d)
    }
}

/// Lower a [`Scene3D`] to a VRML97 [`Document`].
pub fn scene_to_document(scene: &Scene3D, opts: &EncodeOptions) -> Result<Document> {
    let mut e = Enc::new(scene, opts);
    e.run()?;
    Ok(e.doc)
}

/// Make `s` a valid VRML Id (§4.3.1 / Annex A `Id`).
pub fn sanitize_id(s: &str) -> String {
    let mut out: String = s
        .chars()
        .map(|c| {
            if (c as u32) <= 0x20
                || matches!(
                    c,
                    '"' | '#' | '\'' | ',' | '.' | '[' | '\\' | ']' | '{' | '}' | '\u{7f}'
                )
            {
                '_'
            } else {
                c
            }
        })
        .collect();
    if out.is_empty() || out.starts_with(|c: char| c.is_ascii_digit() || c == '+' || c == '-') {
        out.insert(0, '_');
    }
    if matches!(
        out.as_str(),
        "DEF"
            | "EXTERNPROTO"
            | "FALSE"
            | "IS"
            | "NULL"
            | "PROTO"
            | "ROUTE"
            | "TO"
            | "TRUE"
            | "USE"
            | "eventIn"
            | "eventOut"
            | "exposedField"
            | "field"
    ) {
        out.push('_');
    }
    out
}

struct Enc<'s> {
    scene: &'s Scene3D,
    opts: &'s EncodeOptions,
    doc: Document,
    used_names: HashSet<String>,
    /// Scene node → DEF name (animation targets and named nodes).
    node_names: HashMap<u32, String>,
    /// Scene node → emitted VRML node (Transform / Viewpoint / …).
    emitted: HashMap<u32, NodeId>,
    /// Scene node → kind of emitted node (for route field names).
    emitted_kind: HashMap<u32, &'static str>,
    /// Mesh → shapes (shared through USE).
    shapes: HashMap<u32, Vec<NodeId>>,
    /// Mesh → per-primitive Coordinate node (for morph animation).
    coords: HashMap<u32, Vec<Option<NodeId>>>,
    appearances: HashMap<(u32, bool), NodeId>,
    textures: HashMap<u32, Option<NodeId>>,
    directional: Vec<NodeId>,
    statements_tail: Vec<Statement>,
}

impl<'s> Enc<'s> {
    fn new(scene: &'s Scene3D, opts: &'s EncodeOptions) -> Self {
        let mut doc = Document::new();
        doc.header = Header {
            comment: opts.header_comment.clone(),
            ..Header::vrml97()
        };
        Self {
            scene,
            opts,
            doc,
            used_names: HashSet::new(),
            node_names: HashMap::new(),
            emitted: HashMap::new(),
            emitted_kind: HashMap::new(),
            shapes: HashMap::new(),
            coords: HashMap::new(),
            appearances: HashMap::new(),
            textures: HashMap::new(),
            directional: Vec::new(),
            statements_tail: Vec::new(),
        }
    }

    fn unique(&mut self, base: &str) -> String {
        let base = sanitize_id(base);
        let mut name = base.clone();
        let mut k = 1;
        while self.used_names.contains(&name) {
            name = format!("{base}_{k}");
            k += 1;
        }
        self.used_names.insert(name.clone());
        name
    }

    fn add(&mut self, n: Node) -> NodeId {
        self.doc.add_node(n)
    }

    fn run(&mut self) -> Result<()> {
        // DEF names: every named node plus every animation target.
        let mut targets: HashSet<u32> = HashSet::new();
        for a in &self.scene.animations {
            for c in &a.channels {
                targets.insert(c.target.node.0);
            }
        }
        for (i, n) in self.scene.nodes.iter().enumerate() {
            if n.name.is_some() || targets.contains(&(i as u32)) {
                let base = n.name.clone().unwrap_or_else(|| format!("N{i}"));
                let name = self.unique(&base);
                self.node_names.insert(i as u32, name);
            }
        }
        // Environment nodes preserved by the decoder.
        let mut stmts = Vec::new();
        for key in [
            "vrml:worldInfo",
            "vrml:navigationInfo",
            "vrml:background",
            "vrml:fog",
        ] {
            if let Some(Value::Array(items)) = self.scene.extras.get(key) {
                for v in items {
                    if let Some(id) = self.json_node(v, 0) {
                        stmts.push(Statement::Node(id));
                    }
                }
            }
        }
        let world = self.scene.world_node_transforms();
        let mut visiting = Vec::new();
        for &r in &self.scene.roots {
            if let Some(id) = self.emit_node(r, &world, &mut visiting)? {
                stmts.push(Statement::Node(id));
            }
        }
        for d in std::mem::take(&mut self.directional) {
            stmts.push(Statement::Node(d));
        }
        self.animations(&mut stmts);
        stmts.append(&mut self.statements_tail);
        self.doc.statements = stmts;
        Ok(())
    }

    fn emit_node(
        &mut self,
        sid: SceneNodeId,
        world: &[Option<[[f32; 4]; 4]>],
        visiting: &mut Vec<u32>,
    ) -> Result<Option<NodeId>> {
        let i = sid.0;
        let Some(sn) = self.scene.nodes.get(i as usize) else {
            return Ok(None);
        };
        if visiting.contains(&i) || visiting.len() > 256 {
            return Ok(None);
        }
        visiting.push(i);
        let (t, r, s) = trs(&sn.transform);
        let identity = t == [0.0; 3] && r == [0.0, 0.0, 0.0, 1.0] && s == [1.0; 3];
        let name = self.node_names.get(&i).cloned();
        let bare = sn.mesh.is_none() && sn.children.is_empty();

        // Directional lights: hoisted to the root, world direction.
        if let Some(lid) = sn.light {
            if let Some(Light::Directional { color, intensity }) =
                self.scene.lights.get(lid.0 as usize)
            {
                let dir = match world.get(i as usize).copied().flatten() {
                    Some(m) => [-m[0][2], -m[1][2], -m[2][2]],
                    None => quat_rotate(r, [0.0, 0.0, -1.0]),
                };
                let mut n = Node::new("DirectionalLight");
                n.def_name = name.clone().filter(|_| bare);
                self.light_common(&mut n, sn, *color, *intensity);
                n.set_field("direction", FieldValue::sf_vec3f(normalize3(dir)));
                let id = self.add(n);
                self.directional.push(id);
                if bare && sn.camera.is_none() {
                    visiting.pop();
                    return Ok(None);
                }
            }
        }

        // Camera-only node → Viewpoint (position / orientation carry
        // the node TRS).
        if let (Some(cid), true, None) = (sn.camera, bare, sn.light) {
            if let Some(cam) = self.scene.cameras.get(cid.0 as usize) {
                let mut n = Node::new("Viewpoint");
                n.def_name = name.clone();
                n.set_field("position", FieldValue::sf_vec3f(t));
                n.set_field(
                    "orientation",
                    FieldValue::sf_rotation(quat_to_axis_angle(r)),
                );
                n.set_field("fieldOfView", FieldValue::sf_float(camera_fov(cam)));
                if let Some(desc) = sn
                    .extras
                    .get("vrml:viewpoint")
                    .and_then(|v| v.get("description"))
                    .and_then(Value::as_str)
                    .filter(|d| !d.is_empty())
                    .or(sn.name.as_deref())
                {
                    n.set_field("description", FieldValue::sf_string(desc));
                }
                let id = self.add(n);
                self.emitted.insert(i, id);
                self.emitted_kind.insert(i, "Viewpoint");
                visiting.pop();
                return Ok(Some(id));
            }
        }

        // Point / spot light-only node → light with location/direction.
        if let (Some(lid), true, None) = (sn.light, bare, sn.camera) {
            match self.scene.lights.get(lid.0 as usize) {
                Some(Light::Point {
                    color,
                    intensity,
                    range,
                }) => {
                    let mut n = Node::new("PointLight");
                    n.def_name = name.clone();
                    self.light_common(&mut n, sn, *color, *intensity);
                    n.set_field("location", FieldValue::sf_vec3f(t));
                    self.light_range(&mut n, sn, *range);
                    visiting.pop();
                    return Ok(Some(self.add(n)));
                }
                Some(Light::Spot {
                    color,
                    intensity,
                    range,
                    inner_cone_angle,
                    outer_cone_angle,
                }) => {
                    let mut n = Node::new("SpotLight");
                    n.def_name = name.clone();
                    self.light_common(&mut n, sn, *color, *intensity);
                    n.set_field("location", FieldValue::sf_vec3f(t));
                    n.set_field(
                        "direction",
                        FieldValue::sf_vec3f(normalize3(quat_rotate(r, [0.0, 0.0, -1.0]))),
                    );
                    n.set_field("beamWidth", FieldValue::sf_float(*inner_cone_angle));
                    n.set_field("cutOffAngle", FieldValue::sf_float(*outer_cone_angle));
                    self.light_range(&mut n, sn, *range);
                    visiting.pop();
                    return Ok(Some(self.add(n)));
                }
                _ => {
                    visiting.pop();
                    return Ok(None);
                }
            }
        }

        // General case: grouping node.
        let mut kids: Vec<NodeId> = Vec::new();
        if let Some(mid) = sn.mesh {
            kids.extend(self.mesh_shapes(mid.0, sn.weights.as_slice())?);
        }
        if let Some(cid) = sn.camera {
            if let Some(cam) = self.scene.cameras.get(cid.0 as usize) {
                let mut v = Node::new("Viewpoint");
                v.set_field("position", FieldValue::sf_vec3f([0.0; 3]));
                v.set_field("fieldOfView", FieldValue::sf_float(camera_fov(cam)));
                kids.push(self.add(v));
            }
        }
        if let Some(lid) = sn.light {
            match self.scene.lights.get(lid.0 as usize).copied() {
                Some(Light::Point {
                    color,
                    intensity,
                    range,
                }) => {
                    let mut n = Node::new("PointLight");
                    self.light_common(&mut n, sn, color, intensity);
                    self.light_range(&mut n, sn, range);
                    kids.push(self.add(n));
                }
                Some(Light::Spot {
                    color,
                    intensity,
                    range,
                    inner_cone_angle,
                    outer_cone_angle,
                }) => {
                    let mut n = Node::new("SpotLight");
                    self.light_common(&mut n, sn, color, intensity);
                    n.set_field("beamWidth", FieldValue::sf_float(inner_cone_angle));
                    n.set_field("cutOffAngle", FieldValue::sf_float(outer_cone_angle));
                    self.light_range(&mut n, sn, range);
                    kids.push(self.add(n));
                }
                _ => {}
            }
        }
        let mut child_ids = Vec::new();
        for &c in &sn.children {
            if let Some(id) = self.emit_node(c, world, visiting)? {
                child_ids.push((c, id));
            }
        }

        // Wrapper semantics recorded by the decoder.
        let ex = &sn.extras;
        let mut wrapper: Option<Node> = None;
        if let Some(sw) = ex.get("vrml:switch") {
            let mut n = Node::new("Switch");
            let mut which = -1i32;
            let mut choice = Vec::new();
            for (k, (c, id)) in child_ids.iter().enumerate() {
                let inactive = self.scene.nodes[c.0 as usize]
                    .extras
                    .get("vrml:inactive")
                    .and_then(Value::as_bool)
                    .unwrap_or(false);
                if !inactive && which < 0 {
                    which = k as i32;
                }
                choice.push(*id);
            }
            let _ = sw;
            // Mesh shapes of the Switch node itself follow the choices.
            choice.append(&mut kids);
            n.set_field("choice", FieldValue::mf_node(choice));
            n.set_field("whichChoice", FieldValue::sf_int32(which));
            wrapper = Some(n);
        } else if let Some(lod) = ex.get("vrml:lod") {
            let mut n = Node::new("LOD");
            let mut level: Vec<NodeId> = child_ids.iter().map(|(_, id)| *id).collect();
            level.append(&mut kids);
            let range: Vec<f32> = lod
                .get("range")
                .and_then(Value::as_array)
                .map(|a| {
                    a.iter()
                        .filter_map(Value::as_f64)
                        .map(|f| f as f32)
                        .collect()
                })
                .unwrap_or_default();
            let range: Vec<f32> = range
                .into_iter()
                .take(level.len().saturating_sub(1))
                .collect();
            n.set_field("level", FieldValue::mf_node(level));
            if let Some(c) = json_vec3(lod.get("center")) {
                n.set_field("center", FieldValue::sf_vec3f(c));
            }
            n.set_field("range", FieldValue::mf_float(range));
            wrapper = Some(n);
        } else {
            kids.extend(child_ids.iter().map(|(_, id)| *id));
            if let Some(a) = ex.get("vrml:anchor") {
                let mut n = Node::new("Anchor");
                n.set_field("url", FieldValue::mf_string(json_strings(a.get("url"))));
                let d = a.get("description").and_then(Value::as_str).unwrap_or("");
                if !d.is_empty() {
                    n.set_field("description", FieldValue::sf_string(d));
                }
                let p = json_strings(a.get("parameter"));
                if !p.is_empty() {
                    n.set_field("parameter", FieldValue::mf_string(p));
                }
                wrapper = Some(n);
            } else if let Some(b) = ex.get("vrml:billboard") {
                let mut n = Node::new("Billboard");
                if let Some(axis) = json_vec3(b.get("axisOfRotation")) {
                    n.set_field("axisOfRotation", FieldValue::sf_vec3f(axis));
                }
                wrapper = Some(n);
            } else if let Some(c) = ex.get("vrml:collision") {
                let mut n = Node::new("Collision");
                if let Some(false) = c.get("collide").and_then(Value::as_bool) {
                    n.set_field("collide", FieldValue::sf_bool(false));
                }
                wrapper = Some(n);
            } else if let Some(inl) = ex.get("vrml:inline") {
                if ex.get("vrml:inlineResolved").is_none() && kids.is_empty() {
                    let mut n = Node::new("Inline");
                    n.set_field("url", FieldValue::mf_string(json_strings(inl.get("url"))));
                    wrapper = Some(n);
                }
            }
        }
        if let Some(w) = &mut wrapper {
            if !matches!(w.type_name.as_str(), "Switch" | "LOD" | "Inline") {
                w.set_field("children", FieldValue::mf_node(std::mem::take(&mut kids)));
            }
        }

        let animated = self.node_names.contains_key(&i)
            && self
                .scene
                .animations
                .iter()
                .any(|a| a.channels.iter().any(|c| c.target.node.0 == i));
        let vrml_tf = ex.get("vrml:transform");
        let node = if identity && !animated && vrml_tf.is_none() {
            match wrapper {
                Some(mut w) => {
                    w.def_name = name;
                    w
                }
                None => {
                    let mut g = Node::new("Group");
                    g.def_name = name;
                    g.set_field("children", FieldValue::mf_node(kids));
                    g
                }
            }
        } else {
            let mut tf = Node::new("Transform");
            tf.def_name = name;
            if let (Some(v), Transform::Matrix(_)) = (vrml_tf, &sn.transform) {
                // Restore the decoder's original center / scaleOrientation form.
                for k in ["translation", "center", "scale"] {
                    if let Some(x) = json_vec3(v.get(k)) {
                        tf.set_field(k, FieldValue::sf_vec3f(x));
                    }
                }
                for k in ["rotation", "scaleOrientation"] {
                    if let Some(x) = json_vec4(v.get(k)) {
                        tf.set_field(k, FieldValue::sf_rotation(x));
                    }
                }
            } else {
                if t != [0.0; 3] || animated {
                    tf.set_field("translation", FieldValue::sf_vec3f(t));
                }
                if r != [0.0, 0.0, 0.0, 1.0] || animated {
                    tf.set_field("rotation", FieldValue::sf_rotation(quat_to_axis_angle(r)));
                }
                if s != [1.0; 3] || animated {
                    tf.set_field("scale", FieldValue::sf_vec3f(s));
                }
            }
            let inner = match wrapper {
                Some(w) => vec![self.add(w)],
                None => kids,
            };
            tf.set_field("children", FieldValue::mf_node(inner));
            tf
        };
        let kind = if node.type_name == "Transform" {
            "Transform"
        } else {
            "Group"
        };
        let id = self.add(node);
        self.emitted.insert(i, id);
        self.emitted_kind.insert(i, kind);
        visiting.pop();
        Ok(Some(id))
    }

    fn light_common(
        &self,
        n: &mut Node,
        sn: &oxideav_mesh3d::Node,
        color: [f32; 3],
        intensity: f32,
    ) {
        let info = sn.extras.get("vrml:light");
        let on = info
            .and_then(|v| v.get("on"))
            .and_then(Value::as_bool)
            .unwrap_or(true);
        let raw = if on {
            intensity
        } else {
            info.and_then(|v| v.get("intensity"))
                .and_then(Value::as_f64)
                .map(|f| f as f32)
                .unwrap_or(intensity)
        };
        n.set_field(
            "color",
            FieldValue::sf_color(color.map(|c| c.clamp(0.0, 1.0))),
        );
        n.set_field("intensity", FieldValue::sf_float(raw.clamp(0.0, 1.0)));
        if !on {
            n.set_field("on", FieldValue::sf_bool(false));
        }
        if let Some(a) = info
            .and_then(|v| v.get("ambientIntensity"))
            .and_then(Value::as_f64)
            .filter(|a| *a != 0.0)
        {
            n.set_field("ambientIntensity", FieldValue::sf_float(a as f32));
        }
        if n.type_name != "DirectionalLight" {
            if let Some(att) = json_vec3(info.and_then(|v| v.get("attenuation"))) {
                n.set_field("attenuation", FieldValue::sf_vec3f(att));
            }
        }
    }

    fn light_range(&self, n: &mut Node, sn: &oxideav_mesh3d::Node, range: Option<f32>) {
        let radius = range.or_else(|| {
            sn.extras
                .get("vrml:light")
                .and_then(|v| v.get("radius"))
                .and_then(Value::as_f64)
                .map(|f| f as f32)
        });
        // glTF "no range" = unlimited; VRML needs a finite radius.
        let radius = radius
            .filter(|r| r.is_finite() && *r >= 0.0)
            .unwrap_or(1.0e6);
        n.set_field("radius", FieldValue::sf_float(radius));
    }

    /// Shapes for a mesh (built once, re-used through USE).
    fn mesh_shapes(&mut self, mid: u32, node_weights: &[f32]) -> Result<Vec<NodeId>> {
        if let Some(s) = self.shapes.get(&mid) {
            return Ok(s.clone());
        }
        let scene = self.scene;
        let Some(mesh) = scene.meshes.get(mid as usize) else {
            return Ok(Vec::new());
        };
        let weights = if node_weights.is_empty() {
            mesh.weights.as_slice()
        } else {
            node_weights
        };
        let shared = scene
            .nodes
            .iter()
            .filter(|n| n.mesh.map(|m| m.0) == Some(mid))
            .count()
            > 1;
        let morph_animated = scene.animations.iter().any(|a| {
            a.channels.iter().any(|c| {
                c.target.property == AnimationProperty::MorphWeights
                    && scene
                        .nodes
                        .get(c.target.node.0 as usize)
                        .and_then(|n| n.mesh)
                        == Some(oxideav_mesh3d::MeshId(mid))
            })
        });
        let mut shapes = Vec::new();
        let mut coords = Vec::new();
        for (pi, prim) in mesh.primitives.iter().enumerate() {
            let (geom, coord) = self.geometry(prim, weights, morph_animated)?;
            let Some(geom) = geom else {
                coords.push(None);
                continue;
            };
            let mut shape = Node::new("Shape");
            if shared {
                let base = match &mesh.name {
                    Some(n) if mesh.primitives.len() == 1 => n.clone(),
                    Some(n) => format!("{n}_{pi}"),
                    None => format!("MESH{mid}_{pi}"),
                };
                shape.def_name = Some(self.unique(&base));
            }
            let lines_or_points = !matches!(
                prim.topology,
                Topology::Triangles | Topology::TriangleStrip | Topology::TriangleFan
            );
            if let Some(app) = self.appearance(prim, lines_or_points)? {
                shape.set_field("appearance", FieldValue::sf_node(Some(app)));
            }
            shape.set_field("geometry", FieldValue::sf_node(Some(geom)));
            shapes.push(self.add(shape));
            coords.push(coord);
        }
        self.shapes.insert(mid, shapes.clone());
        self.coords.insert(mid, coords);
        Ok(shapes)
    }

    /// Geometry node for one primitive (+ its Coordinate node).
    fn geometry(
        &mut self,
        prim: &Primitive,
        weights: &[f32],
        name_coord: bool,
    ) -> Result<(Option<NodeId>, Option<NodeId>)> {
        let positions: Vec<[f32; 3]> =
            if !prim.targets.is_empty() && weights.iter().any(|w| *w != 0.0) {
                prim.apply_morph_weights(weights).positions
            } else {
                prim.positions.clone()
            };
        if positions.is_empty() {
            return Ok((None, None));
        }
        let mut coord = Node::new("Coordinate");
        coord.set_field("point", FieldValue::mf_vec3f(&positions));
        if name_coord {
            coord.def_name = Some(self.unique("COORD"));
        }
        let coord_id = self.add(coord);
        let seq: Vec<u32> = match &prim.indices {
            Some(oxideav_mesh3d::Indices::U16(v)) => v.iter().map(|&i| i as u32).collect(),
            Some(oxideav_mesh3d::Indices::U32(v)) => v.clone(),
            None => (0..positions.len() as u32).collect(),
        };
        let n = positions.len() as u32;
        let colors = prim.colors.first().filter(|c| c.len() == positions.len());
        // A VRML Color node *replaces* the diffuse colour (§4.14.3)
        // while glTF COLOR_0 multiplies the base colour: bake the factor.
        let base = prim
            .material
            .and_then(|m| self.scene.materials.get(m.0 as usize))
            .filter(|m| !m.extras.contains_key("vrml:material"))
            .map(|m| [m.base_color[0], m.base_color[1], m.base_color[2]])
            .unwrap_or([1.0; 3]);
        let color_node = colors.map(|c| {
            let rgb: Vec<[f32; 3]> = c
                .iter()
                .map(|v| [v[0] * base[0], v[1] * base[1], v[2] * base[2]])
                .collect();
            let mut node = Node::new("Color");
            node.set_field("color", FieldValue::mf_color(&rgb));
            node
        });
        let geom = match prim.topology {
            Topology::Triangles | Topology::TriangleStrip | Topology::TriangleFan => {
                let mut index = Vec::new();
                for t in prim.triangle_indices() {
                    if t.iter().any(|&i| i >= n) {
                        continue;
                    }
                    index.extend([t[0] as i32, t[1] as i32, t[2] as i32, -1]);
                }
                let mut ifs = Node::new("IndexedFaceSet");
                ifs.set_field("coord", FieldValue::sf_node(Some(coord_id)));
                ifs.set_field("coordIndex", FieldValue::mf_int32(index));
                if let Some(normals) = prim.normals.as_ref().filter(|v| v.len() == positions.len())
                {
                    let mut nn = Node::new("Normal");
                    nn.set_field("vector", FieldValue::mf_vec3f(normals));
                    let nid = self.add(nn);
                    ifs.set_field("normal", FieldValue::sf_node(Some(nid)));
                }
                let mat = prim
                    .material
                    .and_then(|m| self.scene.materials.get(m.0 as usize));
                if let Some(uvs) = self.uv_channel(prim, mat) {
                    if uvs.len() == positions.len() {
                        let st: Vec<[f32; 2]> = uvs.iter().map(|uv| [uv[0], 1.0 - uv[1]]).collect();
                        let mut tc = Node::new("TextureCoordinate");
                        tc.set_field("point", FieldValue::mf_vec2f(&st));
                        let tid = self.add(tc);
                        ifs.set_field("texCoord", FieldValue::sf_node(Some(tid)));
                    }
                }
                if let Some(c) = color_node {
                    let cid = self.add(c);
                    ifs.set_field("color", FieldValue::sf_node(Some(cid)));
                }
                let double = mat.is_some_and(|m| m.double_sided);
                if double {
                    ifs.set_field("solid", FieldValue::sf_bool(false));
                }
                ifs
            }
            Topology::Lines | Topology::LineStrip | Topology::LineLoop => {
                let mut index = Vec::new();
                let valid: Vec<i32> = seq.iter().filter(|&&i| i < n).map(|&i| i as i32).collect();
                match prim.topology {
                    Topology::Lines => {
                        for p in valid.chunks_exact(2) {
                            index.extend([p[0], p[1], -1]);
                        }
                    }
                    _ => {
                        if valid.len() >= 2 {
                            index.extend(valid.iter().copied());
                            if prim.topology == Topology::LineLoop {
                                index.push(valid[0]);
                            }
                            index.push(-1);
                        }
                    }
                }
                let mut ils = Node::new("IndexedLineSet");
                ils.set_field("coord", FieldValue::sf_node(Some(coord_id)));
                ils.set_field("coordIndex", FieldValue::mf_int32(index));
                if let Some(c) = color_node {
                    let cid = self.add(c);
                    ils.set_field("color", FieldValue::sf_node(Some(cid)));
                }
                ils
            }
            Topology::Points => {
                let mut ps = Node::new("PointSet");
                ps.set_field("coord", FieldValue::sf_node(Some(coord_id)));
                if let Some(c) = color_node {
                    let cid = self.add(c);
                    ps.set_field("color", FieldValue::sf_node(Some(cid)));
                }
                ps
            }
        };
        Ok((Some(self.add(geom)), Some(coord_id)))
    }

    /// UVs of the base-colour texture's set with its
    /// KHR_texture_transform baked in.
    fn uv_channel(&self, prim: &Primitive, mat: Option<&Material>) -> Option<Vec<[f32; 2]>> {
        let tref = mat.and_then(|m| m.base_color_texture.as_ref());
        let set = tref.map(|t| t.effective_uv_set()).unwrap_or(0) as usize;
        let uvs = prim.uvs.get(set).or(prim.uvs.first())?;
        Some(match tref.and_then(|t| t.transform) {
            Some(tt) => uvs.iter().map(|uv| tt.apply(*uv)).collect(),
            None => uvs.clone(),
        })
    }

    fn appearance(&mut self, prim: &Primitive, lines_or_points: bool) -> Result<Option<NodeId>> {
        let Some(mid) = prim.material else {
            return Ok(None);
        };
        let key = (mid.0, lines_or_points);
        if let Some(a) = self.appearances.get(&key) {
            return Ok(Some(*a));
        }
        let scene = self.scene;
        let Some(m) = scene.materials.get(mid.0 as usize) else {
            return Ok(None);
        };
        let shared = scene
            .meshes
            .iter()
            .flat_map(|me| me.primitives.iter())
            .filter(|p| p.material == Some(mid))
            .count()
            > 1;
        let mut app = Node::new("Appearance");
        if shared {
            let base = m
                .extras
                .get("vrml:appearance")
                .and_then(Value::as_str)
                .map(str::to_owned)
                .or_else(|| m.name.clone().map(|n| format!("{n}_app")))
                .unwrap_or_else(|| format!("APP{}", mid.0));
            app.def_name = Some(self.unique(&base));
        }
        let tex = match m.base_color_texture {
            Some(t) => self.texture(t.texture.0)?,
            None => None,
        };
        let skip_material = m.ext.unlit
            && m.base_color[..3] == [1.0, 1.0, 1.0]
            && !m.extras.contains_key("vrml:material");
        if !skip_material {
            let mut mn = Node::new("Material");
            if let Some(name) = &m.name {
                mn.def_name = Some(self.unique(name));
            }
            if let Some(orig) = m.extras.get("vrml:material") {
                for (k, v) in [
                    ("diffuseColor", orig.get("diffuseColor")),
                    ("emissiveColor", orig.get("emissiveColor")),
                    ("specularColor", orig.get("specularColor")),
                ] {
                    if let Some(c) = json_vec3(v) {
                        mn.set_field(k, FieldValue::sf_color(c));
                    }
                }
                for k in ["ambientIntensity", "shininess", "transparency"] {
                    if let Some(f) = orig.get(k).and_then(Value::as_f64) {
                        mn.set_field(k, FieldValue::sf_float(f as f32));
                    }
                }
            } else {
                let rgb =
                    [m.base_color[0], m.base_color[1], m.base_color[2]].map(|c| c.clamp(0.0, 1.0));
                let alpha = if m.alpha_mode == AlphaMode::Opaque {
                    1.0
                } else {
                    m.base_color[3].clamp(0.0, 1.0)
                };
                let strength = m.ext.emissive_strength.unwrap_or(1.0);
                let emissive = m.emissive_factor.map(|c| (c * strength).clamp(0.0, 1.0));
                if m.ext.unlit || lines_or_points {
                    mn.set_field("diffuseColor", FieldValue::sf_color(rgb));
                    mn.set_field("emissiveColor", FieldValue::sf_color(rgb));
                } else {
                    mn.set_field("diffuseColor", FieldValue::sf_color(rgb));
                    if emissive != [0.0; 3] {
                        mn.set_field("emissiveColor", FieldValue::sf_color(emissive));
                    }
                    mn.set_field(
                        "shininess",
                        FieldValue::sf_float(roughness_to_shininess(m.roughness)),
                    );
                    let spec = rgb.map(|c| 0.04 * (1.0 - m.metallic) + c * m.metallic);
                    mn.set_field("specularColor", FieldValue::sf_color(spec));
                }
                if alpha < 1.0 {
                    mn.set_field("transparency", FieldValue::sf_float(1.0 - alpha));
                }
            }
            let mid_node = self.add(mn);
            app.set_field("material", FieldValue::sf_node(Some(mid_node)));
        }
        if let Some(t) = tex {
            app.set_field("texture", FieldValue::sf_node(Some(t)));
        }
        let id = self.add(app);
        self.appearances.insert(key, id);
        Ok(Some(id))
    }

    fn texture(&mut self, tid: u32) -> Result<Option<NodeId>> {
        if let Some(t) = self.textures.get(&tid) {
            return Ok(*t);
        }
        let scene = self.scene;
        let Some(tex) = scene.textures.get(tid as usize) else {
            return Ok(None);
        };
        let mut node = match &tex.image {
            ImageData::External { uri, .. } => {
                let mut n = Node::new("ImageTexture");
                n.set_field("url", FieldValue::mf_string(vec![uri.clone()]));
                Some(n)
            }
            ImageData::Source(src) => {
                let mut bytes = Vec::new();
                let ok = src
                    .open()
                    .and_then(|mut r| std::io::Read::read_to_end(&mut r, &mut bytes))
                    .is_ok();
                let mime = src.mime().unwrap_or("application/octet-stream").to_owned();
                match (ok, self.opts.textures) {
                    (false, _) | (_, TextureEmbedding::Skip) => None,
                    (true, TextureEmbedding::PixelTexture) if png_to_image(&bytes).is_some() => {
                        let img = png_to_image(&bytes).unwrap_or_default();
                        let mut n = Node::new("PixelTexture");
                        n.set_field("image", FieldValue::sf_image(img));
                        Some(n)
                    }
                    (true, _) => {
                        let mut n = Node::new("ImageTexture");
                        n.set_field("url", FieldValue::mf_string(vec![data_uri(&mime, &bytes)]));
                        Some(n)
                    }
                }
            }
            #[cfg(feature = "registry")]
            ImageData::Embedded(_) => None,
        };
        if let Some(n) = &mut node {
            if tex.sampler.wrap_s == WrapMode::ClampToEdge {
                n.set_field("repeatS", FieldValue::sf_bool(false));
            }
            if tex.sampler.wrap_t == WrapMode::ClampToEdge {
                n.set_field("repeatT", FieldValue::sf_bool(false));
            }
            if let Some(name) = &tex.name {
                n.def_name = Some(self.unique(name));
            }
        }
        let id = node.map(|n| self.add(n));
        self.textures.insert(tid, id);
        Ok(id)
    }

    fn animations(&mut self, stmts: &mut Vec<Statement>) {
        let scene = self.scene;
        for (ai, anim) in scene.animations.iter().enumerate() {
            let max_t = anim
                .channels
                .iter()
                .flat_map(|c| c.sampler.keyframes.iter().copied())
                .filter(|t| t.is_finite())
                .fold(0.0f32, f32::max);
            let cycle = if max_t > 0.0 { max_t } else { 1.0 };
            let ts_name = self.unique(anim.name.as_deref().unwrap_or(&format!("ANIM{ai}")));
            let info = scene
                .extras
                .get("vrml:timeSensors")
                .and_then(Value::as_array)
                .and_then(|a| {
                    a.iter().find(|v| {
                        v.get("name").and_then(Value::as_str) == anim.name.as_deref()
                            && anim.name.is_some()
                    })
                });
            let mut ts = Node::new("TimeSensor");
            ts.def_name = Some(ts_name.clone());
            ts.set_field("cycleInterval", FieldValue::sf_time(cycle as f64));
            let looped = info
                .and_then(|v| v.get("loop"))
                .and_then(Value::as_bool)
                .unwrap_or(true);
            ts.set_field("loop", FieldValue::sf_bool(looped));
            let ts_id = self.add(ts);
            stmts.push(Statement::Node(ts_id));
            for (ci, ch) in anim.channels.iter().enumerate() {
                let target = ch.target.node.0;
                let Some(&tnode) = self.emitted.get(&target) else {
                    continue;
                };
                let kind = self.emitted_kind.get(&target).copied().unwrap_or("Group");
                let s = &ch.sampler;
                let n_frames = s.keyframes.len();
                if n_frames == 0 {
                    continue;
                }
                let per_frame = |values_len: usize| -> usize {
                    let factor = if s.interpolation == Interpolation::CubicSpline {
                        3
                    } else {
                        1
                    };
                    values_len / (n_frames * factor).max(1)
                };
                // Centre value of keyframe k (cubic: skip tangents).
                let centre = |k: usize, stride: usize| -> usize {
                    if s.interpolation == Interpolation::CubicSpline {
                        (3 * k + 1) * stride
                    } else {
                        k * stride
                    }
                };
                let step = s.interpolation == Interpolation::Step;
                let keys_for = |n: usize| -> Vec<(f32, usize)> {
                    // (key fraction, source keyframe index)
                    let mut out = Vec::new();
                    for k in 0..n {
                        let f = s.keyframes[k] / cycle;
                        if step && k > 0 {
                            out.push((f, k - 1));
                        }
                        out.push((f, k));
                    }
                    out
                };
                let (interp_type, field, value): (&str, &str, Option<FieldValue>) =
                    match (&s.values, ch.target.property) {
                        (
                            AnimationValues::Vec3(v),
                            AnimationProperty::Translation | AnimationProperty::Scale,
                        ) => {
                            let stride = per_frame(v.len());
                            if stride != 1 {
                                continue;
                            }
                            let kv: Vec<[f32; 3]> = keys_for(n_frames)
                                .iter()
                                .filter_map(|(_, k)| v.get(centre(*k, 1)).copied())
                                .collect();
                            let field = match (ch.target.property, kind) {
                                (AnimationProperty::Scale, "Transform") => "set_scale",
                                (AnimationProperty::Scale, _) => continue,
                                (_, "Viewpoint") => "set_position",
                                (_, "Transform") => "set_translation",
                                _ => continue,
                            };
                            (
                                "PositionInterpolator",
                                field,
                                Some(FieldValue::mf_vec3f(&kv)),
                            )
                        }
                        (AnimationValues::Quat(v), AnimationProperty::Rotation) => {
                            let kv: Vec<[f32; 4]> = keys_for(n_frames)
                                .iter()
                                .filter_map(|(_, k)| v.get(centre(*k, 1)).copied())
                                .map(quat_to_axis_angle)
                                .collect();
                            let field = match kind {
                                "Viewpoint" => "set_orientation",
                                "Transform" => "set_rotation",
                                _ => continue,
                            };
                            (
                                "OrientationInterpolator",
                                field,
                                Some(FieldValue::mf_rotation(&kv)),
                            )
                        }
                        (AnimationValues::Scalar(w), AnimationProperty::MorphWeights) => {
                            self.morph_route(
                                stmts,
                                &ts_name,
                                ci,
                                target,
                                w,
                                &keys_for(n_frames),
                                per_frame(w.len()),
                                &centre,
                            );
                            continue;
                        }
                        _ => ("", "", None),
                    };
                let Some(value) = value else { continue };
                let keys: Vec<f32> = keys_for(n_frames).iter().map(|(f, _)| *f).collect();
                if keys.len() != value.len() {
                    continue;
                }
                let iname = self.unique(&format!("{ts_name}_{ci}"));
                let mut interp = Node::new(interp_type);
                interp.def_name = Some(iname.clone());
                interp.set_field("key", FieldValue::mf_float(keys));
                interp.set_field("keyValue", value);
                let iid = self.add(interp);
                stmts.push(Statement::Node(iid));
                let tname = self
                    .doc
                    .node(tnode)
                    .and_then(|n| n.def_name.clone())
                    .unwrap_or_default();
                self.statements_tail.push(route(
                    &ts_name,
                    ts_id,
                    "fraction_changed",
                    &iname,
                    iid,
                    "set_fraction",
                ));
                self.statements_tail.push(route(
                    &iname,
                    iid,
                    "value_changed",
                    &tname,
                    tnode,
                    field,
                ));
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn morph_route(
        &mut self,
        stmts: &mut Vec<Statement>,
        ts_name: &str,
        ci: usize,
        target: u32,
        weights: &[f32],
        keys: &[(f32, usize)],
        stride: usize,
        centre: &dyn Fn(usize, usize) -> usize,
    ) {
        let scene = self.scene;
        let Some(mid) = scene.nodes.get(target as usize).and_then(|n| n.mesh) else {
            return;
        };
        let Some(mesh) = scene.meshes.get(mid.0 as usize) else {
            return;
        };
        let coords = self.coords.get(&mid.0).cloned().unwrap_or_default();
        let ts_id = self
            .doc
            .nodes
            .iter()
            .position(|n| n.def_name.as_deref() == Some(ts_name))
            .map(|i| NodeId(i as u32));
        let Some(ts_id) = ts_id else { return };
        for (pi, prim) in mesh.primitives.iter().enumerate() {
            let Some(Some(coord)) = coords.get(pi).copied() else {
                continue;
            };
            if prim.targets.is_empty() || stride != prim.targets.len() {
                continue;
            }
            let mut kv: Vec<[f32; 3]> = Vec::new();
            for (_, k) in keys {
                let off = centre(*k, stride);
                let Some(w) = weights.get(off..off + stride) else {
                    return;
                };
                kv.extend(prim.apply_morph_weights(w).positions);
            }
            let iname = self.unique(&format!("{ts_name}_{ci}_{pi}"));
            let mut interp = Node::new("CoordinateInterpolator");
            interp.def_name = Some(iname.clone());
            interp.set_field(
                "key",
                FieldValue::mf_float(keys.iter().map(|(f, _)| *f).collect()),
            );
            interp.set_field("keyValue", FieldValue::mf_vec3f(&kv));
            let iid = self.add(interp);
            stmts.push(Statement::Node(iid));
            let cname = self
                .doc
                .node(coord)
                .and_then(|n| n.def_name.clone())
                .unwrap_or_default();
            self.statements_tail.push(route(
                ts_name,
                ts_id,
                "fraction_changed",
                &iname,
                iid,
                "set_fraction",
            ));
            self.statements_tail.push(route(
                &iname,
                iid,
                "value_changed",
                &cname,
                coord,
                "set_point",
            ));
        }
    }

    /// Rebuild a standard node from its decoder JSON dump.
    fn json_node(&mut self, v: &Value, depth: usize) -> Option<NodeId> {
        if depth > 8 {
            return None;
        }
        let ty = v.get("type")?.as_str()?;
        let schema = vrml97_node(ty)?;
        let mut n = Node::new(ty);
        if let Some(def) = v.get("def").and_then(Value::as_str) {
            n.def_name = Some(self.unique(def));
        }
        if let Some(Value::Object(fields)) = v.get("fields") {
            for (k, fv) in fields {
                let Some(fs) = schema.field(k) else { continue };
                if !fs.access.has_value() {
                    continue;
                }
                if let Some(val) = self.json_value(fs.field_type, fv, depth) {
                    n.set_field(k, val);
                }
            }
        }
        Some(self.add(n))
    }

    fn json_value(&mut self, ty: FieldType, v: &Value, depth: usize) -> Option<FieldValue> {
        let mut flat = Vec::new();
        flatten(v, &mut flat);
        let mut out = FieldValue::empty(ty);
        match (&mut out.data, ty.element().0) {
            (FieldData::Bools(d), Scalar::Bool) => {
                d.extend(flat.iter().filter_map(|x| x.as_bool()))
            }
            (FieldData::Int32s(d), Scalar::Int32) => {
                d.extend(flat.iter().filter_map(|x| x.as_i64()).map(|x| x as i32))
            }
            (FieldData::Floats(d), Scalar::Float) => {
                d.extend(flat.iter().filter_map(|x| x.as_f64()).map(|x| x as f32))
            }
            (FieldData::Doubles(d), Scalar::Double) => {
                d.extend(flat.iter().filter_map(|x| x.as_f64()))
            }
            (FieldData::Strings(d), Scalar::String) => {
                d.extend(flat.iter().filter_map(|x| x.as_str()).map(str::to_owned))
            }
            (FieldData::Nodes(_), Scalar::Node) => {
                let items: Vec<&Value> = match v {
                    Value::Array(a) => a.iter().collect(),
                    Value::Null => Vec::new(),
                    other => vec![other],
                };
                let ids: Vec<NodeId> = items
                    .into_iter()
                    .filter_map(|x| self.json_node(x, depth + 1))
                    .collect();
                out.data = FieldData::Nodes(ids);
            }
            (FieldData::Images(d), Scalar::Image) => d.push(Image::default()),
            _ => return None,
        }
        let comps = ty.element().1.max(1);
        if let FieldData::Floats(f) = &mut out.data {
            f.truncate(f.len() / comps * comps);
        }
        Some(out)
    }
}

fn flatten<'v>(v: &'v Value, out: &mut Vec<&'v Value>) {
    match v {
        Value::Array(a) => a.iter().for_each(|x| flatten(x, out)),
        other => out.push(other),
    }
}

fn route(from: &str, from_id: NodeId, ff: &str, to: &str, to_id: NodeId, tf: &str) -> Statement {
    Statement::Route(Route {
        from_node: from.to_owned(),
        from_field: ff.to_owned(),
        to_node: to.to_owned(),
        to_field: tf.to_owned(),
        from_id: Some(from_id),
        to_id: Some(to_id),
    })
}

fn trs(t: &Transform) -> ([f32; 3], [f32; 4], [f32; 3]) {
    match Transform::from_matrix(t.to_matrix()) {
        Transform::Trs {
            translation,
            rotation,
            scale,
        } => match t {
            Transform::Trs {
                translation,
                rotation,
                scale,
            } => (*translation, *rotation, *scale),
            Transform::Matrix(_) => (translation, rotation, scale),
        },
        Transform::Matrix(_) => ([0.0; 3], [0.0, 0.0, 0.0, 1.0], [1.0; 3]),
    }
}

fn camera_fov(cam: &Camera) -> f32 {
    match cam {
        Camera::Perspective { yfov, .. } if *yfov > 0.0 && *yfov < std::f32::consts::PI => *yfov,
        Camera::Orthographic { .. } | Camera::Perspective { .. } => std::f32::consts::FRAC_PI_4,
    }
}

/// Unit direction with float noise (|c| < 1e-6) snapped to zero, so
/// axis-aligned directions survive a rotate-and-back exactly.
fn normalize3(v: [f32; 3]) -> [f32; 3] {
    let v = v.map(|c| if c.abs() < 1e-6 { 0.0 } else { c });
    let n = crate::convert::math::normalize(v).unwrap_or([0.0, 0.0, -1.0]);
    n.map(|c| {
        if (c.abs() - 1.0).abs() < 1e-6 {
            c.signum()
        } else {
            c
        }
    })
}

fn json_vec3(v: Option<&Value>) -> Option<[f32; 3]> {
    let a = v?.as_array()?;
    if a.len() != 3 {
        return None;
    }
    let mut out = [0.0; 3];
    for (o, x) in out.iter_mut().zip(a) {
        *o = x.as_f64()? as f32;
    }
    Some(out)
}

fn json_vec4(v: Option<&Value>) -> Option<[f32; 4]> {
    let a = v?.as_array()?;
    if a.len() != 4 {
        return None;
    }
    let mut out = [0.0; 4];
    for (o, x) in out.iter_mut().zip(a) {
        *o = x.as_f64()? as f32;
    }
    Some(out)
}

fn json_strings(v: Option<&Value>) -> Vec<String> {
    match v {
        Some(Value::Array(a)) => a
            .iter()
            .filter_map(Value::as_str)
            .map(str::to_owned)
            .collect(),
        Some(Value::String(s)) => vec![s.clone()],
        _ => Vec::new(),
    }
}
