//! VRML scene graph → [`Scene3D`] conversion.
//!
//! Works on a PROTO-expanded [`Document`]
//! ([`expand_protos`](crate::syntax::expand_protos)). VRML is Y-up,
//! right-handed and in metres (ISO/IEC 14772-1 §4.4.5), which is the
//! mesh3d default orientation, so geometry is never re-oriented.
//!
//! Mapping summary:
//!
//! | VRML | Scene3D |
//! |------|---------|
//! | Transform | node with TRS (or a matrix when `center` / `scaleOrientation` are used) |
//! | Group / Anchor / Billboard / Collision | identity node (+ `vrml:*` extras) |
//! | Switch / LOD | identity node holding the active choice / finest level |
//! | Inline | resolved through a [`UrlResolver`], else `vrml:inline` extras |
//! | Shape children of one grouping node | one mesh on that node, one primitive per Shape |
//! | Material | metallic-roughness approximation (originals in `vrml:material`) |
//! | ImageTexture / PixelTexture / MovieTexture | texture (PixelTexture → in-memory PNG) |
//! | Viewpoint | perspective camera on a child node |
//! | Directional / Point / Spot light | punctual light on a child node (`vrml:light` extras) |
//! | TimeSensor + Position / Orientation / CoordinateInterpolator ROUTEs | animation channels / morph targets |
//! | Background, Fog, NavigationInfo, WorldInfo, routes, behaviour nodes | scene extras |

mod fields;
pub(crate) mod geometry;
pub(crate) mod image;
pub(crate) mod math;
mod mesh;

use std::collections::{HashMap, HashSet};
use std::fmt;
use std::sync::Arc;

use oxideav_mesh3d::{
    AlphaMode, Animation, AnimationChannel, AnimationProperty, AnimationSampler, AnimationValues,
    Camera, Interpolation, Light, Material, MaterialId, Mesh, MeshId, MorphTarget,
    Node as SceneNode, NodeId as SceneNodeId, Primitive, Sampler, Scene3D, Texture, TextureId,
    TextureRef, Transform, WrapMode,
};
use serde_json::{json, Map, Value};

use crate::ast::{Document, FieldBinding, FieldData, FieldValue, Node, NodeId, Route};
use crate::error::{Error, Result};
use fields as fl;
pub use geometry::Tessellation;
use geometry::{GeomCtx, TexTransform};
use math::{axis_angle_to_quat, quat_between, V3};

/// Fetches the bytes behind a URL (Inline children, textures, …).
///
/// The decoder has no file-system or network context of its own;
/// supply a resolver to follow `Inline` URLs. Returning `None` leaves
/// the reference external.
pub trait UrlResolver: Send + Sync {
    /// Bytes of `url`, or `None` when it cannot (or should not) be
    /// fetched.
    fn resolve(&self, url: &str) -> Option<Vec<u8>>;
}

/// Scene-conversion options.
#[derive(Clone)]
pub struct ConvertOptions {
    /// Tessellation density for Sphere / Cone / Cylinder.
    pub tessellation: Tessellation,
    /// Convert every Switch choice (inactive ones flagged with
    /// `vrml:inactive`) instead of only `whichChoice`.
    pub all_switch_choices: bool,
    /// Convert every LOD level (coarser ones flagged with
    /// `vrml:inactive`) instead of only the finest.
    pub all_lod_levels: bool,
    /// Maximum scene nodes created (bounds `USE` instancing blow-up).
    pub max_scene_nodes: usize,
    /// Maximum faces / vertices / indices generated per geometry node
    /// (bounds Extrusion `spine × crossSection` and similar products).
    pub max_generated: usize,
    /// Resolver for `Inline` URLs; `None` keeps them as references.
    pub resolver: Option<Arc<dyn UrlResolver>>,
    /// Maximum `Inline` nesting followed through the resolver.
    pub max_inline_depth: usize,
}

impl fmt::Debug for ConvertOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ConvertOptions")
            .field("tessellation", &self.tessellation)
            .field("all_switch_choices", &self.all_switch_choices)
            .field("all_lod_levels", &self.all_lod_levels)
            .field("max_scene_nodes", &self.max_scene_nodes)
            .field("max_generated", &self.max_generated)
            .field("resolver", &self.resolver.is_some())
            .field("max_inline_depth", &self.max_inline_depth)
            .finish()
    }
}

impl Default for ConvertOptions {
    fn default() -> Self {
        Self {
            tessellation: Tessellation::default(),
            all_switch_choices: false,
            all_lod_levels: false,
            max_scene_nodes: 1_000_000,
            max_generated: geometry::MAX_GENERATED,
            resolver: None,
            max_inline_depth: 8,
        }
    }
}

/// Convert a (PROTO-expanded) document into a [`Scene3D`].
pub fn document_to_scene(doc: &Document, opts: &ConvertOptions) -> Result<Scene3D> {
    document_to_scene_at_depth(doc, opts, 0)
}

fn document_to_scene_at_depth(
    doc: &Document,
    opts: &ConvertOptions,
    inline_depth: usize,
) -> Result<Scene3D> {
    let mut c = Converter::new(doc, opts, inline_depth);
    c.run()?;
    Ok(c.scene)
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct MatKey {
    appearance: Option<NodeId>,
    solid: bool,
    unlit: bool,
    has_colors: bool,
}

struct Converter<'d> {
    doc: &'d Document,
    opts: &'d ConvertOptions,
    scene: Scene3D,
    inline_depth: usize,
    path: Vec<NodeId>,
    meshes: HashMap<Vec<NodeId>, Option<MeshId>>,
    materials: HashMap<MatKey, MaterialId>,
    textures: HashMap<NodeId, Option<TextureId>>,
    /// VRML node → every scene node instantiating it.
    instances: HashMap<NodeId, Vec<SceneNodeId>>,
    /// Scene nodes whose transform is a plain TRS (animatable).
    trs_nodes: HashSet<SceneNodeId>,
    /// Coordinate node → (mesh, primitive, source coord per vertex).
    coord_users: HashMap<NodeId, Vec<(MeshId, usize, Vec<u32>)>>,
    znear: f32,
    zfar: Option<f32>,
    extras: Map<String, Value>,
}

const BEHAVIOUR_TYPES: &[&str] = &[
    "TimeSensor",
    "PositionInterpolator",
    "OrientationInterpolator",
    "ScalarInterpolator",
    "ColorInterpolator",
    "CoordinateInterpolator",
    "NormalInterpolator",
    "Script",
    "TouchSensor",
    "PlaneSensor",
    "SphereSensor",
    "CylinderSensor",
    "ProximitySensor",
    "VisibilitySensor",
];

/// Standard node types that are not children nodes (§4.6.5): a
/// top-level statement of one of these only matters through DEF / USE.
const NON_CHILD_TYPES: &[&str] = &[
    "Appearance",
    "AudioClip",
    "Box",
    "Color",
    "Cone",
    "Coordinate",
    "Cylinder",
    "ElevationGrid",
    "Extrusion",
    "FontStyle",
    "ImageTexture",
    "IndexedFaceSet",
    "IndexedLineSet",
    "Material",
    "MovieTexture",
    "Normal",
    "PixelTexture",
    "PointSet",
    "Sphere",
    "Text",
    "TextureCoordinate",
    "TextureTransform",
];

impl<'d> Converter<'d> {
    fn new(doc: &'d Document, opts: &'d ConvertOptions, inline_depth: usize) -> Self {
        // Near clip = avatarSize[0] / 2, far = visibilityLimit (§6.29),
        // from the first NavigationInfo in the file.
        let nav = doc.nodes.iter().find(|n| n.type_name == "NavigationInfo");
        let (znear, zfar) = match nav {
            Some(n) => {
                let avatar = fl::floats(n, "avatarSize");
                let near = avatar.first().copied().unwrap_or(0.25) * 0.5;
                let far = fl::f(n, "visibilityLimit", 0.0);
                (
                    if near > 0.0 && near.is_finite() {
                        near
                    } else {
                        0.125
                    },
                    (far > 0.0).then_some(far),
                )
            }
            None => (0.125, None),
        };
        Self {
            doc,
            opts,
            scene: Scene3D::new(),
            inline_depth,
            path: Vec::new(),
            meshes: HashMap::new(),
            materials: HashMap::new(),
            textures: HashMap::new(),
            instances: HashMap::new(),
            trs_nodes: HashSet::new(),
            coord_users: HashMap::new(),
            znear,
            zfar,
            extras: Map::new(),
        }
    }

    fn run(&mut self) -> Result<()> {
        let roots: Vec<NodeId> = self.doc.root_nodes().collect();
        for id in roots {
            if let Some(sid) = self.visit(id)? {
                self.scene.roots.push(sid);
            }
        }
        self.animations();
        let h = &self.doc.header;
        if !h.comment.is_empty() {
            self.extras
                .insert("vrml:headerComment".into(), json!(h.comment));
        }
        let extras = std::mem::take(&mut self.extras);
        self.scene.extras.extend(extras);
        Ok(())
    }

    fn node(&self, id: NodeId) -> Option<&'d Node> {
        self.doc.node(id)
    }

    fn add_scene_node(&mut self, node: SceneNode) -> Result<SceneNodeId> {
        if self.scene.nodes.len() >= self.opts.max_scene_nodes {
            return Err(Error::limit(format!(
                "scene exceeds {} nodes (USE instancing)",
                self.opts.max_scene_nodes
            )));
        }
        Ok(self.scene.add_node(node))
    }

    fn push_extra_list(&mut self, key: &str, v: Value) {
        let e = self
            .extras
            .entry(key.to_owned())
            .or_insert_with(|| Value::Array(Vec::new()));
        if let Value::Array(a) = e {
            a.push(v);
        }
    }

    /// Convert one children-field node. Returns the created scene node
    /// (if the node maps to one).
    fn visit(&mut self, id: NodeId) -> Result<Option<SceneNodeId>> {
        if self.path.contains(&id) {
            return Ok(None); // USE cycle (undefined by the spec)
        }
        let Some(node) = self.node(id) else {
            return Ok(None);
        };
        self.path.push(id);
        let r = self.visit_inner(id, node);
        self.path.pop();
        let sid = r?;
        if let Some(sid) = sid {
            self.instances.entry(id).or_default().push(sid);
        }
        Ok(sid)
    }

    fn visit_inner(&mut self, id: NodeId, node: &'d Node) -> Result<Option<SceneNodeId>> {
        let mut sn = SceneNode::new();
        sn.name = node.def_name.clone();

        let ty = node.type_name.as_str();
        let kids: Vec<NodeId> = match ty {
            "Transform" => {
                let (t, simple) = vrml_transform(node);
                sn.transform = t;
                if !simple {
                    sn.extras.insert(
                        "vrml:transform".into(),
                        json!({
                            "center": fl::v3(node, "center", [0.0; 3]),
                            "rotation": fl::v4(node, "rotation", [0.0, 0.0, 1.0, 0.0]),
                            "scale": fl::v3(node, "scale", [1.0; 3]),
                            "scaleOrientation": fl::v4(node, "scaleOrientation", [0.0, 0.0, 1.0, 0.0]),
                            "translation": fl::v3(node, "translation", [0.0; 3]),
                        }),
                    );
                }
                let kids = fl::children(node, "children").to_vec();
                let sid = self.add_scene_node(sn)?;
                if simple {
                    self.trs_nodes.insert(sid);
                }
                self.group_children(sid, &kids)?;
                return Ok(Some(sid));
            }
            "Group" => fl::children(node, "children").to_vec(),
            "Anchor" => {
                sn.extras.insert(
                    "vrml:anchor".into(),
                    json!({
                        "url": fl::strs(node, "url"),
                        "description": fl::string(node, "description"),
                        "parameter": fl::strs(node, "parameter"),
                    }),
                );
                fl::children(node, "children").to_vec()
            }
            "Billboard" => {
                sn.extras.insert(
                    "vrml:billboard".into(),
                    json!({ "axisOfRotation": fl::v3(node, "axisOfRotation", [0.0, 1.0, 0.0]) }),
                );
                fl::children(node, "children").to_vec()
            }
            "Collision" => {
                sn.extras.insert(
                    "vrml:collision".into(),
                    json!({
                        "collide": fl::b(node, "collide", true),
                        "proxy": fl::child(self.doc, node, "proxy").map(|(p, _)| node_json(self.doc, p, 0)),
                    }),
                );
                fl::children(node, "children").to_vec()
            }
            "Switch" => {
                let choice = fl::children(node, "choice").to_vec();
                let which = fl::i(node, "whichChoice", -1);
                sn.extras.insert(
                    "vrml:switch".into(),
                    json!({ "whichChoice": which, "choices": choice.len() }),
                );
                let sid = self.add_scene_node(sn)?;
                if self.opts.all_switch_choices {
                    for (k, c) in choice.iter().enumerate() {
                        self.attach_child(sid, *c, k as i32 != which)?;
                    }
                } else if which >= 0 {
                    if let Some(c) = choice.get(which as usize) {
                        self.attach_child(sid, *c, false)?;
                    }
                }
                return Ok(Some(sid));
            }
            "LOD" => {
                let levels = fl::children(node, "level").to_vec();
                sn.extras.insert(
                    "vrml:lod".into(),
                    json!({
                        "center": fl::v3(node, "center", [0.0; 3]),
                        "range": fl::floats(node, "range"),
                        "levels": levels.len(),
                    }),
                );
                let sid = self.add_scene_node(sn)?;
                for (k, c) in levels.iter().enumerate() {
                    if k > 0 && !self.opts.all_lod_levels {
                        break;
                    }
                    self.attach_child(sid, *c, k > 0)?;
                }
                return Ok(Some(sid));
            }
            "Inline" => {
                let sid = self.add_scene_node(sn)?;
                self.inline(sid, node)?;
                return Ok(Some(sid));
            }
            "Shape" => {
                // Root-level (or Switch / LOD) Shape: its own node.
                let sid = self.add_scene_node(sn)?;
                if let Some(mesh) = self.mesh_for(&[id])? {
                    self.scene.nodes[sid.0 as usize].mesh = Some(mesh);
                }
                return Ok(Some(sid));
            }
            "Viewpoint" => {
                sn.transform = Transform::Trs {
                    translation: fl::v3(node, "position", [0.0, 0.0, 10.0]),
                    rotation: axis_angle_to_quat(fl::v4(node, "orientation", [0.0, 0.0, 1.0, 0.0])),
                    scale: [1.0; 3],
                };
                let desc = fl::string(node, "description");
                if sn.name.is_none() && !desc.is_empty() {
                    sn.name = Some(desc.to_owned());
                }
                let fov = fl::f(node, "fieldOfView", std::f32::consts::FRAC_PI_4);
                let fov = if fov > 0.0 && fov < std::f32::consts::PI {
                    fov
                } else {
                    std::f32::consts::FRAC_PI_4
                };
                let cam = self.scene.add_camera(Camera::Perspective {
                    aspect_ratio: None,
                    yfov: fov,
                    znear: self.znear,
                    zfar: self.zfar,
                });
                sn.camera = Some(cam);
                sn.extras.insert(
                    "vrml:viewpoint".into(),
                    json!({
                        "description": desc,
                        "fieldOfView": fov,
                        "jump": fl::b(node, "jump", true),
                    }),
                );
                let sid = self.add_scene_node(sn)?;
                self.trs_nodes.insert(sid);
                return Ok(Some(sid));
            }
            "DirectionalLight" | "PointLight" | "SpotLight" => {
                self.light(node, &mut sn);
                let sid = self.add_scene_node(sn)?;
                return Ok(Some(sid));
            }
            "Background" | "Fog" | "NavigationInfo" | "WorldInfo" => {
                let key = match ty {
                    "Background" => "vrml:background",
                    "Fog" => "vrml:fog",
                    "NavigationInfo" => "vrml:navigationInfo",
                    _ => "vrml:worldInfo",
                };
                let v = node_json(self.doc, id, 0);
                self.push_extra_list(key, v);
                return Ok(None);
            }
            t if BEHAVIOUR_TYPES.contains(&t) || NON_CHILD_TYPES.contains(&t) => return Ok(None),
            _ => {
                // Unknown node / unresolved EXTERNPROTO instance / Text,
                // Sound …: keep it as an extras-carrying placeholder and
                // still descend into a `children` field.
                sn.extras
                    .insert("vrml:node".into(), node_json(self.doc, id, 0));
                fl::children(node, "children").to_vec()
            }
        };
        let sid = self.add_scene_node(sn)?;
        self.group_children(sid, &kids)?;
        Ok(Some(sid))
    }

    fn attach_child(&mut self, parent: SceneNodeId, child: NodeId, inactive: bool) -> Result<()> {
        if let Some(sid) = self.visit(child)? {
            if inactive {
                self.scene.nodes[sid.0 as usize]
                    .extras
                    .insert("vrml:inactive".into(), json!(true));
            }
            self.scene.nodes[parent.0 as usize].children.push(sid);
        }
        Ok(())
    }

    /// Convert grouping-node children: every Shape child folds into one
    /// mesh on `parent`, other children become child scene nodes.
    fn group_children(&mut self, parent: SceneNodeId, kids: &[NodeId]) -> Result<()> {
        let mut shapes = Vec::new();
        for &k in kids {
            if self.path.contains(&k) {
                continue;
            }
            match self.node(k) {
                Some(n) if n.type_name == "Shape" => shapes.push(k),
                Some(_) => {
                    if let Some(sid) = self.visit(k)? {
                        self.scene.nodes[parent.0 as usize].children.push(sid);
                    }
                }
                None => {}
            }
        }
        if !shapes.is_empty() {
            if let Some(mesh) = self.mesh_for(&shapes)? {
                self.scene.nodes[parent.0 as usize].mesh = Some(mesh);
            }
            for s in shapes {
                self.instances.entry(s).or_default().push(parent);
            }
        }
        Ok(())
    }

    fn mesh_for(&mut self, shapes: &[NodeId]) -> Result<Option<MeshId>> {
        if let Some(m) = self.meshes.get(shapes) {
            return Ok(*m);
        }
        let mut mesh = Mesh::new(
            shapes
                .iter()
                .find_map(|s| self.node(*s).and_then(|n| n.def_name.clone())),
        );
        let mut users = Vec::new();
        for &s in shapes {
            let Some(shape) = self.node(s) else { continue };
            if let Some((prim, coord)) = self.shape_primitive(shape)? {
                if let Some((cid, src)) = coord {
                    users.push((cid, mesh.primitives.len(), src));
                }
                mesh.primitives.push(prim);
            }
        }
        let id = if mesh.primitives.is_empty() {
            None
        } else {
            let mid = self.scene.add_mesh(mesh);
            for (cid, pi, src) in users {
                self.coord_users
                    .entry(cid)
                    .or_default()
                    .push((mid, pi, src));
            }
            Some(mid)
        };
        self.meshes.insert(shapes.to_vec(), id);
        Ok(id)
    }

    #[allow(clippy::type_complexity)]
    fn shape_primitive(
        &mut self,
        shape: &'d Node,
    ) -> Result<Option<(Primitive, Option<(NodeId, Vec<u32>)>)>> {
        let Some((gid, geom)) = fl::child(self.doc, shape, "geometry") else {
            return Ok(None);
        };
        if self.path.contains(&gid) {
            return Ok(None);
        }
        let app = fl::child(self.doc, shape, "appearance");
        let tex = app.and_then(|(_, a)| fl::child(self.doc, a, "texture"));
        let tt = app
            .and_then(|(_, a)| fl::child(self.doc, a, "textureTransform"))
            .map(|(_, t)| TexTransform::from_node(t));
        let texture = match tex {
            Some((tid, tn)) => self.texture(tid, tn),
            None => None,
        };
        let ctx = GeomCtx {
            need_uvs: texture.is_some(),
            tex_transform: tt,
            tess: self.opts.tessellation,
            max_generated: self.opts.max_generated,
        };
        let Some(out) = geometry::convert(self.doc, geom, &ctx) else {
            if !matches!(
                geom.type_name.as_str(),
                "IndexedFaceSet" | "IndexedLineSet" | "PointSet" | "ElevationGrid" | "Extrusion"
            ) {
                // Text and unknown geometry: keep a record.
                let v = node_json(self.doc, gid, 0);
                self.push_extra_list("vrml:unsupportedGeometry", v);
            }
            return Ok(None);
        };
        let mut prim = out.built.prim;
        // RGB(A) textures replace the per-vertex colour (Table 4.6).
        let rgb_texture = tex.is_some_and(|(_, tn)| match tn.type_name.as_str() {
            "PixelTexture" => tn
                .field("image")
                .and_then(FieldValue::as_image)
                .is_some_and(|i| i.components >= 3),
            _ => true,
        });
        let mut has_colors = out.has_colors;
        if has_colors && texture.is_some() && rgb_texture {
            prim.colors.clear();
            has_colors = false;
        }
        let key = MatKey {
            appearance: app.map(|(id, _)| id),
            solid: out.solid,
            unlit: out.unlit,
            has_colors,
        };
        prim.material = Some(self.material(key, texture, rgb_texture, tt.is_some()));
        if !out.solid {
            prim.extras.insert("vrml:solid".into(), json!(false));
        }
        prim.extras
            .insert("vrml:geometry".into(), json!(geom.type_name));
        if let Some(def) = &geom.def_name {
            prim.extras.insert("vrml:geometryDef".into(), json!(def));
        }
        let coord = fl::child(self.doc, geom, "coord").map(|(cid, _)| cid);
        Ok(Some((prim, coord.map(|c| (c, out.built.source_coord)))))
    }

    fn texture(&mut self, id: NodeId, n: &Node) -> Option<TextureId> {
        if let Some(t) = self.textures.get(&id) {
            return *t;
        }
        let mut tex = match n.type_name.as_str() {
            "ImageTexture" | "MovieTexture" => {
                let url = fl::strs(n, "url").first()?.clone();
                match image::parse_data_uri(&url) {
                    Some((mime, bytes)) => Texture::from_encoded(mime, bytes),
                    None => Texture::from_uri(url),
                }
            }
            "PixelTexture" => {
                let img = n.field("image")?.as_image()?;
                Texture::from_encoded("image/png", image::image_to_png(img)?)
            }
            _ => return None,
        };
        tex.name = n.def_name.clone();
        let wrap = |b| {
            if b {
                WrapMode::Repeat
            } else {
                WrapMode::ClampToEdge
            }
        };
        tex.sampler = Sampler {
            wrap_s: wrap(fl::b(n, "repeatS", true)),
            wrap_t: wrap(fl::b(n, "repeatT", true)),
            ..Sampler::default_sampler()
        };
        let tid = self.scene.add_texture(tex);
        self.textures.insert(id, Some(tid));
        Some(tid)
    }

    fn material(
        &mut self,
        key: MatKey,
        texture: Option<TextureId>,
        rgb_texture: bool,
        baked_tt: bool,
    ) -> MaterialId {
        if let Some(m) = self.materials.get(&key) {
            return *m;
        }
        let app = key.appearance.and_then(|a| self.node(a));
        let mat = app.and_then(|a| fl::child(self.doc, a, "material"));
        let mut m = Material::new();
        m.metallic = 0.0;
        m.double_sided = !key.solid;
        m.name = mat
            .and_then(|(_, n)| n.def_name.clone())
            .or_else(|| app.and_then(|a| a.def_name.clone()));
        match mat {
            None => {
                // Lighting off (§4.14.2): unlit white × colour / texture.
                m.ext.unlit = true;
                m.roughness = 1.0;
            }
            Some((_, mn)) => {
                let diffuse = fl::v3(mn, "diffuseColor", [0.8, 0.8, 0.8]);
                let emissive = fl::v3(mn, "emissiveColor", [0.0; 3]);
                let shininess = fl::f(mn, "shininess", 0.2).clamp(0.0, 1.0);
                let transparency = fl::f(mn, "transparency", 0.0).clamp(0.0, 1.0);
                let alpha = 1.0 - transparency;
                if key.unlit {
                    // Lines / points draw with emissiveColor (§6.24, §6.36).
                    let c = if key.has_colors { [1.0; 3] } else { emissive };
                    m.base_color = [c[0], c[1], c[2], alpha];
                    m.ext.unlit = true;
                } else {
                    let c = if key.has_colors || (texture.is_some() && rgb_texture) {
                        [1.0; 3]
                    } else {
                        diffuse
                    };
                    m.base_color = [c[0], c[1], c[2], alpha];
                    m.emissive_factor = emissive;
                    m.roughness = shininess_to_roughness(shininess);
                }
                if transparency > 0.0 {
                    m.alpha_mode = AlphaMode::Blend;
                }
                m.extras.insert(
                    "vrml:material".into(),
                    json!({
                        "ambientIntensity": fl::f(mn, "ambientIntensity", 0.2),
                        "diffuseColor": diffuse,
                        "emissiveColor": emissive,
                        "shininess": shininess,
                        "specularColor": fl::v3(mn, "specularColor", [0.0; 3]),
                        "transparency": transparency,
                    }),
                );
            }
        }
        if key.unlit && mat.is_none() {
            m.base_color = [1.0; 4];
        }
        if let Some(t) = texture {
            m.base_color_texture = Some(TextureRef::new(t));
            if baked_tt {
                m.extras
                    .insert("vrml:textureTransformBaked".into(), json!(true));
            }
        }
        if let Some(a) = app.and_then(|a| a.def_name.clone()) {
            m.extras.insert("vrml:appearance".into(), json!(a));
        }
        let id = self.scene.add_material(m);
        self.materials.insert(key, id);
        id
    }

    fn light(&mut self, n: &Node, sn: &mut SceneNode) {
        let color = fl::v3(n, "color", [1.0; 3]);
        let on = fl::b(n, "on", true);
        let raw_intensity = fl::f(n, "intensity", 1.0);
        let intensity = if on { raw_intensity } else { 0.0 };
        let radius = fl::f(n, "radius", 100.0);
        let direction = fl::v3(n, "direction", [0.0, 0.0, -1.0]);
        let location = fl::v3(n, "location", [0.0; 3]);
        let mut info = json!({
            "type": n.type_name,
            "on": on,
            "intensity": raw_intensity,
            "ambientIntensity": fl::f(n, "ambientIntensity", 0.0),
        });
        let light = match n.type_name.as_str() {
            "DirectionalLight" => {
                sn.transform = Transform::Trs {
                    translation: [0.0; 3],
                    rotation: quat_between([0.0, 0.0, -1.0], direction),
                    scale: [1.0; 3],
                };
                Light::Directional { color, intensity }
            }
            "PointLight" => {
                sn.transform = Transform::Trs {
                    translation: location,
                    rotation: [0.0, 0.0, 0.0, 1.0],
                    scale: [1.0; 3],
                };
                info["attenuation"] = json!(fl::v3(n, "attenuation", [1.0, 0.0, 0.0]));
                info["radius"] = json!(radius);
                Light::Point {
                    color,
                    intensity,
                    range: (radius > 0.0).then_some(radius),
                }
            }
            _ => {
                sn.transform = Transform::Trs {
                    translation: location,
                    rotation: quat_between([0.0, 0.0, -1.0], direction),
                    scale: [1.0; 3],
                };
                let cutoff = fl::f(n, "cutOffAngle", std::f32::consts::FRAC_PI_4)
                    .clamp(1e-4, std::f32::consts::FRAC_PI_2);
                let beam = fl::f(n, "beamWidth", std::f32::consts::FRAC_PI_2).clamp(0.0, cutoff);
                info["attenuation"] = json!(fl::v3(n, "attenuation", [1.0, 0.0, 0.0]));
                info["radius"] = json!(radius);
                info["beamWidth"] = json!(fl::f(n, "beamWidth", std::f32::consts::FRAC_PI_2));
                info["cutOffAngle"] = json!(fl::f(n, "cutOffAngle", std::f32::consts::FRAC_PI_4));
                Light::Spot {
                    color,
                    intensity,
                    range: (radius > 0.0).then_some(radius),
                    inner_cone_angle: beam.min(cutoff * 0.999),
                    outer_cone_angle: cutoff,
                }
            }
        };
        sn.light = Some(self.scene.add_light(light));
        sn.extras.insert("vrml:light".into(), info);
    }

    fn inline(&mut self, sid: SceneNodeId, n: &Node) -> Result<()> {
        let urls = fl::strs(n, "url").to_vec();
        let info = json!({
            "url": urls,
            "bboxCenter": fl::v3(n, "bboxCenter", [0.0; 3]),
            "bboxSize": fl::v3(n, "bboxSize", [-1.0; 3]),
        });
        self.scene.nodes[sid.0 as usize]
            .extras
            .insert("vrml:inline".into(), info);
        let Some(resolver) = self.opts.resolver.clone() else {
            return Ok(());
        };
        if self.inline_depth >= self.opts.max_inline_depth {
            return Ok(());
        }
        for url in &urls {
            let Some(bytes) = resolver.resolve(url) else {
                continue;
            };
            let doc = crate::decoder::load_expanded(
                &bytes,
                &crate::syntax::ParseLimits::default(),
                &crate::syntax::ExpandLimits::default(),
                Some(&resolver),
            )?;
            let sub = document_to_scene_at_depth(&doc, self.opts, self.inline_depth + 1)?;
            if self.scene.nodes.len() + sub.nodes.len() > self.opts.max_scene_nodes {
                return Err(Error::limit("Inline expansion exceeds the scene node cap"));
            }
            let before = self.scene.roots.len();
            self.scene.append(&sub);
            let new_roots: Vec<SceneNodeId> = self.scene.roots.drain(before..).collect();
            let node = &mut self.scene.nodes[sid.0 as usize];
            node.children.extend(new_roots);
            node.extras.insert("vrml:inlineResolved".into(), json!(url));
            break;
        }
        Ok(())
    }

    // ---- animation -------------------------------------------------

    fn all_routes(&self) -> Vec<Route> {
        let mut routes: Vec<Route> = self.doc.routes().cloned().collect();
        for n in &self.doc.nodes {
            for s in &n.statements {
                if let crate::ast::Statement::Route(r) = s {
                    routes.push(r.clone());
                }
            }
            if let Some(inst) = &n.instance {
                routes.extend(inst.routes.iter().cloned());
            }
        }
        routes
    }

    /// Resolve a route endpoint through prototype IS tables.
    fn endpoint(&self, id: NodeId, field: &str) -> Vec<(NodeId, String)> {
        let base = event_base(field);
        if let Some(inst) = self.node(id).and_then(|n| n.instance.as_ref()) {
            let hits: Vec<(NodeId, String)> = inst
                .is_map
                .iter()
                .filter(|(iface, _, _)| event_base(iface) == base)
                .map(|(_, n, f)| (*n, event_base(f).to_owned()))
                .collect();
            if !hits.is_empty() {
                return hits;
            }
        }
        vec![(id, base.to_owned())]
    }

    fn animations(&mut self) {
        let routes = self.all_routes();
        if routes.is_empty() {
            return;
        }
        let mut route_json = Vec::new();
        // interpolator → driving TimeSensors; interpolator → targets.
        let mut drivers: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        let mut targets: HashMap<NodeId, Vec<(NodeId, String)>> = HashMap::new();
        let mut behaviour: Map<String, Value> = Map::new();
        for r in &routes {
            route_json.push(json!([r.from_node, r.from_field, r.to_node, r.to_field]));
            let (Some(f), Some(t)) = (r.from_id, r.to_id) else {
                continue;
            };
            for (fid, ffield) in self.endpoint(f, &r.from_field) {
                for (tid, tfield) in self.endpoint(t, &r.to_field) {
                    let (Some(fnode), Some(tnode)) = (self.node(fid), self.node(tid)) else {
                        continue;
                    };
                    for (nid, n) in [(fid, fnode), (tid, tnode)] {
                        if BEHAVIOUR_TYPES.contains(&n.type_name.as_str()) {
                            let key = n.def_name.clone().unwrap_or_else(|| format!("_N{}", nid.0));
                            behaviour
                                .entry(key)
                                .or_insert_with(|| node_json(self.doc, nid, 0));
                        }
                    }
                    if fnode.type_name == "TimeSensor"
                        && ffield == "fraction"
                        && tnode.type_name.ends_with("Interpolator")
                        && tfield == "fraction"
                    {
                        drivers.entry(tid).or_default().push(fid);
                    } else if fnode.type_name.ends_with("Interpolator") && ffield == "value" {
                        targets.entry(fid).or_default().push((tid, tfield));
                    }
                }
            }
        }
        self.extras
            .insert("vrml:routes".into(), Value::Array(route_json));
        if !behaviour.is_empty() {
            self.extras
                .insert("vrml:behaviour".into(), Value::Object(behaviour));
        }
        let mut by_sensor: Vec<(NodeId, Animation)> = Vec::new();
        let mut interps: Vec<_> = targets.into_iter().collect();
        interps.sort_by_key(|(k, _)| *k);
        for (interp_id, tlist) in interps {
            let Some(interp) = self.node(interp_id) else {
                continue;
            };
            let Some(sensors) = drivers.get(&interp_id) else {
                continue;
            };
            for &ts_id in sensors {
                let Some(ts) = self.node(ts_id) else { continue };
                let cycle = fl::d64(ts, "cycleInterval", 1.0);
                let cycle = if cycle > 0.0 { cycle as f32 } else { 1.0 };
                let mut channels = Vec::new();
                for (tid, tfield) in &tlist {
                    channels.extend(self.channels_for(interp, *tid, tfield, cycle));
                }
                if channels.is_empty() {
                    continue;
                }
                let anim = match by_sensor.iter_mut().find(|(s, _)| *s == ts_id) {
                    Some((_, a)) => a,
                    None => {
                        let name = ts.def_name.clone();
                        by_sensor.push((ts_id, Animation::new(name)));
                        let info = json!({
                            "name": ts.def_name,
                            "cycleInterval": cycle,
                            "loop": fl::b(ts, "loop", false),
                            "startTime": fl::d64(ts, "startTime", 0.0),
                            "stopTime": fl::d64(ts, "stopTime", 0.0),
                            "enabled": fl::b(ts, "enabled", true),
                        });
                        self.push_extra_list("vrml:timeSensors", info);
                        &mut by_sensor.last_mut().expect("just pushed").1
                    }
                };
                anim.channels.extend(channels);
            }
        }
        for (_, a) in by_sensor {
            self.scene.animations.push(a);
        }
    }

    fn channels_for(
        &mut self,
        interp: &Node,
        target: NodeId,
        field: &str,
        cycle: f32,
    ) -> Vec<AnimationChannel> {
        let Some(tnode) = self.node(target) else {
            return Vec::new();
        };
        let keys = fl::floats(interp, "key");
        let times: Vec<f32> = keys.iter().map(|k| k * cycle).collect();
        let mut out = Vec::new();
        let scene_targets: Vec<SceneNodeId> = self
            .instances
            .get(&target)
            .map(|v| {
                v.iter()
                    .copied()
                    .filter(|s| self.trs_nodes.contains(s))
                    .collect()
            })
            .unwrap_or_default();
        match (interp.type_name.as_str(), tnode.type_name.as_str(), field) {
            ("PositionInterpolator", "Transform", "translation" | "scale")
            | ("PositionInterpolator", "Viewpoint", "position") => {
                let (keyframes, values, interpolation) =
                    sanitize_keys(&times, &fl::v3s(interp, "keyValue"));
                if keyframes.is_empty() {
                    return out;
                }
                let prop = if field == "scale" {
                    AnimationProperty::Scale
                } else {
                    AnimationProperty::Translation
                };
                for s in scene_targets {
                    out.push(AnimationChannel::new(
                        s,
                        prop,
                        AnimationSampler {
                            keyframes: keyframes.clone(),
                            values: AnimationValues::Vec3(values.clone()),
                            interpolation,
                        },
                    ));
                }
            }
            ("OrientationInterpolator", "Transform", "rotation")
            | ("OrientationInterpolator", "Viewpoint", "orientation") => {
                let (keyframes, values, interpolation) =
                    sanitize_keys(&times, &fl::opt_v4s(interp, "keyValue").unwrap_or_default());
                if keyframes.is_empty() {
                    return out;
                }
                let mut quats: Vec<[f32; 4]> =
                    values.iter().map(|r| axis_angle_to_quat(*r)).collect();
                for i in 1..quats.len() {
                    let (a, b) = (quats[i - 1], quats[i]);
                    if a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] < 0.0 {
                        quats[i] = b.map(|x| -x);
                    }
                }
                for s in scene_targets {
                    out.push(AnimationChannel::new(
                        s,
                        AnimationProperty::Rotation,
                        AnimationSampler {
                            keyframes: keyframes.clone(),
                            values: AnimationValues::Quat(quats.clone()),
                            interpolation,
                        },
                    ));
                }
            }
            ("CoordinateInterpolator", "Coordinate", "point") => {
                out.extend(self.morph_channels(interp, target, &times));
            }
            _ => {}
        }
        out
    }

    /// CoordinateInterpolator → one morph target per key plus a
    /// one-hot MorphWeights channel (linear interpolation of one-hot
    /// weights reproduces the linear coordinate interpolation exactly).
    fn morph_channels(
        &mut self,
        interp: &Node,
        coord: NodeId,
        times: &[f32],
    ) -> Vec<AnimationChannel> {
        const MAX_KEYS: usize = 256;
        let mut out = Vec::new();
        let n_keys = times.len();
        let values = fl::v3s(interp, "keyValue");
        let idx: Vec<usize> = (0..n_keys).collect();
        let (times, kept, _) = sanitize_keys(times, &idx);
        if kept.len() != n_keys {
            return out; // non-finite / decreasing keys: leave unmapped
        }
        if n_keys == 0 || n_keys > MAX_KEYS || values.len() % n_keys != 0 {
            return out;
        }
        let per_key = values.len() / n_keys;
        let Some(users) = self.coord_users.get(&coord).cloned() else {
            return out;
        };
        let mut done = HashSet::new();
        for (mid, pi, src) in users {
            let mesh = &mut self.scene.meshes[mid.0 as usize];
            if mesh.primitives.len() != 1 || !done.insert(mid) {
                continue;
            }
            let prim = &mut mesh.primitives[pi];
            let budget = n_keys.saturating_mul(prim.positions.len());
            if !prim.targets.is_empty()
                || budget > self.opts.max_generated
                || src.iter().any(|&s| s as usize >= per_key)
            {
                continue;
            }
            for k in 0..n_keys {
                let mut t = MorphTarget::new();
                t.position = Some(
                    src.iter()
                        .zip(&prim.positions)
                        .map(|(&s, base)| {
                            let v: V3 = values[k * per_key + s as usize];
                            [v[0] - base[0], v[1] - base[1], v[2] - base[2]]
                        })
                        .collect(),
                );
                prim.targets.push(t);
            }
            mesh.weights = vec![0.0; n_keys];
            let mut weights = vec![0.0f32; n_keys * n_keys];
            for k in 0..n_keys {
                weights[k * n_keys + k] = 1.0;
            }
            let nodes: Vec<SceneNodeId> = self
                .scene
                .nodes
                .iter()
                .enumerate()
                .filter(|(_, n)| n.mesh == Some(mid))
                .map(|(i, _)| SceneNodeId(i as u32))
                .collect();
            for s in nodes {
                out.push(AnimationChannel::new(
                    s,
                    AnimationProperty::MorphWeights,
                    AnimationSampler {
                        keyframes: times.clone(),
                        values: AnimationValues::Scalar(weights.clone()),
                        interpolation: Interpolation::Linear,
                    },
                ));
            }
        }
        out
    }
}

/// Smallest f32 strictly greater than `x` (finite `x`).
fn next_up(x: f32) -> f32 {
    if x == 0.0 {
        return f32::from_bits(1);
    }
    let b = x.to_bits();
    f32::from_bits(if x > 0.0 { b + 1 } else { b - 1 })
}

/// Turn VRML interpolator keys into a valid sampler timeline.
///
/// VRML allows repeated keys (a discontinuity, §4.6.8) and leaves
/// decreasing keys undefined; mesh3d requires strictly increasing
/// finite keyframes. The duplicated-key pattern
/// `t0, t1, t1, t2, t2, …` with values `v0, v0, v1, v1, …` is exactly
/// a step function and is mapped back to [`Interpolation::Step`];
/// other repeats are nudged by one ulp and out-of-order / non-finite
/// keys are dropped.
fn sanitize_keys<T: PartialEq + Clone>(
    times: &[f32],
    values: &[T],
) -> (Vec<f32>, Vec<T>, Interpolation) {
    let n = times.len().min(values.len());
    let (times, values) = (&times[..n], &values[..n]);
    if n >= 3 && n % 2 == 1 {
        let step = (1..n).step_by(2).all(|i| {
            times[i] == times[i + 1] && values[i - 1] == values[i] && times[i - 1] < times[i]
        });
        if step && times.iter().all(|t| t.is_finite()) {
            let t: Vec<f32> = (0..n).step_by(2).map(|i| times[i]).collect();
            let v: Vec<T> = (0..n).step_by(2).map(|i| values[i].clone()).collect();
            return (t, v, Interpolation::Step);
        }
    }
    let mut t_out: Vec<f32> = Vec::with_capacity(n);
    let mut v_out = Vec::with_capacity(n);
    for (t, v) in times.iter().zip(values) {
        if !t.is_finite() {
            continue;
        }
        let t = match t_out.last() {
            Some(&prev) if *t < prev => continue,
            Some(&prev) if *t == prev => next_up(prev),
            _ => *t,
        };
        if !t.is_finite() {
            continue;
        }
        t_out.push(t);
        v_out.push(v.clone());
    }
    (t_out, v_out, Interpolation::Linear)
}

/// `set_x` / `x_changed` → `x`.
fn event_base(name: &str) -> &str {
    let n = name.strip_prefix("set_").unwrap_or(name);
    n.strip_suffix("_changed").unwrap_or(n)
}

/// Blinn-Phong exponent (`shininess × 128`) → GGX-style roughness via
/// the common `α = sqrt(2 / (n + 2))` equivalence.
pub(crate) fn shininess_to_roughness(shininess: f32) -> f32 {
    let n = shininess.clamp(0.0, 1.0) * 128.0;
    (2.0 / (n + 2.0)).sqrt().clamp(0.0, 1.0)
}

/// Inverse of [`shininess_to_roughness`].
pub(crate) fn roughness_to_shininess(roughness: f32) -> f32 {
    let r = roughness.clamp(0.01, 1.0);
    let n = 2.0 / (r * r) - 2.0;
    (n / 128.0).clamp(0.0, 1.0)
}

/// Transform node → mesh3d transform. Returns `(transform, is_plain_trs)`.
///
/// §6.52: `P' = T × C × R × SR × S × −SR × −C × P`. With no `center`
/// and no `scaleOrientation` this is exactly glTF's `T × R × S`.
fn vrml_transform(n: &Node) -> (Transform, bool) {
    use math::*;
    let t = fl::v3(n, "translation", [0.0; 3]);
    let r = fl::v4(n, "rotation", [0.0, 0.0, 1.0, 0.0]);
    let s = fl::v3(n, "scale", [1.0; 3]);
    let c = fl::v3(n, "center", [0.0; 3]);
    let sr = fl::v4(n, "scaleOrientation", [0.0, 0.0, 1.0, 0.0]);
    let sr_identity = sr[3] == 0.0 || s[0] == s[1] && s[1] == s[2];
    if c == [0.0; 3] && sr_identity {
        return (
            Transform::Trs {
                translation: t,
                rotation: axis_angle_to_quat(r),
                scale: s,
            },
            true,
        );
    }
    let neg_sr = [sr[0], sr[1], sr[2], -sr[3]];
    let m = [
        m4_translation(t),
        m4_translation(c),
        m4_rotation(r),
        m4_rotation(sr),
        m4_scale(s),
        m4_rotation(neg_sr),
        m4_translation([-c[0], -c[1], -c[2]]),
    ]
    .iter()
    .fold(m4_identity(), |acc, x| m4_mul(&acc, x));
    (Transform::Matrix(m), false)
}

/// Generic JSON dump of a node (type, DEF name, field values; nested
/// nodes inline up to a small depth).
pub(crate) fn node_json(doc: &Document, id: NodeId, depth: usize) -> Value {
    let Some(n) = doc.node(id) else {
        return Value::Null;
    };
    let mut fields = Map::new();
    for f in &n.fields {
        let v = match &f.binding {
            FieldBinding::Is(t) => json!({ "IS": t }),
            FieldBinding::Value(v) => value_json(doc, v, depth),
        };
        fields.insert(f.name.clone(), v);
    }
    for d in &n.interface {
        if let Some(v) = &d.value {
            fields.insert(d.name.clone(), value_json(doc, v, depth));
        }
    }
    let mut out = Map::new();
    out.insert("type".into(), json!(n.type_name));
    if let Some(d) = &n.def_name {
        out.insert("def".into(), json!(d));
    }
    out.insert("fields".into(), Value::Object(fields));
    Value::Object(out)
}

fn value_json(doc: &Document, v: &FieldValue, depth: usize) -> Value {
    let multi = v.ty.is_multi();
    let comps = v.ty.element().1.max(1);
    let group = |flat: Vec<Value>| -> Value {
        if comps == 1 {
            if multi {
                Value::Array(flat)
            } else {
                flat.into_iter().next().unwrap_or(Value::Null)
            }
        } else {
            let chunks: Vec<Value> = flat
                .chunks(comps)
                .map(|c| Value::Array(c.to_vec()))
                .collect();
            if multi {
                Value::Array(chunks)
            } else {
                chunks.into_iter().next().unwrap_or(Value::Null)
            }
        }
    };
    match &v.data {
        FieldData::Bools(b) => group(b.iter().map(|x| json!(x)).collect()),
        FieldData::Int32s(i) => group(i.iter().map(|x| json!(x)).collect()),
        FieldData::Floats(f) => group(f.iter().map(|x| json!(x)).collect()),
        FieldData::Doubles(d) => group(d.iter().map(|x| json!(x)).collect()),
        FieldData::Strings(s) => group(s.iter().map(|x| json!(x)).collect()),
        FieldData::Images(imgs) => group(
            imgs.iter()
                .map(|i| json!([i.width, i.height, i.components, i.pixels.len()]))
                .collect(),
        ),
        FieldData::Nodes(ids) => {
            let items: Vec<Value> = ids
                .iter()
                .map(|id| {
                    if depth >= 4 {
                        json!({ "ref": doc.node(*id).and_then(|n| n.def_name.clone()) })
                    } else {
                        node_json(doc, *id, depth + 1)
                    }
                })
                .collect();
            if multi {
                Value::Array(items)
            } else {
                items.into_iter().next().unwrap_or(Value::Null)
            }
        }
    }
}
