//! Geometry nodes → [`Primitive`]s (ISO/IEC 14772-1 §6.7, 6.11, 6.14,
//! 6.17, 6.18, 6.23, 6.24, 6.36, 6.43).
//!
//! Texture coordinates are produced in VRML's `(s, t)` space (origin
//! bottom-left), run through the Appearance's TextureTransform (§6.49)
//! and finally flipped to glTF's top-left origin (`v = 1 − t`).

use std::f32::consts::{PI, TAU};

use oxideav_mesh3d::{Primitive, Topology};

use super::fields as fl;
use super::math::{add, axis_angle_to_quat, cross, dot, normalize, quat_rotate, scale, sub, V3};
use super::mesh::{pack_indices, Built, PolyMesh};
use crate::ast::{Document, Node};

/// Default cap on generated vertices / corners per geometry node.
pub(crate) const MAX_GENERATED: usize = 16 * 1024 * 1024;

/// Tessellation density for the analytic primitives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tessellation {
    /// Segments around the Y axis (Sphere / Cone / Cylinder).
    pub slices: u32,
    /// Latitude bands of a Sphere.
    pub stacks: u32,
}

impl Default for Tessellation {
    fn default() -> Self {
        Self {
            slices: 32,
            stacks: 16,
        }
    }
}

/// A §6.49 TextureTransform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct TexTransform {
    pub center: [f32; 2],
    pub rotation: f32,
    pub scale: [f32; 2],
    pub translation: [f32; 2],
}

impl TexTransform {
    pub(crate) fn from_node(n: &Node) -> Self {
        Self {
            center: fl::v2(n, "center", [0.0, 0.0]),
            rotation: fl::f(n, "rotation", 0.0),
            scale: fl::v2(n, "scale", [1.0, 1.0]),
            translation: fl::v2(n, "translation", [0.0, 0.0]),
        }
    }

    /// `Tc' = −C × S × R × C × T × Tc`.
    fn apply(&self, st: [f32; 2]) -> [f32; 2] {
        let p = [
            st[0] + self.translation[0] + self.center[0],
            st[1] + self.translation[1] + self.center[1],
        ];
        let (s, c) = self.rotation.sin_cos();
        let r = [c * p[0] - s * p[1], s * p[0] + c * p[1]];
        [
            r[0] * self.scale[0] - self.center[0],
            r[1] * self.scale[1] - self.center[1],
        ]
    }
}

/// Context passed down from the Shape / Appearance.
#[derive(Clone, Copy, Debug)]
pub(crate) struct GeomCtx {
    /// Generate texture coordinates (the appearance has a texture).
    pub need_uvs: bool,
    pub tex_transform: Option<TexTransform>,
    pub tess: Tessellation,
    /// Cap on generated faces / vertices / indices per geometry node.
    pub max_generated: usize,
}

impl GeomCtx {
    /// VRML `(s, t)` → glTF `(u, v)`.
    pub(crate) fn uv(&self, st: [f32; 2]) -> [f32; 2] {
        let st = match &self.tex_transform {
            Some(t) => t.apply(st),
            None => st,
        };
        [st[0], 1.0 - st[1]]
    }
}

/// Geometry conversion result.
#[derive(Debug)]
pub(crate) struct GeomOut {
    pub built: Built,
    /// `true` when the geometry is single-sided (`solid TRUE`).
    pub solid: bool,
    /// `true` for lines / points (rendered unlit with emissiveColor).
    pub unlit: bool,
    /// Per-vertex colours present (Color node).
    pub has_colors: bool,
}

pub(crate) fn convert(doc: &Document, n: &Node, ctx: &GeomCtx) -> Option<GeomOut> {
    match n.type_name.as_str() {
        "IndexedFaceSet" => indexed_face_set(doc, n, ctx),
        "ElevationGrid" => elevation_grid(doc, n, ctx),
        "Extrusion" => extrusion(n, ctx),
        "IndexedLineSet" => indexed_line_set(doc, n, ctx.max_generated),
        "PointSet" => point_set(doc, n),
        "Box" => Some(tri(box_prim(n, ctx), true)),
        "Sphere" => Some(tri(sphere(n, ctx), true)),
        "Cylinder" => Some(tri(cylinder(n, ctx), true)),
        "Cone" => Some(tri(cone(n, ctx), true)),
        _ => None,
    }
}

fn tri(prim: Primitive, solid: bool) -> GeomOut {
    GeomOut {
        built: Built {
            prim,
            source_coord: Vec::new(),
        },
        solid,
        unlit: false,
        has_colors: false,
    }
}

fn coords_of(doc: &Document, n: &Node) -> Option<Vec<V3>> {
    let (_, c) = fl::child(doc, n, "coord")?;
    Some(fl::v3s(c, "point"))
}

fn colors_of(doc: &Document, n: &Node) -> Option<Vec<[f32; 4]>> {
    let (_, c) = fl::child(doc, n, "color")?;
    // VRML97 Color is RGB; X3D's ColorRGBA carries alpha.
    if c.type_name == "ColorRGBA" {
        return fl::opt_v4s(c, "color");
    }
    Some(
        fl::v3s(c, "color")
            .into_iter()
            .map(|[r, g, b]| [r, g, b, 1.0])
            .collect(),
    )
}

fn normals_of(doc: &Document, n: &Node) -> Option<Vec<V3>> {
    let (_, c) = fl::child(doc, n, "normal")?;
    Some(fl::v3s(c, "vector"))
}

fn texcoords_of(doc: &Document, n: &Node) -> Option<Vec<[f32; 2]>> {
    let (_, c) = fl::child(doc, n, "texCoord")?;
    fl::opt_v2s(c, "point")
}

/// Attribute lookup for one IFS / ILS stream following the
/// colour-application rules of §6.23.
struct Stream<'a, T: Copy> {
    values: &'a [T],
    index: &'a [i32],
    per_vertex: bool,
}

impl<T: Copy> Stream<'_, T> {
    /// `k` = position in coordIndex, `face` = face ordinal, `coord` =
    /// the coordinate index at `k`.
    fn get(&self, k: usize, face: usize, coord: u32) -> Option<T> {
        let idx = if self.per_vertex {
            if self.index.is_empty() {
                coord as i64
            } else {
                *self.index.get(k)? as i64
            }
        } else if self.index.is_empty() {
            face as i64
        } else {
            *self.index.get(face)? as i64
        };
        if idx < 0 {
            return None;
        }
        self.values.get(idx as usize).copied()
    }
}

/// Split an index list at negative entries. Yields `(start offset,
/// slice)` per polygon / polyline.
fn split_runs(index: &[i32]) -> Vec<(usize, &[i32])> {
    let mut out = Vec::new();
    let mut start = 0;
    for (k, &v) in index.iter().enumerate() {
        if v < 0 {
            out.push((start, &index[start..k]));
            start = k + 1;
        }
    }
    if start < index.len() {
        out.push((start, &index[start..]));
    }
    out
}

/// Default IFS texture mapping (§6.23): S along the longest bounding-box
/// dimension, T along the second, both scaled by the longest size.
fn default_ifs_st(coords: &[V3]) -> impl Fn(V3) -> [f32; 2] {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for p in coords {
        for a in 0..3 {
            min[a] = min[a].min(p[a]);
            max[a] = max[a].max(p[a]);
        }
    }
    let size = [0, 1, 2].map(|a| (max[a] - min[a]).max(0.0));
    let mut order = [0usize, 1, 2];
    // Stable sort keeps X, Y, Z preference on ties.
    order.sort_by(|a, b| {
        size[*b]
            .partial_cmp(&size[*a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let (s_ax, t_ax) = (order[0], order[1]);
    let denom = if size[s_ax] > 0.0 && size[s_ax].is_finite() {
        size[s_ax]
    } else {
        1.0
    };
    move |p: V3| [(p[s_ax] - min[s_ax]) / denom, (p[t_ax] - min[t_ax]) / denom]
}

fn indexed_face_set(doc: &Document, n: &Node, ctx: &GeomCtx) -> Option<GeomOut> {
    let coords = coords_of(doc, n)?;
    let coord_index = fl::ints(n, "coordIndex");
    let colors = colors_of(doc, n);
    let normals = normals_of(doc, n);
    // Explicit texture coordinates are kept even without a texture;
    // default ones are only generated when a texture needs them.
    let tex = texcoords_of(doc, n);
    let want_uvs = ctx.need_uvs || tex.is_some();
    let color_stream = colors.as_deref().map(|values| Stream {
        values,
        index: fl::ints(n, "colorIndex"),
        per_vertex: fl::b(n, "colorPerVertex", true),
    });
    let normal_stream = normals.as_deref().map(|values| Stream {
        values,
        index: fl::ints(n, "normalIndex"),
        per_vertex: fl::b(n, "normalPerVertex", true),
    });
    let tex_index = fl::ints(n, "texCoordIndex");
    let tex_stream = tex.as_deref().map(|values| Stream {
        values,
        index: tex_index,
        per_vertex: true,
    });
    let default_st = default_ifs_st(&coords);

    let mut pm = PolyMesh {
        ccw: fl::b(n, "ccw", true),
        convex: fl::b(n, "convex", true),
        crease_angle: fl::f(n, "creaseAngle", 0.0).max(0.0),
        ..PolyMesh::default()
    };
    let mut c_out = Vec::new();
    let mut n_out = Vec::new();
    let mut t_out = Vec::new();
    for (face_no, (start, run)) in split_runs(coord_index).into_iter().enumerate() {
        if run.len() < 3 || run.iter().any(|&i| i as usize >= coords.len()) {
            continue;
        }
        let face: Vec<u32> = run.iter().map(|&i| i as u32).collect();
        for (j, &ci) in face.iter().enumerate() {
            let k = start + j;
            if let Some(s) = &color_stream {
                c_out.push(s.get(k, face_no, ci).unwrap_or([1.0; 4]));
            }
            if let Some(s) = &normal_stream {
                n_out.push(s.get(k, face_no, ci).unwrap_or([0.0, 0.0, 0.0]));
            }
            if want_uvs {
                let st = match &tex_stream {
                    Some(s) => s.get(k, face_no, ci).unwrap_or([0.0, 0.0]),
                    None => default_st(coords[ci as usize]),
                };
                t_out.push(ctx.uv(st));
            }
        }
        pm.faces.push(face);
        if pm.faces.len() > ctx.max_generated {
            return None;
        }
    }
    let has_colors = color_stream.is_some();
    if has_colors {
        pm.colors = Some(c_out);
    }
    if let Some(s) = &normal_stream {
        let _ = s;
        // Zero-length supplied normals fall back to generation per
        // corner via the builder's facing normal.
        pm.normals = Some(n_out);
    }
    if want_uvs {
        pm.uvs = Some(t_out);
    }
    let solid = fl::b(n, "solid", true);
    pm.coords = coords;
    let mut built = pm.build();
    fix_zero_normals(&mut built.prim);
    Some(GeomOut {
        built,
        solid,
        unlit: false,
        has_colors,
    })
}

/// Replace zero / non-finite normals (missing Normal entries) by the
/// normal of the first triangle using the vertex.
fn fix_zero_normals(prim: &mut Primitive) {
    let tris = prim.triangle_indices();
    let Some(normals) = prim.normals.as_mut() else {
        return;
    };
    let bad = |n: &V3| normalize(*n).is_none();
    // Re-normalise only vectors that are visibly off unit length, so
    // already-unit (e.g. previously exported) normals stay bit-exact.
    let renorm = |n: &mut V3| {
        let l = super::math::len(*n);
        if (l - 1.0).abs() > 1e-5 {
            *n = normalize(*n).unwrap_or(*n);
        }
    };
    if !normals.iter().any(bad) {
        normals.iter_mut().for_each(renorm);
        return;
    }
    let mut face_n: Vec<Option<V3>> = vec![None; normals.len()];
    for t in &tris {
        let [a, b, c] = t.map(|i| prim.positions[i as usize]);
        let fn_ = normalize(cross(sub(b, a), sub(c, a)));
        for &i in t {
            if face_n[i as usize].is_none() {
                face_n[i as usize] = fn_;
            }
        }
    }
    for (i, n) in normals.iter_mut().enumerate() {
        if normalize(*n).is_some() {
            renorm(n);
        } else {
            *n = face_n[i].unwrap_or([0.0, 1.0, 0.0]);
        }
    }
}

fn elevation_grid(doc: &Document, n: &Node, ctx: &GeomCtx) -> Option<GeomOut> {
    let xd = fl::i(n, "xDimension", 0).max(0) as usize;
    let zd = fl::i(n, "zDimension", 0).max(0) as usize;
    let xs = fl::f(n, "xSpacing", 1.0);
    let zs = fl::f(n, "zSpacing", 1.0);
    let height = fl::floats(n, "height");
    if xd < 2 || zd < 2 {
        return None;
    }
    let count = xd.checked_mul(zd)?;
    if count > height.len() || count > ctx.max_generated {
        return None;
    }
    let coords: Vec<V3> = (0..count)
        .map(|k| {
            let (i, j) = (k % xd, k / xd);
            [xs * i as f32, height[k], zs * j as f32]
        })
        .collect();
    let colors = colors_of(doc, n);
    let normals = normals_of(doc, n);
    let tex = texcoords_of(doc, n);
    let want_uvs = ctx.need_uvs || tex.is_some();
    let cpv = fl::b(n, "colorPerVertex", true);
    let npv = fl::b(n, "normalPerVertex", true);
    let mut pm = PolyMesh {
        ccw: fl::b(n, "ccw", true),
        convex: true,
        crease_angle: fl::f(n, "creaseAngle", 0.0).max(0.0),
        ..PolyMesh::default()
    };
    let mut c_out = Vec::new();
    let mut n_out = Vec::new();
    let mut t_out = Vec::new();
    for j in 0..zd - 1 {
        for i in 0..xd - 1 {
            let quad = (i + j * (xd - 1)) as i64;
            // Counter-clockwise seen from +Y (§6.17: default normal +Y).
            let corners = [(i, j), (i, j + 1), (i + 1, j + 1), (i + 1, j)];
            let face: Vec<u32> = corners.iter().map(|(a, b)| (a + b * xd) as u32).collect();
            for (&(ci, cj), &v) in corners.iter().zip(&face) {
                if let Some(c) = &colors {
                    let k = if cpv { v as i64 } else { quad };
                    c_out.push(c.get(k as usize).copied().unwrap_or([1.0; 4]));
                }
                if let Some(nv) = &normals {
                    let k = if npv { v as i64 } else { quad };
                    n_out.push(nv.get(k as usize).copied().unwrap_or([0.0; 3]));
                }
                if want_uvs {
                    let st = match &tex {
                        Some(t) => t.get(v as usize).copied().unwrap_or([0.0; 2]),
                        None => [ci as f32 / (xd - 1) as f32, cj as f32 / (zd - 1) as f32],
                    };
                    t_out.push(ctx.uv(st));
                }
            }
            pm.faces.push(face);
        }
    }
    let has_colors = colors.is_some();
    if has_colors {
        pm.colors = Some(c_out);
    }
    if normals.is_some() {
        pm.normals = Some(n_out);
    }
    if want_uvs {
        pm.uvs = Some(t_out);
    }
    pm.coords = coords;
    let mut built = pm.build();
    fix_zero_normals(&mut built.prim);
    Some(GeomOut {
        built,
        solid: fl::b(n, "solid", true),
        unlit: false,
        has_colors,
    })
}

/// Cumulative normalised arc length along a polyline (0 → 1).
fn arc_params<const N: usize>(pts: &[[f32; N]]) -> Vec<f32> {
    let mut acc = vec![0.0f32; pts.len()];
    for i in 1..pts.len() {
        let d: f32 = (0..N)
            .map(|a| (pts[i][a] - pts[i - 1][a]).powi(2))
            .sum::<f32>()
            .sqrt();
        acc[i] = acc[i - 1] + if d.is_finite() { d } else { 0.0 };
    }
    let total = acc.last().copied().unwrap_or(0.0);
    if total > 0.0 {
        for a in &mut acc {
            *a /= total;
        }
    } else {
        let m = (pts.len().max(2) - 1) as f32;
        for (i, a) in acc.iter_mut().enumerate() {
            *a = i as f32 / m;
        }
    }
    acc
}

/// Spine-aligned cross-section planes (§6.18.2 / §6.18.3): per spine
/// point the `(X, Y, Z)` axes.
fn extrusion_frames(spine: &[V3]) -> Vec<[V3; 3]> {
    let n = spine.len();
    let closed = n > 2 && spine[0] == spine[n - 1];
    let mut ys: Vec<Option<V3>> = (0..n)
        .map(|i| {
            let v = if i == 0 || i == n - 1 {
                if closed {
                    sub(spine[1], spine[n - 2])
                } else if i == 0 {
                    sub(spine[1], spine[0])
                } else {
                    sub(spine[n - 1], spine[n - 2])
                }
            } else {
                sub(spine[i + 1], spine[i - 1])
            };
            normalize(v)
        })
        .collect();
    // Coincident points share the neighbouring SCP's Y axis.
    for i in 1..n {
        if ys[i].is_none() {
            ys[i] = ys[i - 1];
        }
    }
    for i in (0..n.saturating_sub(1)).rev() {
        if ys[i].is_none() {
            ys[i] = ys[i + 1];
        }
    }
    let mut zs: Vec<Option<V3>> = (0..n)
        .map(|i| {
            if i == 0 || i == n - 1 {
                if closed {
                    normalize(cross(sub(spine[1], spine[0]), sub(spine[n - 2], spine[0])))
                } else {
                    None
                }
            } else {
                normalize(cross(
                    sub(spine[i + 1], spine[i]),
                    sub(spine[i - 1], spine[i]),
                ))
            }
        })
        .collect();
    if !closed && n > 2 {
        zs[0] = zs[1];
        zs[n - 1] = zs[n - 2];
    }
    if zs.iter().all(Option::is_none) {
        // Entirely collinear spine: rotate +Y onto the spine direction.
        let dir = ys
            .iter()
            .flatten()
            .next()
            .copied()
            .unwrap_or([0.0, 1.0, 0.0]);
        let q = super::math::quat_between([0.0, 1.0, 0.0], dir);
        let frame = [
            quat_rotate(q, [1.0, 0.0, 0.0]),
            quat_rotate(q, [0.0, 1.0, 0.0]),
            quat_rotate(q, [0.0, 0.0, 1.0]),
        ];
        return vec![frame; n];
    }
    // Undefined Z (collinear triple): previous point's, or for a
    // leading run the first defined one.
    let first = zs.iter().flatten().next().copied();
    let mut prev: Option<V3> = None;
    let mut frames = Vec::with_capacity(n);
    for i in 0..n {
        let mut z = zs[i].or(prev).or(first).unwrap_or([0.0, 0.0, 1.0]);
        if let Some(p) = prev {
            if dot(z, p) < 0.0 {
                z = scale(z, -1.0);
            }
        }
        prev = Some(z);
        let y = ys[i].unwrap_or([0.0, 1.0, 0.0]);
        // Orthonormalise (Z may come from a neighbouring point).
        let z = normalize(sub(z, scale(y, dot(z, y)))).unwrap_or(z);
        let x = cross(y, z);
        frames.push([x, y, z]);
    }
    frames
}

fn extrusion(n: &Node, ctx: &GeomCtx) -> Option<GeomOut> {
    let spine = fl::opt_v3s(n, "spine").unwrap_or_else(|| vec![[0.0; 3], [0.0, 1.0, 0.0]]);
    let cs = fl::opt_v2s(n, "crossSection").unwrap_or_else(|| {
        vec![
            [1.0, 1.0],
            [1.0, -1.0],
            [-1.0, -1.0],
            [-1.0, 1.0],
            [1.0, 1.0],
        ]
    });
    let scales = fl::opt_v2s(n, "scale").unwrap_or_else(|| vec![[1.0, 1.0]]);
    let orients = fl::opt_v4s(n, "orientation").unwrap_or_else(|| vec![[0.0, 0.0, 1.0, 0.0]]);
    let (ns, m) = (spine.len(), cs.len());
    if ns < 2 || m < 2 || ns.checked_mul(m)? > ctx.max_generated {
        return None;
    }
    let frames = extrusion_frames(&spine);
    let pick = |list: &[[f32; 2]], i: usize, d: [f32; 2]| -> [f32; 2] {
        list.get(i).or(list.first()).copied().unwrap_or(d)
    };
    let mut coords = Vec::with_capacity(ns * m);
    for (i, frame) in frames.iter().enumerate() {
        let sc = pick(&scales, i, [1.0, 1.0]);
        let o = orients
            .get(i)
            .or(orients.first())
            .copied()
            .unwrap_or([0.0, 0.0, 1.0, 0.0]);
        let q = axis_angle_to_quat(o);
        for p in &cs {
            let local = quat_rotate(q, [p[0] * sc[0], 0.0, p[1] * sc[1]]);
            let [x, y, z] = *frame;
            let w = add(
                add(scale(x, local[0]), scale(y, local[1])),
                scale(z, local[2]),
            );
            coords.push(add(spine[i], w));
        }
    }
    let u = arc_params(&cs);
    let v = arc_params(&spine);
    let mut pm = PolyMesh {
        ccw: fl::b(n, "ccw", true),
        convex: fl::b(n, "convex", true),
        crease_angle: fl::f(n, "creaseAngle", 0.0).max(0.0),
        ..PolyMesh::default()
    };
    let mut t_out = Vec::new();
    for i in 0..ns - 1 {
        for j in 0..m - 1 {
            let corners = [(i, j), (i, j + 1), (i + 1, j + 1), (i + 1, j)];
            let face: Vec<u32> = corners.iter().map(|(a, b)| (a * m + b) as u32).collect();
            if ctx.need_uvs {
                for &(a, b) in &corners {
                    t_out.push(ctx.uv([u[b], v[a]]));
                }
            }
            pm.faces.push(face);
        }
    }
    // Caps: the cross-section (closing point dropped) at each end.
    let cs_closed = m > 2 && cs[0] == cs[m - 1];
    let cap_len = if cs_closed { m - 1 } else { m };
    if cap_len >= 3 {
        let (mut lo, mut hi) = ([f32::INFINITY; 2], [f32::NEG_INFINITY; 2]);
        for p in &cs {
            for a in 0..2 {
                lo[a] = lo[a].min(p[a]);
                hi[a] = hi[a].max(p[a]);
            }
        }
        let span = (hi[0] - lo[0]).max(hi[1] - lo[1]);
        let span = if span > 0.0 && span.is_finite() {
            span
        } else {
            1.0
        };
        let cap_st = |j: usize| [(cs[j][0] - lo[0]) / span, (cs[j][1] - lo[1]) / span];
        if fl::b(n, "beginCap", true) {
            // Reversed order → normal along −Y of the SCP for ccw.
            let order: Vec<usize> = (0..cap_len).rev().collect();
            pm.faces.push(order.iter().map(|&j| j as u32).collect());
            if ctx.need_uvs {
                t_out.extend(order.iter().map(|&j| ctx.uv(cap_st(j))));
            }
        }
        if fl::b(n, "endCap", true) {
            let base = (ns - 1) * m;
            pm.faces
                .push((0..cap_len).map(|j| (base + j) as u32).collect());
            if ctx.need_uvs {
                t_out.extend((0..cap_len).map(|j| ctx.uv(cap_st(j))));
            }
        }
    }
    if ctx.need_uvs {
        pm.uvs = Some(t_out);
    }
    pm.coords = coords;
    let mut built = pm.build();
    fix_zero_normals(&mut built.prim);
    Some(GeomOut {
        built,
        solid: fl::b(n, "solid", true),
        unlit: false,
        has_colors: false,
    })
}

fn indexed_line_set(doc: &Document, n: &Node, max_generated: usize) -> Option<GeomOut> {
    let coords = coords_of(doc, n)?;
    let colors = colors_of(doc, n);
    let stream = colors.as_deref().map(|values| Stream {
        values,
        index: fl::ints(n, "colorIndex"),
        per_vertex: fl::b(n, "colorPerVertex", true),
    });
    let mut positions = Vec::new();
    let mut out_colors = Vec::new();
    let mut source = Vec::new();
    let mut indices = Vec::new();
    let mut dedup = std::collections::HashMap::new();
    for (line_no, (start, run)) in split_runs(fl::ints(n, "coordIndex"))
        .into_iter()
        .enumerate()
    {
        let mut prev: Option<u32> = None;
        for (j, &ci) in run.iter().enumerate() {
            if ci < 0 || ci as usize >= coords.len() {
                prev = None;
                continue;
            }
            let ci = ci as u32;
            let c = stream
                .as_ref()
                .map(|s| s.get(start + j, line_no, ci).unwrap_or([1.0; 4]));
            let key = (ci, c.map(|c| c.map(f32::to_bits)));
            let idx = *dedup.entry(key).or_insert_with(|| {
                positions.push(coords[ci as usize]);
                out_colors.push(c.unwrap_or([1.0; 4]));
                source.push(ci);
                (positions.len() - 1) as u32
            });
            if let Some(p) = prev {
                indices.extend([p, idx]);
            }
            prev = Some(idx);
        }
        if indices.len() > max_generated {
            return None;
        }
    }
    let mut prim = Primitive::new(Topology::Lines);
    let count = positions.len();
    prim.positions = positions;
    if stream.is_some() {
        prim.colors = vec![out_colors];
    }
    prim.indices = Some(pack_indices(indices, count));
    Some(GeomOut {
        built: Built {
            prim,
            source_coord: source,
        },
        solid: true,
        unlit: true,
        has_colors: stream.is_some(),
    })
}

fn point_set(doc: &Document, n: &Node) -> Option<GeomOut> {
    let coords = coords_of(doc, n)?;
    let colors = colors_of(doc, n).filter(|c| c.len() >= coords.len());
    let mut prim = Primitive::new(Topology::Points);
    let count = coords.len();
    if let Some(mut c) = colors.clone() {
        c.truncate(count);
        prim.colors = vec![c];
    }
    prim.positions = coords;
    Some(GeomOut {
        built: Built {
            prim,
            source_coord: (0..count as u32).collect(),
        },
        solid: true,
        unlit: true,
        has_colors: colors.is_some(),
    })
}

// ---- analytic primitives -------------------------------------------

struct Builder<'c> {
    prim: Primitive,
    pos: Vec<V3>,
    nrm: Vec<V3>,
    uv: Vec<[f32; 2]>,
    idx: Vec<u32>,
    ctx: &'c GeomCtx,
}

impl<'c> Builder<'c> {
    fn new(ctx: &'c GeomCtx) -> Self {
        Self {
            prim: Primitive::new(Topology::Triangles),
            pos: Vec::new(),
            nrm: Vec::new(),
            uv: Vec::new(),
            idx: Vec::new(),
            ctx,
        }
    }

    fn vert(&mut self, p: V3, n: V3, st: [f32; 2]) -> u32 {
        self.pos.push(p);
        self.nrm.push(n);
        self.uv.push(self.ctx.uv(st));
        (self.pos.len() - 1) as u32
    }

    fn tri(&mut self, a: u32, b: u32, c: u32) {
        self.idx.extend([a, b, c]);
    }

    fn finish(mut self) -> Primitive {
        let count = self.pos.len();
        self.prim.positions = self.pos;
        self.prim.normals = Some(self.nrm);
        if self.ctx.need_uvs {
            self.prim.uvs = vec![self.uv];
        }
        self.prim.indices = Some(pack_indices(self.idx, count));
        self.prim
    }
}

/// Positive, finite dimension or the spec default.
fn dim(v: f32, d: f32) -> f32 {
    if v.is_finite() && v > 0.0 {
        v
    } else {
        d
    }
}

fn box_prim(n: &Node, ctx: &GeomCtx) -> Primitive {
    let s = fl::v3(n, "size", [2.0, 2.0, 2.0]);
    let h = [
        dim(s[0], 2.0) / 2.0,
        dim(s[1], 2.0) / 2.0,
        dim(s[2], 2.0) / 2.0,
    ];
    let mut b = Builder::new(ctx);
    // (outward normal, "right", "up") per §6.7 texture orientation.
    let faces: [(V3, V3, V3); 6] = [
        ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        ([0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        ([1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]),
        ([-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
        ([0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
        ([0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ];
    let mul = |v: V3| [v[0] * h[0], v[1] * h[1], v[2] * h[2]];
    for (nrm, r, u) in faces {
        let c = mul(nrm);
        let (r, u) = (mul(r), mul(u));
        let corners = [
            (sub(sub(c, r), u), [0.0, 0.0]),
            (sub(add(c, r), u), [1.0, 0.0]),
            (add(add(c, r), u), [1.0, 1.0]),
            (add(sub(c, r), u), [0.0, 1.0]),
        ];
        let ids = corners.map(|(p, st)| b.vert(p, nrm, st));
        b.tri(ids[0], ids[1], ids[2]);
        b.tri(ids[0], ids[2], ids[3]);
    }
    b.finish()
}

/// Point on a unit circle at texture parameter `s` (0 at −Z, counter-
/// clockwise seen from +Y, §6.43).
fn ring(s: f32) -> (f32, f32) {
    let th = TAU * s;
    (-th.sin(), -th.cos())
}

fn sphere(n: &Node, ctx: &GeomCtx) -> Primitive {
    let r = dim(fl::f(n, "radius", 1.0), 1.0);
    let slices = ctx.tess.slices.clamp(3, 1024) as usize;
    let stacks = ctx.tess.stacks.clamp(2, 1024) as usize;
    let mut b = Builder::new(ctx);
    for j in 0..=stacks {
        let t = j as f32 / stacks as f32;
        let phi = PI * t;
        let (y, rr) = (-phi.cos(), phi.sin());
        for i in 0..=slices {
            let s = i as f32 / slices as f32;
            let (x, z) = ring(s);
            let nrm = [x * rr, y, z * rr];
            b.vert(scale(nrm, r), nrm, [s, t]);
        }
    }
    let w = (slices + 1) as u32;
    for j in 0..stacks as u32 {
        for i in 0..slices as u32 {
            let a = j * w + i;
            let (bb, c, d) = (a + 1, a + w + 1, a + w);
            if j != 0 {
                b.tri(a, bb, c);
            }
            if j + 1 != stacks as u32 {
                b.tri(a, c, d);
            }
        }
    }
    b.finish()
}

fn cap(b: &mut Builder<'_>, y: f32, r: f32, slices: usize, top: bool) {
    let nrm = if top {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, -1.0, 0.0]
    };
    // Top: right side up when tilted toward +Z (t grows toward −Z);
    // bottom: right side up when tilted toward −Z (t grows toward +Z).
    let st = |x: f32, z: f32| {
        if top {
            [0.5 + x / 2.0, 0.5 - z / 2.0]
        } else {
            [0.5 + x / 2.0, 0.5 + z / 2.0]
        }
    };
    let center = b.vert([0.0, y, 0.0], nrm, [0.5, 0.5]);
    let first = b.pos.len() as u32;
    for i in 0..slices {
        let (x, z) = ring(i as f32 / slices as f32);
        b.vert([x * r, y, z * r], nrm, st(x, z));
    }
    for i in 0..slices as u32 {
        let (a, c) = (first + i, first + (i + 1) % slices as u32);
        if top {
            b.tri(center, a, c);
        } else {
            b.tri(center, c, a);
        }
    }
}

fn cylinder(n: &Node, ctx: &GeomCtx) -> Primitive {
    let r = dim(fl::f(n, "radius", 1.0), 1.0);
    let h = dim(fl::f(n, "height", 2.0), 2.0) / 2.0;
    let slices = ctx.tess.slices.clamp(3, 1024) as usize;
    let mut b = Builder::new(ctx);
    if fl::b(n, "side", true) {
        let first = b.pos.len() as u32;
        for i in 0..=slices {
            let s = i as f32 / slices as f32;
            let (x, z) = ring(s);
            b.vert([x * r, -h, z * r], [x, 0.0, z], [s, 0.0]);
            b.vert([x * r, h, z * r], [x, 0.0, z], [s, 1.0]);
        }
        for i in 0..slices as u32 {
            let a = first + 2 * i;
            b.tri(a, a + 2, a + 3);
            b.tri(a, a + 3, a + 1);
        }
    }
    if fl::b(n, "top", true) {
        cap(&mut b, h, r, slices, true);
    }
    if fl::b(n, "bottom", true) {
        cap(&mut b, -h, r, slices, false);
    }
    b.finish()
}

fn cone(n: &Node, ctx: &GeomCtx) -> Primitive {
    let r = dim(fl::f(n, "bottomRadius", 1.0), 1.0);
    let height = dim(fl::f(n, "height", 2.0), 2.0);
    let h = height / 2.0;
    let slices = ctx.tess.slices.clamp(3, 1024) as usize;
    let mut b = Builder::new(ctx);
    if fl::b(n, "side", true) {
        for i in 0..slices {
            let s0 = i as f32 / slices as f32;
            let s1 = (i + 1) as f32 / slices as f32;
            let sm = (s0 + s1) / 2.0;
            let nrm = |s: f32| {
                let (x, z) = ring(s);
                normalize([x * height, r, z * height]).unwrap_or([0.0, 1.0, 0.0])
            };
            let (x0, z0) = ring(s0);
            let (x1, z1) = ring(s1);
            let a = b.vert([x0 * r, -h, z0 * r], nrm(s0), [s0, 0.0]);
            let c = b.vert([x1 * r, -h, z1 * r], nrm(s1), [s1, 0.0]);
            let apex = b.vert([0.0, h, 0.0], nrm(sm), [sm, 1.0]);
            b.tri(a, c, apex);
        }
    }
    if fl::b(n, "bottom", true) {
        cap(&mut b, -h, r, slices, false);
    }
    b.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::parse;

    fn ctx(need_uvs: bool) -> GeomCtx {
        GeomCtx {
            need_uvs,
            tex_transform: None,
            tess: Tessellation::default(),
            max_generated: MAX_GENERATED,
        }
    }

    fn geom(src: &str) -> GeomOut {
        let doc = parse(&format!("#VRML V2.0 utf8\n{src}")).unwrap();
        let id = doc.root_nodes().next().unwrap();
        convert(&doc, doc.node(id).unwrap(), &ctx(true)).unwrap()
    }

    #[test]
    fn primitives_are_outward_and_closed() {
        for src in [
            "Box { size 2 4 6 }",
            "Sphere { radius 2 }",
            "Cylinder { radius 1 height 3 }",
            "Cone { bottomRadius 1 height 2 }",
        ] {
            let g = geom(src);
            let p = &g.built.prim;
            assert!(p.volume() > 0.0, "{src}: volume {}", p.signed_volume());
            assert!(p.signed_volume() > 0.0, "{src}: inward winding");
        }
        let b = geom("Box { size 2 4 6 }");
        assert!((b.built.prim.volume() - 48.0).abs() < 1e-3);
        let bb = b.built.prim.bounding_box().unwrap();
        assert_eq!(bb.max, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn sphere_seam_at_back() {
        let g = geom("Sphere {}");
        let p = &g.built.prim;
        let uvs = &p.uvs[0];
        // A vertex at u = 0.5 on the equator sits at +Z (front).
        let i = uvs
            .iter()
            .position(|uv| (uv[0] - 0.5).abs() < 1e-6 && (uv[1] - 0.5).abs() < 1e-6)
            .unwrap();
        let v = p.positions[i];
        assert!((v[2] - 1.0).abs() < 1e-5, "{v:?}");
    }

    #[test]
    fn elevation_grid_faces_up() {
        let g = geom("ElevationGrid { xDimension 3 zDimension 2 height [0 0 0 0 0 0] }");
        let n = g.built.prim.normals.as_ref().unwrap();
        assert!(n.iter().all(|v| (v[1] - 1.0).abs() < 1e-6));
        assert_eq!(g.built.prim.triangle_count(), 4);
    }

    #[test]
    fn default_extrusion_is_unit_box() {
        let g = geom("Extrusion { }");
        let p = &g.built.prim;
        let bb = p.bounding_box().unwrap();
        assert_eq!(bb.min, [-1.0, 0.0, -1.0]);
        assert_eq!(bb.max, [1.0, 1.0, 1.0]);
        assert!((p.volume() - 4.0).abs() < 1e-4, "{}", p.volume());
        assert!(p.signed_volume() > 0.0);
    }

    #[test]
    fn ifs_per_face_colors_and_texcoords() {
        let g = geom(
            "IndexedFaceSet {
               coord Coordinate { point [0 0 0, 1 0 0, 1 1 0, 0 1 0, 2 0 0, 2 1 0] }
               coordIndex [0 1 2 3 -1 1 4 5 2]
               color Color { color [1 0 0, 0 0 1] }
               colorPerVertex FALSE
               texCoord TextureCoordinate { point [0 0, 1 0, 1 1, 0 1] }
               texCoordIndex [0 1 2 3 -1 0 1 2 3]
             }",
        );
        let p = &g.built.prim;
        assert!(g.has_colors);
        assert_eq!(p.triangle_count(), 4);
        assert_eq!(p.positions.len(), 8);
        let c = &p.colors[0];
        assert!(c.contains(&[1.0, 0.0, 0.0, 1.0]) && c.contains(&[0.0, 0.0, 1.0, 1.0]));
        // t = 1 at the top maps to glTF v = 0.
        let top = p
            .positions
            .iter()
            .position(|q| *q == [0.0, 1.0, 0.0])
            .unwrap();
        assert_eq!(p.uvs[0][top], [0.0, 0.0]);
    }

    #[test]
    fn texture_transform_matrix_order() {
        let t = TexTransform {
            center: [0.0, 0.0],
            rotation: 0.0,
            scale: [2.0, 2.0],
            translation: [0.5, 0.0],
        };
        // Translate then scale.
        assert_eq!(t.apply([0.0, 0.0]), [1.0, 0.0]);
    }

    #[test]
    fn line_and_point_sets() {
        let g = geom(
            "IndexedLineSet { coord Coordinate { point [0 0 0, 1 0 0, 1 1 0] }
                              coordIndex [0 1 2 -1 2 0] }",
        );
        assert_eq!(g.built.prim.topology, Topology::Lines);
        assert_eq!(g.built.prim.indices.as_ref().unwrap().len(), 6);
        let g = geom("PointSet { coord Coordinate { point [0 0 0, 1 0 0] } }");
        assert_eq!(g.built.prim.positions.len(), 2);
    }
}
