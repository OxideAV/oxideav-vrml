//! Polygon-soup → triangle primitive builder shared by IndexedFaceSet,
//! ElevationGrid and Extrusion.
//!
//! Implements the shape-hint semantics of ISO/IEC 14772-1 §4.6.3.4
//! (`ccw`, `convex`) and the crease-angle normal generation of
//! §4.6.3.5: when no normals are supplied, the normal at a face corner
//! is the (area-weighted) average of the geometric normals of the faces
//! sharing that coordinate whose angle to this face is below
//! `creaseAngle` — vertices are split wherever a crease separates them.
//! Non-convex polygons (`convex FALSE`) are triangulated by ear
//! clipping in their best-fit plane.

use std::collections::HashMap;

use oxideav_mesh3d::{Indices, Primitive, Topology};

use super::math::{add, dot, normalize, scale, V3};

/// Per-corner attribute streams are aligned with the flattened corner
/// list (`faces` concatenated).
#[derive(Debug, Default)]
pub(crate) struct PolyMesh {
    pub coords: Vec<V3>,
    /// Faces as coordinate indices (already validated against `coords`).
    pub faces: Vec<Vec<u32>>,
    pub normals: Option<Vec<V3>>,
    pub colors: Option<Vec<[f32; 4]>>,
    pub uvs: Option<Vec<[f32; 2]>>,
    pub ccw: bool,
    pub convex: bool,
    pub crease_angle: f32,
}

/// Result of [`PolyMesh::build`].
#[derive(Debug)]
pub(crate) struct Built {
    pub prim: Primitive,
    /// Source coordinate index of every output vertex (used to derive
    /// CoordinateInterpolator morph targets).
    pub source_coord: Vec<u32>,
}

/// Output-vertex identity: coordinate + normal / colour / uv bits.
type VertexKey = (u32, [u32; 3], [u32; 4], [u32; 2]);

/// Work budget for crease-angle neighbour scans (pairs examined).
const CREASE_BUDGET: u64 = 64 * 1024 * 1024;
/// Polygons larger than this are fan-triangulated even when concave
/// (ear clipping is quadratic).
const EAR_CLIP_MAX: usize = 4096;

/// Newell's method: area-weighted normal (length = 2 × area) following
/// the right-hand rule over the vertex order.
pub(crate) fn newell(points: impl Iterator<Item = V3> + Clone) -> V3 {
    let mut n = [0.0f32; 3];
    let first = points.clone().next();
    let mut it = points.peekable();
    while let Some(a) = it.next() {
        let b = match it.peek() {
            Some(b) => *b,
            None => match first {
                Some(f) => f,
                None => break,
            },
        };
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    n
}

impl PolyMesh {
    fn face_points<'a>(&'a self, f: &'a [u32]) -> impl Iterator<Item = V3> + Clone + 'a {
        f.iter().map(move |&i| self.coords[i as usize])
    }

    /// Triangulate and attribute the polygon soup.
    pub(crate) fn build(&self) -> Built {
        // Facing normals (RH rule; flipped for ccw FALSE).
        let face_n: Vec<V3> = self
            .faces
            .iter()
            .map(|f| {
                let n = newell(self.face_points(f));
                if self.ccw {
                    n
                } else {
                    scale(n, -1.0)
                }
            })
            .collect();
        let corner_normals = match &self.normals {
            Some(n) => n.clone(),
            None => self.generate_normals(&face_n),
        };

        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut colors = Vec::new();
        let mut uvs = Vec::new();
        let mut source_coord = Vec::new();
        let mut indices: Vec<u32> = Vec::new();
        let mut dedup: HashMap<VertexKey, u32> = HashMap::new();

        let mut corner = 0usize;
        for (fi, face) in self.faces.iter().enumerate() {
            let mut local = Vec::with_capacity(face.len());
            for &ci in face {
                let n = corner_normals
                    .get(corner)
                    .copied()
                    .unwrap_or([0.0, 1.0, 0.0]);
                let c = self
                    .colors
                    .as_ref()
                    .and_then(|c| c.get(corner).copied())
                    .unwrap_or([1.0; 4]);
                let t = self
                    .uvs
                    .as_ref()
                    .and_then(|u| u.get(corner).copied())
                    .unwrap_or([0.0; 2]);
                let key = (
                    ci,
                    n.map(f32::to_bits),
                    if self.colors.is_some() {
                        c.map(f32::to_bits)
                    } else {
                        [0; 4]
                    },
                    if self.uvs.is_some() {
                        t.map(f32::to_bits)
                    } else {
                        [0; 2]
                    },
                );
                let idx = *dedup.entry(key).or_insert_with(|| {
                    positions.push(self.coords[ci as usize]);
                    normals.push(n);
                    colors.push(c);
                    uvs.push(t);
                    source_coord.push(ci);
                    (positions.len() - 1) as u32
                });
                local.push(idx);
                corner += 1;
            }
            for [a, b, c] in self.triangulate(face, face_n[fi]) {
                let (a, b, c) = (local[a], local[b], local[c]);
                if self.ccw {
                    indices.extend([a, b, c]);
                } else {
                    indices.extend([a, c, b]);
                }
            }
        }

        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = positions;
        prim.normals = Some(normals);
        if self.colors.is_some() {
            prim.colors = vec![colors];
        }
        if self.uvs.is_some() {
            prim.uvs = vec![uvs];
        }
        prim.indices = Some(pack_indices(indices, prim.positions.len()));
        Built { prim, source_coord }
    }

    fn generate_normals(&self, face_n: &[V3]) -> Vec<V3> {
        let unit: Vec<Option<V3>> = face_n.iter().map(|n| normalize(*n)).collect();
        // vertex → incident faces (CSR).
        let nv = self.coords.len();
        let mut counts = vec![0u32; nv + 1];
        for f in &self.faces {
            for &v in f {
                counts[v as usize + 1] += 1;
            }
        }
        for i in 0..nv {
            counts[i + 1] += counts[i];
        }
        let mut fill = counts.clone();
        let mut incident = vec![0u32; counts[nv] as usize];
        for (fi, f) in self.faces.iter().enumerate() {
            for &v in f {
                let slot = &mut fill[v as usize];
                incident[*slot as usize] = fi as u32;
                *slot += 1;
            }
        }
        let all_smooth = self.crease_angle >= std::f32::consts::PI;
        let cos_crease = self.crease_angle.cos();
        // Per-vertex full sums (fast path for creaseAngle ≥ π).
        let vertex_sum = |v: usize| -> V3 {
            let mut s = [0.0; 3];
            for &g in &incident[counts[v] as usize..counts[v + 1] as usize] {
                s = add(s, face_n[g as usize]);
            }
            s
        };
        let mut budget = CREASE_BUDGET;
        let mut out = Vec::new();
        for (fi, f) in self.faces.iter().enumerate() {
            let own = face_n[fi];
            let fallback = unit[fi].unwrap_or([0.0, 1.0, 0.0]);
            for &v in f {
                let v = v as usize;
                let range = counts[v] as usize..counts[v + 1] as usize;
                let n = if self.crease_angle <= 0.0 {
                    own
                } else if all_smooth {
                    vertex_sum(v)
                } else if budget < range.len() as u64 {
                    own
                } else {
                    budget -= range.len() as u64;
                    let mut s = own;
                    if let Some(u) = unit[fi] {
                        for &g in &incident[range] {
                            let g = g as usize;
                            if g == fi {
                                continue;
                            }
                            if let Some(ug) = unit[g] {
                                if dot(u, ug) > cos_crease {
                                    s = add(s, face_n[g]);
                                }
                            }
                        }
                    }
                    s
                };
                out.push(normalize(n).unwrap_or(fallback));
            }
        }
        out
    }

    /// Local-corner triangles of one face.
    fn triangulate(&self, face: &[u32], normal: V3) -> Vec<[usize; 3]> {
        let n = face.len();
        if n < 3 {
            return Vec::new();
        }
        if n == 3 {
            return vec![[0, 1, 2]];
        }
        if self.convex || n > EAR_CLIP_MAX {
            return (1..n - 1).map(|i| [0, i, i + 1]).collect();
        }
        // Project onto the plane perpendicular to the dominant axis of
        // the (RH-rule) polygon normal; that keeps the polygon's own
        // orientation positive in 2-D.
        let rh = if self.ccw {
            normal
        } else {
            scale(normal, -1.0)
        };
        let ax = [rh[0].abs(), rh[1].abs(), rh[2].abs()];
        let (u, v, flip) = if ax[0] >= ax[1] && ax[0] >= ax[2] {
            (1, 2, rh[0] < 0.0)
        } else if ax[1] >= ax[2] {
            (2, 0, rh[1] < 0.0)
        } else {
            (0, 1, rh[2] < 0.0)
        };
        let pts: Vec<[f32; 2]> = face
            .iter()
            .map(|&i| {
                let p = self.coords[i as usize];
                if flip {
                    [p[v], p[u]]
                } else {
                    [p[u], p[v]]
                }
            })
            .collect();
        ear_clip(&pts)
    }
}

fn cross2(o: [f32; 2], a: [f32; 2], b: [f32; 2]) -> f32 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

fn point_in_tri(p: [f32; 2], a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> bool {
    let d1 = cross2(a, b, p);
    let d2 = cross2(b, c, p);
    let d3 = cross2(c, a, p);
    d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0
}

/// Ear clipping of a simple polygon whose 2-D winding is
/// counter-clockwise (positive area). Falls back to a fan for whatever
/// remains if no ear can be found (degenerate / self-intersecting).
pub(crate) fn ear_clip(pts: &[[f32; 2]]) -> Vec<[usize; 3]> {
    let mut idx: Vec<usize> = (0..pts.len()).collect();
    // Make the working order CCW.
    let area: f32 = (0..pts.len())
        .map(|i| {
            let a = pts[i];
            let b = pts[(i + 1) % pts.len()];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    let reversed = area < 0.0;
    if reversed {
        idx.reverse();
    }
    let mut out = Vec::with_capacity(pts.len().saturating_sub(2));
    let mut guard = 0usize;
    while idx.len() > 3 && guard < pts.len() * pts.len() {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        for i in 0..m {
            let (ia, ib, ic) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            let (a, b, c) = (pts[ia], pts[ib], pts[ic]);
            if cross2(a, b, c) <= 0.0 {
                continue; // reflex or degenerate
            }
            let blocked = idx
                .iter()
                .any(|&j| j != ia && j != ib && j != ic && point_in_tri(pts[j], a, b, c));
            if blocked {
                continue;
            }
            out.push([ia, ib, ic]);
            idx.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            break;
        }
    }
    if idx.len() >= 3 {
        for i in 1..idx.len() - 1 {
            out.push([idx[0], idx[i], idx[i + 1]]);
        }
    }
    if reversed {
        // Restore the caller's winding.
        for t in &mut out {
            t.swap(1, 2);
        }
    }
    out
}

pub(crate) fn pack_indices(indices: Vec<u32>, vertex_count: usize) -> Indices {
    if vertex_count <= u16::MAX as usize + 1 {
        Indices::U16(indices.into_iter().map(|i| i as u16).collect())
    } else {
        Indices::U32(indices)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad_pair(crease: f32) -> Built {
        // Two quads meeting at a 90° edge along x = 0.
        let coords = vec![
            [-1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, -1.0],
            [0.0, -1.0, 1.0],
            [0.0, -1.0, -1.0],
        ];
        PolyMesh {
            coords,
            faces: vec![vec![0, 1, 2, 3], vec![1, 4, 5, 2]],
            ccw: true,
            convex: true,
            crease_angle: crease,
            ..PolyMesh::default()
        }
        .build()
    }

    #[test]
    fn crease_splits_and_smooths() {
        let sharp = quad_pair(0.5);
        assert_eq!(sharp.prim.positions.len(), 8);
        let smooth = quad_pair(2.0);
        assert_eq!(smooth.prim.positions.len(), 6);
        let n = smooth.prim.normals.as_ref().unwrap();
        let s = std::f32::consts::FRAC_1_SQRT_2;
        assert!(n
            .iter()
            .any(|v| (v[0] - s).abs() < 1e-5 && (v[1] - s).abs() < 1e-5));
    }

    #[test]
    fn concave_polygon_ear_clip() {
        // An L-shape (concave) in the XY plane, CCW seen from +Z.
        let coords = vec![
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 2.0, 0.0],
            [0.0, 2.0, 0.0],
        ];
        let b = PolyMesh {
            coords,
            faces: vec![vec![0, 1, 2, 3, 4, 5]],
            ccw: true,
            convex: false,
            ..PolyMesh::default()
        }
        .build();
        assert_eq!(b.prim.triangle_count(), 4);
        // Total area must be 3 (fan triangulation would overlap).
        let area = b.prim.surface_area();
        assert!((area - 3.0).abs() < 1e-5, "{area}");
    }

    #[test]
    fn ccw_false_flips_winding_and_normals() {
        let b = PolyMesh {
            coords: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            faces: vec![vec![0, 1, 2]],
            ccw: false,
            convex: true,
            ..PolyMesh::default()
        }
        .build();
        assert_eq!(b.prim.normals.as_ref().unwrap()[0], [0.0, 0.0, -1.0]);
        assert_eq!(b.prim.triangle_indices(), vec![[0, 2, 1]]);
    }
}
