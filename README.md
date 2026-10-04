# oxideav-vrml

Pure-Rust VRML97 (ISO/IEC 14772-1:1997) reader and writer — implements
`oxideav-mesh3d`'s `Mesh3DDecoder` / `Mesh3DEncoder` traits.

The crate has two layers:

* **`syntax` + `ast`** — a format-generic lexer, typed parser, writer
  and PROTO expander for the UTF-8 grammar of Annex A. It knows nothing
  about the 3-D scene model, preserves unknown nodes, and also speaks
  the X3D ClassicVRML dialect so an X3D reader can reuse it.
* **`convert` / `decoder` / `encoder`** — the mapping between the VRML
  scene graph and `oxideav_mesh3d::Scene3D` (glTF-2.0-aligned, Y-up,
  metres — the same conventions as VRML, so no re-orientation happens).

## Status

| Area | Support |
|------|---------|
| Lexer / parser | All 20 VRML97 field types (SF/MF Bool, Color, Float, Image, Int32, Node, Rotation, String, Time, Vec2f, Vec3f), MF values with or without brackets, comments, `DEF` / `USE` (shared arena ids), `PROTO` / `EXTERNPROTO` with `IS`, `ROUTE` (resolved to node ids), Script body interface declarations, unknown node types (fields typed by shape inference), UTF-8 with Latin-1 fallback, gzip (`.wrz`) input |
| X3D ClassicVRML dialect | access keywords `inputOnly` … `inputOutput`, X3D field types (MFBool, SFDouble, SFVec3d, SFColorRGBA, SFMatrix4f, …), `PROFILE` / `COMPONENT` / `META` / `UNIT` / `IMPORT` / `EXPORT` |
| Writer | canonical indented text, automatic `DEF` / `USE` for shared nodes, shortest round-trip floats; `parse(write(doc)) == doc` |
| PROTO expansion | instance values / interface defaults substituted through `IS`, nested prototypes, extra body roots + body routes kept, `IS` table for forwarding routes to instance interfaces; `EXTERNPROTO` instances kept (unexpanded) |
| Grouping | Transform (TRS, or matrix when `center` / `scaleOrientation` are used), Group, Anchor, Billboard, Collision, Switch (active or all choices), LOD (finest or all levels), Inline (through a `UrlResolver`, otherwise kept as a reference) |
| Geometry | IndexedFaceSet (convex / concave ear clipping, ccw, solid → double-sided, creaseAngle normal generation splitting vertices across creases, per-vertex / per-face colours and normals, indexed or implicit texture coordinates, default bounding-box texture mapping), IndexedLineSet, PointSet, ElevationGrid, Extrusion (spine-aligned cross-section planes per §6.18, scale / orientation, caps, texture coordinates), Box / Sphere / Cone / Cylinder tessellated per spec dimensions and texture layout |
| Appearance | Material (Phong → metallic-roughness approximation, originals in `vrml:material` extras), lighting-off → unlit, Color-node / RGB-texture diffuse replacement (Table 4.6), ImageTexture (URI or `data:` URI), PixelTexture (→ in-memory PNG), MovieTexture (URI), TextureTransform (baked into UVs), repeatS / repeatT |
| Cameras / lights | Viewpoint → perspective camera (near / far from NavigationInfo); Directional / Point / Spot lights (`on`, attenuation, ambientIntensity, radius in `vrml:light` extras) |
| Animation | TimeSensor + PositionInterpolator / OrientationInterpolator → translation / scale / rotation channels; CoordinateInterpolator → morph targets with one-hot weight channels; repeated keys → step interpolation; routes forwarded through PROTO `IS` interfaces. Other interpolators / sensors / scripts kept in `vrml:behaviour`, all routes in `vrml:routes` |
| Environment | Background, Fog, NavigationInfo, WorldInfo preserved as scene extras (and re-emitted by the encoder) |
| Encoder | Transform / Group hierarchy, shared meshes and appearances via `DEF` / `USE`, IndexedFaceSet with explicit normals / UVs / colours, IndexedLineSet, PointSet, Viewpoint, lights, PixelTexture / `data:` URI embedding, animations as TimeSensor + interpolators + ROUTEs (including morph targets as CoordinateInterpolator); optional gzip output |
| Not yet | Text / FontStyle geometry (recorded in `vrml:unsupportedGeometry`), Sound / AudioClip, sensors and Script execution, EXTERNPROTO resolution, ColorInterpolator / NormalInterpolator / ScalarInterpolator animation mapping |

Hostile input is bounded everywhere (`ParseLimits`, `ExpandLimits`,
scene-node and generated-geometry caps, gzip inflate cap); the crate
contains no `unsafe` and a cargo-fuzz harness lives in `fuzz/`.

## Usage

```rust
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_vrml::{VrmlDecoder, VrmlEncoder};

let bytes = std::fs::read("world.wrl")?; // or a gzip-compressed .wrz
let scene = VrmlDecoder::new().decode(&bytes)?;
let out = VrmlEncoder::new().encode(&scene)?;
std::fs::write("copy.wrl", out)?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

Syntax layer only:

```rust
use oxideav_vrml::syntax::{expand_protos, parse, write_document};

let doc = parse("#VRML V2.0 utf8\nDEF B Box { size 1 2 3 }\nShape { geometry USE B }")?;
let flat = expand_protos(&doc)?;
println!("{}", write_document(&flat));
# Ok::<(), oxideav_vrml::Error>(())
```

`Inline` URLs are followed when a resolver is supplied:

```rust
use std::sync::Arc;
use oxideav_vrml::{UrlResolver, VrmlDecoder};

struct Dir;
impl UrlResolver for Dir {
    fn resolve(&self, url: &str) -> Option<Vec<u8>> {
        std::fs::read(url).ok()
    }
}
let decoder = VrmlDecoder::new().with_resolver(Arc::new(Dir));
```

### Standalone build

`oxideav-core` is gated behind the default-on `registry` feature, which
also provides `register(&mut Mesh3DRegistry)` (format ids `vrml` —
`.wrl` / `.wrz` / `.vrml` — and `wrz` for gzip output). With
`default-features = false` everything else stays available and errors
use the crate-local `oxideav_vrml::Error`.

## Clean-room note

Implemented from the ISO/IEC 14772-1:1997 text published by the Web3D
Consortium (mirrored with provenance in the OxideAV `docs` repository
under `3d/vrml/`) and general published literature only. No source code
of other VRML / X3D implementations was consulted.

## License

MIT — see `LICENSE`.
