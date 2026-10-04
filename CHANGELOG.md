# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

- Crate scaffold.
- `syntax` + `ast`: format-generic VRML97 (ISO/IEC 14772-1 Annex A)
  lexer, typed recursive-descent parser, writer and PROTO expander.
  All SF/MF field types, `DEF` / `USE` (arena ids, sharing preserved),
  `PROTO` / `EXTERNPROTO` with `IS` mapping, `ROUTE` (resolved to node
  ids), Script-style body interface declarations, MF values with or
  without brackets, comments, unknown node types preserved with
  inferred field types. Pluggable `NodeCatalog` (the 54 standard VRML97
  node interfaces built in) and an `X3dClassic` dialect (X3D access
  keywords, field types, `PROFILE` / `COMPONENT` / `META` / `UNIT` /
  `IMPORT` / `EXPORT`) for reuse by an X3D ClassicVRML reader.
  Hostile-input caps (`ParseLimits`, `ExpandLimits`): nesting depth,
  node / PROTO counts, field sizes, SFImage dimensions, exponential
  PROTO expansion.
- `convert` + `VrmlDecoder`: VRML97 → `Scene3D` (Y-up, metres).
  Grouping nodes (Transform incl. center / scaleOrientation, Group,
  Anchor, Billboard, Collision, Switch, LOD, Inline via `UrlResolver`),
  IndexedFaceSet (crease-angle normals with vertex splitting, concave
  ear clipping, ccw / solid / convex hints, per-vertex / per-face colour
  and normal bindings, default texture mapping), IndexedLineSet,
  PointSet, ElevationGrid, Extrusion (§6.18 SCP algorithm), tessellated
  Box / Sphere / Cone / Cylinder with spec texture layout, Material →
  metallic-roughness (originals in extras), ImageTexture / PixelTexture
  (→ PNG) / MovieTexture / TextureTransform, Viewpoint, lights,
  Background / Fog / NavigationInfo / WorldInfo extras, TimeSensor +
  Position / Orientation / CoordinateInterpolator ROUTEs → animation
  channels and morph targets; gzip input; Latin-1 fallback.
- `VrmlEncoder`: `Scene3D` → VRML97 text (or gzip) with DEF / USE for
  shared meshes and appearances, IndexedFaceSet / IndexedLineSet /
  PointSet, Viewpoint, lights, PixelTexture / `data:` URI textures and
  TimeSensor + interpolator + ROUTE animations; `register()` for the
  mesh3d registry (`vrml`, `wrz`).
