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
