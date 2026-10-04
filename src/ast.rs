//! Typed, generic VRML scene-graph tree (the parser's output and the
//! writer's input).
//!
//! The tree is deliberately *format-generic*: it models the ISO/IEC
//! 14772-1 Annex A grammar (nodes, fields, `DEF` / `USE`, `PROTO` /
//! `EXTERNPROTO`, `IS`, `ROUTE`) without attaching any rendering
//! semantics, and it preserves node types it does not know. It also
//! carries the additional field types and statements of the X3D
//! ClassicVRML encoding (ISO/IEC 19776-2) so a sibling X3D crate can
//! reuse the same parser and writer.
//!
//! ## Arena layout
//!
//! Every node lives in [`Document::nodes`] and is addressed by a
//! [`NodeId`]. `SFNode` / `MFNode` values store ids, so `USE` simply
//! re-references the id of the `DEF`'d node — the instancing (shared
//! node, multiple parents) semantics of ISO/IEC 14772-1 §4.6.2 are kept
//! exactly. The writer emits `DEF name …` at the first occurrence of a
//! named node in document order and `USE name` afterwards.
//!
//! Note that `USE` inside a node's own subtree makes the graph cyclic
//! (the spec leaves that undefined); consumers walking the tree must
//! guard against revisiting a node on the current path.

/// Index into [`Document::nodes`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeId(pub u32);

/// Index into [`Document::protos`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ProtoId(pub u32);

/// Index into [`Document::extern_protos`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ExternProtoId(pub u32);

/// Interface access type of a field / event declaration.
///
/// VRML97 spells these `eventIn` / `eventOut` / `field` /
/// `exposedField`; X3D (ISO/IEC 19776-2) renamed them `inputOnly` /
/// `outputOnly` / `initializeOnly` / `inputOutput`. The parser accepts
/// either spelling according to its [`Dialect`](crate::syntax::Dialect).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AccessType {
    /// `eventIn` / `inputOnly`.
    EventIn,
    /// `eventOut` / `outputOnly`.
    EventOut,
    /// `field` / `initializeOnly`.
    Field,
    /// `exposedField` / `inputOutput`.
    ExposedField,
}

impl AccessType {
    /// VRML97 keyword.
    pub fn vrml97_keyword(self) -> &'static str {
        match self {
            Self::EventIn => "eventIn",
            Self::EventOut => "eventOut",
            Self::Field => "field",
            Self::ExposedField => "exposedField",
        }
    }

    /// X3D ClassicVRML keyword.
    pub fn x3d_keyword(self) -> &'static str {
        match self {
            Self::EventIn => "inputOnly",
            Self::EventOut => "outputOnly",
            Self::Field => "initializeOnly",
            Self::ExposedField => "inputOutput",
        }
    }

    /// Parse either spelling.
    pub fn from_keyword(word: &str) -> Option<Self> {
        Some(match word {
            "eventIn" | "inputOnly" => Self::EventIn,
            "eventOut" | "outputOnly" => Self::EventOut,
            "field" | "initializeOnly" => Self::Field,
            "exposedField" | "inputOutput" => Self::ExposedField,
            _ => return None,
        })
    }

    /// `true` for the two access types that carry a stored value
    /// (`field` / `exposedField`).
    pub fn has_value(self) -> bool {
        matches!(self, Self::Field | Self::ExposedField)
    }
}

/// An `SFImage` value (ISO/IEC 14772-1 §5.5).
///
/// `pixels` holds `width × height` packed pixel values, bottom row
/// first (left to right, bottom to top). Each value packs
/// `components` bytes, most significant first: intensity; intensity +
/// alpha; R G B; or R G B A.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Image {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Components per pixel (0 for the empty image, else 1..=4).
    pub components: u32,
    /// Packed pixel values, `width × height` entries.
    pub pixels: Vec<u32>,
}

macro_rules! field_types {
    ($( $(#[$m:meta])* $name:ident = $text:literal, multi = $multi:literal, vrml97 = $v97:literal; )*) => {
        /// Every field type of VRML97 (ISO/IEC 14772-1 §5) plus the
        /// additional X3D ClassicVRML field types.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
        pub enum FieldType {
            $( $(#[$m])* $name, )*
        }

        impl FieldType {
            /// All field types, VRML97 ones first.
            pub const ALL: &'static [FieldType] = &[$(FieldType::$name,)*];

            /// Canonical spelling (`"SFVec3f"`, …).
            pub fn name(self) -> &'static str {
                match self { $( Self::$name => $text, )* }
            }

            /// Parse a type keyword.
            pub fn from_name(name: &str) -> Option<Self> {
                match name { $( $text => Some(Self::$name), )* _ => None }
            }

            /// `true` for `MF*` types.
            pub fn is_multi(self) -> bool {
                match self { $( Self::$name => $multi, )* }
            }

            /// `true` when the type exists in VRML97 (ISO/IEC 14772-1).
            pub fn is_vrml97(self) -> bool {
                match self { $( Self::$name => $v97, )* }
            }
        }
    };
}

field_types! {
    /// Single boolean.
    SFBool = "SFBool", multi = false, vrml97 = true;
    /// RGB colour.
    SFColor = "SFColor", multi = false, vrml97 = true;
    /// Single-precision float.
    SFFloat = "SFFloat", multi = false, vrml97 = true;
    /// Uncompressed image.
    SFImage = "SFImage", multi = false, vrml97 = true;
    /// 32-bit integer.
    SFInt32 = "SFInt32", multi = false, vrml97 = true;
    /// Node reference (or `NULL`).
    SFNode = "SFNode", multi = false, vrml97 = true;
    /// Axis + angle rotation.
    SFRotation = "SFRotation", multi = false, vrml97 = true;
    /// UTF-8 string.
    SFString = "SFString", multi = false, vrml97 = true;
    /// Time in seconds (double precision).
    SFTime = "SFTime", multi = false, vrml97 = true;
    /// 2-D float vector.
    SFVec2f = "SFVec2f", multi = false, vrml97 = true;
    /// 3-D float vector.
    SFVec3f = "SFVec3f", multi = false, vrml97 = true;
    /// List of colours.
    MFColor = "MFColor", multi = true, vrml97 = true;
    /// List of floats.
    MFFloat = "MFFloat", multi = true, vrml97 = true;
    /// List of integers.
    MFInt32 = "MFInt32", multi = true, vrml97 = true;
    /// List of nodes.
    MFNode = "MFNode", multi = true, vrml97 = true;
    /// List of rotations.
    MFRotation = "MFRotation", multi = true, vrml97 = true;
    /// List of strings.
    MFString = "MFString", multi = true, vrml97 = true;
    /// List of times.
    MFTime = "MFTime", multi = true, vrml97 = true;
    /// List of 2-D vectors.
    MFVec2f = "MFVec2f", multi = true, vrml97 = true;
    /// List of 3-D vectors.
    MFVec3f = "MFVec3f", multi = true, vrml97 = true;
    /// X3D: list of booleans.
    MFBool = "MFBool", multi = true, vrml97 = false;
    /// X3D: list of images.
    MFImage = "MFImage", multi = true, vrml97 = false;
    /// X3D: RGBA colour.
    SFColorRGBA = "SFColorRGBA", multi = false, vrml97 = false;
    /// X3D: list of RGBA colours.
    MFColorRGBA = "MFColorRGBA", multi = true, vrml97 = false;
    /// X3D: double-precision float.
    SFDouble = "SFDouble", multi = false, vrml97 = false;
    /// X3D: list of doubles.
    MFDouble = "MFDouble", multi = true, vrml97 = false;
    /// X3D: 2-D double vector.
    SFVec2d = "SFVec2d", multi = false, vrml97 = false;
    /// X3D: list of 2-D double vectors.
    MFVec2d = "MFVec2d", multi = true, vrml97 = false;
    /// X3D: 3-D double vector.
    SFVec3d = "SFVec3d", multi = false, vrml97 = false;
    /// X3D: list of 3-D double vectors.
    MFVec3d = "MFVec3d", multi = true, vrml97 = false;
    /// X3D: 4-D float vector.
    SFVec4f = "SFVec4f", multi = false, vrml97 = false;
    /// X3D: list of 4-D float vectors.
    MFVec4f = "MFVec4f", multi = true, vrml97 = false;
    /// X3D: 4-D double vector.
    SFVec4d = "SFVec4d", multi = false, vrml97 = false;
    /// X3D: list of 4-D double vectors.
    MFVec4d = "MFVec4d", multi = true, vrml97 = false;
    /// X3D: 3×3 float matrix (row-major as written).
    SFMatrix3f = "SFMatrix3f", multi = false, vrml97 = false;
    /// X3D: list of 3×3 float matrices.
    MFMatrix3f = "MFMatrix3f", multi = true, vrml97 = false;
    /// X3D: 3×3 double matrix.
    SFMatrix3d = "SFMatrix3d", multi = false, vrml97 = false;
    /// X3D: list of 3×3 double matrices.
    MFMatrix3d = "MFMatrix3d", multi = true, vrml97 = false;
    /// X3D: 4×4 float matrix.
    SFMatrix4f = "SFMatrix4f", multi = false, vrml97 = false;
    /// X3D: list of 4×4 float matrices.
    MFMatrix4f = "MFMatrix4f", multi = true, vrml97 = false;
    /// X3D: 4×4 double matrix.
    SFMatrix4d = "SFMatrix4d", multi = false, vrml97 = false;
    /// X3D: list of 4×4 double matrices.
    MFMatrix4d = "MFMatrix4d", multi = true, vrml97 = false;
}

/// Scalar class of one component of a field type — drives the
/// generic numeric value parser and writer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scalar {
    /// `TRUE` / `FALSE`.
    Bool,
    /// `int32` (decimal or `0x` hex).
    Int32,
    /// Single-precision float.
    Float,
    /// Double-precision float (SFTime / SFDouble / *d types).
    Double,
    /// Quoted string.
    String,
    /// `SFImage` (variable length).
    Image,
    /// Node statement or `NULL`.
    Node,
}

impl FieldType {
    /// Scalar class and component count of one element of this type
    /// (e.g. `SFVec3f` → `(Float, 3)`; `SFImage` → `(Image, 1)`).
    pub fn element(self) -> (Scalar, usize) {
        use FieldType::*;
        match self {
            SFBool | MFBool => (Scalar::Bool, 1),
            SFInt32 | MFInt32 => (Scalar::Int32, 1),
            SFFloat | MFFloat => (Scalar::Float, 1),
            SFTime | MFTime | SFDouble | MFDouble => (Scalar::Double, 1),
            SFString | MFString => (Scalar::String, 1),
            SFImage | MFImage => (Scalar::Image, 1),
            SFNode | MFNode => (Scalar::Node, 1),
            SFVec2f | MFVec2f => (Scalar::Float, 2),
            SFVec3f | MFVec3f | SFColor | MFColor => (Scalar::Float, 3),
            SFRotation | MFRotation | SFColorRGBA | MFColorRGBA | SFVec4f | MFVec4f => {
                (Scalar::Float, 4)
            }
            SFVec2d | MFVec2d => (Scalar::Double, 2),
            SFVec3d | MFVec3d => (Scalar::Double, 3),
            SFVec4d | MFVec4d => (Scalar::Double, 4),
            SFMatrix3f | MFMatrix3f => (Scalar::Float, 9),
            SFMatrix3d | MFMatrix3d => (Scalar::Double, 9),
            SFMatrix4f | MFMatrix4f => (Scalar::Float, 16),
            SFMatrix4d | MFMatrix4d => (Scalar::Double, 16),
        }
    }

    /// The SF counterpart of an MF type (identity for SF types).
    pub fn single(self) -> FieldType {
        use FieldType::*;
        match self {
            MFBool => SFBool,
            MFColor => SFColor,
            MFFloat => SFFloat,
            MFImage => SFImage,
            MFInt32 => SFInt32,
            MFNode => SFNode,
            MFRotation => SFRotation,
            MFString => SFString,
            MFTime => SFTime,
            MFVec2f => SFVec2f,
            MFVec3f => SFVec3f,
            MFColorRGBA => SFColorRGBA,
            MFDouble => SFDouble,
            MFVec2d => SFVec2d,
            MFVec3d => SFVec3d,
            MFVec4f => SFVec4f,
            MFVec4d => SFVec4d,
            MFMatrix3f => SFMatrix3f,
            MFMatrix3d => SFMatrix3d,
            MFMatrix4f => SFMatrix4f,
            MFMatrix4d => SFMatrix4d,
            other => other,
        }
    }
}

/// A typed field value.
///
/// Values are stored flat per scalar class: `floats` for the float
/// families (`SFVec3f` = 3 floats, `MFVec3f` = 3·n floats, …),
/// `doubles` for times / doubles, and so on. The [`FieldValue::ty`]
/// tag says how to group them; the typed accessors
/// ([`FieldValue::as_vec3f`], [`FieldValue::vec3f_list`], …) do the
/// grouping for callers.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldValue {
    /// Field type.
    pub ty: FieldType,
    /// Payload.
    pub data: FieldData,
}

/// Flat payload of a [`FieldValue`].
#[derive(Clone, Debug, PartialEq)]
pub enum FieldData {
    /// `SFBool` / `MFBool`.
    Bools(Vec<bool>),
    /// `SFInt32` / `MFInt32`.
    Int32s(Vec<i32>),
    /// Every single-precision family, flattened.
    Floats(Vec<f32>),
    /// Every double-precision family (incl. `SFTime`), flattened.
    Doubles(Vec<f64>),
    /// `SFString` / `MFString`.
    Strings(Vec<String>),
    /// `SFImage` / `MFImage`.
    Images(Vec<Image>),
    /// `SFNode` (empty = `NULL`) / `MFNode`.
    Nodes(Vec<NodeId>),
}

impl FieldValue {
    /// Empty value of `ty` (zero elements; `NULL` for `SFNode`).
    pub fn empty(ty: FieldType) -> Self {
        let data = match ty.element().0 {
            Scalar::Bool => FieldData::Bools(Vec::new()),
            Scalar::Int32 => FieldData::Int32s(Vec::new()),
            Scalar::Float => FieldData::Floats(Vec::new()),
            Scalar::Double => FieldData::Doubles(Vec::new()),
            Scalar::String => FieldData::Strings(Vec::new()),
            Scalar::Image => FieldData::Images(Vec::new()),
            Scalar::Node => FieldData::Nodes(Vec::new()),
        };
        Self { ty, data }
    }

    /// `SFBool`.
    pub fn sf_bool(v: bool) -> Self {
        Self {
            ty: FieldType::SFBool,
            data: FieldData::Bools(vec![v]),
        }
    }

    /// `SFInt32`.
    pub fn sf_int32(v: i32) -> Self {
        Self {
            ty: FieldType::SFInt32,
            data: FieldData::Int32s(vec![v]),
        }
    }

    /// `MFInt32`.
    pub fn mf_int32(v: Vec<i32>) -> Self {
        Self {
            ty: FieldType::MFInt32,
            data: FieldData::Int32s(v),
        }
    }

    /// `SFFloat`.
    pub fn sf_float(v: f32) -> Self {
        Self {
            ty: FieldType::SFFloat,
            data: FieldData::Floats(vec![v]),
        }
    }

    /// `SFTime`.
    pub fn sf_time(v: f64) -> Self {
        Self {
            ty: FieldType::SFTime,
            data: FieldData::Doubles(vec![v]),
        }
    }

    /// Any float-family type from flat components (`ty` must be a float
    /// type such as `SFVec3f`, `MFFloat`, `MFRotation`, …).
    pub fn floats(ty: FieldType, v: Vec<f32>) -> Self {
        Self {
            ty,
            data: FieldData::Floats(v),
        }
    }

    /// `MFFloat`.
    pub fn mf_float(v: Vec<f32>) -> Self {
        Self::floats(FieldType::MFFloat, v)
    }

    /// `SFVec2f`.
    pub fn sf_vec2f(v: [f32; 2]) -> Self {
        Self::floats(FieldType::SFVec2f, v.to_vec())
    }

    /// `SFVec3f`.
    pub fn sf_vec3f(v: [f32; 3]) -> Self {
        Self::floats(FieldType::SFVec3f, v.to_vec())
    }

    /// `SFColor`.
    pub fn sf_color(v: [f32; 3]) -> Self {
        Self::floats(FieldType::SFColor, v.to_vec())
    }

    /// `SFRotation` (axis x y z, angle).
    pub fn sf_rotation(v: [f32; 4]) -> Self {
        Self::floats(FieldType::SFRotation, v.to_vec())
    }

    /// `MFVec2f`.
    pub fn mf_vec2f(v: &[[f32; 2]]) -> Self {
        Self::floats(FieldType::MFVec2f, v.iter().flatten().copied().collect())
    }

    /// `MFVec3f`.
    pub fn mf_vec3f(v: &[[f32; 3]]) -> Self {
        Self::floats(FieldType::MFVec3f, v.iter().flatten().copied().collect())
    }

    /// `MFColor`.
    pub fn mf_color(v: &[[f32; 3]]) -> Self {
        Self::floats(FieldType::MFColor, v.iter().flatten().copied().collect())
    }

    /// `MFRotation`.
    pub fn mf_rotation(v: &[[f32; 4]]) -> Self {
        Self::floats(FieldType::MFRotation, v.iter().flatten().copied().collect())
    }

    /// `SFString`.
    pub fn sf_string(v: impl Into<String>) -> Self {
        Self {
            ty: FieldType::SFString,
            data: FieldData::Strings(vec![v.into()]),
        }
    }

    /// `MFString`.
    pub fn mf_string(v: Vec<String>) -> Self {
        Self {
            ty: FieldType::MFString,
            data: FieldData::Strings(v),
        }
    }

    /// `SFImage`.
    pub fn sf_image(v: Image) -> Self {
        Self {
            ty: FieldType::SFImage,
            data: FieldData::Images(vec![v]),
        }
    }

    /// `SFNode` (`None` = `NULL`).
    pub fn sf_node(v: Option<NodeId>) -> Self {
        Self {
            ty: FieldType::SFNode,
            data: FieldData::Nodes(v.into_iter().collect()),
        }
    }

    /// `MFNode`.
    pub fn mf_node(v: Vec<NodeId>) -> Self {
        Self {
            ty: FieldType::MFNode,
            data: FieldData::Nodes(v),
        }
    }

    /// Number of elements (vectors / rotations / strings / nodes …).
    pub fn len(&self) -> usize {
        let comps = self.ty.element().1.max(1);
        match &self.data {
            FieldData::Bools(v) => v.len(),
            FieldData::Int32s(v) => v.len(),
            FieldData::Floats(v) => v.len() / comps,
            FieldData::Doubles(v) => v.len() / comps,
            FieldData::Strings(v) => v.len(),
            FieldData::Images(v) => v.len(),
            FieldData::Nodes(v) => v.len(),
        }
    }

    /// `true` when there are no elements.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// First boolean.
    pub fn as_bool(&self) -> Option<bool> {
        match &self.data {
            FieldData::Bools(v) => v.first().copied(),
            _ => None,
        }
    }

    /// First integer.
    pub fn as_int32(&self) -> Option<i32> {
        match &self.data {
            FieldData::Int32s(v) => v.first().copied(),
            _ => None,
        }
    }

    /// Integer list.
    pub fn int32_list(&self) -> Option<&[i32]> {
        match &self.data {
            FieldData::Int32s(v) => Some(v),
            _ => None,
        }
    }

    /// Raw float components (any float family).
    pub fn float_list(&self) -> Option<&[f32]> {
        match &self.data {
            FieldData::Floats(v) => Some(v),
            _ => None,
        }
    }

    /// Raw double components (times / doubles).
    pub fn double_list(&self) -> Option<&[f64]> {
        match &self.data {
            FieldData::Doubles(v) => Some(v),
            _ => None,
        }
    }

    /// First float (also accepts a double-family value).
    pub fn as_float(&self) -> Option<f32> {
        match &self.data {
            FieldData::Floats(v) => v.first().copied(),
            FieldData::Doubles(v) => v.first().map(|d| *d as f32),
            FieldData::Int32s(v) => v.first().map(|i| *i as f32),
            _ => None,
        }
    }

    /// First double (also accepts a float-family value).
    pub fn as_double(&self) -> Option<f64> {
        match &self.data {
            FieldData::Doubles(v) => v.first().copied(),
            FieldData::Floats(v) => v.first().map(|f| *f as f64),
            FieldData::Int32s(v) => v.first().map(|i| *i as f64),
            _ => None,
        }
    }

    fn fixed<const N: usize>(&self) -> Option<[f32; N]> {
        let v = self.float_list()?;
        if v.len() < N {
            return None;
        }
        let mut out = [0.0; N];
        out.copy_from_slice(&v[..N]);
        Some(out)
    }

    fn chunks<const N: usize>(&self) -> Option<Vec<[f32; N]>> {
        let v = self.float_list()?;
        Some(
            v.chunks_exact(N)
                .map(|c| {
                    let mut out = [0.0; N];
                    out.copy_from_slice(c);
                    out
                })
                .collect(),
        )
    }

    /// First 2-vector.
    pub fn as_vec2f(&self) -> Option<[f32; 2]> {
        self.fixed::<2>()
    }

    /// First 3-vector / colour.
    pub fn as_vec3f(&self) -> Option<[f32; 3]> {
        self.fixed::<3>()
    }

    /// First 4-vector / rotation.
    pub fn as_vec4f(&self) -> Option<[f32; 4]> {
        self.fixed::<4>()
    }

    /// 2-vector list.
    pub fn vec2f_list(&self) -> Option<Vec<[f32; 2]>> {
        self.chunks::<2>()
    }

    /// 3-vector / colour list.
    pub fn vec3f_list(&self) -> Option<Vec<[f32; 3]>> {
        self.chunks::<3>()
    }

    /// 4-vector / rotation list.
    pub fn vec4f_list(&self) -> Option<Vec<[f32; 4]>> {
        self.chunks::<4>()
    }

    /// First string.
    pub fn as_string(&self) -> Option<&str> {
        match &self.data {
            FieldData::Strings(v) => v.first().map(String::as_str),
            _ => None,
        }
    }

    /// String list.
    pub fn string_list(&self) -> Option<&[String]> {
        match &self.data {
            FieldData::Strings(v) => Some(v),
            _ => None,
        }
    }

    /// First image.
    pub fn as_image(&self) -> Option<&Image> {
        match &self.data {
            FieldData::Images(v) => v.first(),
            _ => None,
        }
    }

    /// First node (`None` for `NULL` or a non-node value).
    pub fn as_node(&self) -> Option<NodeId> {
        match &self.data {
            FieldData::Nodes(v) => v.first().copied(),
            _ => None,
        }
    }

    /// Node list (empty for a non-node value).
    pub fn node_list(&self) -> &[NodeId] {
        match &self.data {
            FieldData::Nodes(v) => v,
            _ => &[],
        }
    }
}

/// How a field statement inside a node body binds its value.
#[derive(Clone, Debug, PartialEq)]
pub enum FieldBinding {
    /// `name value`.
    Value(FieldValue),
    /// `name IS protoInterfaceName` (only meaningful inside a PROTO
    /// body).
    Is(String),
}

/// One field statement of a node body.
#[derive(Clone, Debug, PartialEq)]
pub struct Field {
    /// Field / event name.
    pub name: String,
    /// Value or `IS` association.
    pub binding: FieldBinding,
    /// `true` when the node type did not declare this field and the
    /// parser inferred the type from the value's shape (unknown node
    /// types, non-standard fields). Writers emit it the same way.
    pub inferred: bool,
}

impl Field {
    /// Construct a value-bound, schema-typed field.
    pub fn value(name: impl Into<String>, value: FieldValue) -> Self {
        Self {
            name: name.into(),
            binding: FieldBinding::Value(value),
            inferred: false,
        }
    }

    /// Construct an `IS`-bound field.
    pub fn is(name: impl Into<String>, target: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            binding: FieldBinding::Is(target.into()),
            inferred: false,
        }
    }
}

/// Where a node's type comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeOrigin {
    /// A type listed in the parser's [`NodeCatalog`](crate::syntax::NodeCatalog)
    /// (the 54 standard VRML97 nodes by default).
    Builtin,
    /// An instance of a `PROTO` declared in scope.
    Proto(ProtoId),
    /// An instance of an `EXTERNPROTO` declared in scope.
    ExternProto(ExternProtoId),
    /// A node type the parser has no declaration for; its fields were
    /// parsed with inferred types.
    Unknown,
}

/// Bookkeeping attached to the root node of an expanded prototype
/// instance by [`expand_protos`](crate::syntax::expand_protos).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ProtoInstance {
    /// Name of the instantiated prototype.
    pub proto_name: String,
    /// The other root nodes of the prototype body (not part of the
    /// transformation hierarchy, but reachable by the instance's
    /// routes — e.g. internal TimeSensors and interpolators).
    pub extra_roots: Vec<NodeId>,
    /// Routes declared inside the prototype body, re-targeted at the
    /// expanded nodes.
    pub routes: Vec<Route>,
    /// `IS` associations: `(interface name, inner node, inner field)`.
    /// Routes addressed to the instance's interface are forwarded
    /// through this table.
    pub is_map: Vec<(String, NodeId, String)>,
}

/// One node statement.
#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    /// Node type name (`"Transform"`, a PROTO name, …).
    pub type_name: String,
    /// `DEF` name, if any.
    pub def_name: Option<String>,
    /// Field statements in source order.
    pub fields: Vec<Field>,
    /// Interface declarations inside the body (Script nodes, and any
    /// other node using the `scriptBodyElement` form).
    pub interface: Vec<InterfaceDecl>,
    /// `ROUTE` / `PROTO` / `EXTERNPROTO` statements that appeared
    /// inside the body.
    pub statements: Vec<Statement>,
    /// Type provenance.
    pub origin: NodeOrigin,
    /// Set on the root node of an expanded prototype instance.
    pub instance: Option<Box<ProtoInstance>>,
}

impl Node {
    /// New empty node of the given type.
    pub fn new(type_name: impl Into<String>) -> Self {
        Self {
            type_name: type_name.into(),
            def_name: None,
            fields: Vec::new(),
            interface: Vec::new(),
            statements: Vec::new(),
            origin: NodeOrigin::Builtin,
            instance: None,
        }
    }

    /// Builder: set the `DEF` name.
    pub fn with_def(mut self, name: impl Into<String>) -> Self {
        self.def_name = Some(name.into());
        self
    }

    /// Builder: append a value-bound field.
    pub fn with_field(mut self, name: impl Into<String>, value: FieldValue) -> Self {
        self.fields.push(Field::value(name, value));
        self
    }

    /// Value of the (last) field statement named `name`, if it is
    /// value-bound.
    pub fn field(&self, name: &str) -> Option<&FieldValue> {
        self.fields.iter().rev().find_map(|f| match &f.binding {
            FieldBinding::Value(v) if f.name == name => Some(v),
            _ => None,
        })
    }

    /// Mutable value of the (last) field named `name`.
    pub fn field_mut(&mut self, name: &str) -> Option<&mut FieldValue> {
        self.fields
            .iter_mut()
            .rev()
            .find_map(|f| match &mut f.binding {
                FieldBinding::Value(v) if f.name == name => Some(v),
                _ => None,
            })
    }

    /// Replace (or append) a value-bound field.
    pub fn set_field(&mut self, name: &str, value: FieldValue) {
        if let Some(slot) = self.field_mut(name) {
            *slot = value;
        } else {
            self.fields.push(Field::value(name, value));
        }
    }

    /// Every node referenced by this node's `SFNode` / `MFNode` field
    /// values, in field order.
    pub fn child_ids(&self) -> impl Iterator<Item = NodeId> + '_ {
        self.fields.iter().flat_map(|f| match &f.binding {
            FieldBinding::Value(v) => v.node_list().iter().copied(),
            FieldBinding::Is(_) => [].iter().copied(),
        })
    }
}

/// An interface declaration (`PROTO` / `EXTERNPROTO` interface, or a
/// Script-style declaration inside a node body).
#[derive(Clone, Debug, PartialEq)]
pub struct InterfaceDecl {
    /// Access type.
    pub access: AccessType,
    /// Field type.
    pub field_type: FieldType,
    /// Field / event name.
    pub name: String,
    /// Initial value (`field` / `exposedField` in a PROTO or Script;
    /// `None` for events and EXTERNPROTO declarations).
    pub value: Option<FieldValue>,
    /// `IS` target for the `scriptBodyElement` form
    /// (`field SFBool x IS y`).
    pub is: Option<String>,
}

/// A `PROTO` declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct ProtoDecl {
    /// Prototype (node type) name.
    pub name: String,
    /// Interface declarations.
    pub interface: Vec<InterfaceDecl>,
    /// Body statements. The first [`Statement::Node`] is the root that
    /// determines the prototype's node type.
    pub body: Vec<Statement>,
}

impl ProtoDecl {
    /// Interface declaration named `name` (also matching the
    /// `set_` / `_changed` event spellings of an exposedField).
    pub fn decl(&self, name: &str) -> Option<&InterfaceDecl> {
        find_decl(&self.interface, name)
    }
}

/// An `EXTERNPROTO` declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct ExternProtoDecl {
    /// Prototype (node type) name.
    pub name: String,
    /// Interface declarations (no values).
    pub interface: Vec<InterfaceDecl>,
    /// Implementation URLs in preference order.
    pub urls: Vec<String>,
}

pub(crate) fn find_decl<'a>(decls: &'a [InterfaceDecl], name: &str) -> Option<&'a InterfaceDecl> {
    if let Some(d) = decls.iter().find(|d| d.name == name) {
        return Some(d);
    }
    let base = name
        .strip_prefix("set_")
        .or_else(|| name.strip_suffix("_changed"))?;
    decls
        .iter()
        .find(|d| d.name == base && d.access == AccessType::ExposedField)
}

/// A `ROUTE` statement.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Route {
    /// Source node `DEF` name.
    pub from_node: String,
    /// Source eventOut name.
    pub from_field: String,
    /// Destination node `DEF` name.
    pub to_node: String,
    /// Destination eventIn name.
    pub to_field: String,
    /// Source node resolved at parse time (`None` when the name was
    /// not defined before the route, which the spec forbids).
    pub from_id: Option<NodeId>,
    /// Destination node resolved at parse time.
    pub to_id: Option<NodeId>,
}

/// One top-level (or PROTO-body, or node-body) statement.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Statement {
    /// Node statement (`node`, `DEF name node`, or `USE name` when
    /// the id was already emitted earlier in document order).
    Node(NodeId),
    /// `PROTO` declaration.
    Proto(ProtoId),
    /// `EXTERNPROTO` declaration.
    ExternProto(ExternProtoId),
    /// `ROUTE` statement.
    Route(Route),
    /// X3D: `PROFILE name`.
    Profile(String),
    /// X3D: `COMPONENT name:level`.
    Component {
        /// Component name.
        name: String,
        /// Support level.
        level: i32,
    },
    /// X3D: `META "name" "content"`.
    Meta {
        /// Metadata key.
        name: String,
        /// Metadata value.
        content: String,
    },
    /// X3D 3.3+: `UNIT category name conversionFactor`.
    Unit {
        /// Unit category (`length`, `angle`, …).
        category: String,
        /// Unit name.
        name: String,
        /// Conversion factor to the base unit.
        factor: f64,
    },
    /// X3D: `IMPORT inlineDef.exportedName [AS localName]`.
    Import {
        /// DEF name of the Inline node.
        inline_def: String,
        /// Name exported by the inlined file.
        exported: String,
        /// Local alias (`AS`).
        as_name: Option<String>,
    },
    /// X3D: `EXPORT localDef [AS exportedName]`.
    Export {
        /// Local DEF name.
        local: String,
        /// Exported alias (`AS`).
        as_name: Option<String>,
    },
}

/// File header line (`#VRML V2.0 utf8 [comment]`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Header {
    /// Format word after `#` (`"VRML"`, or `"X3D"` for ClassicVRML).
    pub format: String,
    /// Version word (`"V2.0"`, `"V3.3"`, …).
    pub version: String,
    /// Encoding word (`"utf8"`).
    pub encoding: String,
    /// Optional trailing comment (without leading whitespace).
    pub comment: String,
}

impl Default for Header {
    fn default() -> Self {
        Self::vrml97()
    }
}

impl Header {
    /// `#VRML V2.0 utf8`.
    pub fn vrml97() -> Self {
        Self {
            format: "VRML".into(),
            version: "V2.0".into(),
            encoding: "utf8".into(),
            comment: String::new(),
        }
    }

    /// `true` for a VRML97 UTF-8 header.
    pub fn is_vrml97(&self) -> bool {
        self.format == "VRML" && self.version == "V2.0" && self.encoding == "utf8"
    }
}

/// A parsed VRML file.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Document {
    /// Header line.
    pub header: Header,
    /// Node arena.
    pub nodes: Vec<Node>,
    /// PROTO declaration arena (top-level, nested and node-body ones).
    pub protos: Vec<ProtoDecl>,
    /// EXTERNPROTO declaration arena.
    pub extern_protos: Vec<ExternProtoDecl>,
    /// Top-level statements in source order.
    pub statements: Vec<Statement>,
}

impl Document {
    /// Empty VRML97 document.
    pub fn new() -> Self {
        Self::default()
    }

    /// Push a node into the arena and return its id.
    pub fn add_node(&mut self, node: Node) -> NodeId {
        let id = NodeId(self.nodes.len() as u32);
        self.nodes.push(node);
        id
    }

    /// Push a PROTO declaration and return its id.
    pub fn add_proto(&mut self, proto: ProtoDecl) -> ProtoId {
        let id = ProtoId(self.protos.len() as u32);
        self.protos.push(proto);
        id
    }

    /// Push an EXTERNPROTO declaration and return its id.
    pub fn add_extern_proto(&mut self, proto: ExternProtoDecl) -> ExternProtoId {
        let id = ExternProtoId(self.extern_protos.len() as u32);
        self.extern_protos.push(proto);
        id
    }

    /// Node by id.
    pub fn node(&self, id: NodeId) -> Option<&Node> {
        self.nodes.get(id.0 as usize)
    }

    /// Mutable node by id.
    pub fn node_mut(&mut self, id: NodeId) -> Option<&mut Node> {
        self.nodes.get_mut(id.0 as usize)
    }

    /// PROTO declaration by id.
    pub fn proto(&self, id: ProtoId) -> Option<&ProtoDecl> {
        self.protos.get(id.0 as usize)
    }

    /// EXTERNPROTO declaration by id.
    pub fn extern_proto(&self, id: ExternProtoId) -> Option<&ExternProtoDecl> {
        self.extern_protos.get(id.0 as usize)
    }

    /// Root node ids (top-level node statements, including `USE`).
    pub fn root_nodes(&self) -> impl Iterator<Item = NodeId> + '_ {
        self.statements.iter().filter_map(|s| match s {
            Statement::Node(id) => Some(*id),
            _ => None,
        })
    }

    /// Top-level routes.
    pub fn routes(&self) -> impl Iterator<Item = &Route> + '_ {
        self.statements.iter().filter_map(|s| match s {
            Statement::Route(r) => Some(r),
            _ => None,
        })
    }
}
