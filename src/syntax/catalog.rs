//! Node-type catalogue: the semantic knowledge the parser needs.
//!
//! ISO/IEC 14772-1 Annex A.1.2: "It is not possible to parse VRML files
//! using a context-free grammar. Semantic knowledge of the names and
//! types of fields, eventIns, and eventOuts for each node type (either
//! built-in or user-defined using PROTO or EXTERNPROTO) shall be used
//! during parsing". PROTO / EXTERNPROTO interfaces are tracked by the
//! parser itself; built-in node types come from a [`NodeCatalog`].
//!
//! [`Vrml97Catalog`] carries the 54 standard node interfaces of clause
//! 6 (transcribed from the node interface blocks of the standard,
//! including the default values as written there). Sibling crates
//! (e.g. X3D ClassicVRML) plug in their own catalogue.

use crate::ast::{AccessType, FieldType};

/// One field / event of a built-in node interface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FieldSchema {
    /// Access type.
    pub access: AccessType,
    /// Field type.
    pub field_type: FieldType,
    /// Field / event name.
    pub name: &'static str,
    /// Default value in VRML syntax (empty for events).
    pub default: &'static str,
}

/// Interface of one built-in node type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeSchema {
    /// Node type name.
    pub name: &'static str,
    /// Interface, in the order the standard lists it.
    pub fields: &'static [FieldSchema],
}

impl NodeSchema {
    /// Look a field / event up by name, also matching the `set_x` /
    /// `x_changed` spellings of an exposedField `x`
    /// (ISO/IEC 14772-1 §4.7).
    pub fn field(&self, name: &str) -> Option<&'static FieldSchema> {
        if let Some(f) = self.fields.iter().find(|f| f.name == name) {
            return Some(f);
        }
        let base = name
            .strip_prefix("set_")
            .or_else(|| name.strip_suffix("_changed"))?;
        self.fields
            .iter()
            .find(|f| f.name == base && f.access == AccessType::ExposedField)
    }
}

/// Source of built-in node interfaces for the parser.
pub trait NodeCatalog {
    /// Schema of the built-in node type `name`, if known.
    fn node(&self, name: &str) -> Option<&NodeSchema>;

    /// Type of field / event `field` of built-in node type `node`.
    fn field_type(&self, node: &str, field: &str) -> Option<FieldType> {
        self.node(node)?.field(field).map(|f| f.field_type)
    }
}

/// The 54 standard VRML97 node types (ISO/IEC 14772-1 clause 6).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Vrml97Catalog;

impl NodeCatalog for Vrml97Catalog {
    fn node(&self, name: &str) -> Option<&NodeSchema> {
        vrml97_node(name)
    }
}

/// Look up a standard VRML97 node interface.
pub fn vrml97_node(name: &str) -> Option<&'static NodeSchema> {
    VRML97_NODES
        .binary_search_by(|n| n.name.cmp(name))
        .ok()
        .map(|i| &VRML97_NODES[i])
}

/// All standard VRML97 node interfaces, sorted by name.
pub fn vrml97_nodes() -> &'static [NodeSchema] {
    VRML97_NODES
}

const fn fd(
    access: AccessType,
    field_type: FieldType,
    name: &'static str,
    default: &'static str,
) -> FieldSchema {
    FieldSchema {
        access,
        field_type,
        name,
        default,
    }
}

use AccessType::{EventIn, EventOut, ExposedField, Field};
use FieldType::*;

static VRML97_NODES: &[NodeSchema] = &[
    NodeSchema {
        name: "Anchor",
        fields: &[
            fd(EventIn, MFNode, "addChildren", ""),
            fd(EventIn, MFNode, "removeChildren", ""),
            fd(ExposedField, MFNode, "children", "[]"),
            fd(ExposedField, SFString, "description", "\"\""),
            fd(ExposedField, MFString, "parameter", "[]"),
            fd(ExposedField, MFString, "url", "[]"),
            fd(Field, SFVec3f, "bboxCenter", "0 0 0"),
            fd(Field, SFVec3f, "bboxSize", "-1 -1 -1"),
        ],
    },
    NodeSchema {
        name: "Appearance",
        fields: &[
            fd(ExposedField, SFNode, "material", "NULL"),
            fd(ExposedField, SFNode, "texture", "NULL"),
            fd(ExposedField, SFNode, "textureTransform", "NULL"),
        ],
    },
    NodeSchema {
        name: "AudioClip",
        fields: &[
            fd(ExposedField, SFString, "description", "\"\""),
            fd(ExposedField, SFBool, "loop", "FALSE"),
            fd(ExposedField, SFFloat, "pitch", "1.0"),
            fd(ExposedField, SFTime, "startTime", "0"),
            fd(ExposedField, SFTime, "stopTime", "0"),
            fd(ExposedField, MFString, "url", "[]"),
            fd(EventOut, SFTime, "duration_changed", ""),
            fd(EventOut, SFBool, "isActive", ""),
        ],
    },
    NodeSchema {
        name: "Background",
        fields: &[
            fd(EventIn, SFBool, "set_bind", ""),
            fd(ExposedField, MFFloat, "groundAngle", "[]"),
            fd(ExposedField, MFColor, "groundColor", "[]"),
            fd(ExposedField, MFString, "backUrl", "[]"),
            fd(ExposedField, MFString, "bottomUrl", "[]"),
            fd(ExposedField, MFString, "frontUrl", "[]"),
            fd(ExposedField, MFString, "leftUrl", "[]"),
            fd(ExposedField, MFString, "rightUrl", "[]"),
            fd(ExposedField, MFString, "topUrl", "[]"),
            fd(ExposedField, MFFloat, "skyAngle", "[]"),
            fd(ExposedField, MFColor, "skyColor", "0 0 0"),
            fd(EventOut, SFBool, "isBound", ""),
        ],
    },
    NodeSchema {
        name: "Billboard",
        fields: &[
            fd(EventIn, MFNode, "addChildren", ""),
            fd(EventIn, MFNode, "removeChildren", ""),
            fd(ExposedField, SFVec3f, "axisOfRotation", "0 1 0"),
            fd(ExposedField, MFNode, "children", "[]"),
            fd(Field, SFVec3f, "bboxCenter", "0 0 0"),
            fd(Field, SFVec3f, "bboxSize", "-1 -1 -1"),
        ],
    },
    NodeSchema {
        name: "Box",
        fields: &[fd(Field, SFVec3f, "size", "2 2 2")],
    },
    NodeSchema {
        name: "Collision",
        fields: &[
            fd(EventIn, MFNode, "addChildren", ""),
            fd(EventIn, MFNode, "removeChildren", ""),
            fd(ExposedField, MFNode, "children", "[]"),
            fd(ExposedField, SFBool, "collide", "TRUE"),
            fd(Field, SFVec3f, "bboxCenter", "0 0 0"),
            fd(Field, SFVec3f, "bboxSize", "-1 -1 -1"),
            fd(Field, SFNode, "proxy", "NULL"),
            fd(EventOut, SFTime, "collideTime", ""),
        ],
    },
    NodeSchema {
        name: "Color",
        fields: &[fd(ExposedField, MFColor, "color", "[]")],
    },
    NodeSchema {
        name: "ColorInterpolator",
        fields: &[
            fd(EventIn, SFFloat, "set_fraction", ""),
            fd(ExposedField, MFFloat, "key", "[]"),
            fd(ExposedField, MFColor, "keyValue", "[]"),
            fd(EventOut, SFColor, "value_changed", ""),
        ],
    },
    NodeSchema {
        name: "Cone",
        fields: &[
            fd(Field, SFFloat, "bottomRadius", "1"),
            fd(Field, SFFloat, "height", "2"),
            fd(Field, SFBool, "side", "TRUE"),
            fd(Field, SFBool, "bottom", "TRUE"),
        ],
    },
    NodeSchema {
        name: "Coordinate",
        fields: &[fd(ExposedField, MFVec3f, "point", "[]")],
    },
    NodeSchema {
        name: "CoordinateInterpolator",
        fields: &[
            fd(EventIn, SFFloat, "set_fraction", ""),
            fd(ExposedField, MFFloat, "key", "[]"),
            fd(ExposedField, MFVec3f, "keyValue", "[]"),
            fd(EventOut, MFVec3f, "value_changed", ""),
        ],
    },
    NodeSchema {
        name: "Cylinder",
        fields: &[
            fd(Field, SFBool, "bottom", "TRUE"),
            fd(Field, SFFloat, "height", "2"),
            fd(Field, SFFloat, "radius", "1"),
            fd(Field, SFBool, "side", "TRUE"),
            fd(Field, SFBool, "top", "TRUE"),
        ],
    },
    NodeSchema {
        name: "CylinderSensor",
        fields: &[
            fd(ExposedField, SFBool, "autoOffset", "TRUE"),
            fd(ExposedField, SFFloat, "diskAngle", "0.262"),
            fd(ExposedField, SFBool, "enabled", "TRUE"),
            fd(ExposedField, SFFloat, "maxAngle", "-1"),
            fd(ExposedField, SFFloat, "minAngle", "0"),
            fd(ExposedField, SFFloat, "offset", "0"),
            fd(EventOut, SFBool, "isActive", ""),
            fd(EventOut, SFRotation, "rotation_changed", ""),
            fd(EventOut, SFVec3f, "trackPoint_changed", ""),
        ],
    },
    NodeSchema {
        name: "DirectionalLight",
        fields: &[
            fd(ExposedField, SFFloat, "ambientIntensity", "0"),
            fd(ExposedField, SFColor, "color", "1 1 1"),
            fd(ExposedField, SFVec3f, "direction", "0 0 -1"),
            fd(ExposedField, SFFloat, "intensity", "1"),
            fd(ExposedField, SFBool, "on", "TRUE"),
        ],
    },
    NodeSchema {
        name: "ElevationGrid",
        fields: &[
            fd(EventIn, MFFloat, "set_height", ""),
            fd(ExposedField, SFNode, "color", "NULL"),
            fd(ExposedField, SFNode, "normal", "NULL"),
            fd(ExposedField, SFNode, "texCoord", "NULL"),
            fd(Field, MFFloat, "height", "[]"),
            fd(Field, SFBool, "ccw", "TRUE"),
            fd(Field, SFBool, "colorPerVertex", "TRUE"),
            fd(Field, SFFloat, "creaseAngle", "0"),
            fd(Field, SFBool, "normalPerVertex", "TRUE"),
            fd(Field, SFBool, "solid", "TRUE"),
            fd(Field, SFInt32, "xDimension", "0"),
            fd(Field, SFFloat, "xSpacing", "1.0"),
            fd(Field, SFInt32, "zDimension", "0"),
            fd(Field, SFFloat, "zSpacing", "1.0"),
        ],
    },
    NodeSchema {
        name: "Extrusion",
        fields: &[
            fd(EventIn, MFVec2f, "set_crossSection", ""),
            fd(EventIn, MFRotation, "set_orientation", ""),
            fd(EventIn, MFVec2f, "set_scale", ""),
            fd(EventIn, MFVec3f, "set_spine", ""),
            fd(Field, SFBool, "beginCap", "TRUE"),
            fd(Field, SFBool, "ccw", "TRUE"),
            fd(Field, SFBool, "convex", "TRUE"),
            fd(Field, SFFloat, "creaseAngle", "0"),
            fd(
                Field,
                MFVec2f,
                "crossSection",
                "[ 1 1, 1 -1, -1 -1, -1 1, 1 1 ]",
            ),
            fd(Field, SFBool, "endCap", "TRUE"),
            fd(Field, MFRotation, "orientation", "0 0 1 0"),
            fd(Field, MFVec2f, "scale", "1 1"),
            fd(Field, SFBool, "solid", "TRUE"),
            fd(Field, MFVec3f, "spine", "[ 0 0 0, 0 1 0 ]"),
        ],
    },
    NodeSchema {
        name: "Fog",
        fields: &[
            fd(ExposedField, SFColor, "color", "1 1 1"),
            fd(ExposedField, SFString, "fogType", "\"LINEAR\""),
            fd(ExposedField, SFFloat, "visibilityRange", "0"),
            fd(EventIn, SFBool, "set_bind", ""),
            fd(EventOut, SFBool, "isBound", ""),
        ],
    },
    NodeSchema {
        name: "FontStyle",
        fields: &[
            fd(Field, MFString, "family", "\"SERIF\""),
            fd(Field, SFBool, "horizontal", "TRUE"),
            fd(Field, MFString, "justify", "\"BEGIN\""),
            fd(Field, SFString, "language", "\"\""),
            fd(Field, SFBool, "leftToRight", "TRUE"),
            fd(Field, SFFloat, "size", "1.0"),
            fd(Field, SFFloat, "spacing", "1.0"),
            fd(Field, SFString, "style", "\"PLAIN\""),
            fd(Field, SFBool, "topToBottom", "TRUE"),
        ],
    },
    NodeSchema {
        name: "Group",
        fields: &[
            fd(EventIn, MFNode, "addChildren", ""),
            fd(EventIn, MFNode, "removeChildren", ""),
            fd(ExposedField, MFNode, "children", "[]"),
            fd(Field, SFVec3f, "bboxCenter", "0 0 0"),
            fd(Field, SFVec3f, "bboxSize", "-1 -1 -1"),
        ],
    },
    NodeSchema {
        name: "ImageTexture",
        fields: &[
            fd(ExposedField, MFString, "url", "[]"),
            fd(Field, SFBool, "repeatS", "TRUE"),
            fd(Field, SFBool, "repeatT", "TRUE"),
        ],
    },
    NodeSchema {
        name: "IndexedFaceSet",
        fields: &[
            fd(EventIn, MFInt32, "set_colorIndex", ""),
            fd(EventIn, MFInt32, "set_coordIndex", ""),
            fd(EventIn, MFInt32, "set_normalIndex", ""),
            fd(EventIn, MFInt32, "set_texCoordIndex", ""),
            fd(ExposedField, SFNode, "color", "NULL"),
            fd(ExposedField, SFNode, "coord", "NULL"),
            fd(ExposedField, SFNode, "normal", "NULL"),
            fd(ExposedField, SFNode, "texCoord", "NULL"),
            fd(Field, SFBool, "ccw", "TRUE"),
            fd(Field, MFInt32, "colorIndex", "[]"),
            fd(Field, SFBool, "colorPerVertex", "TRUE"),
            fd(Field, SFBool, "convex", "TRUE"),
            fd(Field, MFInt32, "coordIndex", "[]"),
            fd(Field, SFFloat, "creaseAngle", "0"),
            fd(Field, MFInt32, "normalIndex", "[]"),
            fd(Field, SFBool, "normalPerVertex", "TRUE"),
            fd(Field, SFBool, "solid", "TRUE"),
            fd(Field, MFInt32, "texCoordIndex", "[]"),
        ],
    },
    NodeSchema {
        name: "IndexedLineSet",
        fields: &[
            fd(EventIn, MFInt32, "set_colorIndex", ""),
            fd(EventIn, MFInt32, "set_coordIndex", ""),
            fd(ExposedField, SFNode, "color", "NULL"),
            fd(ExposedField, SFNode, "coord", "NULL"),
            fd(Field, MFInt32, "colorIndex", "[]"),
            fd(Field, SFBool, "colorPerVertex", "TRUE"),
            fd(Field, MFInt32, "coordIndex", "[]"),
        ],
    },
    NodeSchema {
        name: "Inline",
        fields: &[
            fd(ExposedField, MFString, "url", "[]"),
            fd(Field, SFVec3f, "bboxCenter", "0 0 0"),
            fd(Field, SFVec3f, "bboxSize", "-1 -1 -1"),
        ],
    },
    NodeSchema {
        name: "LOD",
        fields: &[
            fd(ExposedField, MFNode, "level", "[]"),
            fd(Field, SFVec3f, "center", "0 0 0"),
            fd(Field, MFFloat, "range", "[]"),
        ],
    },
    NodeSchema {
        name: "Material",
        fields: &[
            fd(ExposedField, SFFloat, "ambientIntensity", "0.2"),
            fd(ExposedField, SFColor, "diffuseColor", "0.8 0.8 0.8"),
            fd(ExposedField, SFColor, "emissiveColor", "0 0 0"),
            fd(ExposedField, SFFloat, "shininess", "0.2"),
            fd(ExposedField, SFColor, "specularColor", "0 0 0"),
            fd(ExposedField, SFFloat, "transparency", "0"),
        ],
    },
    NodeSchema {
        name: "MovieTexture",
        fields: &[
            fd(ExposedField, SFBool, "loop", "FALSE"),
            fd(ExposedField, SFFloat, "speed", "1.0"),
            fd(ExposedField, SFTime, "startTime", "0"),
            fd(ExposedField, SFTime, "stopTime", "0"),
            fd(ExposedField, MFString, "url", "[]"),
            fd(Field, SFBool, "repeatS", "TRUE"),
            fd(Field, SFBool, "repeatT", "TRUE"),
            fd(EventOut, SFTime, "duration_changed", ""),
            fd(EventOut, SFBool, "isActive", ""),
        ],
    },
    NodeSchema {
        name: "NavigationInfo",
        fields: &[
            fd(EventIn, SFBool, "set_bind", ""),
            fd(ExposedField, MFFloat, "avatarSize", "[0.25, 1.6, 0.75]"),
            fd(ExposedField, SFBool, "headlight", "TRUE"),
            fd(ExposedField, SFFloat, "speed", "1.0"),
            fd(ExposedField, MFString, "type", "[\"WALK\", \"ANY\"]"),
            fd(ExposedField, SFFloat, "visibilityLimit", "0.0"),
            fd(EventOut, SFBool, "isBound", ""),
        ],
    },
    NodeSchema {
        name: "Normal",
        fields: &[fd(ExposedField, MFVec3f, "vector", "[]")],
    },
    NodeSchema {
        name: "NormalInterpolator",
        fields: &[
            fd(EventIn, SFFloat, "set_fraction", ""),
            fd(ExposedField, MFFloat, "key", "[]"),
            fd(ExposedField, MFVec3f, "keyValue", "[]"),
            fd(EventOut, MFVec3f, "value_changed", ""),
        ],
    },
    NodeSchema {
        name: "OrientationInterpolator",
        fields: &[
            fd(EventIn, SFFloat, "set_fraction", ""),
            fd(ExposedField, MFFloat, "key", "[]"),
            fd(ExposedField, MFRotation, "keyValue", "[]"),
            fd(EventOut, SFRotation, "value_changed", ""),
        ],
    },
    NodeSchema {
        name: "PixelTexture",
        fields: &[
            fd(ExposedField, SFImage, "image", "0 0 0"),
            fd(Field, SFBool, "repeatS", "TRUE"),
            fd(Field, SFBool, "repeatT", "TRUE"),
        ],
    },
    NodeSchema {
        name: "PlaneSensor",
        fields: &[
            fd(ExposedField, SFBool, "autoOffset", "TRUE"),
            fd(ExposedField, SFBool, "enabled", "TRUE"),
            fd(ExposedField, SFVec2f, "maxPosition", "-1 -1"),
            fd(ExposedField, SFVec2f, "minPosition", "0 0"),
            fd(ExposedField, SFVec3f, "offset", "0 0 0"),
            fd(EventOut, SFBool, "isActive", ""),
            fd(EventOut, SFVec3f, "trackPoint_changed", ""),
            fd(EventOut, SFVec3f, "translation_changed", ""),
        ],
    },
    NodeSchema {
        name: "PointLight",
        fields: &[
            fd(ExposedField, SFFloat, "ambientIntensity", "0"),
            fd(ExposedField, SFVec3f, "attenuation", "1 0 0"),
            fd(ExposedField, SFColor, "color", "1 1 1"),
            fd(ExposedField, SFFloat, "intensity", "1"),
            fd(ExposedField, SFVec3f, "location", "0 0 0"),
            fd(ExposedField, SFBool, "on", "TRUE"),
            fd(ExposedField, SFFloat, "radius", "100"),
        ],
    },
    NodeSchema {
        name: "PointSet",
        fields: &[
            fd(ExposedField, SFNode, "color", "NULL"),
            fd(ExposedField, SFNode, "coord", "NULL"),
        ],
    },
    NodeSchema {
        name: "PositionInterpolator",
        fields: &[
            fd(EventIn, SFFloat, "set_fraction", ""),
            fd(ExposedField, MFFloat, "key", "[]"),
            fd(ExposedField, MFVec3f, "keyValue", "[]"),
            fd(EventOut, SFVec3f, "value_changed", ""),
        ],
    },
    NodeSchema {
        name: "ProximitySensor",
        fields: &[
            fd(ExposedField, SFVec3f, "center", "0 0 0"),
            fd(ExposedField, SFVec3f, "size", "0 0 0"),
            fd(ExposedField, SFBool, "enabled", "TRUE"),
            fd(EventOut, SFBool, "isActive", ""),
            fd(EventOut, SFVec3f, "position_changed", ""),
            fd(EventOut, SFRotation, "orientation_changed", ""),
            fd(EventOut, SFTime, "enterTime", ""),
            fd(EventOut, SFTime, "exitTime", ""),
        ],
    },
    NodeSchema {
        name: "ScalarInterpolator",
        fields: &[
            fd(EventIn, SFFloat, "set_fraction", ""),
            fd(ExposedField, MFFloat, "key", "[]"),
            fd(ExposedField, MFFloat, "keyValue", "[]"),
            fd(EventOut, SFFloat, "value_changed", ""),
        ],
    },
    NodeSchema {
        name: "Script",
        fields: &[
            fd(ExposedField, MFString, "url", "[]"),
            fd(Field, SFBool, "directOutput", "FALSE"),
            fd(Field, SFBool, "mustEvaluate", "FALSE"),
        ],
    },
    NodeSchema {
        name: "Shape",
        fields: &[
            fd(ExposedField, SFNode, "appearance", "NULL"),
            fd(ExposedField, SFNode, "geometry", "NULL"),
        ],
    },
    NodeSchema {
        name: "Sound",
        fields: &[
            fd(ExposedField, SFVec3f, "direction", "0 0 1"),
            fd(ExposedField, SFFloat, "intensity", "1"),
            fd(ExposedField, SFVec3f, "location", "0 0 0"),
            fd(ExposedField, SFFloat, "maxBack", "10"),
            fd(ExposedField, SFFloat, "maxFront", "10"),
            fd(ExposedField, SFFloat, "minBack", "1"),
            fd(ExposedField, SFFloat, "minFront", "1"),
            fd(ExposedField, SFFloat, "priority", "0"),
            fd(ExposedField, SFNode, "source", "NULL"),
            fd(Field, SFBool, "spatialize", "TRUE"),
        ],
    },
    NodeSchema {
        name: "Sphere",
        fields: &[fd(Field, SFFloat, "radius", "1")],
    },
    NodeSchema {
        name: "SphereSensor",
        fields: &[
            fd(ExposedField, SFBool, "autoOffset", "TRUE"),
            fd(ExposedField, SFBool, "enabled", "TRUE"),
            fd(ExposedField, SFRotation, "offset", "0 1 0 0"),
            fd(EventOut, SFBool, "isActive", ""),
            fd(EventOut, SFRotation, "rotation_changed", ""),
            fd(EventOut, SFVec3f, "trackPoint_changed", ""),
        ],
    },
    NodeSchema {
        name: "SpotLight",
        fields: &[
            fd(ExposedField, SFFloat, "ambientIntensity", "0"),
            fd(ExposedField, SFVec3f, "attenuation", "1 0 0"),
            fd(ExposedField, SFFloat, "beamWidth", "1.570796"),
            fd(ExposedField, SFColor, "color", "1 1 1"),
            fd(ExposedField, SFFloat, "cutOffAngle", "0.785398"),
            fd(ExposedField, SFVec3f, "direction", "0 0 -1"),
            fd(ExposedField, SFFloat, "intensity", "1"),
            fd(ExposedField, SFVec3f, "location", "0 0 0"),
            fd(ExposedField, SFBool, "on", "TRUE"),
            fd(ExposedField, SFFloat, "radius", "100"),
        ],
    },
    NodeSchema {
        name: "Switch",
        fields: &[
            fd(ExposedField, MFNode, "choice", "[]"),
            fd(ExposedField, SFInt32, "whichChoice", "-1"),
        ],
    },
    NodeSchema {
        name: "Text",
        fields: &[
            fd(ExposedField, MFString, "string", "[]"),
            fd(ExposedField, SFNode, "fontStyle", "NULL"),
            fd(ExposedField, MFFloat, "length", "[]"),
            fd(ExposedField, SFFloat, "maxExtent", "0.0"),
        ],
    },
    NodeSchema {
        name: "TextureCoordinate",
        fields: &[fd(ExposedField, MFVec2f, "point", "[]")],
    },
    NodeSchema {
        name: "TextureTransform",
        fields: &[
            fd(ExposedField, SFVec2f, "center", "0 0"),
            fd(ExposedField, SFFloat, "rotation", "0"),
            fd(ExposedField, SFVec2f, "scale", "1 1"),
            fd(ExposedField, SFVec2f, "translation", "0 0"),
        ],
    },
    NodeSchema {
        name: "TimeSensor",
        fields: &[
            fd(ExposedField, SFTime, "cycleInterval", "1"),
            fd(ExposedField, SFBool, "enabled", "TRUE"),
            fd(ExposedField, SFBool, "loop", "FALSE"),
            fd(ExposedField, SFTime, "startTime", "0"),
            fd(ExposedField, SFTime, "stopTime", "0"),
            fd(EventOut, SFTime, "cycleTime", ""),
            fd(EventOut, SFFloat, "fraction_changed", ""),
            fd(EventOut, SFBool, "isActive", ""),
            fd(EventOut, SFTime, "time", ""),
        ],
    },
    NodeSchema {
        name: "TouchSensor",
        fields: &[
            fd(ExposedField, SFBool, "enabled", "TRUE"),
            fd(EventOut, SFVec3f, "hitNormal_changed", ""),
            fd(EventOut, SFVec3f, "hitPoint_changed", ""),
            fd(EventOut, SFVec2f, "hitTexCoord_changed", ""),
            fd(EventOut, SFBool, "isActive", ""),
            fd(EventOut, SFBool, "isOver", ""),
            fd(EventOut, SFTime, "touchTime", ""),
        ],
    },
    NodeSchema {
        name: "Transform",
        fields: &[
            fd(EventIn, MFNode, "addChildren", ""),
            fd(EventIn, MFNode, "removeChildren", ""),
            fd(ExposedField, SFVec3f, "center", "0 0 0"),
            fd(ExposedField, MFNode, "children", "[]"),
            fd(ExposedField, SFRotation, "rotation", "0 0 1 0"),
            fd(ExposedField, SFVec3f, "scale", "1 1 1"),
            fd(ExposedField, SFRotation, "scaleOrientation", "0 0 1 0"),
            fd(ExposedField, SFVec3f, "translation", "0 0 0"),
            fd(Field, SFVec3f, "bboxCenter", "0 0 0"),
            fd(Field, SFVec3f, "bboxSize", "-1 -1 -1"),
        ],
    },
    NodeSchema {
        name: "Viewpoint",
        fields: &[
            fd(EventIn, SFBool, "set_bind", ""),
            fd(ExposedField, SFFloat, "fieldOfView", "0.785398"),
            fd(ExposedField, SFBool, "jump", "TRUE"),
            fd(ExposedField, SFRotation, "orientation", "0 0 1 0"),
            fd(ExposedField, SFVec3f, "position", "0 0 10"),
            fd(Field, SFString, "description", "\"\""),
            fd(EventOut, SFTime, "bindTime", ""),
            fd(EventOut, SFBool, "isBound", ""),
        ],
    },
    NodeSchema {
        name: "VisibilitySensor",
        fields: &[
            fd(ExposedField, SFVec3f, "center", "0 0 0"),
            fd(ExposedField, SFBool, "enabled", "TRUE"),
            fd(ExposedField, SFVec3f, "size", "0 0 0"),
            fd(EventOut, SFTime, "enterTime", ""),
            fd(EventOut, SFTime, "exitTime", ""),
            fd(EventOut, SFBool, "isActive", ""),
        ],
    },
    NodeSchema {
        name: "WorldInfo",
        fields: &[
            fd(Field, MFString, "info", "[]"),
            fd(Field, SFString, "title", "\"\""),
        ],
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_is_sorted_and_complete() {
        assert_eq!(VRML97_NODES.len(), 54);
        for w in VRML97_NODES.windows(2) {
            assert!(w[0].name < w[1].name, "{} !< {}", w[0].name, w[1].name);
        }
    }

    #[test]
    fn exposed_field_event_spellings() {
        let t = vrml97_node("Transform").unwrap();
        assert_eq!(t.field("set_translation").unwrap().name, "translation");
        assert_eq!(t.field("rotation_changed").unwrap().name, "rotation");
        assert!(t.field("set_bboxSize").is_none());
        assert_eq!(
            Vrml97Catalog.field_type("IndexedFaceSet", "coordIndex"),
            Some(FieldType::MFInt32)
        );
    }
}
