//! Defaulting field accessors over [`crate::ast::Node`].
//!
//! Every accessor takes the default from the node interface of
//! ISO/IEC 14772-1 clause 6 (passed in by the caller), so a field
//! that is absent — or present with the wrong shape — reads as the
//! spec default instead of failing.

use crate::ast::{Document, Node, NodeId};

pub(crate) fn b(n: &Node, k: &str, d: bool) -> bool {
    n.field(k).and_then(|v| v.as_bool()).unwrap_or(d)
}

pub(crate) fn f(n: &Node, k: &str, d: f32) -> f32 {
    n.field(k)
        .and_then(|v| v.as_float())
        .filter(|x| x.is_finite())
        .unwrap_or(d)
}

pub(crate) fn d64(n: &Node, k: &str, d: f64) -> f64 {
    n.field(k)
        .and_then(|v| v.as_double())
        .filter(|x| x.is_finite())
        .unwrap_or(d)
}

pub(crate) fn i(n: &Node, k: &str, d: i32) -> i32 {
    n.field(k).and_then(|v| v.as_int32()).unwrap_or(d)
}

pub(crate) fn v2(n: &Node, k: &str, d: [f32; 2]) -> [f32; 2] {
    n.field(k)
        .and_then(|v| v.as_vec2f())
        .filter(|a| a.iter().all(|x| x.is_finite()))
        .unwrap_or(d)
}

pub(crate) fn v3(n: &Node, k: &str, d: [f32; 3]) -> [f32; 3] {
    n.field(k)
        .and_then(|v| v.as_vec3f())
        .filter(|a| a.iter().all(|x| x.is_finite()))
        .unwrap_or(d)
}

pub(crate) fn v4(n: &Node, k: &str, d: [f32; 4]) -> [f32; 4] {
    n.field(k)
        .and_then(|v| v.as_vec4f())
        .filter(|a| a.iter().all(|x| x.is_finite()))
        .unwrap_or(d)
}

pub(crate) fn ints<'a>(n: &'a Node, k: &str) -> &'a [i32] {
    n.field(k).and_then(|v| v.int32_list()).unwrap_or(&[])
}

pub(crate) fn floats<'a>(n: &'a Node, k: &str) -> &'a [f32] {
    n.field(k).and_then(|v| v.float_list()).unwrap_or(&[])
}

pub(crate) fn opt_v2s(n: &Node, k: &str) -> Option<Vec<[f32; 2]>> {
    n.field(k).and_then(|v| v.vec2f_list())
}

pub(crate) fn opt_v3s(n: &Node, k: &str) -> Option<Vec<[f32; 3]>> {
    n.field(k).and_then(|v| v.vec3f_list())
}

pub(crate) fn opt_v4s(n: &Node, k: &str) -> Option<Vec<[f32; 4]>> {
    n.field(k).and_then(|v| v.vec4f_list())
}

pub(crate) fn v3s(n: &Node, k: &str) -> Vec<[f32; 3]> {
    opt_v3s(n, k).unwrap_or_default()
}

pub(crate) fn strs<'a>(n: &'a Node, k: &str) -> &'a [String] {
    n.field(k).and_then(|v| v.string_list()).unwrap_or(&[])
}

pub(crate) fn string<'a>(n: &'a Node, k: &str) -> &'a str {
    n.field(k).and_then(|v| v.as_string()).unwrap_or("")
}

/// The node in SFNode field `k`, if any.
pub(crate) fn child<'d>(doc: &'d Document, n: &Node, k: &str) -> Option<(NodeId, &'d Node)> {
    let id = n.field(k)?.as_node()?;
    Some((id, doc.node(id)?))
}

/// The nodes in MFNode field `k`.
pub(crate) fn children<'a>(n: &'a Node, k: &str) -> &'a [NodeId] {
    n.field(k).map(|v| v.node_list()).unwrap_or(&[])
}
