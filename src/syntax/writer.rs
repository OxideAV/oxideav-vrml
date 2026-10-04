//! Serialiser: [`Document`] → UTF-8 VRML text.
//!
//! Output is canonical, indented VRML97 (or X3D ClassicVRML with the
//! [`Dialect::X3dClassic`] dialect). Shared nodes are written as
//! `DEF name …` at their first occurrence in document order and as
//! `USE name` afterwards; shared nodes without a `DEF` name get a
//! synthetic one (`_N<index>`), as do nodes referenced by a `ROUTE`
//! through an id only.
//!
//! Floats are written with Rust's shortest round-trip representation,
//! so `parse(write(doc))` reproduces every value bit-for-bit;
//! non-finite values (which VRML cannot express) are written as `0`.

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;

use crate::ast::{
    Document, FieldBinding, FieldData, FieldType, FieldValue, Image, InterfaceDecl, Node, NodeId,
    Route, Statement,
};
use crate::syntax::lexer::escape;
use crate::syntax::parser::Dialect;

/// Writer configuration.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WriteOptions {
    /// Grammar dialect (selects the access-type keywords).
    pub dialect: Dialect,
    /// Spaces per indentation level.
    pub indent: usize,
    /// Maximum elements of an MF value per output line.
    pub elements_per_line: usize,
}

impl Default for WriteOptions {
    fn default() -> Self {
        Self {
            dialect: Dialect::Vrml97,
            indent: 2,
            elements_per_line: 4,
        }
    }
}

/// Serialise with default options.
pub fn write_document(doc: &Document) -> String {
    write_document_with(doc, &WriteOptions::default())
}

/// Serialise with explicit options.
pub fn write_document_with(doc: &Document, opts: &WriteOptions) -> String {
    let mut w = Writer::new(doc, opts);
    let h = &doc.header;
    let _ = write!(w.out, "#{} {} {}", h.format, h.version, h.encoding);
    if !h.comment.is_empty() {
        let comment: String = h
            .comment
            .chars()
            .filter(|c| *c != '\n' && *c != '\r')
            .collect();
        let _ = write!(w.out, " {comment}");
    }
    w.out.push('\n');
    w.statements(&doc.statements, 0);
    w.out
}

/// Format one float the way the writer does.
pub fn format_f32(v: f32) -> String {
    if !v.is_finite() {
        return "0".into();
    }
    if v == 0.0 {
        return "0".into();
    }
    format_shortest(format!("{v}"))
}

/// Format one double the way the writer does.
pub fn format_f64(v: f64) -> String {
    if !v.is_finite() {
        return "0".into();
    }
    if v == 0.0 {
        return "0".into();
    }
    format_shortest(format!("{v}"))
}

/// Switch very long plain decimal renderings (`1e30` → 31 digits) to
/// exponent form while keeping the shortest round-trip digits.
fn format_shortest(plain: String) -> String {
    if plain.len() <= 12 {
        return plain;
    }
    // `{:e}` also yields the shortest round-trip mantissa.
    let exp = match plain.parse::<f64>() {
        Ok(v) => format!("{v:e}"),
        Err(_) => return plain,
    };
    if exp.len() < plain.len() {
        exp
    } else {
        plain
    }
}

struct Writer<'d> {
    doc: &'d Document,
    opts: &'d WriteOptions,
    out: String,
    emitted: HashSet<NodeId>,
    names: HashMap<NodeId, String>,
}

impl<'d> Writer<'d> {
    fn new(doc: &'d Document, opts: &'d WriteOptions) -> Self {
        let mut w = Self {
            doc,
            opts,
            out: String::new(),
            emitted: HashSet::new(),
            names: HashMap::new(),
        };
        w.assign_names();
        w
    }

    /// Give every node that is referenced more than once (or by a
    /// route) a DEF name.
    fn assign_names(&mut self) {
        let mut refs: HashMap<NodeId, usize> = HashMap::new();
        let mut routed: HashSet<NodeId> = HashSet::new();
        let note_routes = |r: &Route, routed: &mut HashSet<NodeId>| {
            routed.extend(r.from_id);
            routed.extend(r.to_id);
        };
        let count_stmt = |s: &Statement, refs: &mut HashMap<NodeId, usize>| {
            if let Statement::Node(id) = s {
                *refs.entry(*id).or_default() += 1;
            }
        };
        for s in &self.doc.statements {
            count_stmt(s, &mut refs);
            if let Statement::Route(r) = s {
                note_routes(r, &mut routed);
            }
        }
        for p in &self.doc.protos {
            for s in &p.body {
                count_stmt(s, &mut refs);
                if let Statement::Route(r) = s {
                    note_routes(r, &mut routed);
                }
            }
            for d in &p.interface {
                if let Some(v) = &d.value {
                    for id in v.node_list() {
                        *refs.entry(*id).or_default() += 1;
                    }
                }
            }
        }
        for n in &self.doc.nodes {
            for id in n.child_ids() {
                *refs.entry(id).or_default() += 1;
            }
            for d in &n.interface {
                if let Some(v) = &d.value {
                    for id in v.node_list() {
                        *refs.entry(id.to_owned()).or_default() += 1;
                    }
                }
            }
            for s in &n.statements {
                if let Statement::Route(r) = s {
                    note_routes(r, &mut routed);
                }
            }
        }
        let mut used: HashSet<String> = self
            .doc
            .nodes
            .iter()
            .filter_map(|n| n.def_name.clone())
            .collect();
        let mut ids: Vec<NodeId> = refs
            .iter()
            .filter(|(_, c)| **c > 1)
            .map(|(id, _)| *id)
            .chain(routed.iter().copied())
            .collect();
        ids.sort();
        ids.dedup();
        for id in ids {
            let Some(node) = self.doc.node(id) else {
                continue;
            };
            if node.def_name.is_some() {
                continue;
            }
            let mut name = format!("_N{}", id.0);
            while used.contains(&name) {
                name.push('_');
            }
            used.insert(name.clone());
            self.names.insert(id, name);
        }
    }

    fn name_of(&self, id: NodeId) -> Option<&str> {
        self.doc
            .node(id)
            .and_then(|n| n.def_name.as_deref())
            .or_else(|| self.names.get(&id).map(String::as_str))
    }

    fn pad(&mut self, level: usize) {
        for _ in 0..level * self.opts.indent {
            self.out.push(' ');
        }
    }

    fn statements(&mut self, stmts: &[Statement], level: usize) {
        for s in stmts {
            self.statement(s, level);
        }
    }

    fn statement(&mut self, s: &Statement, level: usize) {
        match s {
            Statement::Node(id) => {
                self.pad(level);
                self.node(*id, level);
                self.out.push('\n');
            }
            Statement::Proto(pid) => self.proto(pid.0 as usize, level),
            Statement::ExternProto(eid) => self.externproto(eid.0 as usize, level),
            Statement::Route(r) => {
                self.pad(level);
                let from = r
                    .from_id
                    .and_then(|i| self.name_of(i))
                    .unwrap_or(&r.from_node)
                    .to_owned();
                let to = r
                    .to_id
                    .and_then(|i| self.name_of(i))
                    .unwrap_or(&r.to_node)
                    .to_owned();
                let _ = writeln!(
                    self.out,
                    "ROUTE {from}.{} TO {to}.{}",
                    r.from_field, r.to_field
                );
            }
            Statement::Profile(p) => {
                self.pad(level);
                let _ = writeln!(self.out, "PROFILE {p}");
            }
            Statement::Component { name, level: l } => {
                self.pad(level);
                let _ = writeln!(self.out, "COMPONENT {name}:{l}");
            }
            Statement::Meta { name, content } => {
                self.pad(level);
                let _ = writeln!(
                    self.out,
                    "META \"{}\" \"{}\"",
                    escape(name),
                    escape(content)
                );
            }
            Statement::Unit {
                category,
                name,
                factor,
            } => {
                self.pad(level);
                let _ = writeln!(self.out, "UNIT {category} {name} {}", format_f64(*factor));
            }
            Statement::Import {
                inline_def,
                exported,
                as_name,
            } => {
                self.pad(level);
                let _ = write!(self.out, "IMPORT {inline_def}.{exported}");
                if let Some(a) = as_name {
                    let _ = write!(self.out, " AS {a}");
                }
                self.out.push('\n');
            }
            Statement::Export { local, as_name } => {
                self.pad(level);
                let _ = write!(self.out, "EXPORT {local}");
                if let Some(a) = as_name {
                    let _ = write!(self.out, " AS {a}");
                }
                self.out.push('\n');
            }
        }
    }

    fn keyword(&self, d: &InterfaceDecl) -> &'static str {
        match self.opts.dialect {
            Dialect::Vrml97 => d.access.vrml97_keyword(),
            Dialect::X3dClassic => d.access.x3d_keyword(),
        }
    }

    fn interface_decl(&mut self, d: &InterfaceDecl, level: usize) {
        self.pad(level);
        let kw = self.keyword(d);
        let _ = write!(self.out, "{kw} {} {}", d.field_type.name(), d.name);
        if let Some(is) = &d.is {
            let _ = write!(self.out, " IS {is}");
        } else if let Some(v) = &d.value {
            self.out.push(' ');
            self.value(v, level);
        }
        self.out.push('\n');
    }

    fn proto(&mut self, idx: usize, level: usize) {
        let doc = self.doc;
        let Some(p) = doc.protos.get(idx) else {
            return;
        };
        self.pad(level);
        let _ = writeln!(self.out, "PROTO {} [", p.name);
        for d in &p.interface {
            self.interface_decl(d, level + 1);
        }
        self.pad(level);
        self.out.push_str("] {\n");
        self.statements(&p.body, level + 1);
        self.pad(level);
        self.out.push_str("}\n");
    }

    fn externproto(&mut self, idx: usize, level: usize) {
        let doc = self.doc;
        let Some(p) = doc.extern_protos.get(idx) else {
            return;
        };
        self.pad(level);
        let _ = writeln!(self.out, "EXTERNPROTO {} [", p.name);
        for d in &p.interface {
            self.interface_decl(d, level + 1);
        }
        self.pad(level);
        self.out.push_str("] ");
        self.strings(&p.urls, true, level);
        self.out.push('\n');
    }

    fn node(&mut self, id: NodeId, level: usize) {
        let doc = self.doc;
        let Some(node) = doc.node(id) else {
            self.out.push_str("NULL");
            return;
        };
        if self.emitted.contains(&id) {
            match self.name_of(id).map(str::to_owned) {
                Some(name) => {
                    let _ = write!(self.out, "USE {name}");
                }
                None => self.out.push_str("NULL"),
            }
            return;
        }
        self.emitted.insert(id);
        if let Some(name) = self.name_of(id).map(str::to_owned) {
            let _ = write!(self.out, "DEF {name} ");
        }
        self.node_body(node, level);
    }

    fn node_body(&mut self, node: &Node, level: usize) {
        let _ = write!(self.out, "{} {{", node.type_name);
        if node.fields.is_empty() && node.interface.is_empty() && node.statements.is_empty() {
            self.out.push_str(" }");
            return;
        }
        self.out.push('\n');
        for d in &node.interface {
            self.interface_decl(d, level + 1);
        }
        for f in &node.fields {
            self.pad(level + 1);
            let _ = write!(self.out, "{} ", f.name);
            match &f.binding {
                FieldBinding::Is(t) => {
                    let _ = write!(self.out, "IS {t}");
                }
                FieldBinding::Value(v) => self.value(v, level + 1),
            }
            self.out.push('\n');
        }
        self.statements(&node.statements, level + 1);
        self.pad(level);
        self.out.push('}');
    }

    fn strings(&mut self, v: &[String], force_brackets: bool, level: usize) {
        if v.len() == 1 && !force_brackets {
            let _ = write!(self.out, "\"{}\"", escape(&v[0]));
            return;
        }
        if v.is_empty() {
            self.out.push_str("[]");
            return;
        }
        self.out.push('[');
        let multiline = v.len() > 1;
        for (i, s) in v.iter().enumerate() {
            if multiline {
                self.out.push('\n');
                self.pad(level + 1);
            } else {
                self.out.push(' ');
            }
            let _ = write!(self.out, "\"{}\"", escape(s));
            if i + 1 < v.len() {
                self.out.push(',');
            }
        }
        if multiline {
            self.out.push('\n');
            self.pad(level);
        } else {
            self.out.push(' ');
        }
        self.out.push(']');
    }

    fn image(&mut self, img: &Image) {
        let _ = write!(self.out, "{} {} {}", img.width, img.height, img.components);
        for p in &img.pixels {
            let _ = write!(self.out, " 0x{p:X}");
        }
    }

    fn value(&mut self, v: &FieldValue, level: usize) {
        let multi = v.ty.is_multi();
        let comps = v.ty.element().1.max(1);
        match &v.data {
            FieldData::Nodes(ids) => {
                if !multi {
                    match ids.first() {
                        Some(id) => self.node(*id, level),
                        None => self.out.push_str("NULL"),
                    }
                    return;
                }
                if ids.is_empty() {
                    self.out.push_str("[]");
                    return;
                }
                self.out.push_str("[\n");
                for id in ids {
                    self.pad(level + 1);
                    self.node(*id, level + 1);
                    self.out.push('\n');
                }
                self.pad(level);
                self.out.push(']');
            }
            FieldData::Strings(s) => self.strings(s, multi && s.len() != 1, level),
            FieldData::Images(imgs) => {
                if !multi {
                    match imgs.first() {
                        Some(img) => self.image(img),
                        None => self.out.push_str("0 0 0"),
                    }
                    return;
                }
                self.out.push('[');
                for (i, img) in imgs.iter().enumerate() {
                    if i > 0 {
                        self.out.push(',');
                    }
                    self.out.push(' ');
                    self.image(img);
                }
                self.out.push_str(" ]");
            }
            FieldData::Bools(b) => {
                let items: Vec<String> = b
                    .iter()
                    .map(|x| if *x { "TRUE" } else { "FALSE" }.to_owned())
                    .collect();
                self.scalars(&items, 1, multi, level);
            }
            FieldData::Int32s(i) => {
                let items: Vec<String> = i.iter().map(|x| x.to_string()).collect();
                // coordIndex-style lists read best broken after `-1`.
                if multi && v.ty == FieldType::MFInt32 && i.contains(&-1) && i.len() > 8 {
                    self.index_list(&items, level);
                } else {
                    self.scalars(&items, 1, multi, level);
                }
            }
            FieldData::Floats(f) => {
                let items: Vec<String> = f.iter().map(|x| format_f32(*x)).collect();
                self.scalars(&items, comps, multi, level);
            }
            FieldData::Doubles(d) => {
                let items: Vec<String> = d.iter().map(|x| format_f64(*x)).collect();
                self.scalars(&items, comps, multi, level);
            }
        }
    }

    fn scalars(&mut self, items: &[String], comps: usize, multi: bool, level: usize) {
        let elements = items.len() / comps;
        let joined = |chunk: &[String]| chunk.join(" ");
        if !multi || elements == 1 {
            if items.is_empty() && multi {
                self.out.push_str("[]");
            } else {
                self.out.push_str(&joined(items));
            }
            return;
        }
        if elements == 0 {
            self.out.push_str("[]");
            return;
        }
        let per_line = self.opts.elements_per_line.max(1);
        if elements <= per_line {
            self.out.push_str("[ ");
            let parts: Vec<String> = items.chunks(comps).map(joined).collect();
            self.out.push_str(&parts.join(", "));
            self.out.push_str(" ]");
            return;
        }
        self.out.push('[');
        for (i, chunk) in items.chunks(comps).enumerate() {
            if i % per_line == 0 {
                self.out.push('\n');
                self.pad(level + 1);
            } else {
                self.out.push(' ');
            }
            self.out.push_str(&joined(chunk));
            if (i + 1) * comps < items.len() {
                self.out.push(',');
            }
        }
        self.out.push('\n');
        self.pad(level);
        self.out.push(']');
    }

    fn index_list(&mut self, items: &[String], level: usize) {
        self.out.push('[');
        let mut line_start = true;
        for (i, it) in items.iter().enumerate() {
            if line_start {
                self.out.push('\n');
                self.pad(level + 1);
                line_start = false;
            } else {
                self.out.push(' ');
            }
            self.out.push_str(it);
            if i + 1 < items.len() {
                self.out.push(',');
            }
            if it == "-1" {
                line_start = true;
            }
        }
        self.out.push('\n');
        self.pad(level);
        self.out.push(']');
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::parser::parse;

    #[test]
    fn round_trip_is_stable() {
        let src = "#VRML V2.0 utf8\n\
            PROTO P [ field SFFloat r 1 eventIn SFBool go ] { Sphere { radius IS r } }\n\
            DEF G Group { children [ DEF S Shape { geometry P { r 2.5 } } USE S ] }\n\
            DEF T TimeSensor { cycleInterval 3.0000001 loop TRUE }\n\
            Foo { bar [ 1 2 3 4 5 ] baz \"q\\\"uote\" }\n\
            ROUTE T.isActive TO T.set_loop\n";
        let d1 = parse(src).unwrap();
        let text = write_document(&d1);
        let d2 = parse(&text).unwrap();
        assert_eq!(d1, d2, "{text}");
        assert_eq!(write_document(&d2), text);
    }

    #[test]
    fn floats_are_shortest_round_trip() {
        for v in [0.1f32, 1e-7, 3.4028235e38, -2.5, 1e30] {
            let s = format_f32(v);
            assert_eq!(s.parse::<f32>().unwrap(), v, "{s}");
            assert!(crate::syntax::lexer::is_float_literal(&s), "{s}");
        }
        assert_eq!(format_f32(f32::NAN), "0");
    }
}
