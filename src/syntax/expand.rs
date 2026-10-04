//! PROTO instantiation (ISO/IEC 14772-1 §4.8.3).
//!
//! [`expand_protos`] rewrites a parsed [`Document`] into an equivalent
//! one without prototype instances: every instance of a `PROTO` is
//! replaced by a copy of the prototype body's first node, with each
//! `field IS name` association substituted by the instance's value for
//! `name` (or the interface default). The other body roots and body
//! routes are kept on the copy's [`ProtoInstance`] record so behaviour
//! wired inside a prototype (TimeSensor → interpolator → transform)
//! survives, and routes addressed to the instance's interface can be
//! forwarded through [`ProtoInstance::is_map`].
//!
//! `DEF` / `USE` sharing is preserved within each copy (each instance
//! gets its own copy of the body, per §4.8.2 "each prototype instance
//! can be considered to be a complete copy of the prototype").
//! `EXTERNPROTO` instances are kept as-is (their implementation lives
//! in another file).
//!
//! The output contains no `PROTO` statements. Expansion is bounded by
//! [`ExpandLimits`] — nested prototypes can otherwise blow up
//! exponentially ("billion laughs").

use std::collections::HashMap;

use crate::ast::{
    Document, ExternProtoId, Field, FieldBinding, FieldData, FieldValue, InterfaceDecl, Node,
    NodeId, NodeOrigin, ProtoId, ProtoInstance, Route, Statement,
};
use crate::error::{Error, Result};

/// Caps for [`expand_protos`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExpandLimits {
    /// Maximum nodes in the expanded document.
    pub max_nodes: usize,
    /// Maximum nesting of prototype instantiation / node copying.
    pub max_depth: usize,
}

impl Default for ExpandLimits {
    fn default() -> Self {
        Self {
            max_nodes: 4_000_000,
            max_depth: 512,
        }
    }
}

/// Expand every PROTO instance with default limits.
pub fn expand_protos(doc: &Document) -> Result<Document> {
    expand_protos_with(doc, &ExpandLimits::default())
}

/// Expand every PROTO instance.
pub fn expand_protos_with(doc: &Document, limits: &ExpandLimits) -> Result<Document> {
    let mut ex = Expander {
        src: doc,
        out: Document {
            header: doc.header.clone(),
            ..Document::default()
        },
        limits: *limits,
        depth: 0,
        extern_map: HashMap::new(),
    };
    let mut env = Env::default();
    let stmts = ex.statements(&doc.statements, &mut env)?;
    ex.out.statements = stmts;
    Ok(ex.out)
}

/// Copy environment of one name scope (the file, or one prototype
/// instance body).
#[derive(Default)]
struct Env {
    /// Source id → expanded id.
    map: HashMap<NodeId, NodeId>,
    /// Interface values of the instance being expanded (already copied
    /// into the output arena).
    is_values: Option<HashMap<String, FieldValue>>,
    /// IS associations recorded while copying the body.
    is_map: Vec<(String, NodeId, String)>,
}

struct Expander<'a> {
    src: &'a Document,
    out: Document,
    limits: ExpandLimits,
    depth: usize,
    extern_map: HashMap<ExternProtoId, ExternProtoId>,
}

impl Expander<'_> {
    fn statements(&mut self, stmts: &[Statement], env: &mut Env) -> Result<Vec<Statement>> {
        let mut out = Vec::new();
        for s in stmts {
            match s {
                Statement::Node(id) => out.push(Statement::Node(self.copy(*id, env)?)),
                Statement::Proto(_) => {}
                Statement::ExternProto(eid) => {
                    out.push(Statement::ExternProto(self.extern_proto(*eid, env)?));
                }
                Statement::Route(r) => out.push(Statement::Route(remap_route(r, env))),
                other => out.push(other.clone()),
            }
        }
        Ok(out)
    }

    fn extern_proto(&mut self, eid: ExternProtoId, env: &mut Env) -> Result<ExternProtoId> {
        if let Some(n) = self.extern_map.get(&eid) {
            return Ok(*n);
        }
        let Some(decl) = self.src.extern_proto(eid) else {
            return Err(Error::invalid("dangling EXTERNPROTO id"));
        };
        let mut decl = decl.clone();
        for d in &mut decl.interface {
            if let Some(v) = d.value.take() {
                d.value = Some(self.value(&v, env)?);
            }
        }
        let new = self.out.add_extern_proto(decl);
        self.extern_map.insert(eid, new);
        Ok(new)
    }

    fn reserve(&mut self) -> Result<NodeId> {
        if self.out.nodes.len() >= self.limits.max_nodes {
            return Err(Error::limit(format!(
                "PROTO expansion exceeds {} nodes",
                self.limits.max_nodes
            )));
        }
        Ok(self.out.add_node(Node::new("")))
    }

    fn enter(&mut self) -> Result<()> {
        self.depth += 1;
        if self.depth > self.limits.max_depth {
            return Err(Error::limit(format!(
                "PROTO expansion nesting exceeds {}",
                self.limits.max_depth
            )));
        }
        Ok(())
    }

    fn value(&mut self, v: &FieldValue, env: &mut Env) -> Result<FieldValue> {
        match &v.data {
            FieldData::Nodes(ids) => {
                let mut out = Vec::with_capacity(ids.len());
                for id in ids {
                    out.push(self.copy(*id, env)?);
                }
                Ok(FieldValue {
                    ty: v.ty,
                    data: FieldData::Nodes(out),
                })
            }
            _ => Ok(v.clone()),
        }
    }

    /// Copy (or instantiate) source node `id` into the output arena.
    fn copy(&mut self, id: NodeId, env: &mut Env) -> Result<NodeId> {
        if let Some(n) = env.map.get(&id) {
            return Ok(*n);
        }
        let src = self.src;
        let Some(node) = src.node(id) else {
            return Err(Error::invalid("dangling node id"));
        };
        self.enter()?;
        let r = match node.origin {
            NodeOrigin::Proto(pid) => self.instantiate(id, node, pid, env),
            _ => self.copy_plain(id, node, env),
        };
        self.depth -= 1;
        r
    }

    fn copy_plain(&mut self, id: NodeId, node: &Node, env: &mut Env) -> Result<NodeId> {
        let new = self.reserve()?;
        env.map.insert(id, new);
        let mut out = Node::new(node.type_name.clone());
        out.def_name = node.def_name.clone();
        out.origin = match node.origin {
            NodeOrigin::ExternProto(eid) => NodeOrigin::ExternProto(self.extern_proto(eid, env)?),
            other => other,
        };
        for f in &node.fields {
            match &f.binding {
                FieldBinding::Value(v) => {
                    out.fields.push(Field {
                        name: f.name.clone(),
                        binding: FieldBinding::Value(self.value(v, env)?),
                        inferred: f.inferred,
                    });
                }
                FieldBinding::Is(target) => {
                    env.is_map.push((target.clone(), new, f.name.clone()));
                    match env.is_values.as_ref().and_then(|m| m.get(target)) {
                        Some(v) => out.fields.push(Field {
                            name: f.name.clone(),
                            binding: FieldBinding::Value(v.clone()),
                            inferred: f.inferred,
                        }),
                        // Event-only association, or IS outside a PROTO
                        // body: nothing to substitute — keep it.
                        None if env.is_values.is_none() => out.fields.push(f.clone()),
                        None => {}
                    }
                }
            }
        }
        for d in &node.interface {
            let mut nd = InterfaceDecl {
                access: d.access,
                field_type: d.field_type,
                name: d.name.clone(),
                value: None,
                is: None,
            };
            if let Some(target) = &d.is {
                env.is_map.push((target.clone(), new, d.name.clone()));
                match env.is_values.as_ref().and_then(|m| m.get(target)) {
                    Some(v) => nd.value = Some(v.clone()),
                    None if env.is_values.is_none() => nd.is = Some(target.clone()),
                    None => {}
                }
            } else if let Some(v) = &d.value {
                nd.value = Some(self.value(v, env)?);
            }
            out.interface.push(nd);
        }
        for s in &node.statements {
            match s {
                Statement::Route(r) => out.statements.push(Statement::Route(remap_route(r, env))),
                Statement::ExternProto(eid) => out
                    .statements
                    .push(Statement::ExternProto(self.extern_proto(*eid, env)?)),
                _ => {}
            }
        }
        self.out.nodes[new.0 as usize] = out;
        Ok(new)
    }

    fn instantiate(
        &mut self,
        id: NodeId,
        node: &Node,
        pid: ProtoId,
        env: &mut Env,
    ) -> Result<NodeId> {
        let src = self.src;
        let Some(proto) = src.proto(pid) else {
            return Err(Error::invalid("dangling PROTO id"));
        };
        // 1. Interface values: instance value (copied in the *outer*
        //    environment, so IS chains through nested bodies resolve)
        //    or the declared default.
        let mut values: HashMap<String, FieldValue> = HashMap::new();
        for d in &proto.interface {
            let given = node.fields.iter().rev().find(|f| f.name == d.name);
            let v = match given.map(|f| &f.binding) {
                Some(FieldBinding::Value(v)) => Some(self.value(v, env)?),
                Some(FieldBinding::Is(t)) => env.is_values.as_ref().and_then(|m| m.get(t)).cloned(),
                None => match &d.value {
                    Some(def) => {
                        // Defaults live in the declaring scope: copy them
                        // with a fresh map per instance.
                        let mut denv = Env::default();
                        Some(self.value(def, &mut denv)?)
                    }
                    None => None,
                },
            };
            if let Some(v) = v {
                values.insert(d.name.clone(), v);
            }
        }
        // 2. Copy the body in a fresh scope.
        let mut body_env = Env {
            map: HashMap::new(),
            is_values: Some(values),
            is_map: Vec::new(),
        };
        let mut root: Option<NodeId> = None;
        let mut extra_roots = Vec::new();
        let mut routes = Vec::new();
        for s in &proto.body {
            match s {
                Statement::Node(nid) => {
                    let copied = self.copy(*nid, &mut body_env)?;
                    if root.is_none() {
                        root = Some(copied);
                        // USE of the instance elsewhere shares the copy.
                        env.map.insert(id, copied);
                    } else {
                        extra_roots.push(copied);
                    }
                }
                Statement::Route(r) => routes.push(r.clone()),
                Statement::ExternProto(eid) => {
                    self.extern_proto(*eid, &mut body_env)?;
                }
                _ => {}
            }
        }
        let routes = routes
            .iter()
            .map(|r| remap_route(r, &body_env))
            .collect::<Vec<_>>();
        let Some(root) = root else {
            return Err(Error::invalid(format!(
                "PROTO `{}` body has no root node",
                proto.name
            )));
        };
        let info = ProtoInstance {
            proto_name: proto.name.clone(),
            extra_roots,
            routes,
            is_map: body_env.is_map,
        };
        let out_node = &mut self.out.nodes[root.0 as usize];
        if node.def_name.is_some() {
            out_node.def_name = node.def_name.clone();
        }
        let nested = out_node.instance.take();
        out_node.instance = Some(Box::new(ProtoInstance {
            extra_roots: info
                .extra_roots
                .into_iter()
                .chain(nested.iter().flat_map(|n| n.extra_roots.clone()))
                .collect(),
            routes: info
                .routes
                .into_iter()
                .chain(nested.iter().flat_map(|n| n.routes.clone()))
                .collect(),
            ..info
        }));
        Ok(root)
    }
}

fn remap_route(r: &Route, env: &Env) -> Route {
    Route {
        from_id: r.from_id.and_then(|i| env.map.get(&i).copied()),
        to_id: r.to_id.and_then(|i| env.map.get(&i).copied()),
        ..r.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::parser::parse;

    #[test]
    fn substitutes_is_values_and_defaults() {
        let d = parse(
            "#VRML V2.0 utf8
             PROTO Ball [ field SFFloat r 1 field SFColor c 1 0 0 ] {
               Shape { appearance Appearance { material Material { diffuseColor IS c } }
                       geometry Sphere { radius IS r } }
             }
             Ball { r 3 }
             Ball { }",
        )
        .unwrap();
        let e = expand_protos(&d).unwrap();
        assert!(e.protos.is_empty());
        let roots: Vec<_> = e.root_nodes().collect();
        assert_eq!(roots.len(), 2);
        let radius = |root: NodeId| {
            let shape = e.node(root).unwrap();
            assert_eq!(shape.type_name, "Shape");
            let g = shape.field("geometry").unwrap().as_node().unwrap();
            e.node(g)
                .unwrap()
                .field("radius")
                .unwrap()
                .as_float()
                .unwrap()
        };
        assert_eq!(radius(roots[0]), 3.0);
        assert_eq!(radius(roots[1]), 1.0);
        let inst = e.node(roots[0]).unwrap().instance.as_ref().unwrap();
        assert_eq!(inst.proto_name, "Ball");
        assert!(inst
            .is_map
            .iter()
            .any(|(n, _, f)| n == "r" && f == "radius"));
    }

    #[test]
    fn nested_protos_and_routes() {
        let d = parse(
            "#VRML V2.0 utf8
             PROTO Inner [ exposedField SFVec3f t 0 0 0 ] { Transform { translation IS t } }
             PROTO Outer [ field SFVec3f p 1 2 3 ] {
               Group { children Inner { t IS p } }
               DEF TS TimeSensor { }
               DEF PI PositionInterpolator { }
               ROUTE TS.fraction_changed TO PI.set_fraction
             }
             DEF O Outer { }",
        )
        .unwrap();
        let e = expand_protos(&d).unwrap();
        let root = e.root_nodes().next().unwrap();
        let g = e.node(root).unwrap();
        assert_eq!(g.def_name.as_deref(), Some("O"));
        let inst = g.instance.as_ref().unwrap();
        assert_eq!(inst.extra_roots.len(), 2);
        assert_eq!(inst.routes.len(), 1);
        assert!(inst.routes[0].from_id.is_some() && inst.routes[0].to_id.is_some());
        let t = g.field("children").unwrap().node_list()[0];
        let t = e.node(t).unwrap();
        assert_eq!(t.type_name, "Transform");
        assert_eq!(
            t.field("translation").unwrap().as_vec3f(),
            Some([1.0, 2.0, 3.0])
        );
    }

    #[test]
    fn exponential_expansion_is_capped() {
        let mut s = String::from("#VRML V2.0 utf8\nPROTO P0 [] { Group {} }\n");
        for i in 1..40 {
            s.push_str(&format!(
                "PROTO P{i} [] {{ Group {{ children [ P{p} {{}} P{p} {{}} ] }} }}\n",
                p = i - 1
            ));
        }
        s.push_str("P39 {}\n");
        let d = parse(&s).unwrap();
        let limits = ExpandLimits {
            max_nodes: 10_000,
            ..ExpandLimits::default()
        };
        assert!(matches!(
            expand_protos_with(&d, &limits),
            Err(Error::LimitExceeded(_))
        ));
    }
}
