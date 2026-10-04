//! Recursive-descent parser for the ISO/IEC 14772-1 Annex A grammar.
//!
//! The parser is *typed*: field values are parsed according to the
//! field's declared type, looked up (in order) in the enclosing node's
//! body-level interface declarations (Script), the PROTO / EXTERNPROTO
//! interface of a prototyped node, and the [`NodeCatalog`]. Fields of
//! unknown node types — and undeclared fields — are parsed with a
//! shape-based type inference so unknown content is preserved rather
//! than rejected ([`Field::inferred`] is set).
//!
//! Hostile-input hardening: nesting depth, node count, PROTO count and
//! per-field element counts are bounded by [`ParseLimits`]; no
//! allocation is sized from an untrusted count before the matching
//! data has actually been read.

use std::collections::HashMap;

use crate::ast::{
    AccessType, Document, ExternProtoDecl, ExternProtoId, Field, FieldBinding, FieldData,
    FieldType, FieldValue, Header, Image, InterfaceDecl, Node, NodeId, NodeOrigin, ProtoDecl,
    ProtoId, Route, Scalar, Statement,
};
use crate::error::{Error, Result};
use crate::syntax::catalog::{NodeCatalog, Vrml97Catalog};
use crate::syntax::lexer::{
    is_float_literal, parse_double, parse_int32, parse_int_wide, unescape, Lexer, Token, TokenKind,
};

/// Grammar dialect.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Dialect {
    /// ISO/IEC 14772-1 (VRML97) UTF-8 encoding.
    #[default]
    Vrml97,
    /// ISO/IEC 19776-2 X3D ClassicVRML encoding: additionally accepts
    /// the `inputOnly` / `outputOnly` / `initializeOnly` / `inputOutput`
    /// access keywords, the X3D field types, and the `PROFILE`,
    /// `COMPONENT`, `META`, `UNIT`, `IMPORT` and `EXPORT` statements.
    X3dClassic,
}

/// Resource caps for hostile input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParseLimits {
    /// Maximum node / PROTO nesting depth.
    pub max_depth: usize,
    /// Maximum number of node statements (excluding `USE`).
    pub max_nodes: usize,
    /// Maximum number of PROTO + EXTERNPROTO declarations.
    pub max_protos: usize,
    /// Maximum scalar elements in one field value.
    pub max_field_elements: usize,
    /// Maximum `width × height` of one SFImage.
    pub max_image_pixels: u64,
}

impl Default for ParseLimits {
    fn default() -> Self {
        Self {
            max_depth: 128,
            max_nodes: 4_000_000,
            max_protos: 100_000,
            max_field_elements: 256 * 1024 * 1024,
            max_image_pixels: 64 * 1024 * 1024,
        }
    }
}

/// Parser configuration.
#[derive(Clone, Copy)]
pub struct ParseOptions<'c> {
    /// Grammar dialect.
    pub dialect: Dialect,
    /// Resource caps.
    pub limits: ParseLimits,
    /// Built-in node interfaces.
    pub catalog: &'c dyn NodeCatalog,
    /// Require (and parse) the `#<FORMAT> <version> <encoding>` header
    /// line. When `false` a missing header is tolerated (useful for
    /// parsing fragments such as `createVrmlFromString` input).
    pub require_header: bool,
}

impl std::fmt::Debug for ParseOptions<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParseOptions")
            .field("dialect", &self.dialect)
            .field("limits", &self.limits)
            .field("require_header", &self.require_header)
            .finish_non_exhaustive()
    }
}

impl Default for ParseOptions<'_> {
    fn default() -> Self {
        Self {
            dialect: Dialect::Vrml97,
            limits: ParseLimits::default(),
            catalog: &Vrml97Catalog,
            require_header: true,
        }
    }
}

/// Parse a complete VRML97 file with the default options.
pub fn parse(src: &str) -> Result<Document> {
    parse_with(src, &ParseOptions::default())
}

/// Parse a complete file with explicit options.
pub fn parse_with(src: &str, options: &ParseOptions<'_>) -> Result<Document> {
    let src = src.strip_prefix('\u{feff}').unwrap_or(src);
    let (header, body) = split_header(src, options.require_header)?;
    let mut p = Parser::new(body, options);
    p.doc.header = header;
    p.advance()?;
    let stmts = p.parse_statements(true)?;
    p.doc.statements = stmts;
    Ok(p.doc)
}

/// Parse a standalone field value of type `ty` (e.g. a catalog default
/// such as `"[ 1 1, 1 -1 ]"`). Node-typed values are parsed into the
/// returned document's arena.
pub fn parse_field_value(src: &str, ty: FieldType) -> Result<(FieldValue, Document)> {
    let options = ParseOptions {
        require_header: false,
        dialect: Dialect::X3dClassic,
        ..ParseOptions::default()
    };
    let mut p = Parser::new(src, &options);
    p.advance()?;
    let v = p.parse_typed_value(ty)?;
    if p.tok.kind != TokenKind::Eof {
        return Err(p.err_here("trailing input after field value"));
    }
    Ok((v, p.doc))
}

/// Split the header line off. Returns `(header, rest)`; the rest keeps
/// the header's line terminator so token line numbers stay exact.
fn split_header(src: &str, require: bool) -> Result<(Header, &str)> {
    if !src.starts_with('#') {
        if require {
            return Err(Error::invalid(
                "missing header line (expected `#VRML V2.0 utf8`)",
            ));
        }
        return Ok((Header::default(), src));
    }
    let end = src.find(['\n', '\r']).unwrap_or(src.len());
    let line = &src[1..end];
    let mut words = line.split([' ', '\t']).filter(|w| !w.is_empty());
    let format = words.next().unwrap_or("").to_owned();
    let version = words.next().unwrap_or("").to_owned();
    let encoding = words.next().unwrap_or("").to_owned();
    if require && (format.is_empty() || version.is_empty() || encoding.is_empty()) {
        return Err(Error::invalid(format!("malformed header line `#{line}`")));
    }
    // Comment = everything after the encoding word.
    let comment = match line.find(encoding.as_str()) {
        Some(pos) if !encoding.is_empty() => line[pos + encoding.len()..].trim().to_owned(),
        _ => String::new(),
    };
    let header = Header {
        format,
        version,
        encoding,
        comment,
    };
    Ok((header, &src[end..]))
}

struct Scope {
    defs: HashMap<String, NodeId>,
    protos: HashMap<String, NodeOrigin>,
}

impl Scope {
    fn new() -> Self {
        Self {
            defs: HashMap::new(),
            protos: HashMap::new(),
        }
    }
}

struct Parser<'a, 'c> {
    lx: Lexer<'a>,
    tok: Token<'a>,
    peeked: Option<Token<'a>>,
    doc: Document,
    scopes: Vec<Scope>,
    depth: usize,
    node_count: usize,
    opts: &'c ParseOptions<'c>,
}

impl<'a, 'c> Parser<'a, 'c> {
    fn new(src: &'a str, opts: &'c ParseOptions<'c>) -> Self {
        Self {
            lx: Lexer::new(src).with_colon_terminal(opts.dialect == Dialect::X3dClassic),
            tok: Token {
                kind: TokenKind::Eof,
                text: "",
                line: 1,
                column: 1,
            },
            peeked: None,
            doc: Document::new(),
            scopes: vec![Scope::new()],
            depth: 0,
            node_count: 0,
            opts,
        }
    }

    fn x3d(&self) -> bool {
        self.opts.dialect == Dialect::X3dClassic
    }

    fn advance(&mut self) -> Result<()> {
        self.tok = match self.peeked.take() {
            Some(t) => t,
            None => self.lx.next_token()?,
        };
        Ok(())
    }

    fn peek(&mut self) -> Result<Token<'a>> {
        if self.peeked.is_none() {
            self.peeked = Some(self.lx.next_token()?);
        }
        // Just populated above.
        Ok(self.peeked.unwrap_or(self.tok))
    }

    fn err_at(&self, t: &Token<'_>, msg: impl Into<String>) -> Error {
        Error::syntax(t.line, t.column, msg)
    }

    fn err_here(&self, msg: impl Into<String>) -> Error {
        let msg = msg.into();
        let found = match self.tok.kind {
            TokenKind::Eof => "end of input".to_owned(),
            TokenKind::String => "a string".to_owned(),
            _ => format!("`{}`", self.tok.text),
        };
        self.err_at(&self.tok, format!("{msg} (found {found})"))
    }

    fn expect(&mut self, kind: TokenKind, what: &str) -> Result<Token<'a>> {
        if self.tok.kind != kind {
            return Err(self.err_here(format!("expected {what}")));
        }
        let t = self.tok;
        self.advance()?;
        Ok(t)
    }

    fn expect_id(&mut self, what: &str) -> Result<&'a str> {
        if self.tok.kind != TokenKind::Id {
            return Err(self.err_here(format!("expected {what}")));
        }
        let t = self.tok.text;
        self.advance()?;
        Ok(t)
    }

    fn enter(&mut self) -> Result<()> {
        self.depth += 1;
        if self.depth > self.opts.limits.max_depth {
            return Err(Error::limit(format!(
                "nesting depth exceeds {}",
                self.opts.limits.max_depth
            )));
        }
        Ok(())
    }

    fn leave(&mut self) {
        self.depth -= 1;
    }

    fn is_keyword(word: &str) -> bool {
        matches!(
            word,
            "DEF"
                | "EXTERNPROTO"
                | "FALSE"
                | "IS"
                | "NULL"
                | "PROTO"
                | "ROUTE"
                | "TO"
                | "TRUE"
                | "USE"
                | "eventIn"
                | "eventOut"
                | "exposedField"
                | "field"
        )
    }

    fn access_keyword(&self, word: &str) -> Option<AccessType> {
        match word {
            "eventIn" | "eventOut" | "field" | "exposedField" => AccessType::from_keyword(word),
            "inputOnly" | "outputOnly" | "initializeOnly" | "inputOutput" if self.x3d() => {
                AccessType::from_keyword(word)
            }
            _ => None,
        }
    }

    fn field_type_kw(&mut self) -> Result<FieldType> {
        let t = self.tok;
        let name = self.expect_id("field type")?;
        match FieldType::from_name(name) {
            Some(ft) if ft.is_vrml97() || self.x3d() => Ok(ft),
            _ => Err(self.err_at(&t, format!("unknown field type `{name}`"))),
        }
    }

    fn lookup_proto(&self, name: &str) -> Option<NodeOrigin> {
        self.scopes
            .iter()
            .rev()
            .find_map(|s| s.protos.get(name).copied())
    }

    fn lookup_def(&self, name: &str) -> Option<NodeId> {
        self.scopes.last().and_then(|s| s.defs.get(name).copied())
    }

    // ---- statements -------------------------------------------------

    /// Parse statements until EOF (`top`) or `}` (PROTO body).
    fn parse_statements(&mut self, top: bool) -> Result<Vec<Statement>> {
        let mut out = Vec::new();
        loop {
            match self.tok.kind {
                TokenKind::Eof if top => return Ok(out),
                TokenKind::RBrace if !top => return Ok(out),
                TokenKind::Id => {
                    if let Some(s) = self.parse_statement()? {
                        out.push(s);
                    }
                }
                _ => return Err(self.err_here("expected a statement")),
            }
        }
    }

    fn parse_statement(&mut self) -> Result<Option<Statement>> {
        match self.tok.text {
            "PROTO" => Ok(Some(Statement::Proto(self.parse_proto()?))),
            "EXTERNPROTO" => Ok(Some(Statement::ExternProto(self.parse_externproto()?))),
            "ROUTE" => Ok(Some(Statement::Route(self.parse_route()?))),
            "PROFILE" if self.x3d() => {
                self.advance()?;
                Ok(Some(Statement::Profile(
                    self.expect_id("profile name")?.into(),
                )))
            }
            "COMPONENT" if self.x3d() => {
                self.advance()?;
                let name = self.expect_id("component name")?.to_owned();
                self.expect(TokenKind::Colon, "`:`")?;
                let t = self.expect(TokenKind::Number, "component level")?;
                let level =
                    parse_int32(t.text).ok_or_else(|| self.err_at(&t, "bad component level"))?;
                Ok(Some(Statement::Component { name, level }))
            }
            "META" if self.x3d() => {
                self.advance()?;
                let name = unescape(self.expect(TokenKind::String, "META name")?.text);
                let content = unescape(self.expect(TokenKind::String, "META content")?.text);
                Ok(Some(Statement::Meta { name, content }))
            }
            "UNIT" if self.x3d() => {
                self.advance()?;
                let category = self.expect_id("unit category")?.to_owned();
                let name = self.expect_id("unit name")?.to_owned();
                let t = self.expect(TokenKind::Number, "conversion factor")?;
                let factor =
                    parse_double(t.text).ok_or_else(|| self.err_at(&t, "bad conversion factor"))?;
                Ok(Some(Statement::Unit {
                    category,
                    name,
                    factor,
                }))
            }
            "IMPORT" if self.x3d() => {
                self.advance()?;
                let inline_def = self.expect_id("inline DEF name")?.to_owned();
                self.expect(TokenKind::Period, "`.`")?;
                let exported = self.expect_id("exported name")?.to_owned();
                let as_name = self.parse_opt_as()?;
                Ok(Some(Statement::Import {
                    inline_def,
                    exported,
                    as_name,
                }))
            }
            "EXPORT" if self.x3d() => {
                self.advance()?;
                let local = self.expect_id("DEF name")?.to_owned();
                let as_name = self.parse_opt_as()?;
                Ok(Some(Statement::Export { local, as_name }))
            }
            _ => {
                let id = self.parse_node_statement()?;
                Ok(Some(Statement::Node(id)))
            }
        }
    }

    fn parse_opt_as(&mut self) -> Result<Option<String>> {
        if self.tok.is_id("AS") {
            self.advance()?;
            Ok(Some(self.expect_id("alias")?.to_owned()))
        } else {
            Ok(None)
        }
    }

    fn parse_route(&mut self) -> Result<Route> {
        self.advance()?; // ROUTE
        let from_node = self.expect_id("node name")?.to_owned();
        self.expect(TokenKind::Period, "`.`")?;
        let from_field = self.expect_id("eventOut name")?.to_owned();
        if !self.tok.is_id("TO") {
            return Err(self.err_here("expected `TO`"));
        }
        self.advance()?;
        let to_node = self.expect_id("node name")?.to_owned();
        self.expect(TokenKind::Period, "`.`")?;
        let to_field = self.expect_id("eventIn name")?.to_owned();
        let from_id = self.lookup_def(&from_node);
        let to_id = self.lookup_def(&to_node);
        Ok(Route {
            from_node,
            from_field,
            to_node,
            to_field,
            from_id,
            to_id,
        })
    }

    fn check_proto_budget(&self) -> Result<()> {
        if self.doc.protos.len() + self.doc.extern_protos.len() >= self.opts.limits.max_protos {
            return Err(Error::limit(format!(
                "more than {} PROTO declarations",
                self.opts.limits.max_protos
            )));
        }
        Ok(())
    }

    fn parse_proto(&mut self) -> Result<ProtoId> {
        self.check_proto_budget()?;
        self.advance()?; // PROTO
        let name = self.expect_id("prototype name")?.to_owned();
        self.expect(TokenKind::LBracket, "`[`")?;
        let mut interface = Vec::new();
        while self.tok.kind != TokenKind::RBracket {
            interface.push(self.parse_interface_decl(true, false)?);
        }
        self.advance()?; // ]
        self.expect(TokenKind::LBrace, "`{`")?;
        self.enter()?;
        // Reserve the id first so nested declarations get later ids.
        let id = self.doc.add_proto(ProtoDecl {
            name: name.clone(),
            interface,
            body: Vec::new(),
        });
        self.scopes.push(Scope::new());
        let body = self.parse_statements(false);
        self.scopes.pop();
        let body = body?;
        self.leave();
        self.expect(TokenKind::RBrace, "`}`")?;
        if !body.iter().any(|s| matches!(s, Statement::Node(_))) {
            return Err(self.err_here(format!("PROTO `{name}` body has no root node")));
        }
        self.doc.protos[id.0 as usize].body = body;
        if let Some(scope) = self.scopes.last_mut() {
            scope.protos.insert(name, NodeOrigin::Proto(id));
        }
        Ok(id)
    }

    fn parse_externproto(&mut self) -> Result<ExternProtoId> {
        self.check_proto_budget()?;
        self.advance()?; // EXTERNPROTO
        let name = self.expect_id("prototype name")?.to_owned();
        self.expect(TokenKind::LBracket, "`[`")?;
        let mut interface = Vec::new();
        while self.tok.kind != TokenKind::RBracket {
            interface.push(self.parse_interface_decl(false, false)?);
        }
        self.advance()?; // ]
        let urls = self.parse_typed_value(FieldType::MFString)?;
        let urls = match urls.data {
            FieldData::Strings(v) => v,
            _ => Vec::new(),
        };
        let id = self.doc.add_extern_proto(ExternProtoDecl {
            name: name.clone(),
            interface,
            urls,
        });
        if let Some(scope) = self.scopes.last_mut() {
            scope.protos.insert(name, NodeOrigin::ExternProto(id));
        }
        Ok(id)
    }

    /// `access type name [value | IS name]`.
    fn parse_interface_decl(&mut self, with_values: bool, allow_is: bool) -> Result<InterfaceDecl> {
        let t = self.tok;
        let access = if t.kind == TokenKind::Id {
            self.access_keyword(t.text)
        } else {
            None
        };
        let Some(access) = access else {
            return Err(self.err_here("expected an interface declaration"));
        };
        self.advance()?;
        let field_type = self.field_type_kw()?;
        let name = self.expect_id("field name")?.to_owned();
        let mut decl = InterfaceDecl {
            access,
            field_type,
            name,
            value: None,
            is: None,
        };
        if allow_is && self.tok.is_id("IS") {
            self.advance()?;
            decl.is = Some(self.expect_id("interface name")?.to_owned());
            return Ok(decl);
        }
        if with_values && access.has_value() {
            decl.value = Some(self.parse_typed_value(field_type)?);
        }
        Ok(decl)
    }

    // ---- nodes ------------------------------------------------------

    /// `node | DEF name node | USE name`.
    fn parse_node_statement(&mut self) -> Result<NodeId> {
        if self.tok.is_id("USE") {
            self.advance()?;
            let t = self.tok;
            let name = self.expect_id("node name after USE")?;
            return self
                .lookup_def(name)
                .ok_or_else(|| self.err_at(&t, format!("USE of undefined node name `{name}`")));
        }
        let def_name = if self.tok.is_id("DEF") {
            self.advance()?;
            Some(self.expect_id("node name after DEF")?.to_owned())
        } else {
            None
        };
        self.parse_node(def_name)
    }

    fn parse_node(&mut self, def_name: Option<String>) -> Result<NodeId> {
        let t = self.tok;
        let type_name = self.expect_id("node type")?;
        if Self::is_keyword(type_name) {
            return Err(self.err_at(&t, format!("unexpected keyword `{type_name}`")));
        }
        self.expect(TokenKind::LBrace, "`{` after node type")?;
        self.node_count += 1;
        if self.node_count > self.opts.limits.max_nodes {
            return Err(Error::limit(format!(
                "more than {} nodes",
                self.opts.limits.max_nodes
            )));
        }
        let origin = match self.lookup_proto(type_name) {
            Some(o) => o,
            None if self.opts.catalog.node(type_name).is_some() => NodeOrigin::Builtin,
            None => NodeOrigin::Unknown,
        };
        let mut node = Node::new(type_name);
        node.origin = origin;
        node.def_name = def_name.clone();
        // Register the DEF before the body so `USE` inside the body (and
        // ROUTEs in the body) resolve, as §4.6.2 describes.
        let id = self.doc.add_node(Node::new(type_name));
        if let Some(name) = &def_name {
            if let Some(scope) = self.scopes.last_mut() {
                scope.defs.insert(name.clone(), id);
            }
        }
        self.enter()?;
        while self.tok.kind != TokenKind::RBrace {
            if self.tok.kind != TokenKind::Id {
                return Err(self.err_here("expected a field name or `}`"));
            }
            match self.tok.text {
                "ROUTE" => {
                    let r = self.parse_route()?;
                    node.statements.push(Statement::Route(r));
                }
                "PROTO" => {
                    let p = self.parse_proto()?;
                    node.statements.push(Statement::Proto(p));
                }
                "EXTERNPROTO" => {
                    let p = self.parse_externproto()?;
                    node.statements.push(Statement::ExternProto(p));
                }
                w if self.access_keyword(w).is_some() => {
                    let d = self.parse_interface_decl(true, true)?;
                    node.interface.push(d);
                }
                _ => {
                    let f = self.parse_field(&node)?;
                    node.fields.push(f);
                }
            }
        }
        self.leave();
        self.advance()?; // }
        self.doc.nodes[id.0 as usize] = node;
        Ok(id)
    }

    fn declared_type(&self, node: &Node, field: &str) -> Option<FieldType> {
        if let Some(d) = crate::ast::find_decl(&node.interface, field) {
            return Some(d.field_type);
        }
        match node.origin {
            NodeOrigin::Proto(pid) => self
                .doc
                .proto(pid)
                .and_then(|p| p.decl(field))
                .map(|d| d.field_type),
            NodeOrigin::ExternProto(eid) => self
                .doc
                .extern_proto(eid)
                .and_then(|p| crate::ast::find_decl(&p.interface, field))
                .map(|d| d.field_type),
            NodeOrigin::Builtin | NodeOrigin::Unknown => {
                self.opts.catalog.field_type(&node.type_name, field)
            }
        }
    }

    fn parse_field(&mut self, node: &Node) -> Result<Field> {
        let name = self.tok.text.to_owned();
        self.advance()?;
        if self.tok.is_id("IS") {
            self.advance()?;
            let target = self.expect_id("interface name after IS")?.to_owned();
            return Ok(Field {
                name,
                binding: FieldBinding::Is(target),
                inferred: false,
            });
        }
        match self.declared_type(node, &name) {
            Some(ty) => Ok(Field {
                name,
                binding: FieldBinding::Value(self.parse_typed_value(ty)?),
                inferred: false,
            }),
            None => Ok(Field {
                name,
                binding: FieldBinding::Value(self.parse_inferred_value()?),
                inferred: true,
            }),
        }
    }

    // ---- values -----------------------------------------------------

    fn parse_typed_value(&mut self, ty: FieldType) -> Result<FieldValue> {
        let (scalar, comps) = ty.element();
        let mut value = FieldValue::empty(ty);
        if scalar == Scalar::Node && !ty.is_multi() {
            if self.tok.is_id("NULL") {
                self.advance()?;
                return Ok(value);
            }
            let id = self.parse_node_statement()?;
            value.data = FieldData::Nodes(vec![id]);
            return Ok(value);
        }
        if ty.is_multi() && self.tok.kind == TokenKind::LBracket {
            self.advance()?;
            while self.tok.kind != TokenKind::RBracket {
                if self.tok.kind == TokenKind::Eof {
                    return Err(self.err_here("unterminated `[`"));
                }
                if scalar == Scalar::Node && self.tok.is_id("NULL") {
                    self.advance()?;
                    continue;
                }
                self.parse_element(&mut value.data, scalar, comps)?;
                self.check_elements(&value)?;
            }
            self.advance()?; // ]
        } else {
            self.parse_element(&mut value.data, scalar, comps)?;
        }
        Ok(value)
    }

    fn check_elements(&self, v: &FieldValue) -> Result<()> {
        let n = match &v.data {
            FieldData::Floats(x) => x.len(),
            FieldData::Doubles(x) => x.len(),
            FieldData::Int32s(x) => x.len(),
            FieldData::Bools(x) => x.len(),
            FieldData::Strings(x) => x.len(),
            FieldData::Images(x) => x.len(),
            FieldData::Nodes(x) => x.len(),
        };
        if n > self.opts.limits.max_field_elements {
            return Err(Error::limit(format!(
                "field value exceeds {} elements",
                self.opts.limits.max_field_elements
            )));
        }
        Ok(())
    }

    fn number(&mut self, what: &str) -> Result<Token<'a>> {
        if self.tok.kind != TokenKind::Number {
            return Err(self.err_here(format!("expected {what}")));
        }
        let t = self.tok;
        self.advance()?;
        Ok(t)
    }

    fn parse_element(&mut self, data: &mut FieldData, scalar: Scalar, comps: usize) -> Result<()> {
        match (scalar, data) {
            (Scalar::Bool, FieldData::Bools(v)) => {
                let b = match self.tok.text {
                    "TRUE" if self.tok.kind == TokenKind::Id => true,
                    "FALSE" if self.tok.kind == TokenKind::Id => false,
                    _ => return Err(self.err_here("expected TRUE or FALSE")),
                };
                self.advance()?;
                v.push(b);
            }
            (Scalar::Int32, FieldData::Int32s(v)) => {
                let t = self.number("an integer")?;
                let i = parse_int32(t.text)
                    .ok_or_else(|| self.err_at(&t, format!("invalid int32 `{}`", t.text)))?;
                v.push(i);
            }
            (Scalar::Float, FieldData::Floats(v)) => {
                for _ in 0..comps {
                    let t = self.number("a number")?;
                    let f = parse_double(t.text)
                        .ok_or_else(|| self.err_at(&t, format!("invalid number `{}`", t.text)))?;
                    v.push(f as f32);
                }
            }
            (Scalar::Double, FieldData::Doubles(v)) => {
                for _ in 0..comps {
                    let t = self.number("a number")?;
                    let f = parse_double(t.text)
                        .ok_or_else(|| self.err_at(&t, format!("invalid number `{}`", t.text)))?;
                    v.push(f);
                }
            }
            (Scalar::String, FieldData::Strings(v)) => {
                let t = self.expect(TokenKind::String, "a quoted string")?;
                v.push(unescape(t.text));
            }
            (Scalar::Image, FieldData::Images(v)) => {
                v.push(self.parse_image()?);
            }
            (Scalar::Node, FieldData::Nodes(v)) => {
                v.push(self.parse_node_statement()?);
            }
            _ => return Err(self.err_here("internal: value/type mismatch")),
        }
        Ok(())
    }

    fn parse_image(&mut self) -> Result<Image> {
        let mut dims = [0i64; 3];
        for (i, d) in dims.iter_mut().enumerate() {
            let t = self.number(["image width", "image height", "image components"][i])?;
            *d = parse_int_wide(t.text)
                .filter(|v| (0..=u32::MAX as i64).contains(v))
                .ok_or_else(|| self.err_at(&t, "invalid SFImage dimension"))?;
        }
        let [w, h, c] = dims;
        if c > 4 {
            return Err(self.err_here("SFImage component count must be 0..=4"));
        }
        let count = (w as u64).saturating_mul(h as u64);
        if count > self.opts.limits.max_image_pixels {
            return Err(Error::limit(format!(
                "SFImage {w}x{h} exceeds {} pixels",
                self.opts.limits.max_image_pixels
            )));
        }
        // Grow as pixels are actually read; never pre-size from the
        // declared (untrusted) dimensions.
        let mut pixels = Vec::new();
        for _ in 0..count {
            let t = self.number("an SFImage pixel value")?;
            let p = parse_int_wide(t.text)
                .filter(|v| (0..=u32::MAX as i64).contains(v))
                .ok_or_else(|| self.err_at(&t, "invalid SFImage pixel"))?;
            pixels.push(p as u32);
        }
        Ok(Image {
            width: w as u32,
            height: h as u32,
            components: c as u32,
            pixels,
        })
    }

    /// Shape-based inference for undeclared fields.
    fn parse_inferred_value(&mut self) -> Result<FieldValue> {
        match self.tok.kind {
            TokenKind::LBracket => {
                let first = self.peek()?;
                let ty = match first.kind {
                    TokenKind::Number => {
                        if self.scan_numbers(false)?.1 {
                            FieldType::MFInt32
                        } else {
                            FieldType::MFFloat
                        }
                    }
                    TokenKind::String => FieldType::MFString,
                    TokenKind::Id if first.text == "TRUE" || first.text == "FALSE" => {
                        FieldType::MFBool
                    }
                    TokenKind::Id => FieldType::MFNode,
                    _ => FieldType::MFFloat,
                };
                self.parse_typed_value(ty)
            }
            TokenKind::Number => {
                let (count, all_int) = self.scan_numbers(true)?;
                let ty = match count {
                    1 if all_int => FieldType::SFInt32,
                    1 => FieldType::SFFloat,
                    2 => FieldType::SFVec2f,
                    3 => FieldType::SFVec3f,
                    4 => FieldType::SFRotation,
                    _ if all_int => FieldType::MFInt32,
                    _ => FieldType::MFFloat,
                };
                if ty.is_multi() {
                    // Unbracketed run of 5+ numbers: collect them all.
                    let mut value = FieldValue::empty(ty);
                    let scalar = ty.element().0;
                    while self.tok.kind == TokenKind::Number {
                        self.parse_element(&mut value.data, scalar, 1)?;
                        self.check_elements(&value)?;
                    }
                    Ok(value)
                } else {
                    self.parse_typed_value(ty)
                }
            }
            TokenKind::String => self.parse_typed_value(FieldType::SFString),
            TokenKind::Id => match self.tok.text {
                "TRUE" | "FALSE" => self.parse_typed_value(FieldType::SFBool),
                "NULL" | "DEF" | "USE" => self.parse_typed_value(FieldType::SFNode),
                _ => {
                    if self.peek()?.kind == TokenKind::LBrace {
                        self.parse_typed_value(FieldType::SFNode)
                    } else {
                        Err(self.err_here("expected a field value"))
                    }
                }
            },
            _ => Err(self.err_here("expected a field value")),
        }
    }

    /// Look ahead over a run of number tokens — starting at the current
    /// token (`from_current`) or right after it (inside `[`) — without
    /// consuming anything. Returns `(run length, all integer literals)`;
    /// the scan is capped so it stays O(1) per field.
    fn scan_numbers(&mut self, from_current: bool) -> Result<(usize, bool)> {
        const CAP: usize = 4096;
        let mut count = 0usize;
        let mut all_int = true;
        let mut visit = |t: &Token<'_>| -> bool {
            if t.kind != TokenKind::Number || count >= CAP {
                return false;
            }
            count += 1;
            if is_float_literal(t.text) && t.text.contains(['.', 'e', 'E']) {
                all_int = false;
            }
            true
        };
        let mut go = true;
        if from_current {
            go = visit(&self.tok);
        }
        if go {
            if let Some(p) = self.peeked {
                go = visit(&p);
            }
        }
        if go {
            let mut lx = self.lx.clone();
            loop {
                let t = lx.next_token()?;
                if !visit(&t) {
                    break;
                }
            }
        }
        Ok((count, all_int))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HDR: &str = "#VRML V2.0 utf8\n";

    fn p(body: &str) -> Document {
        parse(&format!("{HDR}{body}")).unwrap()
    }

    #[test]
    fn header_and_comment() {
        let d = parse("#VRML V2.0 utf8 made by hand\nGroup {}").unwrap();
        assert!(d.header.is_vrml97());
        assert_eq!(d.header.comment, "made by hand");
        assert!(parse("Group {}").is_err());
    }

    #[test]
    fn typed_fields_and_mf_without_brackets() {
        let d = p("Transform { translation 1 2 3 children Shape { geometry Box { size 1 2 3 } } }");
        let root = d.root_nodes().next().unwrap();
        let t = d.node(root).unwrap();
        assert_eq!(
            t.field("translation").unwrap().as_vec3f(),
            Some([1.0, 2.0, 3.0])
        );
        let kids = t.field("children").unwrap().node_list();
        assert_eq!(kids.len(), 1);
        assert_eq!(d.node(kids[0]).unwrap().type_name, "Shape");
    }

    #[test]
    fn def_use_shares_ids() {
        let d = p("Group { children [ DEF B Box {} Shape { geometry USE B } ] } USE B");
        let roots: Vec<_> = d.root_nodes().collect();
        assert_eq!(roots.len(), 2);
        let g = d.node(roots[0]).unwrap();
        let kids = g.field("children").unwrap().node_list();
        assert_eq!(roots[1], kids[0]);
        let shape = d.node(kids[1]).unwrap();
        assert_eq!(shape.field("geometry").unwrap().as_node(), Some(kids[0]));
    }

    #[test]
    fn undefined_use_is_error() {
        assert!(parse(&format!("{HDR}USE Nope")).is_err());
    }

    #[test]
    fn proto_and_is() {
        let d = p(
            "PROTO Thing [ field SFColor c 1 0 0 exposedField SFVec3f pos 0 0 0 ] {
               Transform { translation IS pos children Shape { appearance Appearance {
                 material Material { diffuseColor IS c } } } }
             }
             Thing { c 0 1 0 pos 1 1 1 }",
        );
        assert_eq!(d.protos.len(), 1);
        let inst = d.root_nodes().next().unwrap();
        let n = d.node(inst).unwrap();
        assert!(matches!(n.origin, NodeOrigin::Proto(_)));
        assert_eq!(n.field("c").unwrap().ty, FieldType::SFColor);
        assert_eq!(n.field("pos").unwrap().as_vec3f(), Some([1.0, 1.0, 1.0]));
    }

    #[test]
    fn externproto_types_fields() {
        let d = p(
            "EXTERNPROTO Ext [ field MFFloat k exposedField SFNode n ] \"lib.wrl#Ext\"
                   Ext { k [ 1 2 3 ] n NULL }",
        );
        assert_eq!(d.extern_protos[0].urls, ["lib.wrl#Ext"]);
        let n = d.node(d.root_nodes().next().unwrap()).unwrap();
        assert_eq!(n.field("k").unwrap().ty, FieldType::MFFloat);
    }

    #[test]
    fn routes_resolve() {
        let d = p("DEF T TimeSensor {} DEF I PositionInterpolator {}
                   ROUTE T.fraction_changed TO I.set_fraction");
        let r = d.routes().next().unwrap();
        assert_eq!(r.from_id, Some(NodeId(0)));
        assert_eq!(r.to_id, Some(NodeId(1)));
    }

    #[test]
    fn unknown_nodes_are_preserved() {
        let d = p("Foo { a 1 b 1.5 c 1 2 3 d \"x\" e [ 1 2 3 4 5 ] f TRUE g Bar { } h [ 0.5 1 ] }");
        let n = d.node(d.root_nodes().next().unwrap()).unwrap();
        assert_eq!(n.origin, NodeOrigin::Unknown);
        let ty = |k: &str| n.field(k).unwrap().ty;
        assert_eq!(ty("a"), FieldType::SFInt32);
        assert_eq!(ty("b"), FieldType::SFFloat);
        assert_eq!(ty("c"), FieldType::SFVec3f);
        assert_eq!(ty("d"), FieldType::SFString);
        assert_eq!(ty("e"), FieldType::MFInt32);
        assert_eq!(ty("f"), FieldType::SFBool);
        assert_eq!(ty("g"), FieldType::SFNode);
        assert_eq!(ty("h"), FieldType::MFFloat);
        assert!(n.fields.iter().all(|f| f.inferred));
    }

    #[test]
    fn script_interface() {
        let d = p(
            "Script { url \"javascript: function f(v) {}\" eventIn SFFloat f
                   field SFNode target NULL eventOut SFVec3f out }",
        );
        let n = d.node(d.root_nodes().next().unwrap()).unwrap();
        assert_eq!(n.interface.len(), 3);
    }

    #[test]
    fn sfimage() {
        let d = p("PixelTexture { image 2 1 3 0xFF0000 0x00FF00 }");
        let n = d.node(d.root_nodes().next().unwrap()).unwrap();
        let img = n.field("image").unwrap().as_image().unwrap();
        assert_eq!(img.pixels, [0xFF0000, 0x00FF00]);
        assert!(parse(&format!("{HDR}PixelTexture {{ image 100000 100000 3 }}")).is_err());
    }

    #[test]
    fn depth_limit() {
        let mut s = String::from(HDR);
        for _ in 0..1000 {
            s.push_str("Group { children ");
        }
        assert!(matches!(parse(&s), Err(Error::LimitExceeded(_))));
    }

    #[test]
    fn field_value_parsing() {
        let (v, _) = parse_field_value("[ 1 1, 1 -1, -1 -1 ]", FieldType::MFVec2f).unwrap();
        assert_eq!(v.vec2f_list().unwrap().len(), 3);
    }

    #[test]
    fn x3d_dialect_statements() {
        let opts = ParseOptions {
            dialect: Dialect::X3dClassic,
            ..ParseOptions::default()
        };
        let d = parse_with(
            "#X3D V3.3 utf8\nPROFILE Interchange\nCOMPONENT Geometry3D:2\nMETA \"a\" \"b\"\n\
             Group { }",
            &opts,
        )
        .unwrap();
        assert_eq!(d.header.format, "X3D");
        assert!(matches!(
            d.statements[1],
            Statement::Component { level: 2, .. }
        ));
    }
}
