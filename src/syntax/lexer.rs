//! Tokeniser for the UTF-8 clear-text encoding (ISO/IEC 14772-1 §4.3.1
//! and Annex A, Table A.1).
//!
//! * Whitespace is space, tab, CR, LF **and comma**; every other
//!   control character is tolerated as whitespace too.
//! * `#` starts a comment running to the next line terminator.
//! * Terminal symbols: `{ } [ ] .` (plus `:` in the X3D dialect, used
//!   by `COMPONENT name:level`).
//! * A *word* is a maximal run of identifier characters (Annex A
//!   `IdRestChars`). Words starting with a digit, `+`, `-` or `.`
//!   (followed by a digit) are numbers; every other word is an
//!   identifier or keyword — the parser decides which.
//! * Strings are double-quoted; `\"` and `\\` are the only escapes.
//!
//! Tokens borrow from the source; string tokens keep their raw
//! (still-escaped) text and are decoded with [`unescape`].

use crate::error::{Error, Result};

/// Token category.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TokenKind {
    /// Identifier or keyword.
    Id,
    /// Numeric literal (int32 / float / double; validated on use).
    Number,
    /// Double-quoted string (raw text between the quotes).
    String,
    /// `{`
    LBrace,
    /// `}`
    RBrace,
    /// `[`
    LBracket,
    /// `]`
    RBracket,
    /// `.`
    Period,
    /// `:` (X3D dialect only).
    Colon,
    /// End of input.
    Eof,
}

/// One token with its source position.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Token<'a> {
    /// Category.
    pub kind: TokenKind,
    /// Raw text (for strings: between the quotes, escapes not yet
    /// decoded).
    pub text: &'a str,
    /// 1-based line.
    pub line: usize,
    /// 1-based column (in characters).
    pub column: usize,
}

impl Token<'_> {
    /// `true` when this is the identifier/keyword `word`.
    pub fn is_id(&self, word: &str) -> bool {
        self.kind == TokenKind::Id && self.text == word
    }
}

/// Streaming tokeniser over a UTF-8 source string.
#[derive(Clone, Debug)]
pub struct Lexer<'a> {
    src: &'a str,
    pos: usize,
    line: usize,
    column: usize,
    colon_terminal: bool,
}

#[inline]
fn is_separator(b: u8) -> bool {
    b <= 0x20 || b == b',' || b == 0x7f
}

/// Bytes that end a word (Annex A `IdRestChars` exclusions, plus the
/// separators). Note `.` is a terminal for identifiers but part of
/// numbers; the number scanner handles it separately.
#[inline]
fn ends_word(b: u8, colon_terminal: bool) -> bool {
    is_separator(b)
        || matches!(
            b,
            b'"' | b'#' | b'\'' | b'.' | b'[' | b'\\' | b']' | b'{' | b'}'
        )
        || (colon_terminal && b == b':')
}

impl<'a> Lexer<'a> {
    /// Lexer over `src`, starting at line 1 column 1.
    pub fn new(src: &'a str) -> Self {
        Self {
            src,
            pos: 0,
            line: 1,
            column: 1,
            colon_terminal: false,
        }
    }

    /// Treat `:` as a terminal symbol (X3D `COMPONENT name:level`).
    pub fn with_colon_terminal(mut self, on: bool) -> Self {
        self.colon_terminal = on;
        self
    }

    /// Byte offset of the next unread character.
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Current 1-based line.
    pub fn line(&self) -> usize {
        self.line
    }

    /// The unread remainder of the source.
    pub fn rest(&self) -> &'a str {
        &self.src[self.pos..]
    }

    fn advance_bytes(&mut self, n: usize) {
        let end = (self.pos + n).min(self.src.len());
        for &b in &self.src.as_bytes()[self.pos..end] {
            if b == b'\n' {
                self.line += 1;
                self.column = 1;
            } else if b == b'\r' {
                // CR LF counts once (at the LF); a lone CR is a line
                // terminator on its own.
                let next = self.src.as_bytes().get(self.pos + 1);
                if next != Some(&b'\n') {
                    self.line += 1;
                    self.column = 1;
                }
            } else if b & 0xC0 != 0x80 {
                self.column += 1;
            }
            self.pos += 1;
        }
    }

    fn skip_whitespace_and_comments(&mut self) {
        let bytes = self.src.as_bytes();
        loop {
            let start = self.pos;
            let mut i = self.pos;
            while i < bytes.len() && is_separator(bytes[i]) {
                i += 1;
            }
            if i < bytes.len() && bytes[i] == b'#' {
                while i < bytes.len() && bytes[i] != b'\n' && bytes[i] != b'\r' {
                    i += 1;
                }
            }
            if i == start {
                return;
            }
            self.advance_bytes(i - start);
        }
    }

    fn err(&self, msg: impl Into<String>) -> Error {
        Error::syntax(self.line, self.column, msg)
    }

    /// Read the next token.
    pub fn next_token(&mut self) -> Result<Token<'a>> {
        self.skip_whitespace_and_comments();
        let bytes = self.src.as_bytes();
        let line = self.line;
        let column = self.column;
        let start = self.pos;
        let Some(&b) = bytes.get(start) else {
            return Ok(Token {
                kind: TokenKind::Eof,
                text: "",
                line,
                column,
            });
        };
        let single = |kind| Token {
            kind,
            text: &self.src[start..start + 1],
            line,
            column,
        };
        let tok = match b {
            b'{' => single(TokenKind::LBrace),
            b'}' => single(TokenKind::RBrace),
            b'[' => single(TokenKind::LBracket),
            b']' => single(TokenKind::RBracket),
            b':' if self.colon_terminal => single(TokenKind::Colon),
            b'"' => {
                let mut i = start + 1;
                loop {
                    match bytes.get(i) {
                        None => return Err(self.err("unterminated string")),
                        Some(b'\\') => i += 2,
                        Some(b'"') => break,
                        Some(_) => i += 1,
                    }
                }
                // `i` may have skipped past the end on a trailing `\`.
                if i >= bytes.len() {
                    return Err(self.err("unterminated string"));
                }
                let tok = Token {
                    kind: TokenKind::String,
                    text: &self.src[start + 1..i],
                    line,
                    column,
                };
                self.advance_bytes(i + 1 - start);
                return Ok(tok);
            }
            b'.' if !bytes.get(start + 1).is_some_and(u8::is_ascii_digit) => {
                single(TokenKind::Period)
            }
            b'0'..=b'9' | b'+' | b'-' | b'.' => {
                let mut i = start + 1;
                while i < bytes.len()
                    && (bytes[i].is_ascii_alphanumeric() || matches!(bytes[i], b'.' | b'+' | b'-'))
                {
                    i += 1;
                }
                let tok = Token {
                    kind: TokenKind::Number,
                    text: &self.src[start..i],
                    line,
                    column,
                };
                self.advance_bytes(i - start);
                return Ok(tok);
            }
            b'\'' | b'\\' => {
                return Err(self.err(format!("unexpected character {:?}", b as char)));
            }
            _ => {
                let mut i = start;
                while i < bytes.len() && !ends_word(bytes[i], self.colon_terminal) {
                    i += 1;
                }
                if i == start {
                    return Err(self.err(format!("unexpected character {:?}", b as char)));
                }
                let tok = Token {
                    kind: TokenKind::Id,
                    text: &self.src[start..i],
                    line,
                    column,
                };
                self.advance_bytes(i - start);
                return Ok(tok);
            }
        };
        self.advance_bytes(1);
        Ok(tok)
    }
}

/// Decode the `\"` / `\\` escapes of a raw string token. Any other
/// backslash sequence keeps the escaped character.
pub fn unescape(raw: &str) -> String {
    if !raw.contains('\\') {
        return raw.to_owned();
    }
    let mut out = String::with_capacity(raw.len());
    let mut chars = raw.chars();
    while let Some(c) = chars.next() {
        if c == '\\' {
            if let Some(n) = chars.next() {
                out.push(n);
            }
        } else {
            out.push(c);
        }
    }
    out
}

/// Escape a string for output between double quotes.
pub fn escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    for c in s.chars() {
        if c == '"' || c == '\\' {
            out.push('\\');
        }
        out.push(c);
    }
    out
}

/// Parse an `int32` literal (decimal or `0x` hexadecimal, optional
/// sign). Hex literals up to `0xFFFFFFFF` wrap into the i32 range so
/// packed SFImage pixels survive.
pub fn parse_int32(text: &str) -> Option<i32> {
    parse_int_wide(text).and_then(|v| {
        if (i32::MIN as i64..=u32::MAX as i64).contains(&v) {
            Some(v as u32 as i32)
        } else {
            None
        }
    })
}

/// Parse an integer literal into i64 (no range check beyond i64).
pub fn parse_int_wide(text: &str) -> Option<i64> {
    let (neg, body) = match text.as_bytes().first()? {
        b'-' => (true, &text[1..]),
        b'+' => (false, &text[1..]),
        _ => (false, text),
    };
    let v = if let Some(hex) = body.strip_prefix("0x").or_else(|| body.strip_prefix("0X")) {
        if hex.is_empty() || hex.len() > 16 {
            return None;
        }
        i64::from_str_radix(hex, 16).ok()?
    } else {
        if body.is_empty() || !body.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        body.parse::<i64>().ok()?
    };
    Some(if neg { -v } else { v })
}

/// `true` if `text` matches the Annex A float/double production
/// `[+-]?(([0-9]+\.?)|([0-9]*\.[0-9]+))([eE][+-]?[0-9]+)?`.
pub fn is_float_literal(text: &str) -> bool {
    let b = text.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let int_start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    let mut digits = i - int_start;
    if i < b.len() && b[i] == b'.' {
        i += 1;
        let frac_start = i;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
        digits += i - frac_start;
    }
    if digits == 0 {
        return false;
    }
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        i += 1;
        if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
            i += 1;
        }
        let exp_start = i;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
        if i == exp_start {
            return false;
        }
    }
    i == b.len()
}

/// Parse a float/double literal. Integer literals (including hex) are
/// accepted too since several real-world exporters write them in float
/// fields.
pub fn parse_double(text: &str) -> Option<f64> {
    if is_float_literal(text) {
        return text.parse::<f64>().ok();
    }
    parse_int_wide(text).map(|v| v as f64)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kinds(src: &str) -> Vec<(TokenKind, String)> {
        let mut lx = Lexer::new(src);
        let mut out = Vec::new();
        loop {
            let t = lx.next_token().unwrap();
            if t.kind == TokenKind::Eof {
                break;
            }
            out.push((t.kind, t.text.to_string()));
        }
        out
    }

    #[test]
    fn route_tokens() {
        let k = kinds("ROUTE A.fraction_changed TO B .set_fraction");
        assert_eq!(k.len(), 8);
        assert_eq!(k[2], (TokenKind::Period, ".".into()));
        assert_eq!(k[6].0, TokenKind::Period);
    }

    #[test]
    fn numbers_commas_and_comments() {
        let k = kinds("[1,-2.5e3 .5,0xFF] # trailing \n +3.");
        let nums: Vec<_> = k
            .iter()
            .filter(|t| t.0 == TokenKind::Number)
            .map(|t| t.1.as_str())
            .collect();
        assert_eq!(nums, ["1", "-2.5e3", ".5", "0xFF", "+3."]);
    }

    #[test]
    fn strings_with_escapes() {
        let k = kinds(r#""He said, \"hi\" # not a comment""#);
        assert_eq!(k.len(), 1);
        assert_eq!(unescape(&k[0].1), r#"He said, "hi" # not a comment"#);
        assert_eq!(escape(r#"a"b\c"#), r#"a\"b\\c"#);
    }

    #[test]
    fn literals() {
        assert_eq!(parse_int32("-0xE20"), Some(-0xE20));
        assert_eq!(parse_int32("0xFFFFFFFF"), Some(-1));
        assert!(is_float_literal("1."));
        assert!(is_float_literal(".0001"));
        assert!(is_float_literal("12.5e-3"));
        assert!(!is_float_literal("."));
        assert!(!is_float_literal("1e"));
        assert!(!is_float_literal("inf"));
        assert_eq!(parse_double("0x10"), Some(16.0));
    }

    #[test]
    fn unterminated_string_errors() {
        assert!(Lexer::new("\"abc").next_token().is_err());
        assert!(Lexer::new("\"abc\\").next_token().is_err());
    }

    #[test]
    fn positions() {
        let mut lx = Lexer::new("a\r\n  b\rc");
        let a = lx.next_token().unwrap();
        let b = lx.next_token().unwrap();
        let c = lx.next_token().unwrap();
        assert_eq!((a.line, a.column), (1, 1));
        assert_eq!((b.line, b.column), (2, 3));
        assert_eq!((c.line, c.column), (3, 1));
    }
}
