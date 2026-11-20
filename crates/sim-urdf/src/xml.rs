//! Just enough XML for URDF: elements, attributes, comments and declarations. Text is ignored.

#[derive(Clone, Debug, PartialEq)]
pub struct Element {
    pub name: String,
    pub attrs: Vec<(String, String)>,
    pub children: Vec<Element>,
    /// 1-based line of the opening tag, for error messages.
    pub line: usize,
}

impl Element {
    pub fn attr(&self, name: &str) -> Option<&str> {
        self.attrs.iter().find(|(k, _)| k == name).map(|(_, v)| v.as_str())
    }

    pub fn child(&self, name: &str) -> Option<&Element> {
        self.children.iter().find(|c| c.name == name)
    }

    pub fn children_named<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Element> + 'a {
        self.children.iter().filter(move |c| c.name == name)
    }
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
}

impl<'a> Parser<'a> {
    fn line(&self) -> usize {
        1 + self.s[..self.i.min(self.s.len())].iter().filter(|&&c| c == b'\n').count()
    }

    fn err(&self, what: &str) -> String {
        format!("xml line {}: {}", self.line(), what)
    }

    fn starts(&self, p: &str) -> bool {
        self.s[self.i..].starts_with(p.as_bytes())
    }

    fn skip_until(&mut self, end: &str) -> Result<(), String> {
        while self.i < self.s.len() {
            if self.starts(end) {
                self.i += end.len();
                return Ok(());
            }
            self.i += 1;
        }
        Err(self.err(&format!("missing {end}")))
    }

    fn skip_space(&mut self) {
        while self.i < self.s.len() && self.s[self.i].is_ascii_whitespace() {
            self.i += 1;
        }
    }

    /// Skips text, comments, declarations and processing instructions up to the next tag.
    fn skip_misc(&mut self) -> Result<(), String> {
        loop {
            while self.i < self.s.len() && self.s[self.i] != b'<' {
                self.i += 1;
            }
            if self.starts("<!--") {
                self.skip_until("-->")?;
            } else if self.starts("<?") {
                self.skip_until("?>")?;
            } else if self.starts("<!") {
                self.skip_until(">")?;
            } else {
                return Ok(());
            }
        }
    }

    fn name(&mut self) -> Result<String, String> {
        let start = self.i;
        while self.i < self.s.len() {
            let c = self.s[self.i];
            if c.is_ascii_alphanumeric() || c == b'_' || c == b'-' || c == b':' || c == b'.' {
                self.i += 1;
            } else {
                break;
            }
        }
        if start == self.i {
            return Err(self.err("expected a name"));
        }
        Ok(String::from_utf8_lossy(&self.s[start..self.i]).into_owned())
    }

    fn element(&mut self) -> Result<Element, String> {
        let line = self.line();
        if !self.starts("<") {
            return Err(self.err("expected an element"));
        }
        self.i += 1;
        let name = self.name()?;
        let mut attrs = vec![];
        loop {
            self.skip_space();
            if self.starts("/>") {
                self.i += 2;
                return Ok(Element { name, attrs, children: vec![], line });
            }
            if self.starts(">") {
                self.i += 1;
                break;
            }
            let key = self.name()?;
            self.skip_space();
            if !self.starts("=") {
                return Err(self.err("expected ="));
            }
            self.i += 1;
            self.skip_space();
            let quote = *self.s.get(self.i).ok_or_else(|| self.err("unexpected end"))?;
            if quote != b'"' && quote != b'\'' {
                return Err(self.err("expected a quoted value"));
            }
            self.i += 1;
            let start = self.i;
            while self.i < self.s.len() && self.s[self.i] != quote {
                self.i += 1;
            }
            if self.i >= self.s.len() {
                return Err(self.err("unterminated value"));
            }
            let raw = String::from_utf8_lossy(&self.s[start..self.i]).into_owned();
            self.i += 1;
            attrs.push((key, unescape(&raw)));
        }
        let mut children = vec![];
        loop {
            self.skip_misc()?;
            if self.i >= self.s.len() {
                return Err(self.err(&format!("missing </{name}>")));
            }
            if self.starts("</") {
                self.i += 2;
                let end = self.name()?;
                if end != name {
                    return Err(self.err(&format!("</{end}> closes <{name}>")));
                }
                self.skip_space();
                if !self.starts(">") {
                    return Err(self.err("expected >"));
                }
                self.i += 1;
                return Ok(Element { name, attrs, children, line });
            }
            children.push(self.element()?);
        }
    }
}

fn unescape(s: &str) -> String {
    s.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"").replace("&apos;", "'").replace("&amp;", "&")
}

/// Parses a document and returns its root element.
pub fn parse(text: &str) -> Result<Element, String> {
    let mut p = Parser { s: text.as_bytes(), i: 0 };
    p.skip_misc()?;
    let root = p.element()?;
    p.skip_misc()?;
    if p.i < p.s.len() {
        return Err(p.err("text after the root element"));
    }
    Ok(root)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nested_with_comments() {
        let doc = r#"<?xml version="1.0"?>
<!-- a robot -->
<robot name="r">
  <link name='a &amp; b'><inertial><mass value="2"/></inertial></link>
  text is ignored
</robot>"#;
        let r = parse(doc).unwrap();
        assert_eq!(r.attr("name"), Some("r"));
        let link = r.child("link").unwrap();
        assert_eq!(link.attr("name"), Some("a & b"));
        assert_eq!(link.child("inertial").unwrap().child("mass").unwrap().attr("value"), Some("2"));
        assert_eq!(link.line, 4);
    }

    #[test]
    fn mismatched_tags_are_errors() {
        assert!(parse("<a><b></a></b>").is_err());
        assert!(parse("<a>").is_err());
    }
}
