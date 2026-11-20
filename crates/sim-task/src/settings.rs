//! Settings files: one `key = value` a line. `#` starts a comment; blank lines are skipped.
//!
//! Reading is strict: a line that is not `key = value`, a key given twice, a value that is not
//! what the key needs, and a key nobody asked for (`finish`) are errors with their line number.

use std::cell::Cell;

pub struct Settings {
    /// Where the settings came from, for errors.
    source: String,
    lines: Vec<(usize, String, String, Cell<bool>)>,
}

impl Settings {
    pub fn read(path: &str) -> Result<Settings, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
        Settings::parse(path, &text)
    }

    pub fn parse(source: &str, text: &str) -> Result<Settings, String> {
        let mut lines: Vec<(usize, String, String, Cell<bool>)> = vec![];
        for (n, line) in text.lines().enumerate() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let at = format!("{source}:{}", n + 1);
            let (k, v) = line.split_once('=').ok_or_else(|| format!("{at}: not `key = value`"))?;
            let (k, v) = (k.trim(), v.trim());
            if k.is_empty() || k.contains(char::is_whitespace) || v.is_empty() {
                return Err(format!("{at}: not `key = value`"));
            }
            if let Some(first) = lines.iter().find(|l| l.1 == k) {
                return Err(format!("{at}: `{k}` is already set on line {}", first.0));
            }
            lines.push((n + 1, k.to_string(), v.to_string(), Cell::new(false)));
        }
        Ok(Settings { source: source.to_string(), lines })
    }

    fn find(&self, key: &str) -> Option<&(usize, String, String, Cell<bool>)> {
        let l = self.lines.iter().find(|l| l.1 == key)?;
        l.3.set(true);
        Some(l)
    }

    fn error(&self, line: usize, key: &str, what: &str) -> String {
        format!("{}:{line}: `{key}` {what}", self.source)
    }

    /// The text of `key`, if it is set.
    pub fn text(&self, key: &str) -> Option<&str> {
        self.find(key).map(|l| l.2.as_str())
    }

    /// The text of `key`, which must be set.
    pub fn required(&self, key: &str) -> Result<&str, String> {
        self.text(key).ok_or_else(|| format!("{}: `{key}` is not set", self.source))
    }

    /// A number at least `min`; `default` when not set.
    pub fn number(&self, key: &str, default: f64, min: f64) -> Result<f64, String> {
        let Some((line, _, v, _)) = self.find(key) else { return Ok(default) };
        match v.parse::<f64>() {
            Ok(x) if x.is_finite() && x >= min => Ok(x),
            _ => Err(self.error(*line, key, &format!("must be a number, {min} or more"))),
        }
    }

    /// A whole number at least `min`; `default` when not set.
    pub fn count(&self, key: &str, default: usize, min: usize) -> Result<usize, String> {
        let Some((line, _, v, _)) = self.find(key) else { return Ok(default) };
        match v.parse::<usize>() {
            Ok(x) if x >= min => Ok(x),
            _ => Err(self.error(*line, key, &format!("must be a whole number, {min} or more"))),
        }
    }

    /// Numbers separated by commas or spaces; `default` when not set.
    pub fn numbers(&self, key: &str, default: &[f64]) -> Result<Vec<f64>, String> {
        let Some((line, _, v, _)) = self.find(key) else { return Ok(default.to_vec()) };
        v.split([',', ' ']).filter(|s| !s.is_empty()).map(|s| s.parse::<f64>().ok().filter(|x| x.is_finite()).ok_or_else(|| self.error(*line, key, "must be numbers"))).collect()
    }

    /// An error for the first key that nothing read: it is misspelt or does not belong here.
    pub fn finish(&self) -> Result<(), String> {
        match self.lines.iter().find(|l| !l.3.get()) {
            Some((line, key, _, _)) => Err(self.error(*line, key, "is not a setting here")),
            None => Ok(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_keys_and_rejects_the_rest() {
        let s = Settings::parse("t", "# a comment\nseconds = 20  # trailing\n\njoints = 40, 0 40,0\nname = a b\n").unwrap();
        assert_eq!(s.number("seconds", 1.0, 0.0).unwrap(), 20.0);
        assert_eq!(s.numbers("joints", &[]).unwrap(), vec![40.0, 0.0, 40.0, 0.0]);
        assert_eq!(s.count("threads", 3, 1).unwrap(), 3, "not set: the default");
        assert_eq!(s.text("name"), Some("a b"));
        assert!(s.finish().is_ok());

        let s = Settings::parse("t", "seconds = -1\nsecnods = 2\n").unwrap();
        assert_eq!(s.number("seconds", 1.0, 0.0).unwrap_err(), "t:1: `seconds` must be a number, 0 or more");
        assert_eq!(s.finish().unwrap_err(), "t:2: `secnods` is not a setting here");
        assert!(s.required("robot").is_err());

        for bad in ["seconds 20", "= 1", "a b = 1", "x =", "x = 1\nx = 2"] {
            assert!(Settings::parse("t", bad).is_err(), "{bad:?} should be an error");
        }
    }
}
