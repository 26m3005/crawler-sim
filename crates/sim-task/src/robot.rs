//! A crawler from a few parameters: a chassis with two main tracks and four flippers, each
//! flipper with its own belt. The parameters (a settings file, `key = value`) give the sizes,
//! masses, lugs and motors; everything that follows from them (joint positions, plates between
//! the belts, the flippers' centres of mass) is worked out, so the robot always fits together.
//!
//! The robot is made as extended URDF text (`urdf`) and read as any robot file. A key left out
//! takes the sample crawler's value, so an empty file is `robots/simple_crawler`.

use std::path::Path;

use sim_core::model::Model;

use crate::settings::Settings;

/// Reads a robot: extended URDF (`.urdf`) or crawler parameters (any other file).
pub fn load(path: &Path) -> Result<Model, String> {
    if path.extension().is_some_and(|e| e.eq_ignore_ascii_case("urdf")) {
        return sim_urdf::load_file(path);
    }
    let shown = path.display().to_string();
    let text = std::fs::read_to_string(path).map_err(|e| format!("{shown}: {e}"))?;
    load_text(&text, &shown)
}

/// A robot from crawler parameters given as text; `source` names it in errors.
pub fn load_text(text: &str, source: &str) -> Result<Model, String> {
    let urdf = urdf(&Settings::parse(source, text)?)?;
    sim_urdf::load_str(&urdf).map_err(|e| format!("{source}: the robot made from it does not read: {e}"))
}

/// A number for the file: derived values are rounded to 1 nm, so sums print as written.
fn n(x: f64) -> String {
    ((x * 1e9).round() / 1e9).to_string()
}

struct Lugs {
    count: usize,
    inner: f64,
    outer: f64,
    profile: String,
    ramp: Option<(f64, f64)>,
}

impl Lugs {
    /// The lugs of the `main` or `flipper` belts, on a belt from `inner` to `outer`.
    fn read(s: &Settings, part: &str, inner: f64, outer: f64) -> Result<Lugs, String> {
        let key = |k: &str| format!("{part}_lug_{k}");
        let count = s.count(&format!("{part}_lugs"), if part == "main" { 16 } else { 10 }, 0)?;
        let height = s.number(&key("height"), 0.015, 0.0)?;
        let length = s.number(&key("length"), 0.03, 0.0)?;
        let shape = s.text(&key("shape")).unwrap_or("rect");
        let (mut top, mut ramp) = (length, None);
        match shape {
            "rect" => {}
            "trapezoid" => top = s.number(&key("top"), length, 0.0)?,
            "ramp" => {
                let r = s.numbers(&key("ramp"), &[])?;
                let [from, inner_height] = r[..] else { return Err(format!("`{}` needs 2 numbers: from, inner_height", key("ramp"))) };
                ramp = Some((from, inner_height));
            }
            _ => return Err(format!("`{}` is rect, trapezoid or ramp, not `{shape}`", key("shape"))),
        }
        // A key for another shape is an error, not ignored.
        for (k, used) in [("top", shape == "trapezoid"), ("ramp", shape == "ramp")] {
            if !used && s.text(&key(k)).is_some() {
                return Err(format!("`{}` is only for {} lugs", key(k), if k == "top" { "trapezoid" } else { "ramp" }));
            }
        }
        let (b, t) = (length / 2.0, top / 2.0);
        let profile = format!("{} 0  {} 0  {} {h}  {} {h}", n(-b), n(b), n(t), n(-t), h = n(height));
        Ok(Lugs { count, inner, outer, profile, ramp })
    }

    fn xml(&self) -> String {
        if self.count == 0 {
            return String::new();
        }
        let ramp = self.ramp.map_or(String::new(), |(f, h)| format!("\n      <ramp from=\"{}\" inner_height=\"{}\"/>", n(f), n(h)));
        format!(
            "\n    <lugs count=\"{}\" spacing=\"even\" inner=\"{}\" outer=\"{}\">\n      <profile points=\"{}\"/>{ramp}\n    </lugs>",
            self.count,
            n(self.inner),
            n(self.outer),
            self.profile
        )
    }
}

/// A belt's inner and outer distance from the centre plane.
fn belt(s: &Settings, key: &str, default: [f64; 2]) -> Result<(f64, f64), String> {
    match s.numbers(key, &default)?[..] {
        [i, o] if 0.0 <= i && i < o => Ok((i, o)),
        _ => Err(format!("`{key}` needs 2 numbers, inner then outer, 0 <= inner < outer")),
    }
}

fn triple(s: &Settings, key: &str, default: [f64; 3]) -> Result<[f64; 3], String> {
    match s.numbers(key, &default)?[..] {
        [a, b, c] if a > 0.0 && b > 0.0 && c > 0.0 => Ok([a, b, c]),
        _ => Err(format!("`{key}` needs 3 numbers more than 0 (ixx, iyy, izz)")),
    }
}

/// The robot the parameters describe, as extended URDF. Reads every key; unknown keys are errors.
pub fn urdf(s: &Settings) -> Result<String, String> {
    let name = s.text("name").unwrap_or("simple_crawler").to_string();
    if !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-') {
        return Err("`name` may use letters, digits, _ and - only".into());
    }
    let chassis_mass = s.number("chassis_mass", 16.0, 0.0)?;
    let ci = triple(s, "chassis_inertia", [0.17, 0.45, 0.55])?;
    let chassis_friction = s.number("chassis_friction", 0.3, 0.0)?;
    let spacing = s.number("main_spacing", 0.5, 0.0)?;
    let main_radius = s.number("main_radius", 0.06, 0.0)?;
    let (mi, mo) = belt(s, "main_belt", [0.04, 0.14])?;
    let main_belt_mass = s.number("main_belt_mass", 1.0, 0.0)?;
    let main_lugs = Lugs::read(s, "main", mi, mo)?;
    let length = s.number("flipper_length", 0.24, 0.0)?;
    let flipper_radius = s.number("flipper_radius", 0.06, 0.0)?;
    let (fi, fo) = belt(s, "flipper_belt", [0.145, 0.185])?;
    let flipper_belt_mass = s.number("flipper_belt_mass", 0.5, 0.0)?;
    let flipper_lugs = Lugs::read(s, "flipper", fi, fo)?;
    let flipper_mass = s.number("flipper_mass", 1.0, 0.0)?;
    let fiv = triple(s, "flipper_inertia", [0.001, 0.006, 0.006])?;
    let range = s.numbers("joint_range", &[-1.57, 1.57])?;
    let velocity = s.number("joint_velocity", 1.5, 0.0)?;
    let effort = s.number("joint_effort", 60.0, 0.0)?;
    let gain = s.number("servo_gain", 4.0, 0.0)?;
    let torque = s.number("drive_torque", 30.0, 0.0)?;
    s.finish()?;
    let [lower, upper] = range[..] else { return Err("`joint_range` needs 2 numbers, lower then upper".into()) };
    if lower >= upper {
        return Err("`joint_range`: lower must be below upper".into());
    }
    if fi < mo {
        return Err(format!("`flipper_belt` must start outside the main belt (inner {fi} < main outer {mo})"));
    }
    if spacing <= 0.0 || length <= 0.0 || main_radius <= 0.0 || flipper_radius <= 0.0 {
        return Err("main_spacing, flipper_length and the radii must be more than 0".into());
    }

    let half = spacing / 2.0;
    let mut x = format!(
        "<?xml version=\"1.0\"?>\n<!-- Made by robot-sim from crawler parameters. -->\n<robot name=\"{name}\">\n  <link name=\"base_link\">\n    <inertial><origin xyz=\"0 0 0\"/><mass value=\"{}\"/><inertia ixx=\"{}\" iyy=\"{}\" izz=\"{}\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial>\n  </link>\n",
        n(chassis_mass),
        n(ci[0]),
        n(ci[1]),
        n(ci[2])
    );
    // (name, front?, left?)
    let flippers = [("front_left", true, true), ("rear_left", false, true), ("front_right", true, false), ("rear_right", false, false)];
    let fy = (fi + fo) / 2.0;
    for (f, front, left) in flippers {
        let (dx, dy) = (if front { 1.0 } else { -1.0 }, if left { 1.0 } else { -1.0 });
        x += &format!(
            "  <link name=\"flipper_{f}\">\n    <inertial><origin xyz=\"{} {} 0\"/><mass value=\"{}\"/><inertia ixx=\"{}\" iyy=\"{}\" izz=\"{}\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial>\n  </link>\n",
            n(dx * length / 2.0),
            n(dy * fy),
            n(flipper_mass),
            n(fiv[0]),
            n(fiv[1]),
            n(fiv[2])
        );
    }
    for (f, front, _) in flippers {
        let dx = if front { 1.0 } else { -1.0 };
        x += &format!(
            "  <joint name=\"{f}\" type=\"revolute\">\n    <parent link=\"base_link\"/><child link=\"flipper_{f}\"/>\n    <origin xyz=\"{} 0 0\"/><axis xyz=\"0 {} 0\"/>\n    <limit lower=\"{}\" upper=\"{}\" velocity=\"{}\" effort=\"{}\"/>\n  </joint>\n",
            n(dx * half),
            if front { "-1" } else { "1" },
            n(lower),
            n(upper),
            n(velocity),
            n(effort)
        );
    }
    for (f, _, _) in flippers {
        x += &format!("  <servo joint=\"{f}\" gain=\"{}\"/>\n", n(gain));
    }
    x += &format!("  <surface link=\"base_link\" friction=\"{}\"/>\n", n(chassis_friction));
    let track = |name: &str, side: &str, link: &str, a: f64, b: f64, r: f64, (i, o): (f64, f64), mass: f64, lugs: &Lugs| {
        format!(
            "  <track name=\"{name}\" side=\"{side}\" link=\"{link}\">\n    <pulley xyz=\"{} 0 0\" radius=\"{r}\"/>\n    <pulley xyz=\"{} 0 0\" radius=\"{r}\"/>\n    <belt inner=\"{}\" outer=\"{}\" mass=\"{}\"/>{}\n  </track>\n",
            n(a),
            n(b),
            n(i),
            n(o),
            n(mass),
            lugs.xml(),
            r = n(r)
        )
    };
    for side in ["left", "right"] {
        x += &track(&format!("{side}_main"), side, "base_link", -half, half, main_radius, (mi, mo), main_belt_mass, &main_lugs);
    }
    for (f, front, left) in flippers {
        let (side, end) = (if left { "left" } else { "right" }, if front { length } else { -length });
        let place = if front { "front" } else { "rear" };
        x += &track(&format!("{side}_{place}_flipper"), side, &format!("flipper_{f}"), 0.0, end, flipper_radius, (fi, fo), flipper_belt_mass, &flipper_lugs);
    }
    for side in ["left", "right"] {
        x += &format!("  <drive name=\"{side}\" max_torque=\"{}\" tracks=\"{side}_main {side}_front_flipper {side}_rear_flipper\"/>\n", n(torque));
    }
    // Plates: the chassis between the main belts, and each side between its main and flipper belts.
    if mi > 0.0 {
        x += &format!("  <plate track=\"left_main\" y=\"{} {}\" inset=\"0.005\"/>\n", n(-mi), n(mi));
    }
    if fi > mo {
        x += &format!("  <plate track=\"left_main\" y=\"{} {}\" inset=\"0.005\"/>\n", n(mo), n(fi));
        x += &format!("  <plate track=\"right_main\" y=\"{} {}\" inset=\"0.005\"/>\n", n(-fi), n(-mo));
    }
    x += "</robot>\n";
    Ok(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn made(text: &str) -> Result<Model, String> {
        sim_urdf::load_str(&urdf(&Settings::parse("t", text)?)?)
    }

    #[test]
    fn the_defaults_are_the_sample_crawler() {
        let sample = sim_urdf::load_str(include_str!("../../../robots/simple_crawler/simple_crawler.urdf")).unwrap();
        assert_eq!(made("").unwrap(), sample);
        let file = load(Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../robots/simple_crawler/simple_crawler.txt"))).unwrap();
        assert_eq!(file, sample, "the sample's parameter file gives the sample too");
    }

    #[test]
    fn lug_shapes() {
        let m = made("main_lug_shape = trapezoid\nmain_lug_top = 0.01\nflipper_lugs = 0\n").unwrap();
        let lugs = m.tracks[0].lugs.as_ref().unwrap();
        assert!((lugs.base - 0.03).abs() < 1e-12 && (lugs.tip - 0.01).abs() < 1e-12);
        assert!(m.tracks.iter().filter(|t| t.lugs.is_none()).count() == 4, "flippers without lugs");
        let m = made("main_lug_shape = ramp\nmain_lug_ramp = 0.1, 0.005\n").unwrap();
        assert!(m.tracks[0].lugs.as_ref().unwrap().ramp.is_some());
    }

    #[test]
    fn mistakes_are_errors() {
        for bad in [
            "main_lug_top = 0.01",                    // top without trapezoid
            "main_lug_shape = round",                 // no such shape
            "main_lug_shape = ramp",                  // ramp without its numbers
            "flipper_belt = 0.1, 0.2",                // overlaps the main belt
            "main_belt = 0.14, 0.04",                 // inner past outer
            "joint_range = 1, -1",
            "chassis_inertia = 1, 2",
            "name = a b",
            "wheel_radius = 0.1",                     // unknown key
        ] {
            assert!(made(bad).is_err(), "{bad:?} should be an error");
        }
    }
}
