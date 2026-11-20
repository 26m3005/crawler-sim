//! Builds a crawler [`Model`] from an extended URDF.
//!
//! Standard URDF gives the links, joints, inertia and collision boxes. Extra elements under
//! `<robot>` describe what URDF cannot (see "ロボットの書き方" in README.md):
//!
//! - `<track>`: a belt round two pulleys, with its lugs (and its own drive, or none)
//! - `<drive>`: one motor turning several tracks together
//! - `<plate>`: a rigid plate with a track's outline, filling what the belts do not wrap
//! - `<servo>`: the position servo of a joint
//! - `<surface>`: contact properties of the chassis
//!
//! Supported: one root link, revolute joints from it (axis along y), and box collisions with
//! no rotation, on any link. Unknown elements and attributes are refused with their line
//! ([`schema`]); only `<visual>`, `<material>`, `<gazebo>` and `<transmission>` are skipped.

mod schema;
mod xml;

use sim_core::math::{v3, V3};
use sim_core::model::{BoxCollider, Drive, Inertial, Joint, Lugs, Model, Plate, Ramp, Side, Surface, Track};
use xml::Element;

pub fn load_file(path: &std::path::Path) -> Result<Model, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    load_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

pub fn load_str(text: &str) -> Result<Model, String> {
    let root = xml::parse(text)?;
    if root.name != "robot" {
        return Err("the root element must be <robot>".into());
    }
    schema::check(&root)?;
    Builder { robot: &root }.build()
}

pub(crate) fn at(e: &Element, msg: impl std::fmt::Display) -> String {
    format!("line {} <{}>: {}", e.line, e.name, msg)
}

pub(crate) fn req<'a>(e: &'a Element, name: &str) -> Result<&'a str, String> {
    e.attr(name).ok_or_else(|| at(e, format!("missing attribute {name}")))
}

pub(crate) fn num(e: &Element, name: &str) -> Result<f64, String> {
    let s = req(e, name)?;
    s.trim().parse().map_err(|_| at(e, format!("{name}=\"{s}\" is not a number")))
}

pub(crate) fn num_or(e: &Element, name: &str, default: f64) -> Result<f64, String> {
    if e.attr(name).is_some() {
        num(e, name)
    } else {
        Ok(default)
    }
}

pub(crate) fn numbers(e: &Element, name: &str) -> Result<Vec<f64>, String> {
    let s = req(e, name)?;
    s.split_whitespace().map(|v| v.parse().map_err(|_| at(e, format!("{name}=\"{s}\" holds a non-number")))).collect()
}

pub(crate) fn vec3(e: &Element, name: &str) -> Result<V3, String> {
    let v = numbers(e, name)?;
    if v.len() != 3 {
        return Err(at(e, format!("{name} needs three numbers")));
    }
    Ok(v3(v[0], v[1], v[2]))
}

/// The `xyz` of an optional `<origin>`, refusing any rotation.
fn origin(parent: &Element) -> Result<V3, String> {
    let Some(o) = parent.child("origin") else { return Ok(V3::ZERO) };
    if let Some(rpy) = o.attr("rpy") {
        if rpy.split_whitespace().any(|v| v.parse::<f64>().map_or(true, |x| x != 0.0)) {
            return Err(at(o, "rotated origins are not supported"));
        }
    }
    if o.attr("xyz").is_some() {
        vec3(o, "xyz")
    } else {
        Ok(V3::ZERO)
    }
}

/// Mass, centre of mass and principal moments (off-diagonal terms are dropped).
fn inertial(link: &Element) -> Result<Option<Inertial>, String> {
    let Some(i) = link.child("inertial") else { return Ok(None) };
    let mass = num(i.child("mass").ok_or_else(|| at(i, "missing <mass>"))?, "value")?;
    let t = i.child("inertia").ok_or_else(|| at(i, "missing <inertia>"))?;
    Ok(Some(Inertial { mass, com: origin(i)?, moments: v3(num(t, "ixx")?, num(t, "iyy")?, num(t, "izz")?) }))
}

pub(crate) fn surface(e: Option<&Element>) -> Result<Surface, String> {
    let d = Surface::default();
    let Some(e) = e else { return Ok(d) };
    Ok(Surface {
        friction: if e.attr("friction").is_some() { Some(num(e, "friction")?) } else { None },
        stiffness: num_or(e, "stiffness", d.stiffness)?,
        damping: num_or(e, "damping", d.damping)?,
    })
}

struct Builder<'a> {
    robot: &'a Element,
}

impl<'a> Builder<'a> {
    fn build(&self) -> Result<Model, String> {
        let links: Vec<&Element> = self.robot.children_named("link").collect();
        let joints_xml: Vec<&Element> = self.robot.children_named("joint").collect();
        let children: Vec<&str> = joints_xml.iter().map(|j| j.child("child").and_then(|c| c.attr("link")).unwrap_or("")).collect();
        let roots: Vec<&&Element> = links.iter().filter(|l| !children.contains(&l.attr("name").unwrap_or(""))).collect();
        let [root] = roots.as_slice() else { return Err(format!("expected one root link, found {}", roots.len())) };
        let root_name = req(root, "name")?;
        let link = |name: &str| links.iter().find(|l| l.attr("name") == Some(name)).copied();

        // Joints, and the bodies they carry.
        let mut joints = vec![];
        let mut joint_links = vec![];
        let base = inertial(root)?.unwrap_or(Inertial { mass: 0.0, com: V3::ZERO, moments: V3::ZERO });
        for j in &joints_xml {
            let name = req(j, "name")?.to_string();
            if req(j, "type")? != "revolute" {
                return Err(at(j, "only revolute joints are supported"));
            }
            let parent = j.child("parent").ok_or_else(|| at(j, "missing <parent>"))?;
            if req(parent, "link")? != root_name {
                return Err(at(j, "joints must hang from the root link"));
            }
            let child_name = req(j.child("child").ok_or_else(|| at(j, "missing <child>"))?, "link")?;
            let child = link(child_name).ok_or_else(|| at(j, format!("no link {child_name}")))?;
            let at_origin = origin(j)?;
            let axis_el = j.child("axis").ok_or_else(|| at(j, "missing <axis>"))?;
            let axis = vec3(axis_el, "xyz")?;
            let limit = j.child("limit").ok_or_else(|| at(j, "missing <limit>"))?;
            let ci = inertial(child)?.ok_or_else(|| at(child, "a joint's child needs <inertial>"))?;
            // Inertia about the axis (along y): the link's own, plus its mass off the axis.
            let about_axis = ci.moments.y + ci.mass * (ci.com.x * ci.com.x + ci.com.z * ci.com.z);
            joints.push(Joint {
                child: ci,
                name,
                origin: at_origin,
                axis,
                lower: num(limit, "lower")?,
                upper: num(limit, "upper")?,
                max_velocity: num(limit, "velocity")?,
                max_effort: num(limit, "effort")?,
                inertia: about_axis,
                servo_gain: 0.0,
            });
            joint_links.push(child_name);
        }
        for s in self.robot.children_named("servo") {
            let jn = req(s, "joint")?;
            let j = joints.iter_mut().find(|j| j.name == jn).ok_or_else(|| at(s, format!("no joint {jn}")))?;
            if j.servo_gain > 0.0 {
                return Err(at(s, format!("joint {jn} already has a <servo>")));
            }
            j.servo_gain = num(s, "gain")?;
            if !(j.servo_gain > 0.0) {
                return Err(at(s, "gain must be positive"));
            }
        }
        if let Some(j) = joints.iter().find(|j| j.servo_gain <= 0.0) {
            return Err(format!("joint {} has no <servo>", j.name));
        }

        let mass = base.mass + joints.iter().map(|j| j.child.mass).sum::<f64>();
        if !(mass > 0.0) {
            return Err("the robot has no mass".into());
        }

        // Collision boxes, each in its link's frame.
        let mut boxes = vec![];
        let carriers = std::iter::once((**root, None)).chain(joint_links.iter().enumerate().map(|(j, n)| (link(n).unwrap(), Some(j))));
        for (l, joint) in carriers {
            for c in l.children_named("collision") {
                let centre = origin(c)?;
                let g = c.child("geometry").ok_or_else(|| at(c, "missing <geometry>"))?;
                let b = g.child("box").ok_or_else(|| at(g, "only <box> collisions are supported"))?;
                let half = vec3(b, "size")? * 0.5;
                boxes.push(BoxCollider { min: centre - half, max: centre + half, joint });
            }
        }

        // Tracks, and the drives that turn them: shared ones first, then one per track that has its own.
        let mut tracks = vec![];
        for t in self.robot.children_named("track") {
            tracks.push(self.track(t, root_name, &joint_links)?);
        }
        let names: Vec<String> = self.robot.children_named("track").map(|t| t.attr("name").unwrap_or("").to_string()).collect();
        let mut drives: Vec<Drive> = vec![];
        let mut owner: Vec<Option<usize>> = vec![None; tracks.len()];
        for d in self.robot.children_named("drive") {
            let mut members = vec![];
            for n in req(d, "tracks")?.split_whitespace() {
                let k = names.iter().position(|x| x == n).ok_or_else(|| at(d, format!("no track {n}")))?;
                if owner[k].is_some() {
                    return Err(at(d, format!("track {n} already has a drive")));
                }
                owner[k] = Some(drives.len());
                members.push(k);
            }
            let Some(&first) = members.first() else { return Err(at(d, "needs at least one track")) };
            let side = tracks[first].side;
            if members.iter().any(|&k| tracks[k].side != side) {
                return Err(at(d, "all its tracks must be on one side"));
            }
            drives.push(Drive { name: req(d, "name")?.to_string(), side, tracks: members, max_torque: num(d, "max_torque")?, radius: tracks[first].outer_radius() });
        }
        for (k, t) in self.robot.children_named("track").enumerate() {
            match (t.child("drive"), owner[k]) {
                (Some(own), None) => {
                    owner[k] = Some(drives.len());
                    drives.push(Drive { name: names[k].clone(), side: tracks[k].side, tracks: vec![k], max_torque: num(own, "max_torque")?, radius: tracks[k].outer_radius() });
                }
                (Some(_), Some(_)) => return Err(at(t, "has its own <drive> and is in a shared one")),
                (None, None) => return Err(at(t, "no <drive> turns it")),
                (None, Some(_)) => {}
            }
            tracks[k].drive = owner[k].unwrap();
        }
        let mut plates = vec![];
        for p in self.robot.children_named("plate") {
            let n = req(p, "track")?;
            let track = names.iter().position(|x| x == n).ok_or_else(|| at(p, format!("no track {n}")))?;
            let y = numbers(p, "y")?;
            let [y0, y1] = y.as_slice() else { return Err(at(p, "y needs two numbers")) };
            plates.push(Plate { track, y0: y0.min(*y1), y1: y0.max(*y1), inset: num_or(p, "inset", 0.0)? });
        }

        // Contact properties of the chassis: only the root link has them (tracks carry their own).
        let mut chassis = None;
        for s in self.robot.children_named("surface") {
            let l = req(s, "link")?;
            if l != root_name {
                return Err(at(s, format!("link {l}: only the root link ({root_name}) takes a <surface>; a track's goes inside its <track>")));
            }
            if chassis.replace(s).is_some() {
                return Err(at(s, "the root link already has a <surface>"));
            }
        }
        let model = Model {
            name: req(self.robot, "name")?.to_string(),
            base,
            mass,
            joints,
            tracks,
            drives,
            boxes,
            plates,
            chassis_surface: surface(chassis)?,
        };
        model.validate()?;
        Ok(model)
    }

    fn track(&self, t: &Element, root: &str, joint_links: &[&str]) -> Result<Track, String> {
        let name = req(t, "name")?.to_string();
        let side = match req(t, "side")? {
            "left" => Side::Left,
            "right" => Side::Right,
            s => return Err(at(t, format!("side=\"{s}\" must be left or right"))),
        };
        let link_name = req(t, "link")?;
        let joint = if link_name == root {
            None
        } else {
            Some(joint_links.iter().position(|l| *l == link_name).ok_or_else(|| at(t, format!("link {link_name} is not the root or a joint's child")))?)
        };
        let pulleys: Vec<&Element> = t.children_named("pulley").collect();
        let [p0, p1] = pulleys.as_slice() else { return Err(at(t, "needs exactly two <pulley>")) };
        let (r0, r1) = (num(p0, "radius")?, num(p1, "radius")?);
        if (r0 - r1).abs() > 1e-12 {
            return Err(at(t, "pulleys of different radii are not supported"));
        }
        let (a, b) = (vec3(p0, "xyz")?, vec3(p1, "xyz")?);
        let belt = t.child("belt").ok_or_else(|| at(t, "missing <belt>"))?;
        let lugs = match t.child("lugs") {
            None => None,
            Some(l) => Some(lugs(l)?),
        };
        Ok(Track {
            name,
            side,
            joint,
            pulleys: [(a.x, a.z), (b.x, b.z)],
            radius: r0,
            inner: num(belt, "inner")?,
            outer: num(belt, "outer")?,
            belt_mass: num(belt, "mass")?,
            lugs,
            drive: 0,
            surface: surface(t.child("surface"))?,
        })
    }
}

/// Lugs from `<lugs count inner outer spacing="even"><profile points="u h ..."/><ramp .../></lugs>`.
/// The profile is a symmetric trapezoid: two root corners at h = 0, two tip corners above.
/// `<ramp from inner_height>` (optional) slopes the top down towards the inner edge.
fn lugs(l: &Element) -> Result<Lugs, String> {
    if l.attr("spacing").unwrap_or("even") != "even" {
        return Err(at(l, "only spacing=\"even\" is supported"));
    }
    let count = req(l, "count")?.parse::<usize>().map_err(|_| at(l, "count must be a whole number"))?;
    let profile = l.child("profile").ok_or_else(|| at(l, "missing <profile>"))?;
    let p = numbers(profile, "points")?;
    if p.len() != 8 {
        return Err(at(profile, "the profile needs four corners (u h)"));
    }
    let pts: Vec<(f64, f64)> = p.chunks(2).map(|c| (c[0], c[1])).collect();
    let root: Vec<f64> = pts.iter().filter(|q| q.1.abs() < 1e-12).map(|q| q.0).collect();
    let tip: Vec<(f64, f64)> = pts.iter().filter(|q| q.1.abs() >= 1e-12).copied().collect();
    let symmetric = |v: &[f64]| v.len() == 2 && (v[0] + v[1]).abs() < 1e-9;
    let tips: Vec<f64> = tip.iter().map(|q| q.0).collect();
    if !symmetric(&root) || !symmetric(&tips) || (tip[0].1 - tip[1].1).abs() > 1e-12 {
        return Err(at(profile, "the profile must be a trapezoid symmetric about u = 0"));
    }
    let ramp = match l.child("ramp") {
        None => None,
        Some(r) => Some(Ramp { from: num(r, "from")?, inner_height: num(r, "inner_height")? }),
    };
    Ok(Lugs { count, base: (root[0] - root[1]).abs(), tip: (tips[0] - tips[1]).abs(), height: tip[0].1, inner: num(l, "inner")?, outer: num(l, "outer")?, ramp })
}

#[cfg(test)]
mod tests {
    use super::*;

    const CRAWLER: &str = include_str!("../../../robots/simple_crawler/simple_crawler.urdf");

    #[test]
    fn loads_the_simple_crawler() {
        let m = load_str(CRAWLER).unwrap();
        assert_eq!(m.tracks.len(), 6);
        assert_eq!(m.joints.len(), 4);
        assert!((m.mass - 20.0).abs() < 1e-9);
        // The lumped values at flipper angle 0.
        let p = m.mass_properties(&[0.0; 4]);
        let i = p.inertia.0;
        assert!((i[0][0] - 0.2829).abs() < 1e-3 && (i[1][1] - 1.0216).abs() < 1e-3 && (i[2][2] - 1.2305).abs() < 1e-3, "{i:?}");
        assert!(p.com.len() < 1e-9);
        for j in &m.joints {
            assert!((j.inertia - 0.0204).abs() < 1e-4, "{} {}", j.name, j.inertia);
        }
        let main = &m.tracks[0];
        let l = main.lugs.unwrap();
        assert_eq!(l.count, 16);
        assert!((main.outer_radius() - 0.075).abs() < 1e-9);
        assert!((l.inner - 0.04).abs() < 1e-9 && (l.outer - 0.14).abs() < 1e-9);
        assert!(l.ramp.is_none());
        assert_eq!(l.pieces().1, 1);
        assert_eq!(m.tracks[2].lugs.unwrap().count, 10);
        // Plates with the belts' outlines: three on the chassis, none on the flippers.
        assert!(m.boxes.is_empty());
        assert_eq!(m.plates.iter().filter(|p| m.tracks[p.track].joint.is_none()).count(), 3);
        assert_eq!(m.plates.iter().filter(|p| m.tracks[p.track].joint.is_some()).count(), 0);
        // Two motors, each turning the main belt and both flippers of its side.
        assert_eq!(m.drives.len(), 2);
        for d in &m.drives {
            assert_eq!(d.tracks.len(), 3);
            assert!(d.tracks.iter().all(|&k| m.tracks[k].side == d.side));
        }
        assert_eq!(m.chassis_surface.friction, Some(0.3));
    }

    #[test]
    fn centre_of_mass_follows_the_flippers() {
        let m = load_str(CRAWLER).unwrap();
        // Both front flippers up by 90 degrees: each 1 kg moves 0.12 m back and 0.12 m up.
        let p = m.mass_properties(&[1.5707963267948966, 0.0, 1.5707963267948966, 0.0]);
        let shift = 2.0 * 1.0 * 0.12 / 20.0;
        assert!((p.com.x + shift).abs() < 1e-9 && (p.com.z - shift).abs() < 1e-9, "{:?}", p.com);
        // The tensor stays symmetric and gains products of inertia in x-z.
        let i = p.inertia.0;
        assert!((i[0][2] - i[2][0]).abs() < 1e-12 && i[0][2].abs() > 1e-4, "{i:?}");
    }

    #[test]
    fn a_track_needs_one_drive() {
        let none = CRAWLER.replace("left_main left_front_flipper left_rear_flipper", "left_main left_front_flipper");
        assert!(load_str(&none).unwrap_err().contains("no <drive>"));
        let twice = CRAWLER.replace("tracks=\"right_main", "tracks=\"left_main right_main");
        assert!(load_str(&twice).is_err());
    }

    #[test]
    fn flipper_raises_with_positive_angle() {
        let m = load_str(CRAWLER).unwrap();
        let q = [0.5, 0.5, 0.5, 0.5];
        let front = m.track_pulleys(2, &q);
        let rear = m.track_pulleys(3, &q);
        assert!(front[1].0 > 0.25 && front[1].1 > 0.1, "{front:?}");
        assert!(rear[1].0 < -0.25 && rear[1].1 > 0.1, "{rear:?}");
    }

    #[test]
    fn refuses_what_it_cannot_do() {
        let bad = CRAWLER.replace("type=\"revolute\"", "type=\"prismatic\"");
        assert!(load_str(&bad).unwrap_err().contains("revolute"));
    }

    /// Opening tags that appear once in `CRAWLER`, to insert elements after.
    const LEFT_MAIN: &str = "<track name=\"left_main\" side=\"left\" link=\"base_link\">";
    const FRONT_LEFT: &str = "<joint name=\"front_left\" type=\"revolute\">";

    /// Loads `CRAWLER` with `from` replaced by `to` (`from` must appear once) and returns the error.
    fn err(from: &str, to: &str) -> String {
        assert_eq!(CRAWLER.matches(from).count(), 1, "{from} should appear once");
        load_str(&CRAWLER.replacen(from, to, 1)).expect_err(&format!("{to} should be refused"))
    }

    #[test]
    fn refuses_unknown_elements_and_attributes() {
        // A misspelt element, at the top and inside a track.
        let e = err("<plate track=\"left_main\" y=\"-0.04 0.04\"", "<plates track=\"left_main\" y=\"-0.04 0.04\"");
        assert!(e.contains("<plates>") && e.contains("unknown element"), "{e}");
        let e = err(LEFT_MAIN, &format!("{LEFT_MAIN}<belts inner=\"0.04\" outer=\"0.14\" mass=\"1.0\"/>"));
        assert!(e.contains("<belts>"), "{e}");
        // A misspelt attribute.
        let e = err("<servo joint=\"front_left\" gain=\"4\"/>", "<servo joint=\"front_left\" gian=\"4\"/>");
        assert!(e.contains("unknown attribute gian"), "{e}");
        // Physics we do not compute is refused, not dropped.
        let e = err(FRONT_LEFT, &format!("{FRONT_LEFT}<dynamics damping=\"0.1\"/>"));
        assert!(e.contains("<dynamics>"), "{e}");
        // The line is given.
        assert!(e.starts_with("line "), "{e}");
    }

    #[test]
    fn refuses_repeated_elements() {
        let e = err(LEFT_MAIN, &format!("{LEFT_MAIN}<belt inner=\"0.04\" outer=\"0.14\" mass=\"1.0\"/>"));
        assert!(e.contains("exactly one <belt>"), "{e}");
        let e = err("<servo joint=\"rear_right\" gain=\"4\"/>", "<servo joint=\"rear_right\" gain=\"4\"/><servo joint=\"rear_right\" gain=\"5\"/>");
        assert!(e.contains("already has a <servo>"), "{e}");
    }

    #[test]
    fn surface_only_on_the_chassis() {
        let e = err("<surface link=\"base_link\" friction=\"0.3\"/>", "<surface link=\"flipper_front_left\" friction=\"0.3\"/>");
        assert!(e.contains("only the root link"), "{e}");
        let e = err("<surface link=\"base_link\" friction=\"0.3\"/>", "<surface link=\"base_link\" friction=\"0.3\"/><surface link=\"base_link\" friction=\"0.5\"/>");
        assert!(e.contains("already has a <surface>"), "{e}");
    }

    #[test]
    fn skips_what_only_other_tools_use() {
        let with_extras = CRAWLER
            .replacen("<link name=\"base_link\">", "<link name=\"base_link\">\n    <visual><geometry><mesh filename=\"chassis.stl\"/></geometry></visual>", 1)
            .replacen("<robot name=\"simple_crawler\">", "<robot name=\"simple_crawler\">\n  <material name=\"grey\"><color rgba=\"0.5 0.5 0.5 1\"/></material>\n  <gazebo/>", 1);
        let m = load_str(&with_extras).unwrap();
        assert_eq!(m, load_str(CRAWLER).unwrap());
    }
}
