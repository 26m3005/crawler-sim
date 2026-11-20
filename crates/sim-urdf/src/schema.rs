//! Which elements and attributes a robot file may hold, checked before anything is built.
//!
//! A misspelt or unsupported element or attribute is an error with its line, never skipped, so
//! a typo cannot quietly change the robot. Standard URDF elements that only other tools use
//! (`<visual>`, `<material>`, `<gazebo>`, `<transmission>`) are skipped whole, so URDF exported
//! from CAD still loads. Whether required attributes are present is checked when building.

use crate::at;
use crate::xml::Element;

/// How many of a child element may appear.
#[derive(Clone, Copy)]
enum Count {
    One,
    Optional,
    Any,
}
use Count::{Any, One, Optional};

/// Elements of standard URDF that only other tools use, skipped with all they hold.
const IGNORED_IN_ROBOT: &[&str] = &["material", "gazebo", "transmission"];
const IGNORED_IN_LINK: &[&str] = &["visual"];

pub fn check(robot: &Element) -> Result<(), String> {
    allow(
        robot,
        &["name"],
        &[("link", Any), ("joint", Any), ("track", Any), ("drive", Any), ("plate", Any), ("servo", Any), ("surface", Any)],
        IGNORED_IN_ROBOT,
    )?;
    for e in &robot.children {
        match e.name.as_str() {
            "link" => link(e)?,
            "joint" => joint(e)?,
            "track" => track(e)?,
            "drive" => leaf(e, &["name", "max_torque", "tracks"])?,
            "plate" => leaf(e, &["track", "y", "inset"])?,
            "servo" => leaf(e, &["joint", "gain"])?,
            "surface" => leaf(e, &["link", "friction", "stiffness", "damping"])?,
            _ => {} // ignored
        }
    }
    Ok(())
}

fn link(e: &Element) -> Result<(), String> {
    allow(e, &["name"], &[("inertial", Optional), ("collision", Any)], IGNORED_IN_LINK)?;
    for c in &e.children {
        match c.name.as_str() {
            "inertial" => {
                allow(c, &[], &[("origin", Optional), ("mass", One), ("inertia", One)], &[])?;
                children(c, |g| match g.name.as_str() {
                    "origin" => origin(g),
                    "mass" => leaf(g, &["value"]),
                    _ => leaf(g, &["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]),
                })?;
            }
            "collision" => {
                allow(c, &["name"], &[("origin", Optional), ("geometry", One)], &[])?;
                children(c, |g| match g.name.as_str() {
                    "origin" => origin(g),
                    _ => {
                        allow(g, &[], &[("box", One)], &[])?;
                        children(g, |b| leaf(b, &["size"]))
                    }
                })?;
            }
            _ => {} // ignored
        }
    }
    Ok(())
}

fn joint(e: &Element) -> Result<(), String> {
    allow(e, &["name", "type"], &[("parent", One), ("child", One), ("origin", Optional), ("axis", One), ("limit", One)], &[])?;
    children(e, |c| match c.name.as_str() {
        "parent" | "child" => leaf(c, &["link"]),
        "origin" => origin(c),
        "axis" => leaf(c, &["xyz"]),
        _ => leaf(c, &["lower", "upper", "velocity", "effort"]),
    })
}

fn track(e: &Element) -> Result<(), String> {
    allow(
        e,
        &["name", "side", "link"],
        &[("pulley", Any), ("belt", One), ("lugs", Optional), ("drive", Optional), ("surface", Optional)],
        &[],
    )?;
    children(e, |c| match c.name.as_str() {
        "pulley" => leaf(c, &["xyz", "radius"]),
        "belt" => leaf(c, &["inner", "outer", "mass"]),
        "drive" => leaf(c, &["max_torque"]),
        "surface" => leaf(c, &["friction", "stiffness", "damping"]),
        _ => {
            allow(c, &["count", "spacing", "inner", "outer"], &[("profile", One), ("ramp", Optional)], &[])?;
            children(c, |p| match p.name.as_str() {
                "profile" => leaf(p, &["points"]),
                _ => leaf(p, &["from", "inner_height"]),
            })
        }
    })
}

fn origin(e: &Element) -> Result<(), String> {
    leaf(e, &["xyz", "rpy"])
}

/// An element with attributes from `attrs` and no children.
fn leaf(e: &Element, attrs: &[&str]) -> Result<(), String> {
    allow(e, attrs, &[], &[])
}

/// Runs `f` on each child that `allow` let through (skipped elements are not visited).
fn children(e: &Element, f: impl Fn(&Element) -> Result<(), String>) -> Result<(), String> {
    e.children.iter().try_for_each(f)
}

/// Every attribute of `e` is in `attrs`; every child is in `kids` (at most as often as its count
/// says, and `One` exactly once) or in `ignored`.
fn allow(e: &Element, attrs: &[&str], kids: &[(&str, Count)], ignored: &[&str]) -> Result<(), String> {
    for (k, _) in &e.attrs {
        if !attrs.contains(&k.as_str()) {
            return Err(at(e, format!("unknown attribute {k}{}", expected(attrs))));
        }
    }
    for c in &e.children {
        if !ignored.contains(&c.name.as_str()) && !kids.iter().any(|(n, _)| *n == c.name) {
            let names: Vec<&str> = kids.iter().map(|(n, _)| *n).collect();
            return Err(at(c, format!("unknown element inside <{}>{}", e.name, expected(&names))));
        }
    }
    for (name, count) in kids {
        let n = e.children_named(name).count();
        let bad = match count {
            One => n != 1,
            Optional => n > 1,
            Any => false,
        };
        if bad {
            let want = if matches!(count, One) { "exactly one" } else { "at most one" };
            return Err(at(e, format!("needs {want} <{name}>, found {n}")));
        }
    }
    Ok(())
}

fn expected(names: &[&str]) -> String {
    if names.is_empty() {
        " (none allowed here)".into()
    } else {
        format!(" (expected one of: {})", names.join(", "))
    }
}
