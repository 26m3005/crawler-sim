//! What a viewer needs to draw a robot, as JSON: bodies, parts fixed to them, and each body's
//! pose per frame. The viewer knows no robot; crawlers are described this way.
//!
//! Scene: `{"bodies": n, "follow": body the camera follows, "parts": [...], "terrain": {...} or null}`. Parts, in their body's frame:
//!
//! | `k` | fields | drawn as |
//! |---|---|---|
//! | `box` | `min`, `max` | an axis-aligned box |
//! | `belt` | `a`, `bb` (pulley centres x, z), `r`, `y0`, `y1` | a belt round two pulleys, across y |
//! | `plate` | same as `belt` | a solid plate with that outline, across y |
//! | `prism` | `pts` (corners u, h), `y0`, `y1` | a cross-section in x-z, across y (a lug) |
//! | `hull8` | `v` (8 corners x, y, z: 4 at one end across y, then the same 4 at the other) | a convex solid with six four-cornered faces (a lug piece whose top slopes) |
//!
//! Every part has a body `b` and a colour `c`. A frame is `{"t": time, "b": [[x, y, z, r00 .. r22], ...]}`.
//! A recording is `{"scene": ..., "frames": [...]}`.

use sim_core::math::{v3, Mat3, V3};
use sim_core::model::Model;
use sim_core::path::BeltPath;
use sim_core::state::State;
use sim_core::terrain::HeightGrid;

/// The look: a white robot with light grey belts and nearly white lugs.
pub const WHITE: &str = "#f1f2f4";
pub const BELT: &str = "#c3c7ce";
pub const FLIPPER_BELT: &str = "#d0d4da";
pub const LUG: &str = "#e6e8ec";

fn v(p: V3) -> String {
    format!("[{},{},{}]", p.x, p.y, p.z)
}

fn pose(p: V3, r: &Mat3) -> String {
    let m = &r.0;
    format!("[{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}]", p.x, p.y, p.z, m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2])
}

/// The terrain as the scene carries it: heights at the cells' centres and, when any top
/// slopes, `grades` (dz/dx, dz/dy of each cell, in the order of the heights).
pub fn terrain(grid: &HeightGrid) -> String {
    let (ox, cell, nx, ny, end) = grid.layout();
    let h: Vec<String> = grid.heights().iter().map(|h| format!("{h:.4}")).collect();
    let grades = if grid.grades().is_empty() {
        String::new()
    } else {
        let g: Vec<String> = grid.grades().iter().map(|(gx, gy)| format!("{gx:.6},{gy:.6}")).collect();
        format!(",\"grades\":[{}]", g.join(","))
    };
    format!("{{\"origin_x\":{ox},\"cell\":{cell},\"nx\":{nx},\"ny\":{ny},\"end\":{end},\"heights\":[{}]{grades}}}", h.join(","))
}

/// A scene from bodies, parts and optional terrain; the camera follows body 0.
pub fn scene(bodies: usize, parts: &[String], terrain: Option<String>) -> String {
    format!("{{\"bodies\":{bodies},\"follow\":0,\"parts\":[{}],\"terrain\":{}}}", parts.join(","), terrain.unwrap_or_else(|| "null".into()))
}

/// A frame from body poses.
pub fn frame(time: f64, bodies: &[String]) -> String {
    format!("{{\"t\":{time:.3},\"b\":[{}]}}", bodies.join(","))
}

/// A whole recording.
pub fn recording(scene: &str, frames: &[String]) -> String {
    format!("{{\"scene\":{scene},\"frames\":[{}]}}\n", frames.join(",\n"))
}

// ---- Crawlers: body 0 is the chassis, then one body per joint (flipper), then one per lug.

fn lug_count(m: &Model) -> usize {
    m.tracks.iter().map(|t| t.lugs.map_or(0, |l| l.count)).sum()
}

pub fn crawler_scene(m: &Model, grid: &HeightGrid) -> String {
    let mut parts = vec![];
    for b in &m.boxes {
        let body = b.joint.map_or(0, |j| j + 1);
        parts.push(format!("{{\"k\":\"box\",\"b\":{body},\"min\":{},\"max\":{},\"c\":\"{WHITE}\"}}", v(b.min), v(b.max)));
    }
    for p in &m.plates {
        let t = &m.tracks[p.track];
        let ([a, b], body) = (t.pulleys, t.joint.map_or(0, |j| j + 1));
        parts.push(format!("{{\"k\":\"plate\",\"b\":{body},\"a\":[{},{}],\"bb\":[{},{}],\"r\":{},\"y0\":{},\"y1\":{},\"c\":\"{WHITE}\"}}", a.0, a.1, b.0, b.1, t.radius - p.inset, p.y0, p.y1));
    }
    let mut lug_body = 1 + m.joints.len();
    for t in &m.tracks {
        let s = t.side.sign();
        let (y0, y1) = ((t.inner * s).min(t.outer * s), (t.inner * s).max(t.outer * s));
        let body = t.joint.map_or(0, |j| j + 1);
        let [a, b] = t.pulleys;
        let colour = if t.joint.is_some() { FLIPPER_BELT } else { BELT };
        parts.push(format!("{{\"k\":\"belt\",\"b\":{body},\"a\":[{},{}],\"bb\":[{},{}],\"r\":{},\"y0\":{y0},\"y1\":{y1},\"c\":\"{colour}\"}}", a.0, a.1, b.0, b.1, t.radius));
        if let Some(l) = t.lugs {
            let (pieces, n) = l.pieces();
            for _ in 0..l.count {
                for &(d0, d1, h0, h1) in &pieces[..n] {
                    let (y0, y1) = (d0 * s, d1 * s);
                    if h0 == h1 {
                        parts.push(format!(
                            "{{\"k\":\"prism\",\"b\":{lug_body},\"pts\":[[{},0],[{},0],[{},{}],[{},{}]],\"y0\":{},\"y1\":{},\"c\":\"{LUG}\"}}",
                            -l.base / 2.0, l.base / 2.0, l.tip / 2.0, h0, -l.tip / 2.0, h0, y0.min(y1), y0.max(y1)
                        ));
                    } else {
                        let (b, tp) = (l.base / 2.0, l.tip / 2.0);
                        let v = [(-b, y0, 0.0), (b, y0, 0.0), (tp, y0, h0), (-tp, y0, h0), (-b, y1, 0.0), (b, y1, 0.0), (tp, y1, h1), (-tp, y1, h1)];
                        let v: Vec<String> = v.iter().map(|q| format!("[{},{},{}]", q.0, q.1, q.2)).collect();
                        parts.push(format!("{{\"k\":\"hull8\",\"b\":{lug_body},\"v\":[{}],\"c\":\"{LUG}\"}}", v.join(",")));
                    }
                }
                lug_body += 1;
            }
        }
    }
    scene(1 + m.joints.len() + lug_count(m), &parts, Some(terrain(grid)))
}

/// Poses of the chassis, the flippers and every lug, as JSON.
pub fn crawler_bodies(m: &Model, s: &State) -> Vec<String> {
    crawler_poses(m, s).iter().map(|(p, r)| pose(*p, r)).collect()
}

/// Poses (position, rotation) of the chassis, the flippers and every lug, in body order.
pub fn crawler_poses(m: &Model, s: &State) -> Vec<(V3, Mat3)> {
    let rc = s.pose.rot.to_mat3();
    let mut out = vec![(s.pose.to_world(V3::ZERO), rc)];
    for (j, joint) in m.joints.iter().enumerate() {
        out.push((s.pose.to_world(joint.origin), rc.mul(&Mat3::rotation(joint.axis, s.joint_q[j]))));
    }
    for (k, t) in m.tracks.iter().enumerate() {
        let Some(l) = t.lugs else { continue };
        let [a, b] = m.track_pulleys(k, &s.joint_q);
        let path = BeltPath { a, b, r: t.radius };
        let pitch = path.len() / l.count as f64;
        for i in 0..l.count {
            let (p, o) = path.at(i as f64 * pitch - s.belt_q[k]);
            // Columns: along the belt, across, outward.
            let r = Mat3([[o.1, 0.0, o.0], [0.0, 1.0, 0.0], [-o.0, 0.0, o.1]]);
            out.push((s.pose.to_world(v3(p.0, 0.0, p.1)), rc.mul(&r)));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crawler_bodies_match_the_scene() {
        let m = sim_urdf::load_str(include_str!("../../../robots/simple_crawler/simple_crawler.urdf")).unwrap();
        let grid = HeightGrid::flat();
        let scene = crawler_scene(&m, &grid);
        let n = 1 + 4 + 2 * 16 + 4 * 10;
        assert!(scene.starts_with(&format!("{{\"bodies\":{n},")), "{}", &scene[..40]);
        let s = State::at_rest(&m, v3(0.0, 0.0, 0.1), vec![0.3; 4]);
        assert_eq!(crawler_bodies(&m, &s).len(), n);
        // The prism and belt parts all name a body that exists.
        assert_eq!(scene.matches("\"k\":\"prism\"").count(), 2 * 16 + 4 * 10);
    }

}
