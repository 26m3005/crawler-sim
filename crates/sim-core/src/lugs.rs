//! The tracks as rigid lugs carried round each belt path, with the belt surface between them.
//!
//! Each lug is a convex solid (its cross-section from the robot file, across its width; two solids
//! when its top slopes towards the inner edge) at the position the belt travel puts it. It meets the block columns by separating axes, so a block
//! edge can press on a lug's side wall: the catch that lifts the robot onto a step.

use crate::collide::{solid_hits, Solid};
use crate::contact::{edges_near, reach, Contact, ContactGenerator, Frame};
use crate::math::{v3, V3};
use crate::path::travel;
use crate::terrain::Terrain;

/// Spacing of the belt-surface sample points between the lugs [m].
const BELT_SAMPLE: f64 = 0.04;

#[derive(Default)]
pub struct MovingLugs {
    hits: Vec<(V3, V3, f64, bool)>,
    near: Vec<V3>,
}

impl MovingLugs {
    pub fn new() -> MovingLugs {
        MovingLugs::default()
    }
}

impl<T: Terrain> ContactGenerator<T> for MovingLugs {
    fn collect(&mut self, f: &Frame<'_>, terrain: &T, out: &mut Vec<Contact>) {
        let rot = f.pose.rot;
        for (k, track) in f.model.tracks.iter().enumerate() {
            let side = track.side.sign();
            let path = f.paths[k].belt;
            if let Some(lugs) = track.lugs {
                let pitch = path.len() / lugs.count as f64;
                let (pieces, n_pieces) = lugs.pieces();
                for i in 0..lugs.count {
                    let (p, o) = path.at(i as f64 * pitch - f.belt_q[k]);
                    let t2 = (o.1, -o.0);
                    let (tx, ty, tz) = travel(o);
                    let u = rot.rot(v3(tx, ty, tz));
                    // The pieces share the corners where the top starts to slope: a point is
                    // one rubber contact, so a corner already found for this lug is not added again.
                    let first = out.len();
                    for &(d0, d1, h0, h1) in &pieces[..n_pieces] {
                        let (y0, y1) = (d0 * side, d1 * side);
                        let local = |e: f64, tip: bool, y: f64, h: f64| {
                            let half = if tip { lugs.tip / 2.0 } else { lugs.base / 2.0 };
                            let r = if tip { h } else { 0.0 };
                            v3(p.0 + o.0 * r + t2.0 * half * e, y, p.1 + o.1 * r + t2.1 * half * e)
                        };
                        let body = [local(-1.0, false, y0, h0), local(1.0, false, y0, h0), local(1.0, true, y0, h0), local(-1.0, true, y0, h0), local(-1.0, false, y1, h1), local(1.0, false, y1, h1), local(1.0, true, y1, h1), local(-1.0, true, y1, h1)];
                        let mut verts = [V3::ZERO; 8];
                        for (w, v) in verts.iter_mut().zip(body.iter()) {
                            *w = f.pose.to_world(*v);
                        }
                        // Side walls: straight up when the profile is a rectangle (always so with a ramp).
                        let slope = (lugs.base - lugs.tip) / 2.0;
                        let h = lugs.height;
                        let wl = (h * h + slope * slope).sqrt();
                        let rot_v = |v: (f64, f64)| rot.rot(v3(v.0, 0.0, v.1));
                        // Tip face: outward, tilted across the width where the height changes.
                        let tip_n = if h0 == h1 {
                            rot_v(o)
                        } else {
                            let s = (h1 - h0) / (y1 - y0);
                            let l = (1.0 + s * s).sqrt();
                            rot.rot(v3(o.0 / l, -s / l, o.1 / l))
                        };
                        let solid = Solid {
                            verts,
                            side_n: [rot_v(((t2.0 * h + o.0 * slope) / wl, (t2.1 * h + o.1 * slope) / wl)), rot_v(((-t2.0 * h + o.0 * slope) / wl, (-t2.1 * h + o.1 * slope) / wl))],
                            tip_n,
                            width_n: rot.rot(v3(0.0, 1.0, 0.0)),
                        };
                        self.hits.clear();
                        solid_hits(&solid, terrain, f.skin, true, &mut self.hits);
                        for &(point, normal, depth, _) in &self.hits {
                            if n_pieces > 1 && out[first..].iter().any(|c| (c.point - point).len() < 1e-9) {
                                continue;
                            }
                            out.push(Contact { point, normal, depth, track: Some(k), joint: None, belt_dir: u, caught: normal.dot(u).abs() > 0.5 });
                        }
                    }
                }
            }
            // The belt surface between the lugs, along both edges of the belt.
            let per = path.len();
            let n = (per / BELT_SAMPLE).round() as usize;
            for i in 0..n {
                let (p, o) = path.at(per * i as f64 / n as f64);
                for y in [track.inner * side, track.outer * side] {
                    let world = f.pose.to_world(v3(p.0, y, p.1));
                    if let Some((depth, normal)) = terrain.inside(world, f.skin) {
                        let (tx, ty, tz) = travel(o);
                        out.push(Contact { point: world, normal, depth, track: Some(k), joint: None, belt_dir: rot.rot(v3(tx, ty, tz)), caught: false });
                    }
                }
            }
        }
        // Block edges against the belt surface.
        edges_near(terrain, f.pose.com, reach(f), &mut self.near);
        for e in &self.near {
            let l = f.pose.to_body(*e);
            for (k, track) in f.model.tracks.iter().enumerate() {
                let y = l.y * track.side.sign();
                if y < track.inner || y > track.outer {
                    continue;
                }
                let path = f.paths[k].belt;
                let (d, _, o) = path.locate((l.x, l.z));
                if d < path.r {
                    let (tx, ty, tz) = travel(o);
                    out.push(Contact { point: *e, normal: -rot.rot(v3(o.0, 0.0, o.1)), depth: path.r - d, track: Some(k), joint: None, belt_dir: rot.rot(v3(tx, ty, tz)), caught: false });
                }
            }
        }
    }
}
