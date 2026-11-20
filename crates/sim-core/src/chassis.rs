//! The rigid bodies: boxes, and plates with a belt's outline, on the chassis and on the flipper
//! links. They fill what the belts do not wrap and carry no belt speed.

use crate::collide::{solid_hits, Solid};
use crate::contact::{Contact, ContactGenerator, Frame};
use crate::math::{v3, V3};
use crate::path::BeltPath;
use crate::terrain::Terrain;

/// Spacing of the sample points round a plate's outline [m].
const SAMPLE: f64 = 0.02;

#[derive(Default)]
pub struct ChassisBoxes {
    hits: Vec<(V3, V3, f64, bool)>,
}

impl ChassisBoxes {
    pub fn new() -> ChassisBoxes {
        ChassisBoxes::default()
    }
}

impl<T: Terrain> ContactGenerator<T> for ChassisBoxes {
    fn collect(&mut self, f: &Frame<'_>, terrain: &T, out: &mut Vec<Contact>) {
        let (m, rot) = (f.model, f.pose.rot);
        for b in &m.boxes {
            let (lo, hi) = (b.min, b.max);
            // Link frame to chassis frame: the joint's turn, or nothing for the chassis.
            let link = |p: V3| b.joint.map_or(p, |j| m.link_point(j, f.joint_q[j], p));
            let axis = |p: V3| rot.rot(b.joint.map_or(p, |j| m.link_rot(j, f.joint_q[j]).mul_v(p)));
            let c = |x: f64, y: f64, z: f64| f.pose.to_world(link(v3(x, y, z)));
            let verts = [c(lo.x, lo.y, hi.z), c(hi.x, lo.y, hi.z), c(hi.x, lo.y, lo.z), c(lo.x, lo.y, lo.z), c(lo.x, hi.y, hi.z), c(hi.x, hi.y, hi.z), c(hi.x, hi.y, lo.z), c(lo.x, hi.y, lo.z)];
            let ax = axis(v3(1.0, 0.0, 0.0));
            let solid = Solid { verts, side_n: [ax, -ax], tip_n: axis(v3(0.0, 0.0, -1.0)), width_n: axis(v3(0.0, 1.0, 0.0)) };
            self.hits.clear();
            solid_hits(&solid, terrain, f.skin, false, &mut self.hits);
            for &(point, normal, depth, _) in &self.hits {
                out.push(Contact { point, normal, depth, track: None, joint: b.joint, belt_dir: V3::ZERO, caught: false });
            }
        }
        // Plates: points round the outline in the terrain, and block edges inside the outline.
        // Only plates whose bounding box reaches down to a block under it are looked at.
        let outline = |k: usize, inset: f64| {
            let p = f.paths[k].belt;
            BeltPath { r: p.r - inset, ..p }
        };
        for p in &m.plates {
            let (path, joint) = (outline(p.track, p.inset), m.tracks[p.track].joint);
            let Some((lo, hi)) = near_terrain(f, terrain, &path, p.y0, p.y1) else { continue };
            let per = path.len();
            let n = (per / SAMPLE).round() as usize;
            for i in 0..n {
                let (q, _) = path.at(per * i as f64 / n as f64);
                for y in [p.y0, (p.y0 + p.y1) / 2.0, p.y1] {
                    let w = f.pose.to_world(v3(q.0, y, q.1));
                    if let Some((depth, normal)) = terrain.inside(w, f.skin) {
                        out.push(Contact { point: w, normal, depth, track: None, joint, belt_dir: V3::ZERO, caught: false });
                    }
                }
            }
            // Block edges inside the box (they are sorted by x), then inside the outline.
            let all = terrain.edges();
            let (from, to) = (all.partition_point(|e| e.x < lo.x), all.partition_point(|e| e.x <= hi.x));
            for e in &all[from..to] {
                if e.y < lo.y || e.y > hi.y || e.z < lo.z || e.z > hi.z {
                    continue;
                }
                let l = f.pose.to_body(*e);
                if l.y < p.y0 || l.y > p.y1 {
                    continue;
                }
                let (d, _, o) = path.locate((l.x, l.z));
                if d < path.r {
                    out.push(Contact { point: *e, normal: -rot.rot(v3(o.0, 0.0, o.1)), depth: path.r - d, track: None, joint, belt_dir: V3::ZERO, caught: false });
                }
            }
        }
    }
}

/// The plate's bounding box in the world, if it reaches down to the top of a block under it;
/// `None` when the plate is above every block there and cannot touch.
fn near_terrain<T: Terrain>(f: &Frame<'_>, terrain: &T, path: &BeltPath, y0: f64, y1: f64) -> Option<(V3, V3)> {
    let (mut lo, mut hi) = (v3(f64::MAX, f64::MAX, f64::MAX), v3(f64::MIN, f64::MIN, f64::MIN));
    for (cx, cz) in [path.a, path.b] {
        for (dx, dz) in [(-path.r, -path.r), (-path.r, path.r), (path.r, -path.r), (path.r, path.r)] {
            for y in [y0, y1] {
                let w = f.pose.to_world(v3(cx + dx, y, cz + dz));
                lo = v3(lo.x.min(w.x), lo.y.min(w.y), lo.z.min(w.z));
                hi = v3(hi.x.max(w.x), hi.y.max(w.y), hi.z.max(w.z));
            }
        }
    }
    let ((i0, j0), (i1, j1)) = (terrain.cell_of(lo.x, lo.y), terrain.cell_of(hi.x, hi.y));
    let mut top = f64::MIN;
    for i in i0..=i1 {
        for j in j0..=j1 {
            top = top.max(terrain.top_max(i, j));
        }
    }
    (lo.z <= top + f.skin).then_some((lo, hi))
}
