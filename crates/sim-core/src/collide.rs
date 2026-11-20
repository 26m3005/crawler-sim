//! A convex solid of eight corners against block columns, by separating axes.

use crate::math::{v3, V3};
use crate::terrain::Terrain;

/// The faces of a convex solid with eight corners, besides the four corners' positions.
///
/// `verts` are world positions: root corners then tip corners at one end of the width
/// (0..4: root -, root +, tip +, tip -), then the same at the other end (4..8).
#[derive(Clone, Copy)]
pub struct Solid {
    pub verts: [V3; 8],
    /// Outward normals of the two side walls (along the belt).
    pub side_n: [V3; 2],
    /// Outward normal of the tip face.
    pub tip_n: V3,
    /// Normal of the end faces across the width.
    pub width_n: V3,
}

/// The way out of one block: direction to push the solid, the depth, and whether it is a side wall.
pub struct Hit {
    pub n: V3,
    pub depth: f64,
    pub wall: bool,
}

pub fn project(p: &[V3], a: V3) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for v in p {
        let d = v.dot(a);
        lo = lo.min(d);
        hi = hi.max(d);
    }
    (lo, hi)
}

/// The eight corners of the column (i, j): its bottom, one metre below the lowest corner of its
/// top, then its top on the top's plane.
pub fn block_corners(terrain: &impl Terrain, i: i32, j: i32) -> [V3; 8] {
    let (bx, by) = terrain.corner(i, j);
    let c = terrain.cell();
    let z = |x: f64, y: f64| terrain.height_in(i, j, x, y);
    let (z0, z1, z2, z3) = (z(bx, by), z(bx + c, by), z(bx + c, by + c), z(bx, by + c));
    let bottom = z0.min(z1).min(z2).min(z3) - 1.0;
    [v3(bx, by, bottom), v3(bx + c, by, bottom), v3(bx + c, by + c, bottom), v3(bx, by + c, bottom), v3(bx, by, z0), v3(bx + c, by, z1), v3(bx + c, by + c, z2), v3(bx, by + c, z3)]
}

/// A column as the separating axes see it: its corners, which side faces (+x, -x, +y, -y)
/// stand above their neighbour, and its top's normal.
pub struct Block {
    pub corners: [V3; 8],
    pub open: [bool; 4],
    pub top_n: V3,
}

impl Block {
    pub fn of(terrain: &impl Terrain, i: i32, j: i32) -> Block {
        Block { corners: block_corners(terrain, i, j), open: terrain.open_faces(i, j), top_n: terrain.top_normal(i, j) }
    }
}

/// The shallowest way to push `s` out of `block`, or `None` if they do not touch.
///
/// The block's top and the solid's tip come first: a contact shallower than `skin` on either
/// stays there, even near an edge, so a graze cannot turn into a push against the belt.
pub fn solid_vs_block(s: &Solid, b: &Block, skin: f64) -> Option<Hit> {
    let (block, open) = (&b.corners, b.open);
    let verts = &s.verts;
    let depth = |a: V3| {
        let (lug_lo, _) = project(verts, a);
        let (_, block_hi) = project(block, a);
        block_hi - lug_lo
    };
    let mut best: Option<Hit> = None;
    let mut consider = |a: V3, wall: bool, margin: f64| -> bool {
        let d = depth(a);
        if d <= 0.0 {
            return false;
        }
        if best.as_ref().map_or(true, |b| d + margin < b.depth) {
            best = Some(Hit { n: a, depth: d, wall });
        }
        true
    };
    if !consider(b.top_n, false, 0.0) || !consider(-s.tip_n, false, 0.0) {
        return None;
    }
    let axes = [v3(1.0, 0.0, 0.0), v3(-1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, -1.0, 0.0)];
    for (a, is_open) in axes.iter().zip(open) {
        // A face against a taller neighbour is no way out, but it still separates.
        if !is_open {
            if depth(*a) <= 0.0 {
                return None;
            }
            continue;
        }
        if !consider(*a, false, skin) {
            return None;
        }
    }
    for a in s.side_n {
        if !consider(-a, true, skin) {
            return None;
        }
    }
    for a in [s.width_n, -s.width_n] {
        if !consider(a, false, skin) {
            return None;
        }
    }
    // Edge against edge: a solid edge across a block edge, pointing away from the block's centre.
    // A tip edge across the width counts too when the top slopes (otherwise it is parallel to the
    // root edges across the width, whose axes the faces already cover).
    let (across_tip, across_root) = (verts[6] - verts[2], verts[4] - verts[0]);
    let sloped = across_tip.cross(across_root).len() > 1e-9 * across_tip.len() * across_root.len();
    let edges = [verts[1] - verts[0], verts[2] - verts[0], verts[3] - verts[1], across_tip];
    let centre = (block[0] + block[6]) * 0.5;
    let solid_c = (verts[0] + verts[7]) * 0.5;
    // The top's edges along x and y, tilted with a sloping top.
    let n = b.top_n;
    let top_edges = if n.z == 1.0 { [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0)] } else { [v3(n.z, 0.0, -n.x), v3(0.0, n.z, -n.y)] };
    for e in &edges[..if sloped { 4 } else { 3 }] {
        let e = *e;
        for b in top_edges {
            let c = e.cross(b);
            let l = c.len();
            if l < 1e-6 {
                continue;
            }
            let a = c * (1.0 / l);
            let a = if (solid_c - centre).dot(a) >= 0.0 { a } else { -a };
            if !consider(a, false, skin * 2.0) {
                return None;
            }
        }
    }
    best
}

/// Axis-aligned bounds of the corners: (x min, x max, y min, y max, z min).
pub fn bounds(verts: &[V3; 8]) -> (f64, f64, f64, f64, f64) {
    let (mut x0, mut x1, mut y0, mut y1, mut z0) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN, f64::MAX);
    for v in verts {
        x0 = x0.min(v.x);
        x1 = x1.max(v.x);
        y0 = y0.min(v.y);
        y1 = y1.max(v.y);
        z0 = z0.min(v.z);
    }
    (x0, x1, y0, y1, z0)
}

/// Contact points of a solid against every column under it: (point, push direction, depth, wall).
///
/// With `block_corners_in` the corners of each column's top that lie inside the solid are
/// added too, so a block edge pressing into a lug's side wall gives a point at the edge.
pub fn solid_hits(s: &Solid, terrain: &impl Terrain, skin: f64, block_corners_in: bool, out: &mut Vec<(V3, V3, f64, bool)>) {
    let (x0, x1, y0, y1, z0) = bounds(&s.verts);
    let (ci0, cj0) = terrain.cell_of(x0, y0);
    let (ci1, cj1) = terrain.cell_of(x1, y1);
    let c = terrain.cell();
    for ci in ci0..=ci1 {
        for cj in cj0..=cj1 {
            if z0 >= terrain.top_max(ci, cj) {
                continue;
            }
            let b = Block::of(terrain, ci, cj);
            let Some(hit) = solid_vs_block(s, &b, skin) else { continue };
            let block = b.corners;
            let (bx, by) = terrain.corner(ci, cj);
            let (_, block_hi) = project(&block, hit.n);
            let mut any = false;
            for v in s.verts.iter() {
                let pen = block_hi - v.dot(hit.n);
                if pen > 0.0 && v.x >= bx && v.x <= bx + c && v.y >= by && v.y <= by + c && v.z <= terrain.height_in(ci, cj, v.x, v.y) {
                    out.push((*v, hit.n, pen.min(hit.depth), hit.wall));
                    any = true;
                }
            }
            if block_corners_in {
                let (solid_lo, _) = project(&s.verts, hit.n);
                let (wa, wb) = (s.verts[0].dot(s.width_n), s.verts[4].dot(s.width_n));
                for v in block[4..8].iter() {
                    let wv = v.dot(s.width_n);
                    let within = (*v - s.verts[2]).dot(s.tip_n) <= 0.0 && (*v - s.verts[1]).dot(s.side_n[0]) <= 0.0 && (*v - s.verts[0]).dot(s.side_n[1]) <= 0.0 && wv >= wa.min(wb) && wv <= wa.max(wb);
                    if within {
                        let pen = v.dot(hit.n) - solid_lo;
                        if pen > 0.0 {
                            out.push((*v, hit.n, pen.min(hit.depth), hit.wall));
                            any = true;
                        }
                    }
                }
            }
            if !any {
                let (qx, qy) = ((x0.max(bx) + x1.min(bx + c)) / 2.0, (y0.max(by) + y1.min(by + c)) / 2.0);
                let top = terrain.height_in(ci, cj, qx, qy);
                let q = v3(qx, qy, if block_corners_in { (z0 + top) / 2.0 } else { top });
                out.push((q, hit.n, hit.depth, hit.wall));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::HeightGrid;

    /// A lug piece whose top slopes across the width: a block under the low end only touches it
    /// once it reaches the sloping top, and pushes along the tilted tip normal, not straight up.
    #[test]
    fn sloped_top_meets_a_block_where_it_is() {
        // Lug upside down over the ground (tip facing -z), width along y from 0 to 0.075,
        // height 0.02 at y = 0.075 falling to 0.005 at y = 0. Root at z = 0.03.
        let (b, h0, h1) = (0.02, 0.005, 0.02);
        let v = |x: f64, y: f64, h: f64| v3(x, y, 0.03 - h);
        let verts = [v(-b, 0.0, 0.0), v(b, 0.0, 0.0), v(b, 0.0, h0), v(-b, 0.0, h0), v(-b, 0.075, 0.0), v(b, 0.075, 0.0), v(b, 0.075, h1), v(-b, 0.075, h1)];
        let s = (h1 - h0) / 0.075;
        let l = (1.0 + s * s).sqrt();
        let solid = Solid { verts, side_n: [v3(1.0, 0.0, 0.0), v3(-1.0, 0.0, 0.0)], tip_n: v3(0.0, -s / l, -1.0 / l), width_n: v3(0.0, 1.0, 0.0) };
        let mut out = vec![];
        // Flat ground at z = 0.012: the high end (lowest point z = 0.01) is in by 2 mm, the low end
        // (z = 0.025) is clear. The contact must be at the high end.
        let g = HeightGrid::new(-1.0, 0.05, 40, 40, vec![0.012; 1600], 0.012);
        solid_hits(&solid, &g, 0.001, true, &mut out);
        assert!(!out.is_empty());
        for &(p, _, d, _) in &out {
            assert!(p.y > 0.05, "contact at y {} should be near the high end", p.y);
            assert!(d <= 0.0021, "depth {d}");
        }
    }
}
