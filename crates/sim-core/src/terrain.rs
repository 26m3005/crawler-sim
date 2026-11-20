//! Terrain made of columns on a square grid. A column's top is a plane: level, or sloping by
//! its grade, so neighbouring columns of the same grade make one smooth slope.
//!
//! Contact generators depend on the [`Terrain`] trait, not on how the heights are stored.

use crate::math::{v3, V3};

/// Two tops closer than this along the edge they share meet without a step [m]. Only sloping
/// tops use it (rounding leaves their shared edges a few ulps apart); level tops compare exactly.
const MEET: f64 = 1e-9;

/// Columns on a square grid, seen from the contact generators.
pub trait Terrain: Sync {
    /// Side of one cell [m].
    fn cell(&self) -> f64;
    /// The cell containing (x, y).
    fn cell_of(&self, x: f64, y: f64) -> (i32, i32);
    /// The (x, y) corner of a cell with the smallest coordinates.
    fn corner(&self, i: i32, j: i32) -> (f64, f64);
    /// Height of a column's top at the centre of its cell. Defined for every cell, inside the grid or not.
    fn top(&self, i: i32, j: i32) -> f64;
    /// How a column's top slopes: (dz/dx, dz/dy). Level by default.
    fn grade(&self, _i: i32, _j: i32) -> (f64, f64) {
        (0.0, 0.0)
    }
    /// Sample points along the exposed top edges of the columns, sorted by x.
    fn edges(&self) -> &[V3];

    /// Height of the plane of a column's top at (x, y).
    fn height_in(&self, i: i32, j: i32, x: f64, y: f64) -> f64 {
        let (gx, gy) = self.grade(i, j);
        let top = self.top(i, j);
        if gx == 0.0 && gy == 0.0 {
            return top;
        }
        let (x0, y0) = self.corner(i, j);
        let h = self.cell() / 2.0;
        top + gx * (x - x0 - h) + gy * (y - y0 - h)
    }

    /// Outward unit normal of a column's top.
    fn top_normal(&self, i: i32, j: i32) -> V3 {
        let (gx, gy) = self.grade(i, j);
        if gx == 0.0 && gy == 0.0 {
            return v3(0.0, 0.0, 1.0);
        }
        let n = v3(-gx, -gy, 1.0);
        n * (1.0 / n.len())
    }

    /// The highest point of a column's top.
    fn top_max(&self, i: i32, j: i32) -> f64 {
        let (gx, gy) = self.grade(i, j);
        self.top(i, j) + (gx.abs() + gy.abs()) * self.cell() / 2.0
    }

    /// Whether column (i, j)'s top stands above column `n`'s at (x, y), on the edge they share.
    fn above(&self, i: i32, j: i32, n: (i32, i32), x: f64, y: f64) -> bool {
        if self.grade(i, j) == (0.0, 0.0) && self.grade(n.0, n.1) == (0.0, 0.0) {
            return self.top(n.0, n.1) < self.top(i, j);
        }
        self.height_in(i, j, x, y) - self.height_in(n.0, n.1, x, y) > MEET
    }

    /// Which of a column's side faces (+x, -x, +y, -y) stand above their neighbour somewhere.
    fn open_faces(&self, i: i32, j: i32) -> [bool; 4] {
        let (x0, y0) = self.corner(i, j);
        let (x1, y1) = (x0 + self.cell(), y0 + self.cell());
        let side = |n: (i32, i32), a: (f64, f64), b: (f64, f64)| self.above(i, j, n, a.0, a.1) || self.above(i, j, n, b.0, b.1);
        [side((i + 1, j), (x1, y0), (x1, y1)), side((i - 1, j), (x0, y0), (x0, y1)), side((i, j + 1), (x0, y1), (x1, y1)), side((i, j - 1), (x0, y0), (x1, y0))]
    }

    /// If `p` is inside a column: (depth, outward normal through the nearest exposed face).
    /// A point less than `skin` under a top is on the top, even near the column's edge.
    fn inside(&self, p: V3, skin: f64) -> Option<(f64, V3)> {
        let (i, j) = self.cell_of(p.x, p.y);
        let top = self.height_in(i, j, p.x, p.y);
        if p.z >= top {
            return None;
        }
        let n = self.top_normal(i, j);
        let mut best = ((top - p.z) * n.z, n);
        if top - p.z < skin {
            return Some(best);
        }
        let (x0, y0) = self.corner(i, j);
        let c = self.cell();
        let sides = [
            (p.x - x0, v3(-1.0, 0.0, 0.0), self.height_in(i - 1, j, x0, p.y)),
            (x0 + c - p.x, v3(1.0, 0.0, 0.0), self.height_in(i + 1, j, x0 + c, p.y)),
            (p.y - y0, v3(0.0, -1.0, 0.0), self.height_in(i, j - 1, p.x, y0)),
            (y0 + c - p.y, v3(0.0, 1.0, 0.0), self.height_in(i, j + 1, p.x, y0 + c)),
        ];
        for (d, n, other) in sides {
            if p.z > other && d < best.0 {
                best = (d, n);
            }
        }
        Some(best)
    }
}

/// Heights on a grid starting at x = `origin_x`, centred on y = 0, with an optional grade per cell.
///
/// Before the grid the ground is level at 0; past its far end it is level at `end`. Past the
/// sides the outermost row continues: along x as it is, across y level at the height of its
/// outer edge, so the terrain has no cliff at its edge.
#[derive(Clone, Debug)]
pub struct HeightGrid {
    origin_x: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    heights: Vec<f64>,
    /// Empty when every top is level.
    grades: Vec<(f64, f64)>,
    end: f64,
    edges: Vec<V3>,
}

impl HeightGrid {
    /// `heights[i * ny + j]` is the cell i along x, j along y, at the centre of the cell. Every top is level.
    pub fn new(origin_x: f64, cell: f64, nx: usize, ny: usize, heights: Vec<f64>, end: f64) -> HeightGrid {
        assert_eq!(heights.len(), nx * ny, "heights must be nx * ny");
        let mut g = HeightGrid { origin_x, cell, nx, ny, heights, grades: vec![], end, edges: vec![] };
        g.find_edges();
        g
    }

    /// The same grid with the tops sloping by `grades[i * ny + j]` = (dz/dx, dz/dy).
    pub fn with_grades(mut self, grades: Vec<(f64, f64)>) -> HeightGrid {
        assert_eq!(grades.len(), self.nx * self.ny, "grades must be nx * ny");
        self.grades = if grades.iter().all(|g| *g == (0.0, 0.0)) { vec![] } else { grades };
        self.find_edges();
        self
    }

    /// Flat ground at height 0.
    pub fn flat() -> HeightGrid {
        HeightGrid::new(1.2, 0.1, 1, 1, vec![0.0], 0.0)
    }

    pub fn heights(&self) -> &[f64] {
        &self.heights
    }

    /// The grade of every cell, in the order of `heights`; empty when every top is level.
    pub fn grades(&self) -> &[(f64, f64)] {
        &self.grades
    }

    /// (origin x, cell, nx, ny, height past the far end), for drawing.
    pub fn layout(&self) -> (f64, f64, usize, usize, f64) {
        (self.origin_x, self.cell, self.nx, self.ny, self.end)
    }

    fn find_edges(&mut self) {
        self.edges = self.sample_edges();
        // Sorted by x, so a robot only looks at the edges within its reach.
        self.edges.sort_by(|a, b| a.x.total_cmp(&b.x));
    }

    fn sample_edges(&self) -> Vec<V3> {
        let step = 0.025f64.min(self.cell / 2.0);
        let n = (self.cell / step).round() as usize;
        let c = self.cell;
        let mut edges = vec![];
        for i in 0..self.nx as i32 {
            for j in 0..self.ny as i32 {
                let (x0, y0) = self.corner(i, j);
                let mut add = |nb: (i32, i32), a: (f64, f64), b: (f64, f64)| {
                    for k in 0..=n {
                        let t = k as f64 / n as f64;
                        let (x, y) = (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
                        if self.above(i, j, nb, x, y) {
                            edges.push(v3(x, y, self.height_in(i, j, x, y)));
                        }
                    }
                };
                add((i - 1, j), (x0, y0), (x0, y0 + c));
                add((i + 1, j), (x0 + c, y0), (x0 + c, y0 + c));
                add((i, j - 1), (x0, y0), (x0 + c, y0));
                add((i, j + 1), (x0, y0 + c), (x0 + c, y0 + c));
            }
        }
        edges
    }

    /// The top of any cell: (height at its centre, grade).
    fn plane(&self, i: i32, j: i32) -> (f64, (f64, f64)) {
        if i < 0 {
            return (0.0, (0.0, 0.0));
        }
        if i >= self.nx as i32 {
            return (self.end, (0.0, 0.0));
        }
        let jc = j.clamp(0, self.ny as i32 - 1);
        let k = i as usize * self.ny + jc as usize;
        let h = self.heights[k];
        let (gx, gy) = if self.grades.is_empty() { (0.0, 0.0) } else { self.grades[k] };
        if jc == j || gy == 0.0 {
            return (h, (gx, gy));
        }
        // Past a side: level across y at the height of the outermost row's outer edge.
        (h + gy * self.cell / 2.0 * if j > jc { 1.0 } else { -1.0 }, (gx, 0.0))
    }
}

impl Terrain for HeightGrid {
    fn cell(&self) -> f64 {
        self.cell
    }

    fn cell_of(&self, x: f64, y: f64) -> (i32, i32) {
        (((x - self.origin_x) / self.cell).floor() as i32, ((y + self.ny as f64 * self.cell / 2.0) / self.cell).floor() as i32)
    }

    fn corner(&self, i: i32, j: i32) -> (f64, f64) {
        (self.origin_x + i as f64 * self.cell, j as f64 * self.cell - self.ny as f64 * self.cell / 2.0)
    }

    fn top(&self, i: i32, j: i32) -> f64 {
        self.plane(i, j).0
    }

    fn grade(&self, i: i32, j: i32) -> (f64, f64) {
        self.plane(i, j).1
    }

    fn edges(&self) -> &[V3] {
        &self.edges
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inside_a_single_block() {
        // One 0.1 m block at x 1.2..1.3, |y| < 0.05.
        let g = HeightGrid::new(1.2, 0.1, 1, 1, vec![0.1], 0.0);
        assert!(g.inside(v3(1.25, 0.0, 0.11), 0.003).is_none());
        let (d, n) = g.inside(v3(1.25, 0.0, 0.099), 0.003).unwrap();
        assert!((d - 0.001).abs() < 1e-12 && n.z == 1.0);
        // Deep and near the front face: pushed out through the front.
        let (d, n) = g.inside(v3(1.201, 0.0, 0.05), 0.003).unwrap();
        assert!((d - 0.001).abs() < 1e-9 && n.x == -1.0);
    }

    #[test]
    fn edges_only_where_exposed() {
        let g = HeightGrid::new(1.2, 0.1, 2, 1, vec![0.1, 0.1], 0.0);
        // Front and back edges are exposed, the joint between the two blocks is not; the sides
        // continue past the grid, so they are not edges.
        let xs: Vec<f64> = g.edges().iter().map(|e| e.x).collect();
        assert!(xs.iter().all(|x| (x - 1.2).abs() < 1e-12 || (x - 1.4).abs() < 1e-12));
    }

    /// A ramp of 0.1 m cells rising 0.2 per metre from 0 at x = 1.2, then level at its top.
    fn ramp() -> HeightGrid {
        let (g, nx, ny) = (0.2, 10, 4);
        let h = (0..nx * ny).map(|k| ((k / ny) as f64 + 0.5) * 0.1 * g).collect();
        HeightGrid::new(1.2, 0.1, nx, ny, h, nx as f64 * 0.1 * g).with_grades(vec![(g, 0.0); nx * ny])
    }

    #[test]
    fn a_ramp_is_one_smooth_plane() {
        let g = ramp();
        // No edges: the cells meet without steps, and the ramp meets the ground at both ends.
        assert!(g.edges().is_empty(), "{} edge points", g.edges().len());
        let (i, j) = g.cell_of(1.73, 0.01);
        assert!((g.height_in(i, j, 1.73, 0.01) - 0.53 * 0.2).abs() < 1e-12);
        // Just under the slope: pushed out along the slope's normal, by the perpendicular depth.
        let (d, n) = g.inside(v3(1.73, 0.0, 0.53 * 0.2 - 0.002), 0.003).unwrap();
        let l = (1.0f64 + 0.04).sqrt();
        assert!((n.x + 0.2 / l).abs() < 1e-12 && (n.z - 1.0 / l).abs() < 1e-12);
        assert!((d - 0.002 / l).abs() < 1e-12);
        // Past the sides the ramp goes on.
        let (i, j) = g.cell_of(1.73, 3.0);
        assert!((g.height_in(i, j, 1.73, 3.0) - 0.53 * 0.2).abs() < 1e-12);
    }

    #[test]
    fn a_side_slope_levels_off_past_the_side() {
        // One row of two cells across y, rising 0.5 per metre to the left (+y).
        let g = HeightGrid::new(1.2, 0.1, 1, 2, vec![-0.025, 0.025], 0.0).with_grades(vec![(0.0, 0.5); 2]);
        let (i, j) = g.cell_of(1.25, 0.12);
        assert_eq!((j, g.grade(i, j)), (2, (0.0, 0.0)));
        assert!((g.top(i, j) - 0.05).abs() < 1e-12, "level at the left edge's height");
        // The ground before and after is level at 0: the left half stands above it, front and back.
        assert_eq!(g.open_faces(0, 1), [true, true, false, false]);
    }
}
