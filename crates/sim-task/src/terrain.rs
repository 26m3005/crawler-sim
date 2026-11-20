//! Terrains from a description and a seed. The same description gives the same terrain.
//!
//! The robot starts at x = 0; a terrain begins `START_X` ahead of it and is `WIDTH` wide.
//! A description in text is a name and `key=value` pairs, such as `stairs rise=0.08 steps=4`
//! (`TerrainSpec::parse`); a key left out takes its default.

use sim_core::terrain::{HeightGrid, Terrain};

/// Where the terrain starts along x [m]; the robot starts at x = 0.
pub const START_X: f64 = 1.2;
/// Width of the terrain across y [m].
pub const WIDTH: f64 = 3.5;

/// A kind of terrain. All are block columns on a square grid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TerrainSpec {
    /// Blocks of random height up to `hmax`, 4 m long.
    Random { cell: f64, hmax: f64, seed: u64 },
    /// `steps` steps up; `skew_deg` turns the edges so one track meets them first.
    Stairs { rise: f64, tread: f64, steps: usize, skew_deg: f64 },
    /// A single step up, across the whole width.
    Step { h: f64 },
    /// A step on the left half only.
    Curb { h: f64 },
    /// Rails of height `h`, every `every` cells of 0.1 m.
    Rails { h: f64, every: usize },
    /// A checkerboard of holes `h` deep.
    Holes { h: f64, cell: f64 },
    /// A smooth slope up, 3 m long, from level ground to level ground.
    Slope { deg: f64 },
    /// A slope across the left half (y > 0), rising to the left, 3 m long.
    SideSlope { deg: f64 },
    /// Bare ground.
    Flat,
    /// Every terrain above in one course (`course`).
    Course { seed: u64 },
}

impl TerrainSpec {
    /// Reads a description such as `step h=0.15` or `course seed=2`.
    pub fn parse(text: &str) -> Result<TerrainSpec, String> {
        let mut words = text.split_whitespace();
        let name = words.next().ok_or("the terrain needs a name")?;
        let mut keys: Vec<(&str, f64)> = vec![];
        for w in words {
            let (k, v) = w.split_once('=').ok_or_else(|| format!("`{w}` is not key=value"))?;
            let v: f64 = v.parse().map_err(|_| format!("`{v}` is not a number"))?;
            if !v.is_finite() || v < 0.0 {
                return Err(format!("{k} must be a number, 0 or more"));
            }
            keys.push((k, v));
        }
        let allowed: &[&str] = match name {
            "random" => &["cell", "hmax", "seed"],
            "stairs" => &["rise", "tread", "steps", "skew_deg"],
            "step" | "curb" => &["h"],
            "rails" => &["h", "every"],
            "holes" => &["h", "cell"],
            "slope" | "side_slope" => &["deg"],
            "flat" => &[],
            "course" => &["seed"],
            _ => return Err(format!("unknown terrain `{name}` (random, stairs, step, curb, rails, holes, slope, side_slope, flat, course)")),
        };
        if let Some((k, _)) = keys.iter().find(|(k, _)| !allowed.contains(k)) {
            return Err(format!("terrain `{name}` has no `{k}` (it has: {})", allowed.join(", ")));
        }
        let get = |k: &str, default: f64| keys.iter().rev().find(|(key, _)| *key == k).map_or(default, |(_, v)| *v);
        let count = |k: &str, default: f64| -> Result<usize, String> {
            let v = get(k, default);
            if v < 1.0 || v.fract() != 0.0 {
                return Err(format!("{k} must be a whole number, 1 or more"));
            }
            Ok(v as usize)
        };
        let spec = match name {
            "random" => TerrainSpec::Random { cell: get("cell", 0.25), hmax: get("hmax", 0.2), seed: get("seed", 0.0) as u64 },
            "stairs" => TerrainSpec::Stairs { rise: get("rise", 0.08), tread: get("tread", 0.25), steps: count("steps", 8.0)?, skew_deg: get("skew_deg", 0.0) },
            "step" => TerrainSpec::Step { h: get("h", 0.1) },
            "curb" => TerrainSpec::Curb { h: get("h", 0.1) },
            "rails" => TerrainSpec::Rails { h: get("h", 0.05), every: count("every", 5.0)? },
            "holes" => TerrainSpec::Holes { h: get("h", 0.1), cell: get("cell", 0.25) },
            "slope" => TerrainSpec::Slope { deg: get("deg", 15.0) },
            "side_slope" => TerrainSpec::SideSlope { deg: get("deg", 15.0) },
            "flat" => TerrainSpec::Flat,
            _ => TerrainSpec::Course { seed: get("seed", 0.0) as u64 },
        };
        let cell = match spec {
            TerrainSpec::Random { cell, .. } | TerrainSpec::Holes { cell, .. } | TerrainSpec::Stairs { tread: cell, .. } => cell,
            _ => 1.0,
        };
        if cell <= 0.0 {
            return Err("cell and tread must be more than 0".into());
        }
        Ok(spec)
    }

    /// A label for grouping results, without the seed.
    pub fn group(&self) -> String {
        match *self {
            TerrainSpec::Random { cell, hmax, .. } => format!("random {cell:.2} m cells, up to {hmax:.2} m"),
            TerrainSpec::Stairs { rise, tread, skew_deg, .. } => {
                let skew = if skew_deg != 0.0 { format!(", skewed {skew_deg:.0} deg") } else { String::new() };
                format!("stairs {:.0} deg{skew}", (rise / tread).atan().to_degrees())
            }
            TerrainSpec::Step { h } => format!("step {h:.2} m"),
            TerrainSpec::Curb { h } => format!("one-side step {h:.2} m"),
            TerrainSpec::Rails { h, every } => format!("rails {h:.2} m, every {:.1} m", every as f64 * 0.1),
            TerrainSpec::Holes { h, cell } => format!("checker holes {h:.2} m deep, {cell:.2} m cells"),
            TerrainSpec::Slope { deg } => format!("slope {deg:.0} deg"),
            TerrainSpec::SideSlope { deg } => format!("side slope {deg:.0} deg"),
            TerrainSpec::Flat => "flat".into(),
            TerrainSpec::Course { .. } => "course".into(),
        }
    }

    /// Where a run counts as through: 0.6 m past the end of the terrain [m].
    pub fn goal_x(&self) -> f64 {
        let length = match *self {
            TerrainSpec::Course { .. } => {
                let (_, cell, nx, _, _) = self.build().layout();
                nx as f64 * cell
            }
            _ => self.cell_and_length().1,
        };
        START_X + length + 0.6
    }

    fn cell_and_length(&self) -> (f64, f64) {
        match *self {
            TerrainSpec::Random { cell, .. } => (cell, 4.0),
            TerrainSpec::Stairs { tread, steps, .. } => (tread, steps as f64 * tread),
            TerrainSpec::Step { .. } | TerrainSpec::Curb { .. } => (0.25, 2.0),
            TerrainSpec::Rails { .. } | TerrainSpec::Flat => (0.1, 4.0),
            TerrainSpec::Holes { cell, .. } => (cell, 4.0),
            TerrainSpec::Slope { .. } | TerrainSpec::SideSlope { .. } => (0.25, 3.0),
            TerrainSpec::Course { .. } => (0.05, 0.0),
        }
    }

    pub fn build(&self) -> HeightGrid {
        if let TerrainSpec::Course { seed } = *self {
            return course(0.05, 1.0, seed);
        }
        let (cell, length) = self.cell_and_length();
        let nx = (length / cell).round() as usize;
        let ny = (WIDTH / cell).round() as usize;
        let mut rng = XorShift::new(if let TerrainSpec::Random { seed, .. } = *self { seed } else { 1 });
        let mut h = vec![0.0; nx * ny];
        let mut grades = vec![(0.0, 0.0); nx * ny];
        for i in 0..nx {
            for j in 0..ny {
                let x = (i as f64 + 0.5) * cell;
                let y = (j as f64 + 0.5) * cell - WIDTH / 2.0;
                match *self {
                    TerrainSpec::Slope { deg } => grades[i * ny + j] = (deg.to_radians().tan(), 0.0),
                    TerrainSpec::SideSlope { deg } if y > 0.0 => grades[i * ny + j] = (0.0, deg.to_radians().tan()),
                    _ => {}
                }
                let on = |b: bool, h: f64| if b { h } else { 0.0 };
                h[i * ny + j] = match *self {
                    TerrainSpec::Random { hmax, .. } => rng.next() * hmax,
                    TerrainSpec::Stairs { rise, steps, skew_deg, .. } => {
                        let skew = skew_deg.to_radians().tan() * y / cell;
                        ((i as f64 + skew).floor().max(-1.0) + 1.0).min(steps as f64 + 1.0) * rise
                    }
                    TerrainSpec::Step { h } => h,
                    TerrainSpec::Curb { h } => on(y > 0.0, h),
                    TerrainSpec::Rails { h, every } => on(i % every == 0, h),
                    TerrainSpec::Holes { h, .. } => on((i + j) % 2 == 0, h),
                    TerrainSpec::Slope { deg } => x * deg.to_radians().tan(),
                    TerrainSpec::SideSlope { deg } => on(y > 0.0, y * deg.to_radians().tan()),
                    TerrainSpec::Flat | TerrainSpec::Course { .. } => 0.0,
                };
            }
        }
        let end = match *self {
            TerrainSpec::Stairs { rise, steps, .. } => rise * steps as f64,
            TerrainSpec::Step { h } | TerrainSpec::Holes { h, .. } => h,
            TerrainSpec::Slope { deg } => length * deg.to_radians().tan(),
            _ => 0.0,
        };
        HeightGrid::new(START_X, cell, nx, ny, h, end).with_grades(grades)
    }
}

/// Every terrain in one course: rails, a step, a curb, a side slope, holes, random blocks, stairs
/// and a slope, in that order, with `gap` m of flat ground after each. Each part starts at the
/// height the one before it ended, so the course climbs. The parts are drawn on one grid of
/// `cell`, which must divide every part's cell.
pub fn course(cell: f64, gap: f64, seed: u64) -> HeightGrid {
    let parts = [
        TerrainSpec::Rails { h: 0.05, every: 5 },
        TerrainSpec::Step { h: 0.1 },
        TerrainSpec::Curb { h: 0.1 },
        // Before the random blocks, where the robot still runs straight: the side slope's entry
        // step grows to the left, and drifting there could make it too high to climb.
        TerrainSpec::SideSlope { deg: 15.0 },
        TerrainSpec::Holes { h: 0.1, cell: 0.25 },
        TerrainSpec::Random { cell: 0.25, hmax: 0.1, seed },
        TerrainSpec::Stairs { rise: 0.08, tread: 0.25, steps: 4, skew_deg: 0.0 },
        TerrainSpec::Slope { deg: 15.0 },
    ];
    let ny = (WIDTH / cell).round() as usize;
    let gap_rows = (gap / cell).round() as usize;
    let (mut h, mut grades, mut base) = (vec![], vec![], 0.0);
    for part in parts {
        let g = part.build();
        let (_, pc, pnx, pny, end) = g.layout();
        let k = (pc / cell).round() as usize;
        assert!((k as f64 * cell - pc).abs() < 1e-9 && pny * k == ny, "cell {cell} does not divide the part's cell {pc}");
        // Each small cell takes the part's plane at its own centre.
        for i in 0..pnx * k {
            for j in 0..ny {
                let (pi, pj) = ((i / k) as i32, (j / k) as i32);
                let (x, y) = (START_X + (i as f64 + 0.5) * cell, (j as f64 + 0.5) * cell - WIDTH / 2.0);
                h.push(g.height_in(pi, pj, x, y) + base);
                grades.push(g.grade(pi, pj));
            }
        }
        base += end;
        h.extend(std::iter::repeat(base).take(gap_rows * ny));
        grades.extend(std::iter::repeat((0.0, 0.0)).take(gap_rows * ny));
    }
    HeightGrid::new(START_X, cell, h.len() / ny, ny, h, base).with_grades(grades)
}

/// A small fast generator; the same as the prototypes, so seeds give the same terrains.
struct XorShift(u64);

impl XorShift {
    fn new(seed: u64) -> XorShift {
        let mut r = XorShift(seed.wrapping_mul(0x9E3779B97F4A7C15) | 1);
        for _ in 0..8 {
            r.next();
        }
        r
    }

    /// Uniform in [0, 1).
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_terrain() {
        let a = TerrainSpec::Random { cell: 0.25, hmax: 0.2, seed: 3 }.build();
        let b = TerrainSpec::Random { cell: 0.25, hmax: 0.2, seed: 3 }.build();
        let c = TerrainSpec::Random { cell: 0.25, hmax: 0.2, seed: 4 }.build();
        assert_eq!(a.heights(), b.heights());
        assert_ne!(a.heights(), c.heights());
        assert!(a.heights().iter().all(|h| (0.0..0.2).contains(h)));
    }

    #[test]
    fn matches_the_earlier_version() {
        // The first heights of random 0.25 m, up to 0.10 m, seed 1, as the earlier version made them.
        let g = TerrainSpec::Random { cell: 0.25, hmax: 0.10, seed: 1 }.build();
        let cm: Vec<i64> = g.heights().iter().take(10).map(|h| (h * 100.0).round() as i64).collect();
        assert_eq!(cm, vec![9, 6, 0, 2, 3, 1, 7, 8, 2, 6]);
    }

    #[test]
    fn reads_descriptions() {
        assert_eq!(TerrainSpec::parse("stairs rise=0.1 steps=4").unwrap(), TerrainSpec::Stairs { rise: 0.1, tread: 0.25, steps: 4, skew_deg: 0.0 });
        assert_eq!(TerrainSpec::parse("  course  ").unwrap(), TerrainSpec::Course { seed: 0 });
        for bad in ["", "hill", "step height=0.1", "step h", "step h=x", "rails every=2.5", "holes cell=0"] {
            assert!(TerrainSpec::parse(bad).is_err(), "{bad:?} should be an error");
        }
    }

    #[test]
    fn the_course_climbs_part_after_part() {
        let g = course(0.05, 1.0, 0);
        let (_, cell, nx, ny, end) = g.layout();
        assert_eq!(ny, 70);
        // Parts 4 + 2 + 2 + 3 + 4 + 4 + 1 + 3 m, each with 1 m of flat ground after it.
        assert!((nx as f64 * cell - 31.0).abs() < 1e-9, "length {}", nx as f64 * cell);
        let expected = 0.1 + 0.1 + 4.0 * 0.08 + 3.0 * 15f64.to_radians().tan();
        assert!((end - expected).abs() < 1e-12);
        assert!((TerrainSpec::Course { seed: 0 }.goal_x() - (START_X + 31.6)).abs() < 1e-9);
    }

    #[test]
    fn a_slope_is_smooth_from_level_ground_to_level_ground() {
        let g = TerrainSpec::Slope { deg: 15.0 }.build();
        let t = 15f64.to_radians().tan();
        assert!(g.edges().is_empty(), "{} edge points on a smooth slope", g.edges().len());
        for x in [START_X + 0.01, START_X + 1.37, START_X + 2.99] {
            let (i, j) = g.cell_of(x, 0.3);
            assert!((g.height_in(i, j, x, 0.3) - (x - START_X) * t).abs() < 1e-12, "height at x {x}");
        }
    }

    #[test]
    fn a_side_slope_rises_to_the_left() {
        let g = TerrainSpec::SideSlope { deg: 15.0 }.build();
        let t = 15f64.to_radians().tan();
        let (i, j) = g.cell_of(START_X + 1.0, 0.6);
        assert!((g.height_in(i, j, START_X + 1.0, 0.6) - 0.6 * t).abs() < 1e-12);
        let (i, j) = g.cell_of(START_X + 1.0, -0.6);
        assert_eq!(g.height_in(i, j, START_X + 1.0, -0.6), 0.0);
        // Its only edges are where the left half starts and ends above the level ground.
        assert!(g.edges().iter().all(|e| e.y > 0.0 && ((e.x - START_X).abs() < 1e-12 || (e.x - START_X - 3.0).abs() < 1e-12)));
    }

    #[test]
    fn the_course_keeps_the_slopes_smooth() {
        let g = course(0.05, 1.0, 0);
        // No edges inside the slope (it starts 27 m in, after the stairs' 1 m of level ground).
        let from = START_X + 4.0 + 1.0 + 2.0 + 1.0 + 2.0 + 1.0 + 3.0 + 1.0 + 4.0 + 1.0 + 4.0 + 1.0 + 1.0 + 1.0;
        assert!(g.edges().iter().all(|e| e.x < from + 0.01 || e.x > from + 2.99), "edges inside the slope");
    }
}
