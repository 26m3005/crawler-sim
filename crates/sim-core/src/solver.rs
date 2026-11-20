//! The contact solver: one crawler against the terrain.
//!
//! The crawler is a [`Body`]. The solver collects where it touches the terrain, then solves all
//! contacts together as soft velocity constraints by repeated sweeps (sequential impulses),
//! running the body's motors and servos between sweeps. The body only has to say how a point on
//! it moves, how heavy it feels in a direction, and how an impulse changes it.

use crate::math::V3;
use crate::model::Surface;
use crate::sim::SimConfig;
use crate::terrain::Terrain;

/// Marks a touch on no particular part of a body.
pub const NO_PART: usize = usize::MAX;

/// Where the terrain touches the body. `normal` pushes the body; `part` is the body's own index
/// (a track, a link); `surface_dir` is how the surface moves per unit of its drive (a belt), else zero.
#[derive(Clone, Copy, Debug)]
pub struct Touch {
    pub point: V3,
    pub normal: V3,
    pub depth: f64,
    pub part: usize,
    pub surface_dir: V3,
    /// A lug's side wall is pressed by a block edge.
    pub caught: bool,
}

pub trait Body<T: Terrain> {
    /// Starts a step: velocity changes without contact (gravity).
    fn begin(&mut self, config: &SimConfig);
    /// Adds where this body presses into the terrain.
    fn terrain_touches(&mut self, terrain: &T, config: &SimConfig, out: &mut Vec<Touch>);
    /// Contact properties of a part.
    fn surface(&self, part: usize) -> Surface;
    /// Registers a contact point for this step and returns a handle to it.
    fn anchor(&mut self, touch: &Touch) -> usize;
    /// Velocity of an anchored point.
    fn velocity(&self, anchor: usize) -> V3;
    /// Inverse effective mass at an anchored point along the unit direction `d`.
    fn inv_mass(&self, anchor: usize, d: V3) -> f64;
    /// Applies an impulse at an anchored point.
    fn apply(&mut self, anchor: usize, impulse: V3);
    /// Motors and servos, once after every sweep over the contacts.
    fn actuate(&mut self, config: &SimConfig);
    /// Ends the step: moves the body by its velocity.
    fn end(&mut self, config: &SimConfig);
}

/// One contact being solved: the body is pushed along `n` at `anchor`.
pub struct Row {
    pub anchor: usize,
    pub part: usize,
    pub n: V3,
    pub caught: bool,
    bias: f64,
    gamma: f64,
    mass_n: f64,
    friction: f64,
    pub lambda_n: f64,
    pub lambda_t: V3,
}

/// Scratch space for solving; keep one per thread.
#[derive(Default)]
pub struct Solver {
    pub rows: Vec<Row>,
    touches: Vec<Touch>,
}

/// The softness of a contact with a spring and damper over a step `h`: how much it gives.
/// Its push-out is depth * h * stiffness * gamma.
fn softness(h: f64, s: &Surface) -> f64 {
    1.0 / (h * (s.damping + h * s.stiffness))
}

impl Solver {
    /// Advances `body` by one step on `terrain`.
    pub fn step<T: Terrain>(&mut self, body: &mut dyn Body<T>, terrain: &T, config: &SimConfig) {
        let h = config.dt;
        self.rows.clear();
        body.begin(config);
        self.touches.clear();
        body.terrain_touches(terrain, config, &mut self.touches);
        for t in &self.touches {
            let s = body.surface(t.part);
            let gamma = softness(h, &s);
            let anchor = body.anchor(t);
            let w = body.inv_mass(anchor, t.normal);
            self.rows.push(Row { anchor, part: t.part, n: t.normal, caught: t.caught, bias: t.depth * h * s.stiffness * gamma, gamma, mass_n: 1.0 / (w + gamma), friction: s.friction_on(config.terrain_friction), lambda_n: 0.0, lambda_t: V3::ZERO });
        }
        for it in 0..config.iterations {
            // Alternate the direction of the sweep, so no contact is always solved first.
            let n = self.rows.len();
            for idx in 0..n {
                let r = if it % 2 == 0 { idx } else { n - 1 - idx };
                self.solve_row(r, body);
            }
            body.actuate(config);
        }
        body.end(config);
    }

    fn solve_row<T: Terrain>(&mut self, r: usize, body: &mut dyn Body<T>) {
        let row = &self.rows[r];
        // Normal: push apart, never pull.
        let vn = body.velocity(row.anchor).dot(row.n);
        let old = row.lambda_n;
        let new = (old + (row.bias - vn - row.gamma * old) * row.mass_n).max(0.0);
        body.apply(row.anchor, row.n * (new - old));

        // Friction: oppose the slip, within the cone of the current normal impulse.
        let v = body.velocity(row.anchor);
        let vt = v - row.n * v.dot(row.n);
        let sl = vt.len();
        let max_f = row.friction * new;
        let mut lt = row.lambda_t;
        if sl > 1e-9 {
            let dir = vt * (1.0 / sl);
            let m_t = 1.0 / body.inv_mass(row.anchor, dir);
            lt = lt - dir * (sl * m_t);
        }
        let ll = lt.len();
        if ll > max_f && ll > 0.0 {
            lt = lt * (max_f / ll);
        }
        body.apply(row.anchor, lt - row.lambda_t);
        let row = &mut self.rows[r];
        row.lambda_n = new;
        row.lambda_t = lt;
    }
}
