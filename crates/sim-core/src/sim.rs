//! A crawler as a [`Body`] for the solver, and [`Simulation`]: one crawler on one terrain.
//!
//! The crawler's chassis, belts and flipper joints all move when a contact pushes a point on a
//! lug or the belt; its motors and servos run between the solver's sweeps.
//!
//! The chassis and the flipper links move as one rigid body whose centre of mass and inertia
//! follow the flipper angles: when a flipper turns, the chassis shifts so the centre of mass
//! stays put, the spin is rescaled to keep the angular momentum, and the servo's torque turns
//! the body the other way.

use crate::contact::{Contact, ContactGenerator, Frame, TrackPaths};
use crate::math::{v3, Mat3, V3};
use crate::model::{Model, Side, Surface};
use crate::path::BeltPath;
use crate::solver::{Body, Solver, Touch, NO_PART};
use crate::state::{Command, State};
use crate::terrain::Terrain;

/// Settings of the solver, separate from the robots and the terrain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimConfig {
    /// Time step [s].
    pub dt: f64,
    /// Sweeps over the contacts per step.
    pub iterations: usize,
    /// Contacts shallower than this stay on a top or tip face [m].
    pub skin: f64,
    pub gravity: V3,
    /// Friction of the terrain, and the default for surfaces that do not set their own.
    pub terrain_friction: f64,
}

impl Default for SimConfig {
    fn default() -> Self {
        SimConfig { dt: 0.001, iterations: 40, skin: 0.003, gravity: v3(0.0, 0.0, -9.81), terrain_friction: 1.0 }
    }
}

/// What happened in one step, for checks and measurements.
#[derive(Clone, Copy, Debug, Default)]
pub struct StepReport {
    pub contacts: usize,
    /// Some lug side wall is pressed by a block edge.
    pub caught: bool,
    /// Total contact force on the tracks and on the chassis [N].
    pub track_force: V3,
    pub chassis_force: V3,
}

/// A contact point on the crawler, prepared for the solver.
struct Anchor {
    r: V3,
    u: V3,
    track: Option<usize>,
    joint: Option<usize>,
    /// The drive turning the point's belt.
    drive: usize,
    /// Point velocity per unit joint speed.
    c: V3,
    /// Inverse masses of the point's belt and joint, 0 when it has none.
    belt_im: f64,
    joint_im: f64,
}

/// A crawler: one rigid chassis, its belts and its flipper joints.
pub struct Crawler<'m, T: Terrain> {
    pub model: &'m Model,
    pub state: State,
    pub cmd: Command,
    generators: Vec<Box<dyn ContactGenerator<T> + 'm>>,
    contacts: Vec<Contact>,
    anchors: Vec<Anchor>,
    paths: Vec<TrackPaths>,
    motor: Vec<f64>,
    servo: Vec<f64>,
    inv_mass: f64,
    /// Mass moving with each drive's belts [kg].
    drive_mass: Vec<f64>,
    /// The inertia in world axes for this step, and its inverse.
    inertia_world: Mat3,
    inv_inertia_world: Mat3,
}

impl<'m, T: Terrain> Crawler<'m, T> {
    pub fn new(model: &'m Model, state: State, generators: Vec<Box<dyn ContactGenerator<T> + 'm>>) -> Self {
        let cmd = Command { belt_speed: [0.0; 2], joint_target: state.joint_q.clone() };
        let drive_mass = model.drives.iter().map(|d| d.tracks.iter().map(|&k| model.tracks[k].belt_mass).sum()).collect();
        Crawler { model, state, cmd, generators, contacts: vec![], anchors: vec![], paths: vec![], motor: vec![], servo: vec![], inv_mass: 1.0 / model.mass, drive_mass, inertia_world: Mat3::IDENTITY, inv_inertia_world: Mat3::IDENTITY }
    }

    /// The whole body's inertia in world axes at the current joint angles.
    fn world_inertia(&self) -> Mat3 {
        let r = self.state.pose.rot.to_mat3();
        r.mul(&self.model.mass_properties(&self.state.joint_q).inertia).mul(&r.transpose())
    }

    /// Adds `dv` to the speed of every belt on drive `d`.
    fn push_drive(&mut self, d: usize, dv: f64) {
        for &k in &self.model.drives[d].tracks {
            self.state.belt_v[k] += dv;
        }
    }

    /// Inverse effective mass of the chassis at offset `r` along `d`.
    fn weight(&self, r: V3, d: V3) -> f64 {
        let c = r.cross(d);
        self.inv_mass + c.dot(self.inv_inertia(c))
    }

    fn inv_inertia(&self, v: V3) -> V3 {
        self.inv_inertia_world.mul_v(v)
    }
}

impl<'m, T: Terrain> Body<T> for Crawler<'m, T> {
    fn begin(&mut self, config: &SimConfig) {
        // The orientation and the joint angles do not change during the sweeps: the inertia once.
        self.inertia_world = self.world_inertia();
        self.inv_inertia_world = self.inertia_world.inverse();
        self.anchors.clear();
        self.motor.clear();
        self.motor.resize(self.model.drives.len(), 0.0);
        self.servo.clear();
        self.servo.resize(self.model.joints.len(), 0.0);
        self.state.vel = self.state.vel + config.gravity * config.dt;
    }

    fn terrain_touches(&mut self, terrain: &T, config: &SimConfig, out: &mut Vec<Touch>) {
        let m = self.model;
        self.paths.clear();
        for (k, t) in m.tracks.iter().enumerate() {
            let [a, b] = m.track_pulleys(k, &self.state.joint_q);
            self.paths.push(TrackPaths { belt: BeltPath { a, b, r: t.radius }, outer: BeltPath { a, b, r: t.outer_radius() } });
        }
        self.contacts.clear();
        let frame = Frame { model: m, pose: &self.state.pose, paths: &self.paths, belt_q: &self.state.belt_q, joint_q: &self.state.joint_q, skin: config.skin };
        for g in self.generators.iter_mut() {
            g.collect(&frame, terrain, &mut self.contacts);
        }
        // Parts: the tracks, then each joint's link; the chassis is NO_PART.
        let n = m.tracks.len();
        for c in &self.contacts {
            let part = c.track.or(c.joint.map(|j| n + j)).unwrap_or(NO_PART);
            out.push(Touch { point: c.point, normal: c.normal, depth: c.depth, part, surface_dir: c.belt_dir, caught: c.caught });
        }
    }

    fn surface(&self, part: usize) -> Surface {
        self.model.tracks.get(part).map_or(self.model.chassis_surface, |t| t.surface)
    }

    fn anchor(&mut self, t: &Touch) -> usize {
        let m = self.model;
        let s = &self.state;
        let n = m.tracks.len();
        let track = (t.part < n).then_some(t.part);
        let joint = match track {
            Some(k) => m.tracks[k].joint,
            None => (t.part != NO_PART).then(|| t.part - n),
        };
        let drive = track.map_or(0, |k| m.tracks[k].drive);
        let c = match joint {
            Some(j) => {
                let jt = &m.joints[j];
                s.pose.rot.rot(jt.axis).cross(t.point - s.pose.to_world(jt.origin))
            }
            None => V3::ZERO,
        };
        self.anchors.push(Anchor {
            r: t.point - s.pose.com,
            u: t.surface_dir,
            track,
            joint,
            drive,
            c,
            belt_im: track.map_or(0.0, |_| 1.0 / self.drive_mass[drive]),
            joint_im: joint.map_or(0.0, |j| 1.0 / m.joints[j].inertia),
        });
        self.anchors.len() - 1
    }

    fn velocity(&self, a: usize) -> V3 {
        let p = &self.anchors[a];
        let s = &self.state;
        let mut v = s.vel + s.ang_vel.cross(p.r);
        if let Some(k) = p.track {
            v = v + p.u * s.belt_v[k];
        }
        if let Some(j) = p.joint {
            v = v + p.c * s.joint_v[j];
        }
        v
    }

    fn inv_mass(&self, a: usize, d: V3) -> f64 {
        let p = &self.anchors[a];
        let (ud, jd) = (d.dot(p.u), d.dot(p.c));
        self.weight(p.r, d) + p.belt_im * ud * ud + jd * jd * p.joint_im
    }

    fn apply(&mut self, a: usize, imp: V3) {
        let p = &self.anchors[a];
        let dw = self.inv_inertia(p.r.cross(imp));
        let (u, c, track, joint, drive, belt_im, joint_im) = (p.u, p.c, p.track, p.joint, p.drive, p.belt_im, p.joint_im);
        let inv_mass = self.inv_mass;
        if track.is_some() {
            self.push_drive(drive, u.dot(imp) * belt_im);
        }
        let s = &mut self.state;
        s.vel = s.vel + imp * inv_mass;
        s.ang_vel = s.ang_vel + dw;
        if let Some(j) = joint {
            s.joint_v[j] += c.dot(imp) * joint_im;
        }
    }

    fn actuate(&mut self, config: &SimConfig) {
        let h = config.dt;
        let m = self.model;
        // Joint servos: toward the target at a limited speed, up to the torque limit. The body
        // takes the opposite turn.
        for (j, joint) in m.joints.iter().enumerate() {
            let s = &mut self.state;
            let want = (joint.servo_gain * (self.cmd.joint_target[j] - s.joint_q[j])).clamp(-joint.max_velocity, joint.max_velocity);
            let limit = joint.max_effort * h;
            let new = (self.servo[j] + (want - s.joint_v[j]) * joint.inertia).clamp(-limit, limit);
            let d = new - self.servo[j];
            s.joint_v[j] += d / joint.inertia;
            let reaction = self.inv_inertia_world.mul_v(s.pose.rot.rot(joint.axis) * -d);
            s.ang_vel = s.ang_vel + reaction;
            self.servo[j] = new;
        }
        // Drive motors: toward the commanded speed, up to the force limit; each turns all its belts.
        for (d, drive) in m.drives.iter().enumerate() {
            let target = match drive.side {
                Side::Left => self.cmd.belt_speed[0],
                Side::Right => self.cmd.belt_speed[1],
            };
            let (mass, limit) = (self.drive_mass[d], drive.max_force() * h);
            let v = self.state.belt_v[drive.tracks[0]];
            let new = (self.motor[d] + (target - v) * mass).clamp(-limit, limit);
            self.push_drive(d, (new - self.motor[d]) / mass);
            self.motor[d] = new;
        }
    }

    fn end(&mut self, config: &SimConfig) {
        let h = config.dt;
        let s = &mut self.state;
        s.pose.com = s.pose.com + s.vel * h;
        s.pose.rot = s.pose.rot.add_rotation(s.ang_vel * h);
        for k in 0..self.model.tracks.len() {
            s.belt_q[k] += s.belt_v[k] * h;
        }
        // The flippers move: keep the centre of mass and the angular momentum.
        let momentum = self.world_inertia().mul_v(self.state.ang_vel);
        let s = &mut self.state;
        for (j, joint) in self.model.joints.iter().enumerate() {
            s.joint_q[j] = (s.joint_q[j] + s.joint_v[j] * h).clamp(joint.lower, joint.upper);
        }
        s.pose.com_offset = self.model.mass_properties(&s.joint_q).com;
        self.state.ang_vel = self.world_inertia().inverse().mul_v(momentum);
        self.state.time += h;
    }
}

/// One crawler on one terrain: the common case, with the state kept by the caller.
pub struct Simulation<'m, T: Terrain> {
    crawler: Crawler<'m, T>,
    config: SimConfig,
    solver: Solver,
}

impl<'m, T: Terrain> Simulation<'m, T> {
    pub fn new(model: &'m Model, config: SimConfig, generators: Vec<Box<dyn ContactGenerator<T> + 'm>>) -> Self {
        let state = State::at_rest(model, V3::ZERO, vec![0.0; model.joints.len()]);
        Simulation { crawler: Crawler::new(model, state, generators), config, solver: Solver::default() }
    }

    /// Advances `state` by one time step.
    pub fn step(&mut self, state: &mut State, cmd: &Command, terrain: &T) -> StepReport {
        std::mem::swap(&mut self.crawler.state, state);
        self.crawler.cmd.clone_from(cmd);
        self.solver.step(&mut self.crawler, terrain, &self.config);
        std::mem::swap(&mut self.crawler.state, state);
        let h = self.config.dt;
        let mut report = StepReport { contacts: self.solver.rows.len(), ..Default::default() };
        for r in &self.solver.rows {
            report.caught |= r.caught;
            let f = (r.n * r.lambda_n + r.lambda_t) * (1.0 / h);
            if r.part < self.crawler.model.tracks.len() {
                report.track_force = report.track_force + f;
            } else {
                report.chassis_force = report.chassis_force + f;
            }
        }
        report
    }
}
