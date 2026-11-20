//! The traversal task: drive straight over a terrain with fixed flipper angles.
//!
//! A run ends when the robot passes the goal, rolls over, stops making progress, or runs out
//! of time. It is a success when it passes the goal without leaving the terrain's width and
//! without tilting past 60 degrees on the way.

use sim_core::math::v3;
use sim_core::model::Model;
use sim_core::path::BeltPath;
use sim_core::sim::{SimConfig, Simulation};
use sim_core::state::{Command, State};
use sim_core::crawler_generators;

use sim_task::terrain::TerrainSpec;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TraverseCase {
    pub terrain: TerrainSpec,
    /// Friction of the terrain.
    pub friction: f64,
    /// Front and rear flipper angles [deg], held for the whole run.
    pub front_deg: f64,
    pub rear_deg: f64,
    /// Starting lug position as a fraction of the lug pitch.
    pub phase: f64,
    /// Belt speed command [m/s], reached over the first second.
    pub belt_speed: f64,
    pub seconds: f64,
    /// Tilts gravity forward by this angle [deg]: on flat ground, a perfectly smooth slope.
    pub tilt_deg: f64,
    /// Record the pose this many times a second; 0 records nothing.
    pub record_hz: f64,
}

impl TraverseCase {
    pub fn new(terrain: TerrainSpec, friction: f64, front_deg: f64) -> TraverseCase {
        TraverseCase { terrain, friction, front_deg, rear_deg: 0.0, phase: 0.0, belt_speed: 0.5, seconds: 30.0, tilt_deg: 0.0, record_hz: 0.0 }
    }
}

/// One recorded instant.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub time: f64,
    pub state: State,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TraverseResult {
    pub case: TraverseCase,
    /// Time the goal was passed [s].
    pub done: Option<f64>,
    /// Time the robot turned over [s].
    pub rolled: Option<f64>,
    /// Time it was found not moving [s].
    pub stuck: Option<f64>,
    pub final_x: f64,
    pub final_y: f64,
    pub max_tilt_deg: f64,
    /// Time some lug side wall was caught on an edge [s].
    pub caught_s: f64,
    /// Seconds of simulated time per second of computing.
    pub speed: f64,
    pub frames: Vec<Frame>,
}

impl TraverseResult {
    pub fn success(&self) -> bool {
        self.done.is_some() && self.final_y.abs() <= 1.5 && self.max_tilt_deg <= 60.0
    }
}

pub fn run(model: &Model, case: &TraverseCase) -> TraverseResult {
    run_with(model, case, &|_| (case.front_deg, case.rear_deg))
}

/// Like `run`, but the front and rear flipper targets [deg] come from `flippers` at every step.
/// The robot starts with the case's angles.
pub fn run_with(model: &Model, case: &TraverseCase, flippers: &dyn Fn(&State) -> (f64, f64)) -> TraverseResult {
    let terrain = case.terrain.build();
    let tilt = case.tilt_deg.to_radians();
    let config = SimConfig { terrain_friction: case.friction, gravity: v3(-9.81 * tilt.sin(), 0.0, -9.81 * tilt.cos()), ..SimConfig::default() };
    let mut sim = Simulation::new(model, config, crawler_generators());
    let h = config.dt;

    // Flipper angles: the front joints get the front angle, the rear ones the rear angle.
    let angles: Vec<f64> = model.joints.iter().map(|j| if j.origin.x >= 0.0 { case.front_deg } else { case.rear_deg }.to_radians()).collect();
    // The prototypes start 2 mm above level ground with the lowest belt point at the lug tips.
    let start_z = model.tracks.iter().map(|t| t.outer_radius()).fold(0.0, f64::max) + 0.002;
    let mut state = State::at_rest(model, v3(0.0, 0.0, start_z), angles.clone());
    for (k, t) in model.tracks.iter().enumerate() {
        if let Some(l) = t.lugs {
            let [a, b] = model.track_pulleys(k, &state.joint_q);
            let pitch = BeltPath { a, b, r: t.radius }.len() / l.count as f64;
            state.belt_q[k] = -case.phase * pitch;
        }
    }

    let steps = (case.seconds / h) as usize;
    let record_every = if case.record_hz > 0.0 { (1.0 / (case.record_hz * h)).round().max(1.0) as usize } else { usize::MAX };
    let goal = case.terrain.goal_x();
    let mut cmd = Command { belt_speed: [0.0; 2], joint_target: angles };
    let (mut done, mut rolled, mut stuck) = (None, None, None);
    let (mut max_tilt, mut caught_steps) = (0.0f64, 0usize);
    let (mut check_x, mut check_step) = (0.0, 0usize);
    let mut frames = vec![];
    let started = std::time::Instant::now();
    for step in 0..steps {
        let t = step as f64 * h;
        let v = case.belt_speed * t.min(1.0);
        cmd.belt_speed = [v, v];
        let (front, rear) = flippers(&state);
        for (target, j) in cmd.joint_target.iter_mut().zip(&model.joints) {
            *target = if j.origin.x >= 0.0 { front } else { rear }.to_radians();
        }
        let report = sim.step(&mut state, &cmd, &terrain);
        if report.caught {
            caught_steps += 1;
        }
        let x = state.pose.com.x;
        max_tilt = max_tilt.max(state.pose.rot.tilt());
        if step % record_every == 0 {
            frames.push(Frame { time: t, state: state.clone() });
        }
        if state.pose.rot.rot(v3(0.0, 0.0, 1.0)).z < -0.2 {
            rolled = Some(t);
            break;
        }
        if step >= check_step + (8.0 / h) as usize {
            if (x - check_x).abs() < 0.02 && t > 8.0 {
                stuck = Some(t);
                break;
            }
            check_x = x;
            check_step = step;
        }
        if x > goal {
            done = Some(t);
            break;
        }
        if !x.is_finite() {
            break;
        }
    }
    let elapsed = started.elapsed().as_secs_f64().max(1e-9);
    TraverseResult {
        case: *case,
        done,
        rolled,
        stuck,
        final_x: state.pose.com.x,
        final_y: state.pose.com.y,
        max_tilt_deg: max_tilt.to_degrees(),
        caught_s: caught_steps as f64 * h,
        speed: state.time / elapsed,
        frames,
    }
}
