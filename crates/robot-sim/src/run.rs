//! One robot driven over a terrain with fixed commands: what `run` and the GUI both do.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use sim_core::model::Model;
use sim_core::terrain::HeightGrid;
use sim_task::settings::Settings;
use sim_task::terrain::TerrainSpec;
use sim_task::world::{rest_z, World};

/// The keys of a run, but `robot` and `record`.
pub struct RunSettings {
    pub terrain: TerrainSpec,
    pub friction: f64,
    pub seconds: f64,
    pub belt: [f64; 2],
    /// Joint targets [rad].
    pub joints: Vec<f64>,
    pub control_dt: f64,
}

impl RunSettings {
    pub fn read(s: &Settings, model: &Model) -> Result<RunSettings, String> {
        let terrain = TerrainSpec::parse(s.text("terrain").unwrap_or("course"))?;
        let friction = s.number("friction", 0.8, 0.0)?;
        let seconds = s.number("seconds", 90.0, 0.0)?;
        let belt = s.numbers("belt", &[0.5, 0.5])?;
        let joints = s.numbers("joints", &vec![0.0; model.joints.len()])?;
        let control_dt = s.number("control_dt", 0.05, 0.0)?;
        let [l, r] = belt[..] else { return Err("`belt` needs 2 speeds, left and right".into()) };
        if joints.len() != model.joints.len() {
            return Err(format!("`joints` needs {} angles, one for each of the robot's joints", model.joints.len()));
        }
        Ok(RunSettings { terrain, friction, seconds, belt: [l, r], joints: joints.iter().map(|d| d.to_radians()).collect(), control_dt })
    }

    pub fn world<'m>(&self, model: &'m Model) -> Result<World<'m>, String> {
        let mut w = World::new(model, 1, 1, self.control_dt)?;
        w.set_terrain(&[true], self.terrain.build(), self.friction)?;
        w.reset(&[true], &[0.0, 0.0, rest_z(model)], &[0.0], &self.joints)?;
        Ok(w)
    }

    /// The robot standing at its start on the terrain, as a recording of one frame.
    pub fn still(&self, model: &Model) -> Result<String, String> {
        let w = self.world(model)?;
        let terrain: Arc<HeightGrid> = w.envs[0].terrain.clone();
        Ok(w.recording_json(&(terrain, vec![w.envs[0].state.clone()])))
    }
}

/// Where the robot is.
#[derive(Clone, Copy, Debug, Default)]
pub struct Progress {
    pub time: f64,
    pub x: f64,
    pub z: f64,
}

pub struct Finished {
    pub at: Progress,
    /// Where the run counts as through [m].
    pub goal: f64,
    /// Simulated seconds per second of computing.
    pub speed: f64,
    pub recording: Option<String>,
}

/// Drives until `seconds`, past the end of the terrain, or until `stop` is set. `progress` hears
/// every command step.
pub fn drive(model: &Model, rs: &RunSettings, record: bool, stop: &AtomicBool, progress: &mut dyn FnMut(Progress)) -> Result<Finished, String> {
    let mut w = rs.world(model)?;
    if record {
        w.record(0)?;
    }
    let goal = rs.terrain.goal_x();
    let at = |w: &World| {
        let s = &w.envs[0].state;
        let o = s.pose.to_world(Default::default());
        Progress { time: s.time, x: o.x, z: o.z }
    };
    let started = std::time::Instant::now();
    for _ in 0..(rs.seconds / w.control_dt).round() as usize {
        let p = at(&w);
        progress(p);
        if p.x > goal || stop.load(Ordering::Relaxed) {
            break;
        }
        w.step(&rs.belt, &rs.joints)?;
    }
    let p = at(&w);
    progress(p);
    let speed = p.time / started.elapsed().as_secs_f64().max(1e-9);
    let recording = if record { w.stop_recording().first().map(|(_, r)| w.recording_json(r)) } else { None };
    Ok(Finished { at: p, goal, speed, recording })
}
