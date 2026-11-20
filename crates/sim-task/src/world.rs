//! Crawlers without a task: the caller gives the terrain, places the robots, sends belt speeds and
//! joint targets, and reads the state. Rewards, observations and resets are the caller's.
//! Many environments step together on several threads.

use std::sync::Arc;

use sim_core::math::{v3, Quat, V3};
use sim_core::model::{Model, Side};
use sim_core::sim::{SimConfig, Simulation};
use sim_core::state::{Command, State};
use sim_core::terrain::HeightGrid;
use sim_core::crawler_generators;

/// One environment: a robot on its terrain.
pub struct WorldEnv<'m> {
    sim: Simulation<'m, HeightGrid>,
    pub terrain: Arc<HeightGrid>,
    pub friction: f64,
    pub state: State,
    /// States since recording started, and the terrain they ran on.
    recording: Option<(Vec<State>, Arc<HeightGrid>)>,
}

impl<'m> WorldEnv<'m> {
    fn new(model: &'m Model) -> WorldEnv<'m> {
        let friction = 1.0;
        WorldEnv {
            sim: sim_for(model, friction),
            terrain: Arc::new(HeightGrid::flat()),
            friction,
            state: State::at_rest(model, V3::ZERO, vec![0.0; model.joints.len()]),
            recording: None,
        }
    }
}

fn sim_for(model: &Model, friction: f64) -> Simulation<'_, HeightGrid> {
    Simulation::new(model, SimConfig { terrain_friction: friction, ..SimConfig::default() }, crawler_generators())
}

/// A finished recording: its terrain and states.
pub type Recording = (Arc<HeightGrid>, Vec<State>);

/// Many environments of one robot.
pub struct World<'m> {
    pub model: &'m Model,
    pub envs: Vec<WorldEnv<'m>>,
    /// Time between commands [s]; each `step` runs this long in physics steps.
    pub control_dt: f64,
    threads: usize,
}

/// The state arrays of every environment, flat, environment after environment.
pub struct StateOut<'a> {
    pub pos: &'a mut [f64],
    pub quat: &'a mut [f64],
    pub vel: &'a mut [f64],
    pub ang_vel: &'a mut [f64],
    pub joint_q: &'a mut [f64],
    pub belt_v: &'a mut [f64],
    pub time: &'a mut [f64],
}

impl<'m> World<'m> {
    pub fn new(model: &'m Model, n: usize, threads: usize, control_dt: f64) -> Result<World<'m>, String> {
        if n == 0 {
            return Err("num_envs must be at least 1".into());
        }
        if !(control_dt >= SimConfig::default().dt && control_dt.is_finite()) {
            return Err(format!("control_dt must be at least the physics step, {} s", SimConfig::default().dt));
        }
        Ok(World { model, envs: (0..n).map(|_| WorldEnv::new(model)).collect(), control_dt, threads: threads.max(1) })
    }

    pub fn joints(&self) -> usize {
        self.model.joints.len()
    }

    pub fn tracks(&self) -> usize {
        self.model.tracks.len()
    }

    /// Gives the chosen environments this terrain and terrain friction. They share one copy.
    pub fn set_terrain(&mut self, chosen: &[bool], grid: HeightGrid, friction: f64) -> Result<(), String> {
        if !(friction >= 0.0 && friction.is_finite()) {
            return Err("friction must be 0 or more".into());
        }
        let grid = Arc::new(grid);
        for (e, _) in self.envs.iter_mut().zip(chosen).filter(|(_, c)| **c) {
            e.terrain = grid.clone();
            if e.friction != friction {
                e.friction = friction;
                e.sim = sim_for(self.model, friction);
            }
            // A recording that has not started moving takes the new terrain.
            if let Some((states, t)) = &mut e.recording {
                if states.len() <= 1 {
                    *t = grid.clone();
                }
            }
        }
        Ok(())
    }

    /// Puts the chosen robots at rest: chassis origin at `pos[k]`, turned by `yaw[k]` about z,
    /// joints at `joint_q[k]`. Arrays hold every environment; rows not chosen are ignored.
    /// Returns the recordings this ends (environment, recording).
    pub fn reset(&mut self, chosen: &[bool], pos: &[f64], yaw: &[f64], joint_q: &[f64]) -> Result<Vec<(usize, Recording)>, String> {
        let nj = self.joints();
        let mut done = vec![];
        for (k, e) in self.envs.iter_mut().enumerate() {
            if !chosen[k] {
                continue;
            }
            let p = &pos[3 * k..3 * k + 3];
            let q = &joint_q[nj * k..nj * (k + 1)];
            if !p.iter().chain(q).chain([&yaw[k]]).all(|x| x.is_finite()) {
                return Err(format!("environment {k}: position, yaw and joint angles must be numbers"));
            }
            let mut s = State::at_rest(self.model, v3(p[0], p[1], p[2]), q.to_vec());
            let rot = Quat::from_axis_angle(v3(0.0, 0.0, 1.0), yaw[k]);
            let origin = v3(p[0], p[1], p[2]);
            s.pose.rot = rot;
            s.pose.com = origin + rot.rot(s.pose.com_offset);
            e.state = s;
            e.sim = sim_for(self.model, e.friction);
            if let Some((states, terrain)) = e.recording.take() {
                if states.len() > 1 {
                    done.push((k, (terrain, states)));
                } else {
                    e.recording = Some((vec![e.state.clone()], e.terrain.clone()));
                }
            }
        }
        Ok(done)
    }

    /// Runs every environment for `control_dt` with belt speeds `belt` (left, right [m/s] each)
    /// and joint targets `joints` ([rad], model joint order).
    pub fn step(&mut self, belt: &[f64], joints: &[f64]) -> Result<(), String> {
        let nj = self.joints();
        if !belt.iter().chain(joints).all(|x| x.is_finite()) {
            return Err("belt speeds and joint targets must be numbers".into());
        }
        let steps = (self.control_dt / SimConfig::default().dt).round().max(1.0) as usize;
        let workers = self.threads.min(self.envs.len());
        let jobs = self.envs.iter_mut().zip(belt.chunks(2)).zip(joints.chunks(nj.max(1)).chain(std::iter::repeat(&[][..])));
        let jobs = std::sync::Mutex::new(jobs);
        std::thread::scope(|s| {
            for _ in 0..workers {
                s.spawn(|| loop {
                    let Some(((e, b), q)) = jobs.lock().unwrap().next() else { break };
                    let cmd = Command { belt_speed: [b[0], b[1]], joint_target: q[..nj].to_vec() };
                    for _ in 0..steps {
                        e.sim.step(&mut e.state, &cmd, &e.terrain);
                    }
                    if let Some((states, _)) = &mut e.recording {
                        states.push(e.state.clone());
                    }
                });
            }
        });
        Ok(())
    }

    /// Writes the state of every environment.
    pub fn state(&self, out: StateOut<'_>) {
        let (nj, nt) = (self.joints(), self.tracks());
        for (k, e) in self.envs.iter().enumerate() {
            let s = &e.state;
            let o = s.pose.to_world(V3::ZERO);
            let r = s.pose.rot;
            out.pos[3 * k..3 * k + 3].copy_from_slice(&[o.x, o.y, o.z]);
            out.quat[4 * k..4 * k + 4].copy_from_slice(&[r.w, r.x, r.y, r.z]);
            out.vel[3 * k..3 * k + 3].copy_from_slice(&[s.vel.x, s.vel.y, s.vel.z]);
            out.ang_vel[3 * k..3 * k + 3].copy_from_slice(&[s.ang_vel.x, s.ang_vel.y, s.ang_vel.z]);
            out.joint_q[nj * k..nj * (k + 1)].copy_from_slice(&s.joint_q);
            out.belt_v[nt * k..nt * (k + 1)].copy_from_slice(&s.belt_v);
            out.time[k] = s.time;
        }
    }

    /// Starts recording environment `k` from its current state (replacing any recording of it).
    pub fn record(&mut self, k: usize) -> Result<(), String> {
        let e = self.envs.get_mut(k).ok_or("no such environment")?;
        e.recording = Some((vec![e.state.clone()], e.terrain.clone()));
        Ok(())
    }

    /// Stops every recording and returns those that moved.
    pub fn stop_recording(&mut self) -> Vec<(usize, Recording)> {
        let mut done = vec![];
        for (k, e) in self.envs.iter_mut().enumerate() {
            if let Some((states, terrain)) = e.recording.take() {
                if states.len() > 1 {
                    done.push((k, (terrain, states)));
                }
            }
        }
        done
    }

    /// A recording as JSON (format in view.rs).
    pub fn recording_json(&self, rec: &Recording) -> String {
        let (terrain, states) = rec;
        let frames: Vec<String> = states.iter().map(|s| crate::view::frame(s.time, &crate::view::crawler_bodies(self.model, s))).collect();
        crate::view::recording(&crate::view::crawler_scene(self.model, terrain), &frames)
    }
}

/// Chassis origin height standing on flat ground: the largest lug tip radius, and a little air.
pub fn rest_z(m: &Model) -> f64 {
    m.tracks.iter().map(|t| t.outer_radius()).fold(0.0, f64::max) + 0.002
}

/// What a program needs to know about the robot, as JSON.
pub fn info_json(m: &Model) -> String {
    let q = |s: &str| format!("\"{}\"", s.replace('\\', "\\\\").replace('"', "\\\""));
    let side = |s: Side| if s.sign() > 0.0 { "left" } else { "right" };
    let joints: Vec<String> = m.joints.iter().map(|j| format!("{{\"name\":{},\"lower\":{},\"upper\":{},\"max_velocity\":{}}}", q(&j.name), j.lower, j.upper, j.max_velocity)).collect();
    let tracks: Vec<String> = m.tracks.iter().map(|t| format!("{{\"name\":{},\"side\":\"{}\"}}", q(&t.name), side(t.side))).collect();
    let drives: Vec<String> = m.drives.iter().map(|d| format!("{{\"name\":{},\"side\":\"{}\",\"max_force\":{}}}", q(&d.name), side(d.side), d.max_force())).collect();
    let rest_z = rest_z(m);
    format!(
        "{{\"kind\":\"crawler\",\"name\":{},\"mass\":{},\"rest_z\":{rest_z},\"joints\":[{}],\"tracks\":[{}],\"drives\":[{}]}}",
        q(&m.name),
        m.mass,
        joints.join(","),
        tracks.join(","),
        drives.join(",")
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn crawler() -> Model {
        sim_urdf::load_str(include_str!("../../../robots/simple_crawler/simple_crawler.urdf")).unwrap()
    }

    #[test]
    fn places_and_turns_the_robot() {
        let m = crawler();
        let mut w = World::new(&m, 2, 2, 0.05).unwrap();
        let yaw = std::f64::consts::FRAC_PI_2;
        w.reset(&[false, true], &[0.0, 0.0, 0.0, 1.0, 2.0, 0.3], &[0.0, yaw], &[0.0; 8]).unwrap();
        let (n, nj, nt) = (2, w.joints(), w.tracks());
        let (mut pos, mut quat, mut vel, mut av, mut jq, mut bv, mut t) = (vec![0.0; 3 * n], vec![0.0; 4 * n], vec![0.0; 3 * n], vec![0.0; 3 * n], vec![0.0; nj * n], vec![0.0; nt * n], vec![0.0; n]);
        w.state(StateOut { pos: &mut pos, quat: &mut quat, vel: &mut vel, ang_vel: &mut av, joint_q: &mut jq, belt_v: &mut bv, time: &mut t });
        assert!((pos[3] - 1.0).abs() < 1e-12 && (pos[4] - 2.0).abs() < 1e-12 && (pos[5] - 0.3).abs() < 1e-12);
        let fwd = w.envs[1].state.pose.rot.rot(v3(1.0, 0.0, 0.0));
        assert!((fwd.y - 1.0).abs() < 1e-12, "turned to face +y");
    }

    #[test]
    fn records_one_episode_from_the_next_reset() {
        let m = crawler();
        let mut w = World::new(&m, 1, 1, 0.05).unwrap();
        w.record(0).unwrap();
        // A reset before moving restarts the recording rather than ending it.
        assert!(w.reset(&[true], &[0.0, 0.0, 0.1], &[0.0], &[0.0; 4]).unwrap().is_empty());
        for _ in 0..3 {
            w.step(&[0.3, 0.3], &[0.0; 4]).unwrap();
        }
        let done = w.reset(&[true], &[0.0, 0.0, 0.1], &[0.0], &[0.0; 4]).unwrap();
        assert_eq!(done.len(), 1);
        assert_eq!(done[0].1 .1.len(), 4);
        assert!(w.recording_json(&done[0].1).starts_with("{\"scene\":"));
        assert!(w.stop_recording().is_empty(), "the recording ended at the reset");
    }
}
