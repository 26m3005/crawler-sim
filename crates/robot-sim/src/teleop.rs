//! Driving a robot by hand (teleoperation): the physics runs in real time on its own thread,
//! takes the newest command the page sent, and keeps every frame for the page to show and for
//! a recording at the end.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sim_core::model::Model;
use sim_core::state::State;
use sim_core::terrain::HeightGrid;
use sim_task::view;

use crate::run::RunSettings;

/// With no command for this long the page is taken to be gone, and the drive ends.
const SILENT: Duration = Duration::from_secs(3);

struct Shared {
    belt: [f64; 2],
    joints: Vec<f64>,
    heard: Instant,
    /// Every frame so far, as the viewer reads them.
    frames: Vec<String>,
    last: State,
    terrain: Arc<HeightGrid>,
    running: bool,
    error: Option<String>,
}

pub struct Teleop {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    model: Model,
    goal: f64,
}

impl Teleop {
    /// Starts at rest at the terrain's start, joints as `rs` says, belts still.
    pub fn start(model: Model, rs: RunSettings) -> Result<Teleop, String> {
        let (s0, terrain) = {
            let w = rs.world(&model)?;
            (w.envs[0].state.clone(), w.envs[0].terrain.clone())
        };
        let shared = Arc::new(Mutex::new(Shared {
            belt: [0.0, 0.0],
            joints: rs.joints.clone(),
            heard: Instant::now(),
            frames: vec![view::frame(s0.time, &view::crawler_bodies(&model, &s0))],
            last: s0,
            terrain,
            running: true,
            error: None,
        }));
        let stop = Arc::new(AtomicBool::new(false));
        let goal = rs.terrain.goal_x();
        let (sh, st, m, dt, seconds) = (shared.clone(), stop.clone(), model.clone(), rs.control_dt, rs.seconds);
        std::thread::spawn(move || {
            // The same world again, here: it borrows the thread's own copy of the robot.
            let mut world = match rs.world(&m) {
                Ok(w) => w,
                Err(e) => {
                    let mut s = sh.lock().unwrap();
                    s.running = false;
                    s.error = Some(e);
                    return;
                }
            };
            let started = Instant::now();
            let mut k: u32 = 0;
            let error = loop {
                // Keep to real time: step k is due at k * dt after the start.
                k += 1;
                let due = started + Duration::from_secs_f64(k as f64 * dt);
                if let Some(wait) = due.checked_duration_since(Instant::now()) {
                    std::thread::sleep(wait);
                }
                let (belt, joints) = {
                    let s = sh.lock().unwrap();
                    if st.load(Ordering::Relaxed) || s.heard.elapsed() > SILENT || k as f64 * dt > seconds {
                        break None;
                    }
                    (s.belt, s.joints.clone())
                };
                if let Err(e) = world.step(&belt, &joints) {
                    break Some(e);
                }
                let state = world.envs[0].state.clone();
                let frame = view::frame(state.time, &view::crawler_bodies(&m, &state));
                let mut s = sh.lock().unwrap();
                s.frames.push(frame);
                s.last = state;
            };
            let mut s = sh.lock().unwrap();
            s.running = false;
            s.error = error;
        });
        Ok(Teleop { shared, stop, goal, model })
    }

    /// The newest command: belt speeds (left, right) and joint targets [rad].
    pub fn command(&self, belt: [f64; 2], joints: Vec<f64>) -> Result<(), String> {
        let mut s = self.shared.lock().unwrap();
        if joints.len() != s.joints.len() || !belt.iter().chain(&joints).all(|x| x.is_finite()) {
            return Err(format!("a command is 2 belt speeds and {} joint angles", s.joints.len()));
        }
        s.belt = belt;
        s.joints = joints;
        s.heard = Instant::now();
        Ok(())
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// The frames from number `from` on, and where the robot is, as JSON.
    pub fn frames(&self, from: usize) -> String {
        let s = self.shared.lock().unwrap();
        let st = &s.last;
        let o = st.pose.to_world(Default::default());
        let error = s.error.as_ref().map_or("null".into(), |e| crate::gui::json_str(e));
        format!(
            "{{\"running\":{},\"error\":{error},\"next\":{},\"t\":{},\"x\":{},\"z\":{},\"tilt_deg\":{},\"goal\":{},\"frames\":[{}]}}",
            s.running,
            s.frames.len(),
            st.time,
            o.x,
            o.z,
            st.pose.rot.tilt().to_degrees(),
            self.goal,
            s.frames.get(from.min(s.frames.len())..).unwrap_or_default().join(",")
        )
    }

    /// Everything driven so far, as a recording.
    pub fn recording(&self) -> String {
        let s = self.shared.lock().unwrap();
        view::recording(&view::crawler_scene(&self.model, &s.terrain), &s.frames)
    }
}

impl Drop for Teleop {
    fn drop(&mut self) {
        self.stop();
    }
}
