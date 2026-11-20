//! How high a single step the sample crawler crosses without learning, run on demand:
//!
//!   cargo test --release -p sim-task --test baseline -- --ignored --nocapture
//!
//! A base line to compare learnt policies with: step heights 0.05 to 0.35 m,
//! friction 0.8 and 0.5, four lug phases, belts at 0.5 m/s, with
//! - fixed front flipper angles (0 / 20 / 40 / 60 degrees, rear at 0), and
//! - a script that knows where the edge is: front flippers up before it; past it, front down
//!   onto the top and rear down to lift the tail; flat again once over.
//! Prints, for each command and height, how many of the four phases succeeded.

mod common;

use common::traverse::{run_with, TraverseCase};
use sim_task::terrain::{TerrainSpec, START_X};

#[derive(Clone, Copy)]
enum Flippers {
    Fixed(f64),
    /// Front angle [deg] on the way to the edge.
    Script(f64),
}

impl Flippers {
    fn name(self) -> String {
        match self {
            Flippers::Fixed(a) => format!("fixed {a:.0}"),
            Flippers::Script(a) => format!("script {a:.0}"),
        }
    }

    fn at(self, x: f64) -> (f64, f64) {
        match self {
            Flippers::Fixed(a) => (a, 0.0),
            Flippers::Script(up) if x < START_X - 0.05 => (up, 0.0),
            Flippers::Script(_) if x < START_X + 0.45 => (-20.0, -45.0),
            Flippers::Script(_) => (0.0, 0.0),
        }
    }
}

#[test]
#[ignore]
fn baseline_single_step() {
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get());
    let model = sim_urdf::load_str(include_str!("../../../robots/simple_crawler/simple_crawler.urdf")).unwrap();
    let commands = [Flippers::Fixed(0.0), Flippers::Fixed(20.0), Flippers::Fixed(40.0), Flippers::Fixed(60.0), Flippers::Script(45.0), Flippers::Script(60.0)];
    let frictions = [0.8, 0.5];
    let heights: Vec<f64> = (0..13).map(|k| 0.05 + 0.025 * k as f64).collect();
    let phases = 4;

    let mut cases = vec![];
    let mut labels = vec![];
    for (c, &cmd) in commands.iter().enumerate() {
        for (f, &friction) in frictions.iter().enumerate() {
            for (h, &height) in heights.iter().enumerate() {
                for p in 0..phases {
                    let mut case = TraverseCase::new(TerrainSpec::Step { h: height }, friction, cmd.at(0.0).0);
                    case.phase = p as f64 / phases as f64;
                    cases.push(case);
                    labels.push((c, f, h));
                }
            }
        }
    }
    eprintln!("{} runs on {threads} threads", cases.len());
    let started = std::time::Instant::now();

    // Each run reads its command from the state, so this is run_all with run_with in it.
    let next = std::sync::atomic::AtomicUsize::new(0);
    let out = std::sync::Mutex::new(vec![None; cases.len()]);
    std::thread::scope(|s| {
        for _ in 0..threads {
            s.spawn(|| loop {
                let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                let Some(case) = cases.get(i) else { break };
                let cmd = commands[labels[i].0];
                let r = run_with(&model, case, &|st| cmd.at(st.pose.com.x));
                out.lock().unwrap()[i] = Some(r);
            });
        }
    });
    let results: Vec<_> = out.into_inner().unwrap().into_iter().map(Option::unwrap).collect();
    let sim_s: f64 = results.iter().map(|r| r.done.or(r.rolled).or(r.stuck).unwrap_or(r.case.seconds)).sum();
    eprintln!("{:.0} s simulated in {:.0} s", sim_s, started.elapsed().as_secs_f64());

    for (f, &friction) in frictions.iter().enumerate() {
        eprintln!("\nfriction {friction}: successes out of {phases} phases (r = rolled, s = stuck, t = tilted past 60 deg)");
        eprint!("{:>8}", "h [m]");
        for c in &commands {
            eprint!("{:>11}", c.name());
        }
        eprintln!();
        for (h, &height) in heights.iter().enumerate() {
            eprint!("{height:>8.3}");
            for c in 0..commands.len() {
                let rs: Vec<_> = results.iter().zip(&labels).filter(|(_, l)| **l == (c, f, h)).map(|(r, _)| r).collect();
                let ok = rs.iter().filter(|r| r.success()).count();
                let rolled = rs.iter().filter(|r| r.rolled.is_some()).count();
                let stuck = rs.iter().filter(|r| r.stuck.is_some()).count();
                let tilted = rs.iter().filter(|r| r.done.is_some() && !r.success()).count();
                let mut why = String::new();
                for (n, ch) in [(rolled, 'r'), (stuck, 's'), (tilted, 't')] {
                    if n > 0 {
                        why.push_str(&format!("{n}{ch}"));
                    }
                }
                eprint!("{:>11}", format!("{ok} {why}"));
            }
            eprintln!();
        }
    }
}
