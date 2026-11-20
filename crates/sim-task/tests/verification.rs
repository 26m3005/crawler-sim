//! Checks of the physics against simple theory and earlier trials, kept as tests.
//! Run with `cargo test --release`; each test runs real simulations for a few seconds.

mod common;

use sim_core::math::v3;
use sim_core::model::Model;
use sim_core::sim::{SimConfig, Simulation};
use sim_core::state::{Command, State};
use sim_core::terrain::HeightGrid;
use sim_core::crawler_generators;
use common::survey::run_all;
use common::terrain::{TerrainSpec, START_X};
use common::traverse::{run, TraverseCase};

fn crawler() -> Model {
    sim_urdf::load_str(include_str!("../../../robots/simple_crawler/simple_crawler.urdf")).unwrap()
}

fn flipper_angles(model: &Model, front_deg: f64, rear_deg: f64) -> Vec<f64> {
    model.joints.iter().map(|j| if j.origin.x >= 0.0 { front_deg } else { rear_deg }.to_radians()).collect()
}

/// At rest on flat ground, the ground carries the robot's weight: 20 kg x 9.81 = 196 N.
#[test]
fn weight_is_carried_at_rest() {
    let model = crawler();
    let ground = HeightGrid::flat();
    let mut sim = Simulation::new(&model, SimConfig::default(), crawler_generators());
    let q = flipper_angles(&model, 40.0, 0.0);
    let mut state = State::at_rest(&model, v3(0.0, 0.0, 0.077), q.clone());
    let cmd = Command { belt_speed: [0.0; 2], joint_target: q };
    let mut z = vec![];
    for step in 0..5000 {
        let r = sim.step(&mut state, &cmd, &ground);
        if step >= 3000 {
            z.push(r.track_force.z + r.chassis_force.z);
        }
    }
    let mean = z.iter().sum::<f64>() / z.len() as f64;
    assert!((mean - model.mass * 9.81).abs() < 2.0, "vertical force {mean:.1} N");
    let drift = (state.pose.com.x.powi(2) + state.pose.com.y.powi(2)).sqrt();
    assert!(drift < 0.005, "drifted {drift:.4} m at rest");
}

/// A smooth slope (gravity tilted on flat ground) needs friction tan(angle): 0.36 at 20 deg,
/// 0.58 at 30 deg. With friction 0.5 the robot climbs the first and not the second.
#[test]
fn smooth_slope_follows_friction_theory() {
    let model = crawler();
    let mut case = TraverseCase::new(TerrainSpec::Flat, 0.5, 40.0);
    case.tilt_deg = 20.0;
    assert!(run(&model, &case).success(), "should climb 20 deg with friction 0.5");
    case.tilt_deg = 30.0;
    case.seconds = 12.0;
    let r = run(&model, &case);
    assert!(r.done.is_none() && r.final_x < 1.0, "climbed 30 deg with friction 0.5 (x {:.2})", r.final_x);
}

/// The same theory on a slope in the terrain, from level ground: friction 0.5 holds up to
/// atan 0.5 = 26.6 deg. The slope has no edge for a lug to catch, so only friction is left.
/// (`caught_s` is not 0 here: while the robot tips onto the slope, the raised front flippers'
/// lugs land on it tip first, pushing along the belt, which the flag also counts.)
#[test]
fn terrain_slope_follows_friction_theory() {
    let model = crawler();
    let up = |deg: f64, seconds: f64| {
        let mut case = TraverseCase::new(TerrainSpec::Slope { deg }, 0.5, 40.0);
        case.seconds = seconds;
        run(&model, &case)
    };
    let r = up(20.0, 30.0);
    assert!(r.success(), "should climb a 20 deg slope with friction 0.5 (x {:.2})", r.final_x);
    assert!((r.max_tilt_deg - 20.0).abs() < 1.0, "should lie on the slope (tilt {:.1})", r.max_tilt_deg);
    let r = up(30.0, 16.0);
    assert!(r.done.is_none() && r.final_x < START_X + 1.0, "climbed a 30 deg slope with friction 0.5 (x {:.2})", r.final_x);
}

/// The lugs' shape is what gets the robot over steps: random steps 0.25 m, up to 0.10 m,
/// seed 1, friction 0.3, front flippers 40 deg. With its lugs the crawler gets through, catching
/// block edges on the way; the same crawler with smooth belts (no `<lugs>`) stops near the start.
#[test]
fn lugs_climb_where_smooth_belts_stop() {
    let model = crawler();
    let terrain = TerrainSpec::Random { cell: 0.25, hmax: 0.10, seed: 1 };
    let case = TraverseCase::new(terrain, 0.3, 40.0);
    let lugs = run(&model, &case);
    let done = lugs.done.expect("the lugs should get through");
    assert!((done - 12.66).abs() < 0.3, "lugs took {done:.2} s");
    assert!(lugs.caught_s > 0.0, "the lugs should catch block edges on the way");
    let mut smooth = model.clone();
    for t in &mut smooth.tracks {
        t.lugs = None;
    }
    let belt = run(&smooth, &case);
    assert!(belt.done.is_none() && belt.final_x < 1.2, "smooth belts got to x {:.2}", belt.final_x);
    assert_eq!(belt.caught_s, 0.0, "smooth belts have no lug to catch");
}

/// The same input gives the same result, on one thread or several.
#[test]
fn deterministic_across_threads() {
    let model = crawler();
    let mut cases = vec![];
    for seed in 1..=3 {
        let mut c = TraverseCase::new(TerrainSpec::Random { cell: 0.25, hmax: 0.20, seed }, 0.5, 40.0);
        c.seconds = 4.0;
        cases.push(c);
    }
    let strip = |v: Vec<common::traverse::TraverseResult>| v.into_iter().map(|r| (r.final_x, r.final_y, r.max_tilt_deg, r.caught_s)).collect::<Vec<_>>();
    let one = strip(run_all(&model, &cases, 1));
    let again = strip(run_all(&model, &cases, 1));
    let many = strip(run_all(&model, &cases, 3));
    assert_eq!(one, again);
    assert_eq!(one, many);
}

/// Pivot turn: left belts back, right belts forward. The robot turns on the spot, to the left.
#[test]
fn pivot_turn_on_the_spot() {
    let model = crawler();
    let ground = HeightGrid::flat();
    let mut sim = Simulation::new(&model, SimConfig::default(), crawler_generators());
    let q = flipper_angles(&model, 40.0, 40.0);
    let mut state = State::at_rest(&model, v3(0.0, 0.0, 0.077), q.clone());
    let mut cmd = Command { belt_speed: [0.0; 2], joint_target: q };
    let yaw = |s: &State| {
        let a = s.pose.rot.rot(v3(1.0, 0.0, 0.0));
        a.y.atan2(a.x)
    };
    let (mut turned, mut last, mut start) = (0.0, 0.0, 0.0);
    for step in 0..8000 {
        let t = step as f64 * 0.001;
        let v = 0.5 * t.min(1.0);
        cmd.belt_speed = [-v, v];
        sim.step(&mut state, &cmd, &ground);
        let a = yaw(&state);
        let mut d = a - last;
        if d > std::f64::consts::PI {
            d -= std::f64::consts::TAU;
        }
        if d < -std::f64::consts::PI {
            d += std::f64::consts::TAU;
        }
        turned += d;
        last = a;
        if step == 3000 {
            start = turned;
        }
    }
    let rate = (turned - start).to_degrees() / 5.0;
    let drift = (state.pose.com.x.powi(2) + state.pose.com.y.powi(2)).sqrt();
    assert!(rate > 10.0, "turned at {rate:.1} deg/s");
    assert!(drift < 0.3, "drifted {drift:.2} m");
    eprintln!("pivot: {rate:.1} deg/s, drift {drift:.3} m");
}

/// A 0.1 m box of 0.5 kg on flat ground with friction 0.5 slides when the slope passes
/// atan(0.5) = 26.6 degrees (the slope is gravity tilted forward).
#[test]
fn box_slides_at_the_friction_angle() {
    use sim_core::math::{v3, V3};
    use sim_core::model::{BoxCollider, Inertial, Surface};
    let i = 0.5 * (0.1f64 * 0.1 + 0.1 * 0.1) / 12.0;
    let cube = Model {
        name: "box".into(),
        base: Inertial { mass: 0.5, com: V3::ZERO, moments: v3(i, i, i) },
        mass: 0.5,
        joints: vec![],
        tracks: vec![],
        drives: vec![],
        boxes: vec![BoxCollider { min: v3(-0.05, -0.05, -0.05), max: v3(0.05, 0.05, 0.05), joint: None }],
        plates: vec![],
        chassis_surface: Surface::default(),
    };
    let ground = HeightGrid::flat();
    let slid = |deg: f64| {
        let t = deg.to_radians();
        let config = SimConfig { terrain_friction: 0.5, gravity: v3(-9.81 * t.sin(), 0.0, -9.81 * t.cos()), ..SimConfig::default() };
        let mut sim = Simulation::new(&cube, config, crawler_generators());
        let mut s = State::at_rest(&cube, v3(0.0, 0.0, 0.05), vec![]);
        let cmd = Command { belt_speed: [0.0; 2], joint_target: vec![] };
        for _ in 0..2000 {
            sim.step(&mut s, &cmd, &ground);
        }
        -s.pose.com.x
    };
    let (below, above) = (slid(25.0), slid(28.0));
    assert!(below < 0.002, "slid {below:.4} m at 25 degrees");
    assert!(above > 0.05, "only {above:.4} m at 28 degrees");
}

/// In the air without gravity, swinging the front flippers up keeps the centre of mass still,
/// turns the body the other way, and keeps the angular momentum (body plus flipper spin) at 0.
#[test]
fn flipper_swing_keeps_centre_of_mass_and_momentum() {
    let model = crawler();
    let config = SimConfig { gravity: v3(0.0, 0.0, 0.0), ..SimConfig::default() };
    let mut sim = Simulation::new(&model, config, crawler_generators());
    let mut state = State::at_rest(&model, v3(0.0, 0.0, 5.0), vec![0.0; 4]);
    let start = state.pose.com;
    let cmd = Command { belt_speed: [0.0; 2], joint_target: flipper_angles(&model, 90.0, 0.0) };
    let mut worst: f64 = 0.0;
    for _ in 0..3000 {
        sim.step(&mut state, &cmd, &HeightGrid::flat());
        let r = state.pose.rot.to_mat3();
        let body = r.mul(&model.mass_properties(&state.joint_q).inertia).mul(&r.transpose()).mul_v(state.ang_vel);
        let spin = model.joints.iter().enumerate().fold(body, |l, (j, jt)| l + state.pose.rot.rot(jt.axis) * (jt.inertia * state.joint_v[j]));
        worst = worst.max(spin.len());
    }
    let moved = (state.pose.com - start).len();
    let pitch = state.pose.rot.tilt().to_degrees();
    eprintln!("front flippers at {:.1} deg, centre of mass moved {moved:.2e} m, body pitched {pitch:.2} deg, largest angular momentum {worst:.2e}", state.joint_q[0].to_degrees());
    assert!(state.joint_q[0] > 1.5, "the flippers did not rise: {:.2}", state.joint_q[0]);
    assert!(moved < 1e-9, "the centre of mass moved {moved:.2e} m");
    assert!(pitch > 0.5, "the body did not turn the other way ({pitch:.2} deg)");
    assert!(worst < 0.02 * 0.05, "angular momentum reached {worst:.2e}");
}

/// One motor per side: the main belt and both flipper belts of a side always run together.
#[test]
fn belts_on_a_side_run_together() {
    let model = crawler();
    let mut sim = Simulation::new(&model, SimConfig::default(), crawler_generators());
    let grid = TerrainSpec::Random { cell: 0.25, hmax: 0.1, seed: 2 }.build();
    let mut state = State::at_rest(&model, v3(0.0, 0.0, 0.2), flipper_angles(&model, 40.0, 0.0));
    let cmd = Command { belt_speed: [0.5, 0.3], joint_target: flipper_angles(&model, 40.0, 0.0) };
    for _ in 0..3000 {
        sim.step(&mut state, &cmd, &grid);
        for d in &model.drives {
            let v0 = state.belt_v[d.tracks[0]];
            assert!(d.tracks.iter().all(|&k| state.belt_v[k] == v0), "{}: {:?}", d.name, state.belt_v);
        }
    }
}
