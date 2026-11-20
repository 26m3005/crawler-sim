//! robot-sim.exe: runs the simulator from a settings file, or from a page in the browser.
//!
//!   robot-sim                     the GUI: a page in the browser (also by double click)
//!   robot-sim run <settings>      drive a robot over a terrain with fixed commands, and record it
//!   robot-sim info <robot>        the robot's joints, tracks and drives, as JSON
//!   robot-sim urdf <parameters>   the robot crawler parameters make, as extended URDF

mod gui;
mod run;
mod teleop;

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;

use sim_task::robot;
use sim_task::settings::Settings;
use sim_task::world::info_json;

const USAGE: &str = "usage:
  robot-sim                     the GUI: a page in the browser (also by double click)
  robot-sim run <settings>      drive a robot over a terrain with fixed commands, and record it
  robot-sim info <robot>        the robot's joints, tracks and drives, as JSON
  robot-sim urdf <parameters>   the robot crawler parameters make, as extended URDF

A robot is extended URDF (.urdf) or crawler parameters (any other file, README.md).

Settings of `run` (paths are relative to the settings file):
  robot      = robots/simple_crawler/simple_crawler.urdf    (required; or a parameter file)
  terrain    = course        a terrain and its keys, such as `stairs rise=0.1 steps=4`
  friction   = 0.8
  seconds    = 90            stop then, or when the robot passes the end of the terrain
  belt       = 0.5, 0.5      left and right belt speeds [m/s]
  joints     = 0, 0, 0, 0    joint targets [deg], in the robot's joint order
  control_dt = 0.05          time between commands [s]
  record     = run.json      write a recording for viewer/viewer.html (not set: none)";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.iter().map(String::as_str).collect::<Vec<_>>()[..] {
        [] => return windowed(gui::serve()),
        ["run", settings] => run(settings),
        ["info", robot] => robot::load(Path::new(robot)).map(|m| println!("{}", info_json(&m))),
        ["urdf", params] => Settings::read(params).and_then(|s| robot::urdf(&s)).map(|u| print!("{u}")),
        // A settings file dropped on the exe: its window closes at the end, so wait for Enter.
        [settings] if !matches!(settings, "run" | "info" | "urdf" | "help" | "-h" | "--help") => return windowed(run(settings)),
        _ => {
            eprintln!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Shows how it went and waits for Enter, so the window does not close before it is read.
fn windowed(result: Result<(), String>) -> ExitCode {
    let code = match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    };
    println!("\nPress Enter to close.");
    let _ = std::io::stdin().read_line(&mut String::new());
    code
}

/// A path in a settings file, relative to the file.
fn beside(settings: &str, path: &str) -> PathBuf {
    Path::new(settings).parent().unwrap_or(Path::new("")).join(path)
}

fn run(path: &str) -> Result<(), String> {
    let s = Settings::read(path)?;
    let model = robot::load(&beside(path, s.required("robot")?))?;
    let rs = run::RunSettings::read(&s, &model).map_err(|e| format!("{path}: {e}"))?;
    let record = s.text("record").map(|r| beside(path, r));
    s.finish()?;
    let mut k = 0;
    let done = run::drive(&model, &rs, record.is_some(), &AtomicBool::new(false), &mut |p| {
        if k % 100 == 0 {
            println!("t {:5.1} s  x {:5.2} m  z {:4.2} m", p.time, p.x, p.z);
        }
        k += 1;
    })?;
    println!("t {:5.1} s  x {:5.2} m (end of the terrain {:.1} m), {:.1} x real time", done.at.time, done.at.x, done.goal, done.speed);
    if let Some(out) = record {
        let json = done.recording.ok_or("nothing to record: the run did not move")?;
        std::fs::write(&out, json).map_err(|e| format!("{}: {e}", out.display()))?;
        println!("recorded {}", out.display());
    }
    Ok(())
}
