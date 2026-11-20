//! The GUI: a small web server on this computer only (127.0.0.1) and a page in the browser.
//!
//! The server touches no files: the page sends the robot and the settings as text, and gets
//! robots, previews and recordings back; opening and saving files is the browser's. Every request
//! needs the token made at start (in the address the browser is sent to), so other pages and
//! programs cannot drive it, and a Host other than this address is refused.

use std::collections::hash_map::RandomState;
use std::hash::{BuildHasher, Hasher};
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use sim_core::model::Model;
use sim_task::robot;
use sim_task::settings::Settings;
use sim_task::world::info_json;

use crate::run::{self, Progress, RunSettings};
use crate::teleop::Teleop;

const PAGE: &str = include_str!("gui.html");
const VIEWER: &str = include_str!("../../../viewer/viewer.html");
const SAMPLE: &str = include_str!("../../../robots/simple_crawler/simple_crawler.txt");
const MAX_BODY: usize = 1 << 20;

#[derive(Default)]
struct Job {
    running: bool,
    progress: Progress,
    goal: f64,
    /// The summary and the recording, or what went wrong.
    result: Option<Result<(String, String), String>>,
}

struct Request {
    method: String,
    path: String,
    query: String,
    host: String,
    token: String,
    body: String,
}

/// Runs the server until the page says quit or the window is closed.
pub fn serve() -> Result<(), String> {
    let listener = TcpListener::bind("127.0.0.1:0").map_err(|e| format!("cannot open the GUI's port: {e}"))?;
    let port = listener.local_addr().map_err(|e| e.to_string())?.port();
    let token = new_token();
    let url = format!("http://127.0.0.1:{port}/?t={token}");
    println!("robot-sim GUI: {url}");
    println!("The page opens in your browser. Closing this window ends robot-sim.");
    let _ = std::process::Command::new("cmd").args(["/C", "start", "", &url]).spawn();

    let mut model: Model = robot::load_text(SAMPLE, "sample")?;
    let job = Arc::new(Mutex::new(Job::default()));
    let stop = Arc::new(AtomicBool::new(false));
    let mut teleop: Option<Teleop> = None;
    for stream in listener.incoming() {
        let Ok(mut stream) = stream else { continue };
        let Some(req) = read_request(&mut stream) else { continue };
        let host_ok = req.host == format!("127.0.0.1:{port}") || req.host == format!("localhost:{port}");
        if !host_ok || (req.token != token && !req.query.split('&').any(|kv| kv == format!("t={token}"))) {
            respond(&mut stream, 403, "text/plain", b"forbidden");
            continue;
        }
        let text = |s: &mut TcpStream, r: Result<String, String>, kind: &str| match r {
            Ok(body) => respond(s, 200, kind, body.as_bytes()),
            Err(e) => respond(s, 400, "text/plain; charset=utf-8", e.as_bytes()),
        };
        match (req.method.as_str(), req.path.as_str()) {
            ("GET", "/") => respond(&mut stream, 200, "text/html; charset=utf-8", PAGE.as_bytes()),
            ("GET", "/viewer.html") => respond(&mut stream, 200, "text/html; charset=utf-8", VIEWER.as_bytes()),
            ("GET", "/api/sample") => respond(&mut stream, 200, "text/plain; charset=utf-8", SAMPLE.as_bytes()),
            ("POST", "/api/robot") => {
                // First line `urdf` or `params`, then the robot file.
                let (kind, body) = req.body.split_once('\n').unwrap_or((&req.body, ""));
                let loaded = match kind.trim() {
                    "urdf" => sim_urdf::load_str(body),
                    _ => robot::load_text(body, "parameters"),
                };
                let r = loaded.map(|m| {
                    let info = info_json(&m);
                    model = m;
                    info
                });
                text(&mut stream, r, "application/json");
            }
            ("POST", "/api/preview") => {
                let r = settings(&req.body, &model).and_then(|rs| rs.still(&model));
                text(&mut stream, r, "application/json");
            }
            ("POST", "/api/run") => {
                let r = settings(&req.body, &model).and_then(|rs| start(&job, &stop, model.clone(), rs));
                text(&mut stream, r.map(|()| "started".into()), "text/plain");
            }
            ("POST", "/api/stop") => {
                stop.store(true, Ordering::Relaxed);
                respond(&mut stream, 200, "text/plain", b"stopping");
            }
            ("GET", "/api/status") => {
                let j = job.lock().unwrap();
                let (error, summary) = match &j.result {
                    Some(Err(e)) => (json_str(e), "null".into()),
                    Some(Ok((s, _))) => ("null".into(), json_str(s)),
                    None => ("null".into(), "null".into()),
                };
                let p = j.progress;
                let body = format!("{{\"running\":{},\"time\":{},\"x\":{},\"goal\":{},\"error\":{error},\"summary\":{summary}}}", j.running, p.time, p.x, j.goal);
                respond(&mut stream, 200, "application/json", body.as_bytes());
            }
            ("GET", "/api/result") => {
                let j = job.lock().unwrap();
                match &j.result {
                    Some(Ok((_, rec))) => respond(&mut stream, 200, "application/json", rec.as_bytes()),
                    _ => respond(&mut stream, 404, "text/plain", b"no recording yet"),
                }
            }
            // Driving by hand: start (the settings), commands, new frames, stop, the recording.
            ("POST", "/api/teleop/start") => {
                let r = settings(&req.body, &model).and_then(|rs| Teleop::start(model.clone(), rs));
                let r = r.map(|t| {
                    teleop = Some(t); // the drive before it, if any, ends as it is dropped
                    "started".to_string()
                });
                text(&mut stream, r, "text/plain");
            }
            ("POST", "/api/teleop/command") => {
                let r = match (&teleop, numbers(&req.body)) {
                    (Some(t), Ok(v)) if v.len() >= 2 => t.command([v[0], v[1]], v[2..].iter().map(|d| d.to_radians()).collect()),
                    (None, _) => Err("not driving".into()),
                    (_, Err(e)) => Err(e),
                    _ => Err("a command is: left right joint angles [deg]...".into()),
                };
                text(&mut stream, r.map(|()| "ok".into()), "text/plain");
            }
            ("GET", "/api/teleop/frames") => {
                let from = req.query.split('&').find_map(|kv| kv.strip_prefix("from=")).and_then(|v| v.parse().ok()).unwrap_or(0);
                let r = teleop.as_ref().map(|t| t.frames(from)).ok_or_else(|| "not driving".to_string());
                text(&mut stream, r, "application/json");
            }
            ("POST", "/api/teleop/stop") => {
                if let Some(t) = &teleop {
                    t.stop();
                }
                respond(&mut stream, 200, "text/plain", b"stopping");
            }
            ("GET", "/api/teleop/recording") => {
                let r = teleop.as_ref().map(|t| t.recording()).ok_or_else(|| "nothing driven".to_string());
                text(&mut stream, r, "application/json");
            }
            ("POST", "/api/quit") => {
                respond(&mut stream, 200, "text/plain", b"bye");
                println!("Ended from the page.");
                std::process::exit(0);
            }
            _ => respond(&mut stream, 404, "text/plain", b"not found"),
        }
    }
    Ok(())
}

/// Numbers separated by spaces.
fn numbers(text: &str) -> Result<Vec<f64>, String> {
    text.split_whitespace().map(|w| w.parse::<f64>().ok().filter(|x| x.is_finite()).ok_or_else(|| format!("`{w}` is not a number"))).collect()
}

/// Run settings from the page (no `robot` or `record`: the robot is the one sent last).
fn settings(text: &str, model: &Model) -> Result<RunSettings, String> {
    let s = Settings::parse("settings", text)?;
    let rs = RunSettings::read(&s, model)?;
    s.finish()?;
    Ok(rs)
}

fn start(job: &Arc<Mutex<Job>>, stop: &Arc<AtomicBool>, model: Model, rs: RunSettings) -> Result<(), String> {
    {
        let mut j = job.lock().unwrap();
        if j.running {
            return Err("a run is going already".into());
        }
        *j = Job { running: true, goal: rs.terrain.goal_x(), ..Job::default() };
    }
    stop.store(false, Ordering::Relaxed);
    let (job, stop) = (job.clone(), stop.clone());
    std::thread::spawn(move || {
        let r = run::drive(&model, &rs, true, &stop, &mut |p| job.lock().unwrap().progress = p);
        let r = r.and_then(|f| {
            let through = if f.at.x > f.goal { "through" } else { "not through" };
            let summary = format!("t {:.1} s, x {:.2} m of {:.1} m ({through}), {:.1} x real time", f.at.time, f.at.x, f.goal, f.speed);
            Ok((summary, f.recording.ok_or("nothing recorded: the run did not move")?))
        });
        let mut j = job.lock().unwrap();
        j.running = false;
        j.result = Some(r);
    });
    Ok(())
}

/// 128 random bits as hex, from the standard library's randomly seeded hasher.
fn new_token() -> String {
    let part = |k: u64| {
        let mut h = RandomState::new().build_hasher();
        h.write_u64(k ^ std::process::id() as u64);
        h.finish()
    };
    format!("{:016x}{:016x}", part(1), part(2))
}

fn read_request(stream: &mut TcpStream) -> Option<Request> {
    let mut r = BufReader::new(stream.try_clone().ok()?);
    let mut line = String::new();
    r.read_line(&mut line).ok()?;
    let mut words = line.split_whitespace();
    let (method, target) = (words.next()?.to_string(), words.next()?.to_string());
    let (path, query) = target.split_once('?').map_or((target.clone(), String::new()), |(p, q)| (p.to_string(), q.to_string()));
    let (mut host, mut token, mut length) = (String::new(), String::new(), 0usize);
    loop {
        let mut h = String::new();
        if r.read_line(&mut h).ok()? == 0 || h.trim().is_empty() {
            break;
        }
        let (k, v) = h.split_once(':')?;
        match k.trim().to_ascii_lowercase().as_str() {
            "host" => host = v.trim().to_string(),
            "x-token" => token = v.trim().to_string(),
            "content-length" => length = v.trim().parse().ok()?,
            _ => {}
        }
    }
    if length > MAX_BODY {
        return None;
    }
    let mut body = vec![0; length];
    r.read_exact(&mut body).ok()?;
    Some(Request { method, path, query, host, token, body: String::from_utf8(body).ok()? })
}

fn respond(stream: &mut TcpStream, status: u16, kind: &str, body: &[u8]) {
    let reason = match status {
        200 => "OK",
        400 => "Bad Request",
        403 => "Forbidden",
        _ => "Not Found",
    };
    let head = format!("HTTP/1.1 {status} {reason}\r\nContent-Type: {kind}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\nConnection: close\r\n\r\n", body.len());
    let _ = stream.write_all(head.as_bytes()).and_then(|_| stream.write_all(body));
}

pub(crate) fn json_str(s: &str) -> String {
    let mut o = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => o += "\\\"",
            '\\' => o += "\\\\",
            '\n' => o += "\\n",
            c if (c as u32) < 0x20 => o += &format!("\\u{:04x}", c as u32),
            c => o.push(c),
        }
    }
    o + "\""
}
