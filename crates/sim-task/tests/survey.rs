//! The traversal survey that guards the crawler's calculation, run on demand after changing it:
//!
//!   cargo test --release -p sim-task --test survey -- --ignored --nocapture
//!   SURVEY_SEEDS=6 cargo test --release ...        # quicker, about ±5 points
//!
//! Random steps, friction 1.0 / 0.7 / 0.5 / 0.3, front flippers 20 / 40 / 60 degrees, two lug
//! phases. Prints the success rates.

mod common;

use common::survey::{run_all, success_rate, survey_set};
use common::traverse::TraverseCase;

#[test]
#[ignore]
fn survey_random_steps() {
    let seeds: u64 = std::env::var("SURVEY_SEEDS").ok().and_then(|v| v.parse().ok()).unwrap_or(30);
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get());
    let model = sim_urdf::load_str(include_str!("../../../robots/simple_crawler/simple_crawler.urdf")).unwrap();
    let frictions = [1.0, 0.7, 0.5, 0.3];
    let phases = 2;
    let mut cases = vec![];
    for t in survey_set(seeds).into_iter().filter(|t| t.group().contains("random")) {
        for &friction in &frictions {
            for front in [20.0, 40.0, 60.0] {
                for p in 0..phases {
                    let mut c = TraverseCase::new(t, friction, front);
                    c.phase = p as f64 / phases as f64;
                    cases.push(c);
                }
            }
        }
    }
    eprintln!("{} runs on {threads} threads", cases.len());
    let results = run_all(&model, &cases, threads);
    let rates: Vec<String> = frictions.iter().map(|&mu| format!("{:3.0}", success_rate(&results.iter().filter(|r| r.case.friction == mu).collect::<Vec<_>>()))).collect();
    eprintln!("success rate %, by friction {frictions:?}: {}", rates.join(" "));
}
