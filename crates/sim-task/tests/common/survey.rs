//! Many runs at once, on several threads. Results come back in the order of the cases.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

use sim_core::model::Model;
use sim_task::terrain::TerrainSpec;

use super::traverse::{run, TraverseCase, TraverseResult};

pub fn run_all(model: &Model, cases: &[TraverseCase], threads: usize) -> Vec<TraverseResult> {
    let next = AtomicUsize::new(0);
    let out = Mutex::new(Vec::with_capacity(cases.len()));
    std::thread::scope(|s| {
        for _ in 0..threads.max(1) {
            s.spawn(|| loop {
                let i = next.fetch_add(1, Ordering::Relaxed);
                let Some(case) = cases.get(i) else { break };
                let r = run(model, case);
                out.lock().unwrap().push((i, r));
            });
        }
    });
    let mut v = out.into_inner().unwrap();
    v.sort_by_key(|x| x.0);
    v.into_iter().map(|x| x.1).collect()
}

/// Success rate in percent.
pub fn success_rate(results: &[&TraverseResult]) -> f64 {
    if results.is_empty() {
        return 0.0;
    }
    100.0 * results.iter().filter(|r| r.success()).count() as f64 / results.len() as f64
}

/// The terrains of the verification of 2026-11-03: random steps with `seeds` seeds each, then the rest.
pub fn survey_set(seeds: u64) -> Vec<TerrainSpec> {
    let mut v = vec![];
    for (cell, hs) in [(0.10, [0.05, 0.10]), (0.25, [0.10, 0.20]), (0.50, [0.20, 0.30])] {
        for hmax in hs {
            for seed in 1..=seeds {
                v.push(TerrainSpec::Random { cell, hmax, seed });
            }
        }
    }
    let stairs = |rise, tread, skew_deg| TerrainSpec::Stairs { rise, tread, steps: 8, skew_deg };
    v.extend([
        stairs(0.15, 0.30, 0.0),
        stairs(0.19, 0.243, 0.0),
        stairs(0.19, 0.197, 0.0),
        stairs(0.15, 0.30, 30.0),
        TerrainSpec::Step { h: 0.20 },
        TerrainSpec::Step { h: 0.30 },
        TerrainSpec::Curb { h: 0.15 },
        TerrainSpec::Curb { h: 0.25 },
        TerrainSpec::Rails { h: 0.05, every: 3 },
        TerrainSpec::Rails { h: 0.10, every: 3 },
        TerrainSpec::Holes { h: 0.15, cell: 0.25 },
        TerrainSpec::Slope { deg: 20.0 },
        TerrainSpec::Slope { deg: 30.0 },
        TerrainSpec::Slope { deg: 40.0 },
    ]);
    v
}
