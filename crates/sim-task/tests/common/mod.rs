//! What the tests share: terrains to cross, one traversal run, and many runs on several threads.
#![allow(dead_code)] // each test file uses only part of it

pub mod survey;
pub mod traverse;

#[allow(unused_imports)]
pub use sim_task::terrain;
