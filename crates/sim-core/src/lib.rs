//! The physics engine: model, state, contacts and the solver.
//!
//! It knows nothing of displays, learning, tasks or particular robots. Robots are data
//! ([`model::Model`]); ways of computing contacts are [`contact::ContactGenerator`]s.

pub mod chassis;
pub mod collide;
pub mod contact;
pub mod lugs;
pub mod math;
pub mod model;
pub mod path;
pub mod sim;
pub mod solver;
pub mod state;
pub mod terrain;

use contact::ContactGenerator;
use terrain::Terrain;

/// The contact generators for a crawler: the chassis boxes, then the tracks as rigid lugs.
pub fn crawler_generators<'m, T: Terrain + 'm>() -> Vec<Box<dyn ContactGenerator<T> + 'm>> {
    vec![Box::new(chassis::ChassisBoxes::new()), Box::new(lugs::MovingLugs::new())]
}
