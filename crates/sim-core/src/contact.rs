//! Contact points, and the trait for whatever produces them.
//!
//! Each way of computing a part of the robot against the terrain (the moving lugs, the chassis
//! boxes) is a [`ContactGenerator`]. New methods are new types; the
//! solver does not change.

use crate::math::V3;
use crate::model::Model;
use crate::path::BeltPath;
use crate::state::Pose;
use crate::terrain::Terrain;

/// One contact point, in world coordinates.
#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub point: V3,
    /// Unit direction the robot is pushed.
    pub normal: V3,
    pub depth: f64,
    /// The track the point is on, or `None` for a rigid body.
    pub track: Option<usize>,
    /// Off the tracks: the joint whose link the point is on, or `None` for the chassis.
    pub joint: Option<usize>,
    /// Direction the point moves per unit of belt speed; zero off the belts.
    pub belt_dir: V3,
    /// The terrain presses on a lug's side wall: the lug has caught an edge.
    pub caught: bool,
}

/// What a generator sees of the robot this step.
pub struct Frame<'a> {
    pub model: &'a Model,
    pub pose: &'a Pose,
    /// Each track's path in the chassis frame at the current joint angles.
    pub paths: &'a [TrackPaths],
    pub belt_q: &'a [f64],
    pub joint_q: &'a [f64],
    /// Contacts shallower than this stay on a top or tip face.
    pub skin: f64,
}

/// A track's belt path, and the path of the lug tips round it.
#[derive(Clone, Copy, Debug)]
pub struct TrackPaths {
    pub belt: BeltPath,
    pub outer: BeltPath,
}

/// Produces contact points between part of the robot and the terrain.
///
/// Implementations add points to `out` and must not remove any. Every point's `normal` is a
/// unit vector, `depth` is positive, and `belt_dir` is zero exactly when `track` is `None`.
pub trait ContactGenerator<T: Terrain>: Send {
    fn collect(&mut self, frame: &Frame<'_>, terrain: &T, out: &mut Vec<Contact>);
}

/// How far any belt surface reaches from the centre of mass at the current joint angles [m].
pub(crate) fn reach(f: &Frame<'_>) -> f64 {
    let c = f.pose.com_offset;
    let mut r: f64 = 0.0;
    for (k, t) in f.model.tracks.iter().enumerate() {
        let p = f.paths[k].outer;
        for (x, z) in [p.a, p.b] {
            let in_plane = (x - c.x).hypot(z - c.z) + p.r;
            r = r.max(in_plane.hypot(t.outer + c.y.abs()));
        }
    }
    r + 0.02
}

/// Block-column edge samples within `radius` of `centre`.
pub(crate) fn edges_near(terrain: &impl Terrain, centre: V3, radius: f64, out: &mut Vec<V3>) {
    out.clear();
    let all = terrain.edges();
    let from = all.partition_point(|e| e.x < centre.x - radius);
    let to = all.partition_point(|e| e.x <= centre.x + radius);
    for e in &all[from..to] {
        let d = *e - centre;
        if d.dot(d) < radius * radius {
            out.push(*e);
        }
    }
}
