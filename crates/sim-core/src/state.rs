//! What changes while simulating, and the commands that drive it.

use crate::math::{Quat, V3};
use crate::model::Model;

/// Pose of the chassis: its centre of mass and orientation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub com: V3,
    pub rot: Quat,
    /// Centre of mass in the chassis frame, to place chassis-frame points.
    pub com_offset: V3,
}

impl Pose {
    /// A chassis-frame point in the world.
    pub fn to_world(&self, p: V3) -> V3 {
        self.com + self.rot.rot(p - self.com_offset)
    }

    /// A world point in the chassis frame.
    pub fn to_body(&self, p: V3) -> V3 {
        self.rot.inv_rot(p - self.com) + self.com_offset
    }
}

/// Everything that changes, for one robot. The model is shared; states are per environment.
#[derive(Clone, Debug, PartialEq)]
pub struct State {
    pub pose: Pose,
    pub vel: V3,
    pub ang_vel: V3,
    /// Travel of each belt along its path [m], and its speed [m/s].
    pub belt_q: Vec<f64>,
    pub belt_v: Vec<f64>,
    /// Joint angles [rad] and speeds [rad/s].
    pub joint_q: Vec<f64>,
    pub joint_v: Vec<f64>,
    /// Simulated time [s].
    pub time: f64,
}

impl State {
    /// At rest with the chassis origin at `origin`, level, at joint angles `joint_q`.
    pub fn at_rest(model: &Model, origin: V3, joint_q: Vec<f64>) -> State {
        assert_eq!(joint_q.len(), model.joints.len(), "one angle per joint");
        let com = model.mass_properties(&joint_q).com;
        let pose = Pose { com: origin + com, rot: Quat::IDENTITY, com_offset: com };
        State {
            pose,
            vel: V3::ZERO,
            ang_vel: V3::ZERO,
            belt_q: vec![0.0; model.tracks.len()],
            belt_v: vec![0.0; model.tracks.len()],
            joint_v: vec![0.0; joint_q.len()],
            joint_q,
            time: 0.0,
        }
    }
}

/// Commands for one step.
#[derive(Clone, Debug, PartialEq)]
pub struct Command {
    /// Belt speed for the left and right tracks [m/s]; positive drives forward.
    pub belt_speed: [f64; 2],
    /// Target angle for each joint [rad].
    pub joint_target: Vec<f64>,
}
