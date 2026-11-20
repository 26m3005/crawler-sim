//! What does not change while simulating: the robot's bodies, joints, tracks and colliders.
//!
//! A model is plain data. It does not know the file format it came from, nor how the tracks
//! are computed; readers build it and contact generators read it.

use crate::math::{Mat3, V3};

/// Mass, centre of mass and principal moments of one rigid part, in its link's frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertial {
    pub mass: f64,
    pub com: V3,
    /// Moments about the part's centre of mass, along the link axes [kg m^2] (products dropped).
    pub moments: V3,
}

/// Contact properties of a surface. `friction: None` leaves the terrain's friction to decide.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Surface {
    pub friction: Option<f64>,
    /// Rubber stiffness [N/m]; must be positive.
    pub stiffness: f64,
    /// Rubber damping [N s/m].
    pub damping: f64,
}

impl Default for Surface {
    fn default() -> Self {
        Surface { friction: None, stiffness: 1.0e5, damping: 1500.0 }
    }
}

impl Surface {
    /// Friction of this surface on terrain of friction `terrain`: the smaller of the two.
    pub fn friction_on(&self, terrain: f64) -> f64 {
        self.friction.map_or(terrain, |f| f.min(terrain))
    }
}

/// A revolute joint from the chassis, such as a flipper. Its axis is parallel to the body y axis.
#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    pub name: String,
    /// Position of the axis in the chassis frame.
    pub origin: V3,
    /// Unit axis in the chassis frame; positive angles turn about it.
    pub axis: V3,
    pub lower: f64,
    pub upper: f64,
    /// Servo speed limit [rad/s].
    pub max_velocity: f64,
    /// Servo torque limit [N m].
    pub max_effort: f64,
    /// The child link's mass, in the link frame (origin on the axis, chassis axes at angle 0).
    pub child: Inertial,
    /// Moment of inertia of the child link about the axis [kg m^2].
    pub inertia: f64,
    /// Servo gain: the commanded speed is gain * angle error [1/s].
    pub servo_gain: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    /// +1 for left (+y), -1 for right.
    pub fn sign(self) -> f64 {
        match self {
            Side::Left => 1.0,
            Side::Right => -1.0,
        }
    }
}

/// Lugs spaced evenly round a belt. The cross-section is a symmetric trapezoid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lugs {
    pub count: usize,
    /// Length along the belt at the root [m].
    pub base: f64,
    /// Length along the belt at the tip [m].
    pub tip: f64,
    /// Height out from the belt surface [m].
    pub height: f64,
    /// Distances from the centre plane, inner then outer edge [m].
    pub inner: f64,
    pub outer: f64,
    /// A top that slopes down towards the inner edge, or `None` for full height across.
    pub ramp: Option<Ramp>,
}

/// The lug top is full height from the outer edge in to `from`, then falls in a straight line
/// to `inner_height` at the inner edge (lugs whose top slopes down towards the inner edge of the belt).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ramp {
    /// Distance from the centre plane where the slope starts [m].
    pub from: f64,
    /// Height at the inner edge [m].
    pub inner_height: f64,
}

impl Lugs {
    /// The lug as convex pieces across its width: (inner distance, outer distance, height at
    /// the inner end, height at the outer end). One piece, or two with a ramp.
    pub fn pieces(&self) -> ([(f64, f64, f64, f64); 2], usize) {
        match self.ramp {
            None => ([(self.inner, self.outer, self.height, self.height); 2], 1),
            Some(r) => ([(self.inner, r.from, r.inner_height, self.height), (r.from, self.outer, self.height, self.height)], 2),
        }
    }
}

/// A belt round two pulleys of equal radius, in the x-z plane of its link.
#[derive(Clone, Debug, PartialEq)]
pub struct Track {
    pub name: String,
    pub side: Side,
    /// The joint carrying the track, or `None` for the chassis.
    pub joint: Option<usize>,
    /// Pulley centres (x, z) in the link frame. The link frame has the chassis orientation at angle 0.
    pub pulleys: [(f64, f64); 2],
    /// Radius of the belt surface [m].
    pub radius: f64,
    /// Distances from the centre plane, inner then outer edge of the belt [m].
    pub inner: f64,
    pub outer: f64,
    /// Mass moving with the belt [kg].
    pub belt_mass: f64,
    pub lugs: Option<Lugs>,
    /// The drive that turns this belt.
    pub drive: usize,
    pub surface: Surface,
}

/// One motor turning one or more belts together: they all run at the same speed.
#[derive(Clone, Debug, PartialEq)]
pub struct Drive {
    pub name: String,
    /// Which belt-speed command it follows.
    pub side: Side,
    pub tracks: Vec<usize>,
    /// Torque limit at the drive pulley [N m], and that pulley's outer radius [m].
    pub max_torque: f64,
    pub radius: f64,
}

impl Drive {
    /// Force limit at the belt surface [N].
    pub fn max_force(&self) -> f64 {
        self.max_torque / self.radius
    }
}

impl Track {
    /// Radius of the outermost surface: the lug tips, or the belt.
    pub fn outer_radius(&self) -> f64 {
        self.radius + self.lugs.map_or(0.0, |l| l.height)
    }
}

/// An axis-aligned box in its link's frame: the chassis, or a joint's child link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxCollider {
    pub min: V3,
    pub max: V3,
    /// The joint whose child carries the box, or `None` for the chassis.
    pub joint: Option<usize>,
}

/// A rigid plate with the outline of a track's belt, on the same link: it fills what the belts do
/// not wrap (between two tracks, or beside one) and turns with the link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plate {
    /// The track whose belt path gives the outline.
    pub track: usize,
    /// Extent across, in the link frame [m].
    pub y0: f64,
    pub y1: f64,
    /// How far inside the belt surface the outline lies [m].
    pub inset: f64,
}

/// The whole robot's mass at some joint angles, in the chassis frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    pub com: V3,
    /// Inertia tensor about `com`, chassis axes [kg m^2].
    pub inertia: Mat3,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Model {
    pub name: String,
    /// The chassis link's own mass.
    pub base: Inertial,
    /// Total mass, chassis and joint links [kg].
    pub mass: f64,
    pub joints: Vec<Joint>,
    pub tracks: Vec<Track>,
    pub drives: Vec<Drive>,
    pub boxes: Vec<BoxCollider>,
    pub plates: Vec<Plate>,
    pub chassis_surface: Surface,
}

impl Model {
    /// Orientation of joint `j`'s link in the chassis frame at angle `q`.
    pub fn link_rot(&self, j: usize, q: f64) -> Mat3 {
        Mat3::rotation(self.joints[j].axis, q)
    }

    /// A point of joint `j`'s link, in the chassis frame.
    pub fn link_point(&self, j: usize, q: f64, p: V3) -> V3 {
        self.joints[j].origin + self.link_rot(j, q).mul_v(p)
    }

    /// Centre of mass and inertia of the whole robot at joint angles `q`.
    pub fn mass_properties(&self, q: &[f64]) -> MassProperties {
        // Each part: (centre in the chassis frame, mass, its own inertia in chassis axes).
        let diag = |d: V3| Mat3([[d.x, 0.0, 0.0], [0.0, d.y, 0.0], [0.0, 0.0, d.z]]);
        let mut parts = vec![(self.base.com, self.base.mass, diag(self.base.moments))];
        for (j, joint) in self.joints.iter().enumerate() {
            let r = self.link_rot(j, q[j]);
            let c = joint.child;
            parts.push((self.link_point(j, q[j], c.com), c.mass, r.mul(&diag(c.moments)).mul(&r.transpose())));
        }
        let com = parts.iter().fold(V3::ZERO, |a, p| a + p.0 * p.1) * (1.0 / self.mass);
        let mut inertia = [[0.0; 3]; 3];
        for (p, m, own) in &parts {
            let d = *p - com;
            let d = [d.x, d.y, d.z];
            let dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            for a in 0..3 {
                for b in 0..3 {
                    let delta = if a == b { 1.0 } else { 0.0 };
                    inertia[a][b] += own.0[a][b] + m * (dd * delta - d[a] * d[b]);
                }
            }
        }
        MassProperties { com, inertia: Mat3(inertia) }
    }

    /// The track's path at the joint angles `q`: pulley centres (x, z) in the chassis frame.
    pub fn track_pulleys(&self, k: usize, q: &[f64]) -> [(f64, f64); 2] {
        let t = &self.tracks[k];
        let Some(j) = t.joint else { return t.pulleys };
        let joint = &self.joints[j];
        // A turn about +y by a takes x toward -z; the axis sign decides the direction.
        let a = q[j] * joint.axis.y;
        let (s, c) = a.sin_cos();
        let place = |p: (f64, f64)| (joint.origin.x + c * p.0 + s * p.1, joint.origin.z - s * p.0 + c * p.1);
        [place(t.pulleys[0]), place(t.pulleys[1])]
    }

    /// Checks the assumptions the engine relies on.
    pub fn validate(&self) -> Result<(), String> {
        let m = self.mass_properties(&vec![0.0; self.joints.len()]).inertia.0;
        if !(self.mass > 0.0) || !(m[0][0] > 0.0 && m[1][1] > 0.0 && m[2][2] > 0.0) {
            return Err("mass and inertia must be positive".into());
        }
        for (d, drive) in self.drives.iter().enumerate() {
            if drive.tracks.is_empty() || !(drive.max_torque > 0.0) || !(drive.radius > 0.0) {
                return Err(format!("drive {}: needs tracks and a positive torque limit", drive.name));
            }
            if drive.tracks.iter().any(|&k| k >= self.tracks.len() || self.tracks[k].drive != d) {
                return Err(format!("drive {}: its tracks do not match", drive.name));
            }
        }
        if self.boxes.iter().any(|b| b.joint.is_some_and(|j| j >= self.joints.len())) {
            return Err("a collision box names an unknown joint".into());
        }
        for p in &self.plates {
            if p.track >= self.tracks.len() || !(p.y0 < p.y1) || !(p.inset >= 0.0 && p.inset < self.tracks[p.track].radius) {
                return Err("a plate needs a track, y0 < y1 and an inset smaller than the belt radius".into());
            }
        }
        let surfaces = std::iter::once(&self.chassis_surface).chain(self.tracks.iter().map(|t| &t.surface));
        if surfaces.into_iter().any(|s| !(s.stiffness > 0.0) || !(s.damping >= 0.0)) {
            return Err("surface stiffness must be positive and damping not negative".into());
        }
        for j in &self.joints {
            if (j.axis.y.abs() - 1.0).abs() > 1e-9 {
                return Err(format!("joint {}: the axis must be along y", j.name));
            }
            if !(j.inertia > 0.0) {
                return Err(format!("joint {}: the child link needs an inertia about the axis", j.name));
            }
        }
        for (k, t) in self.tracks.iter().enumerate() {
            if t.joint.is_some_and(|j| j >= self.joints.len()) {
                return Err(format!("track {}: unknown joint", t.name));
            }
            if t.drive >= self.drives.len() || !self.drives[t.drive].tracks.contains(&k) {
                return Err(format!("track {}: no drive turns it", t.name));
            }
            if !(t.radius > 0.0) || !(t.belt_mass > 0.0) || t.inner >= t.outer {
                return Err(format!("track {}: needs a radius, a belt mass and inner < outer", t.name));
            }
            if t.pulleys[0] == t.pulleys[1] {
                return Err(format!("track {}: the pulleys must be apart", t.name));
            }
            if let Some(l) = t.lugs {
                if l.count == 0 || !(l.height > 0.0) || l.inner >= l.outer {
                    return Err(format!("track {}: bad lugs", t.name));
                }
                if let Some(r) = l.ramp {
                    // A sloping top on a trapezoid would bend the side walls; only straight walls are convex pieces.
                    if !(r.from > l.inner && r.from < l.outer) || !(r.inner_height > 0.0 && r.inner_height <= l.height) || (l.base - l.tip).abs() > 1e-12 {
                        return Err(format!("track {}: bad lug ramp (inner < from < outer, 0 < height at the inner edge <= height, and a rectangular profile)", t.name));
                    }
                }
            }
        }
        Ok(())
    }
}
