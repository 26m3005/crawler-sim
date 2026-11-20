//! Vectors and rotations. Right-handed, z up, metres and radians.

use std::ops::{Add, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub const fn v3(x: f64, y: f64, z: f64) -> V3 {
    V3 { x, y, z }
}

impl V3 {
    pub const ZERO: V3 = v3(0.0, 0.0, 0.0);

    pub fn dot(self, o: V3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: V3) -> V3 {
        v3(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }

    pub fn len(self) -> f64 {
        self.dot(self).sqrt()
    }
}

impl Add for V3 {
    type Output = V3;
    fn add(self, o: V3) -> V3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for V3 {
    type Output = V3;
    fn sub(self, o: V3) -> V3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f64> for V3 {
    type Output = V3;
    fn mul(self, k: f64) -> V3 {
        v3(self.x * k, self.y * k, self.z * k)
    }
}

impl Neg for V3 {
    type Output = V3;
    fn neg(self) -> V3 {
        v3(-self.x, -self.y, -self.z)
    }
}

/// A unit quaternion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quat {
    pub const IDENTITY: Quat = Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    pub fn rot(self, v: V3) -> V3 {
        let u = v3(self.x, self.y, self.z);
        let t = u.cross(v) * 2.0;
        v + t * self.w + u.cross(t)
    }

    pub fn inv_rot(self, v: V3) -> V3 {
        Quat { w: self.w, x: -self.x, y: -self.y, z: -self.z }.rot(v)
    }

    /// The rotation after turning by the small angle vector `w` (world frame), normalised.
    pub fn add_rotation(self, w: V3) -> Quat {
        let (a, b, c, d) = (self.w, self.x, self.y, self.z);
        let n = Quat {
            w: a + 0.5 * (-w.x * b - w.y * c - w.z * d),
            x: b + 0.5 * (w.x * a + w.y * d - w.z * c),
            y: c + 0.5 * (-w.x * d + w.y * a + w.z * b),
            z: d + 0.5 * (w.x * c - w.y * b + w.z * a),
        };
        let l = (n.w * n.w + n.x * n.x + n.y * n.y + n.z * n.z).sqrt();
        Quat { w: n.w / l, x: n.x / l, y: n.y / l, z: n.z / l }
    }

    pub fn from_axis_angle(axis: V3, angle: f64) -> Quat {
        let a = axis * (1.0 / axis.len());
        let (s, c) = (angle / 2.0).sin_cos();
        Quat { w: c, x: a.x * s, y: a.y * s, z: a.z * s }
    }

    /// The same rotation as a matrix.
    pub fn to_mat3(self) -> Mat3 {
        let (a, b, c) = (self.rot(v3(1.0, 0.0, 0.0)), self.rot(v3(0.0, 1.0, 0.0)), self.rot(v3(0.0, 0.0, 1.0)));
        Mat3([[a.x, b.x, c.x], [a.y, b.y, c.y], [a.z, b.z, c.z]])
    }

    /// Angle between the body's z axis and the world's z axis.
    pub fn tilt(self) -> f64 {
        self.rot(v3(0.0, 0.0, 1.0)).z.clamp(-1.0, 1.0).acos()
    }
}

/// A 3x3 matrix, rows first.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3(pub [[f64; 3]; 3]);

impl Mat3 {
    pub const IDENTITY: Mat3 = Mat3([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    pub fn mul_v(&self, v: V3) -> V3 {
        let m = &self.0;
        v3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z, m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z)
    }

    pub fn mul(&self, o: &Mat3) -> Mat3 {
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, x) in row.iter_mut().enumerate() {
                *x = (0..3).map(|k| self.0[i][k] * o.0[k][j]).sum();
            }
        }
        Mat3(r)
    }

    pub fn transpose(&self) -> Mat3 {
        let m = &self.0;
        Mat3([[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]])
    }

    /// Rotation about the unit vector `a` by `t` radians.
    pub fn rotation(a: V3, t: f64) -> Mat3 {
        let (s, c) = t.sin_cos();
        let k = 1.0 - c;
        Mat3([
            [c + a.x * a.x * k, a.x * a.y * k - a.z * s, a.x * a.z * k + a.y * s],
            [a.y * a.x * k + a.z * s, c + a.y * a.y * k, a.y * a.z * k - a.x * s],
            [a.z * a.x * k - a.y * s, a.z * a.y * k + a.x * s, c + a.z * a.z * k],
        ])
    }

    /// The inverse, by cofactors. The matrix must not be singular.
    pub fn inverse(&self) -> Mat3 {
        let m = &self.0;
        let c = |i: usize, j: usize| {
            let (r0, r1, c0, c1) = ((i + 1) % 3, (i + 2) % 3, (j + 1) % 3, (j + 2) % 3);
            m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0]
        };
        let det = m[0][0] * c(0, 0) + m[0][1] * c(0, 1) + m[0][2] * c(0, 2);
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, x) in row.iter_mut().enumerate() {
                *x = c(j, i) / det;
            }
        }
        Mat3(r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotation_round_trip() {
        let q = Quat::from_axis_angle(v3(0.3, -1.0, 0.2), 0.7);
        let v = v3(0.1, 0.2, -0.3);
        let back = q.inv_rot(q.rot(v));
        assert!((back - v).len() < 1e-12);
    }

    #[test]
    fn quarter_turn_about_z() {
        let q = Quat::from_axis_angle(v3(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2);
        let r = q.rot(v3(1.0, 0.0, 0.0));
        assert!((r - v3(0.0, 1.0, 0.0)).len() < 1e-12);
    }

    #[test]
    fn quaternion_matches_rotation_matrix() {
        let q = Quat::from_axis_angle(v3(0.3, 0.5, -0.2), 0.9);
        let r = Mat3::rotation(v3(0.3, 0.5, -0.2) * (1.0 / v3(0.3, 0.5, -0.2).len()), 0.9);
        let v = v3(0.1, -0.4, 0.7);
        assert!((q.rot(v) - r.mul_v(v)).len() < 1e-12);
    }

    #[test]
    fn small_rotation_matches_axis_angle() {
        let w = v3(0.0, 0.0, 1e-4);
        let a = Quat::IDENTITY.add_rotation(w);
        let b = Quat::from_axis_angle(v3(0.0, 0.0, 1.0), 1e-4);
        assert!((a.w - b.w).abs() < 1e-9 && (a.z - b.z).abs() < 1e-9);
    }
}
