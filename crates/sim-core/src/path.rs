//! The closed path of a belt round two pulleys, in the x-z plane of the chassis.

use std::f64::consts::PI;

/// Two pulley centres `a`, `b` of radius `r`, in (x, z).
#[derive(Clone, Copy, Debug)]
pub struct BeltPath {
    pub a: (f64, f64),
    pub b: (f64, f64),
    pub r: f64,
}

impl BeltPath {
    fn span(&self) -> (f64, f64, f64) {
        let (ex, ez) = (self.b.0 - self.a.0, self.b.1 - self.a.1);
        let l = (ex * ex + ez * ez).sqrt();
        (l, ex / l, ez / l)
    }

    pub fn len(&self) -> f64 {
        2.0 * self.span().0 + 2.0 * PI * self.r
    }

    /// The point at path position `s`: (position, outward unit normal). `s` grows counter-clockwise
    /// seen from -y, so the bottom run goes from `a` to `b`; a belt driving forward moves against `s`.
    pub fn at(&self, s: f64) -> ((f64, f64), (f64, f64)) {
        let (l, dx, dz) = self.span();
        let r = self.r;
        let s = s.rem_euclid(2.0 * l + 2.0 * PI * r);
        let o0 = (dz, -dx);
        let arc = PI * r;
        let rot = |o: (f64, f64), a: f64| {
            let (sn, c) = a.sin_cos();
            (c * o.0 - sn * o.1, sn * o.0 + c * o.1)
        };
        let (a, b) = (self.a, self.b);
        if s < l {
            return ((a.0 + dx * s + o0.0 * r, a.1 + dz * s + o0.1 * r), o0);
        }
        if s < l + arc {
            let o = rot(o0, (s - l) / r);
            return ((b.0 + o.0 * r, b.1 + o.1 * r), o);
        }
        if s < 2.0 * l + arc {
            let w = s - l - arc;
            return ((b.0 - dx * w - o0.0 * r, b.1 - dz * w - o0.1 * r), (-o0.0, -o0.1));
        }
        let o = rot((-o0.0, -o0.1), (s - 2.0 * l - arc) / r);
        ((a.0 + o.0 * r, a.1 + o.1 * r), o)
    }

    /// The path position nearest to `p`: (distance from the line between the centres, s, outward normal).
    pub fn locate(&self, p: (f64, f64)) -> (f64, f64, (f64, f64)) {
        let (l, dx, dz) = self.span();
        let o0 = (dz, -dx);
        let arc = PI * self.r;
        let (vx, vz) = (p.0 - self.a.0, p.1 - self.a.1);
        let f = vx * dx + vz * dz;
        if (0.0..=l).contains(&f) {
            let h = vx * o0.0 + vz * o0.1;
            return if h >= 0.0 { (h, f, o0) } else { (-h, l + arc + (l - f), (-o0.0, -o0.1)) };
        }
        let (c, base, start) = if f > l { (self.b, l, o0) } else { (self.a, 2.0 * l + arc, (-o0.0, -o0.1)) };
        let (qx, qz) = (p.0 - c.0, p.1 - c.1);
        let d = (qx * qx + qz * qz).sqrt();
        if d < 1e-9 {
            return (0.0, base, start);
        }
        let o = (qx / d, qz / d);
        let cr = start.0 * o.1 - start.1 * o.0;
        let dt = start.0 * o.0 + start.1 * o.1;
        let ang = cr.atan2(dt).clamp(0.0, PI);
        (d, base + ang * self.r, o)
    }
}

/// Direction of forward belt travel (x, y, z) at a point with outward normal `o`: clockwise in (x, z).
pub fn travel(o: (f64, f64)) -> (f64, f64, f64) {
    (o.1, 0.0, -o.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn path() -> BeltPath {
        BeltPath { a: (-0.27, 0.0), b: (0.27, 0.0), r: 0.07 }
    }

    #[test]
    fn length_of_a_stadium() {
        assert!((path().len() - (4.0 * 0.27 + 2.0 * PI * 0.07)).abs() < 1e-12);
    }

    #[test]
    fn locate_inverts_at() {
        let p = path();
        let n = 50;
        for i in 0..n {
            let s = p.len() * (i as f64 + 0.3) / n as f64;
            let (q, _) = p.at(s);
            let (d, s2, _) = p.locate(q);
            assert!((d - p.r).abs() < 1e-9, "distance at s {s}");
            assert!((s2 - s).abs() < 1e-9, "s {s} came back as {s2}");
        }
    }

    #[test]
    fn bottom_run_drives_forward() {
        // On the bottom run the outward normal is -z, so forward travel is +x.
        let (_, o) = path().at(0.1);
        assert!((o.1 + 1.0).abs() < 1e-12);
        let t = travel(o);
        assert!((t.0 + 1.0).abs() < 1e-12, "the belt surface on the ground moves backward");
    }
}
