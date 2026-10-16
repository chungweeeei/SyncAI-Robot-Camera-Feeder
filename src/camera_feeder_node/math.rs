//! Unit quaternions and 3×3 matrices, as plain `f64` arithmetic.
//!
//! Hand-written rather than `nalgebra`: every release that still builds on the pinned rustc 1.85
//! is two major versions old, and the stabilizer needs only a handful of operations (compose,
//! invert, slerp, rotate, a few matrix products). Conventions, used everywhere downstream:
//!
//! - A [`Quat`] is `w + xi + yj + zk`, unit length, and rotates a vector of the *body* frame into
//!   the *world* frame (`q.rotate(v_body) == v_world`). Composition is `a * b`: apply `b`
//!   first, then `a`.
//! - A [`Mat3`] is row-major; `a * b` is the matrix product and `m.mul_vec(v)` the
//!   matrix-vector product.

use std::ops::Mul;

/// A unit quaternion `w + xi + yj + zk`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quat {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// The rotation by `angle` radians about `axis` (normalized here; a zero axis is the
    /// identity).
    #[must_use]
    pub fn from_axis_angle(axis: [f64; 3], angle: f64) -> Self {
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if norm == 0.0 {
            return Self::IDENTITY;
        }
        let (sin, cos) = (angle / 2.0).sin_cos();
        let s = sin / norm;
        Self {
            w: cos,
            x: axis[0] * s,
            y: axis[1] * s,
            z: axis[2] * s,
        }
    }

    /// The rotation whose axis is `v` and whose angle is `|v|` radians: what integrating an
    /// angular velocity over `dt` produces (`v = omega * dt`).
    #[must_use]
    pub fn from_rotation_vector(v: [f64; 3]) -> Self {
        let angle = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        Self::from_axis_angle(v, angle)
    }

    /// The inverse rotation (for a unit quaternion the conjugate is the inverse).
    #[must_use]
    pub fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Rescaled to unit length; a zero quaternion becomes the identity rather than NaN.
    #[must_use]
    pub fn normalized(self) -> Self {
        let norm = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if norm == 0.0 || !norm.is_finite() {
            return Self::IDENTITY;
        }
        Self {
            w: self.w / norm,
            x: self.x / norm,
            y: self.y / norm,
            z: self.z / norm,
        }
    }

    /// The rotation angle in radians, in `0..=PI` (the shorter way round).
    #[must_use]
    pub fn angle(self) -> f64 {
        let sin_half = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        2.0 * sin_half.atan2(self.w.abs())
    }

    /// Spherical interpolation from `self` (`t = 0`) to `to` (`t = 1`) along the shorter arc.
    #[must_use]
    pub fn slerp(self, to: Self, t: f64) -> Self {
        let mut to = to;
        let mut dot = self.w * to.w + self.x * to.x + self.y * to.y + self.z * to.z;
        if dot < 0.0 {
            // q and -q are the same rotation; pick the sign that takes the shorter arc.
            to = Self {
                w: -to.w,
                x: -to.x,
                y: -to.y,
                z: -to.z,
            };
            dot = -dot;
        }
        let (a, b) = if dot > 0.9995 {
            // Nearly parallel: the sine denominators below lose precision, and linear
            // interpolation is within rounding of the arc anyway.
            (1.0 - t, t)
        } else {
            let theta = dot.clamp(-1.0, 1.0).acos();
            let sin_theta = theta.sin();
            (
                ((1.0 - t) * theta).sin() / sin_theta,
                (t * theta).sin() / sin_theta,
            )
        };
        Self {
            w: a * self.w + b * to.w,
            x: a * self.x + b * to.x,
            y: a * self.y + b * to.y,
            z: a * self.z + b * to.z,
        }
        .normalized()
    }

    /// The rotation matrix `R` with `R v == self.rotate(v)`.
    #[must_use]
    pub fn to_matrix(self) -> Mat3 {
        let Self { w, x, y, z } = self;
        Mat3([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ])
    }

    /// Rotates `v` (body → world, see the module docs).
    #[must_use]
    pub fn rotate(self, v: [f64; 3]) -> [f64; 3] {
        self.to_matrix().mul_vec(v)
    }
}

/// Hamilton product: `a * b` is the rotation that applies `b` first and then `a`.
impl Mul for Quat {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let (a, b) = (self, other);
        Self {
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        }
    }
}

/// A row-major 3×3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3(pub [[f64; 3]; 3]);

/// The matrix product.
impl Mul for Mat3 {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let mut out = [[0.0; 3]; 3];
        for (i, row) in out.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (0..3).map(|k| self.0[i][k] * other.0[k][j]).sum();
            }
        }
        Self(out)
    }
}

impl Mat3 {
    pub const IDENTITY: Self = Self([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    #[must_use]
    pub fn mul_vec(&self, v: [f64; 3]) -> [f64; 3] {
        let row = |r: [f64; 3]| r[0] * v[0] + r[1] * v[1] + r[2] * v[2];
        [row(self.0[0]), row(self.0[1]), row(self.0[2])]
    }

    #[must_use]
    pub fn transpose(&self) -> Self {
        let m = &self.0;
        Self([
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ])
    }

    #[must_use]
    pub fn scale(&self, factor: f64) -> Self {
        Self(self.0.map(|row| row.map(|cell| cell * factor)))
    }

    #[must_use]
    pub fn det(&self) -> f64 {
        let m = &self.0;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// The inverse by cofactors, or `None` for a singular (or non-finite) matrix.
    #[must_use]
    pub fn inverse(&self) -> Option<Self> {
        let det = self.det();
        if det == 0.0 || !det.is_finite() {
            return None;
        }
        let m = &self.0;
        let cofactor = |r1: usize, c1: usize, r2: usize, c2: usize| {
            m[r1][c1] * m[r2][c2] - m[r1][c2] * m[r2][c1]
        };
        // The adjugate (transposed cofactor matrix) over the determinant.
        let adj = Self([
            [
                cofactor(1, 1, 2, 2),
                -cofactor(0, 1, 2, 2),
                cofactor(0, 1, 1, 2),
            ],
            [
                -cofactor(1, 0, 2, 2),
                cofactor(0, 0, 2, 2),
                -cofactor(0, 0, 1, 2),
            ],
            [
                cofactor(1, 0, 2, 1),
                -cofactor(0, 0, 2, 1),
                cofactor(0, 0, 1, 1),
            ],
        ]);
        Some(adj.scale(1.0 / det))
    }

    /// The rotation `Rz(yaw) · Ry(pitch) · Rx(roll)` — the extrinsic x-y-z convention tf2 and
    /// URDF `rpy` use — with the angles in degrees.
    #[must_use]
    pub fn from_rpy_deg(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = roll.to_radians().sin_cos();
        let (sp, cp) = pitch.to_radians().sin_cos();
        let (sy, cy) = yaw.to_radians().sin_cos();
        let rx = Self([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]);
        let ry = Self([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]);
        let rz = Self([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]);
        rz * ry * rx
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    const EPS: f64 = 1e-9;

    fn assert_vec_eq(a: [f64; 3], b: [f64; 3]) {
        for i in 0..3 {
            assert!((a[i] - b[i]).abs() < EPS, "{a:?} != {b:?}");
        }
    }

    fn assert_quat_eq(a: Quat, b: Quat) {
        // q and -q are the same rotation.
        let same = (a.w - b.w).abs() < EPS
            && (a.x - b.x).abs() < EPS
            && (a.y - b.y).abs() < EPS
            && (a.z - b.z).abs() < EPS;
        let negated = (a.w + b.w).abs() < EPS
            && (a.x + b.x).abs() < EPS
            && (a.y + b.y).abs() < EPS
            && (a.z + b.z).abs() < EPS;
        assert!(same || negated, "{a:?} != {b:?}");
    }

    #[test]
    fn a_quarter_turn_about_z_takes_x_to_y() {
        let q = Quat::from_axis_angle([0.0, 0.0, 1.0], FRAC_PI_2);
        assert_vec_eq(q.rotate([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]);
        assert_vec_eq(q.to_matrix().mul_vec([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]);
        assert!((q.angle() - FRAC_PI_2).abs() < EPS);
    }

    #[test]
    fn the_axis_is_normalized_and_a_zero_axis_is_the_identity() {
        let q = Quat::from_axis_angle([0.0, 0.0, 10.0], FRAC_PI_2);
        assert_vec_eq(q.rotate([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]);
        assert_eq!(Quat::from_axis_angle([0.0, 0.0, 0.0], 1.0), Quat::IDENTITY);
        assert_eq!(Quat::from_rotation_vector([0.0; 3]), Quat::IDENTITY);
    }

    #[test]
    fn composition_applies_the_right_factor_first() {
        let about_z = Quat::from_axis_angle([0.0, 0.0, 1.0], FRAC_PI_2);
        let about_x = Quat::from_axis_angle([1.0, 0.0, 0.0], FRAC_PI_2);
        // x --(z turn)--> y --(x turn)--> z
        let both = about_x * about_z;
        assert_vec_eq(both.rotate([1.0, 0.0, 0.0]), [0.0, 0.0, 1.0]);
    }

    #[test]
    fn the_conjugate_undoes_the_rotation() {
        let q = Quat::from_axis_angle([1.0, 2.0, 3.0], 0.7);
        assert_quat_eq(q * q.conjugate(), Quat::IDENTITY);
        assert_vec_eq(
            q.conjugate().rotate(q.rotate([0.3, -0.2, 0.9])),
            [0.3, -0.2, 0.9],
        );
    }

    #[test]
    fn normalizing_restores_unit_length_and_never_yields_nan() {
        let q = Quat {
            w: 2.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
        .normalized();
        assert_eq!(q, Quat::IDENTITY);
        let zero = Quat {
            w: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(zero.normalized(), Quat::IDENTITY);
    }

    #[test]
    fn slerp_hits_both_ends_and_the_midpoint() {
        let a = Quat::IDENTITY;
        let b = Quat::from_axis_angle([0.0, 1.0, 0.0], 1.0);
        assert_quat_eq(a.slerp(b, 0.0), a);
        assert_quat_eq(a.slerp(b, 1.0), b);
        assert_quat_eq(a.slerp(b, 0.5), Quat::from_axis_angle([0.0, 1.0, 0.0], 0.5));
    }

    #[test]
    fn slerp_takes_the_shorter_arc_when_the_target_is_negated() {
        let a = Quat::IDENTITY;
        let b = Quat::from_axis_angle([0.0, 1.0, 0.0], 1.0);
        let minus_b = Quat {
            w: -b.w,
            x: -b.x,
            y: -b.y,
            z: -b.z,
        };
        assert!((a.slerp(minus_b, 0.5).angle() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn the_matrix_inverse_is_the_transpose_for_a_rotation() {
        let r = Quat::from_axis_angle([1.0, 1.0, 0.0], 0.4).to_matrix();
        let inv = r.inverse().unwrap();
        let t = r.transpose();
        for i in 0..3 {
            assert_vec_eq(inv.0[i], t.0[i]);
        }
        assert!((r.det() - 1.0).abs() < EPS);
    }

    #[test]
    fn a_singular_matrix_has_no_inverse() {
        assert!(
            Mat3([[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 0.0, 1.0]])
                .inverse()
                .is_none()
        );
    }

    #[test]
    fn the_general_inverse_round_trips() {
        let k = Mat3([[640.0, 0.0, 640.0], [0.0, 640.0, 360.0], [0.0, 0.0, 1.0]]);
        let product = k * k.inverse().unwrap();
        for i in 0..3 {
            assert_vec_eq(product.0[i], Mat3::IDENTITY.0[i]);
        }
    }

    #[test]
    fn rpy_follows_the_tf2_convention() {
        // Roll alone is a rotation about x; yaw alone about z.
        assert_vec_eq(
            Mat3::from_rpy_deg(90.0, 0.0, 0.0).mul_vec([0.0, 1.0, 0.0]),
            [0.0, 0.0, 1.0],
        );
        assert_vec_eq(
            Mat3::from_rpy_deg(0.0, 0.0, 90.0).mul_vec([1.0, 0.0, 0.0]),
            [0.0, 1.0, 0.0],
        );
        // The REP-103 body -> optical static transform: with rpy (-90, 0, -90) the optical frame's
        // z axis (its third column) is the body's forward (+x) axis, and its y axis is body down.
        let body_from_optical = Mat3::from_rpy_deg(-90.0, 0.0, -90.0);
        assert_vec_eq(body_from_optical.mul_vec([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0]);
        assert_vec_eq(body_from_optical.mul_vec([0.0, 1.0, 0.0]), [0.0, 0.0, -1.0]);
        assert_vec_eq(body_from_optical.mul_vec([1.0, 0.0, 0.0]), [0.0, -1.0, 0.0]);
    }

    #[test]
    fn quaternion_and_matrix_rotations_agree() {
        let q = Quat::from_axis_angle([0.2, -0.5, 0.8], PI / 3.0);
        let v = [0.1, 0.7, -0.4];
        assert_vec_eq(q.rotate(v), q.to_matrix().mul_vec(v));
    }
}
