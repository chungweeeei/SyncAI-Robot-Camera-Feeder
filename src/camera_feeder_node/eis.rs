//! The stabilizer: from the camera's orientation at a frame to the homography that warps the
//! frame into a steadier virtual camera.
//!
//! Gyro-based electronic image stabilization in the Karpenko et al. (2011) / Gyroflow mould. The
//! true orientation `q(t)` is low-passed into the virtual camera's orientation `q_s(t)` (two
//! cascaded first-order stages by default, see [`SmoothingOrder`]); the frame is then
//! re-projected from the real camera into the virtual one. Only rotation is
//! compensated, so the re-projection is a homography `H = K_out · R · K⁻¹` with no depth involved:
//! `K` the intrinsics, `R` the real-to-virtual rotation in the optical frame, and `K_out` the
//! intrinsics scaled by the zoom that hides the borders the warp uncovers.
//!
//! Pure: no GStreamer or ROS types; everything is tested on synthetic orientations.

use super::math::{Mat3, Quat};

/// Pinhole intrinsics, in pixels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Intrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

impl Intrinsics {
    #[must_use]
    pub fn matrix(&self) -> Mat3 {
        Mat3([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ])
    }

    /// `K⁻¹` in closed form.
    #[must_use]
    pub fn inverse(&self) -> Mat3 {
        Mat3([
            [1.0 / self.fx, 0.0, -self.cx / self.fx],
            [0.0, 1.0 / self.fy, -self.cy / self.fy],
            [0.0, 0.0, 1.0],
        ])
    }

    /// The same camera zoomed in about its principal point.
    #[must_use]
    pub fn zoomed(&self, zoom: f64) -> Self {
        Self {
            fx: self.fx * zoom,
            fy: self.fy * zoom,
            cx: self.cx,
            cy: self.cy,
        }
    }
}

/// How many first-order stages the virtual camera's low-pass cascades.
///
/// What the image shows is the virtual camera's own motion, so the residual shake is the
/// low-pass's gain at the gait frequency (~2 Hz on this robot): `1/√(1+(ωτ)²)` per stage. One
/// stage at τ 0.5 s still lets 16 % of a 2 Hz pitch through; two at τ 0.35 s let 5 % through and
/// follow a steady pan 0.7 s behind rather than 0.5 s. Two equal stages are critically damped, so
/// the view never overshoots and swings back when a turn ends.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SmoothingOrder {
    First,
    Second,
}

impl SmoothingOrder {
    #[must_use]
    pub fn stages(self) -> u8 {
        match self {
            Self::First => 1,
            Self::Second => 2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EisConfig {
    pub intrinsics: Intrinsics,
    pub width: u32,
    pub height: u32,
    /// Output magnification, >= 1. The warp uncovers a border of up to
    /// `(1 - 1/zoom)` of each half-size; see [`max_hidden_angle`].
    pub zoom: f64,
    /// Time constant of each first-order stage of the low-pass that the virtual camera follows
    /// the real one with. Larger is steadier, and drifts further from the real framing: a steady
    /// pan is followed `stages × τ` behind.
    pub smoothing_tau_s: f64,
    /// How many such stages; see [`SmoothingOrder`].
    pub smoothing_order: SmoothingOrder,
    /// The rotation taking IMU-frame vectors into the camera optical frame.
    pub cam_from_imu: Mat3,
}

/// The per-frame state of the stabilizer: the virtual camera's orientation.
#[derive(Clone, Debug)]
pub struct Stabilizer {
    config: EisConfig,
    /// The stamp of the previous frame and the output of each low-pass stage there, in the IMU
    /// frame. The last stage is `q_s`; with [`SmoothingOrder::First`] both hold the same value.
    smoothed: Option<(i64, [Quat; 2])>,
    max_angle: f64,
    k_out: Intrinsics,
}

impl Stabilizer {
    #[must_use]
    pub fn new(config: EisConfig) -> Self {
        Self {
            max_angle: max_hidden_angle(
                &config.intrinsics,
                config.zoom,
                config.width,
                config.height,
            ),
            k_out: config.intrinsics.zoomed(config.zoom),
            config,
            smoothed: None,
        }
    }

    /// Forgets the virtual camera, so the next frame starts it at the real orientation again.
    pub fn reset(&mut self) {
        self.smoothed = None;
    }

    #[must_use]
    pub fn config(&self) -> &EisConfig {
        &self.config
    }

    /// The homography from this frame's pixels to the output's, given the IMU-frame orientation
    /// `q_imu` (world_from_imu) at the frame's capture time `stamp_ns`. Advances the virtual
    /// camera, so it is called once per frame, in order.
    pub fn correction(&mut self, stamp_ns: i64, q_imu: Quat) -> Mat3 {
        let stages = match self.smoothed {
            None => [q_imu; 2],
            Some((previous_stamp, [first, second])) => {
                let dt_s = (stamp_ns - previous_stamp).max(0) as f64 / 1e9;
                let tau_s = self.config.smoothing_tau_s;
                // Every stage is clamped toward the real orientation, not only the output: an
                // unclamped first stage winds up during a long turn (it lags rate × τ, past what
                // the zoom hides) and then holds the output at the clamp after the turn has ended.
                let first = clamp_toward(smooth(first, q_imu, dt_s, tau_s), q_imu, self.max_angle);
                let second = match self.config.smoothing_order {
                    SmoothingOrder::First => first,
                    SmoothingOrder::Second => {
                        clamp_toward(smooth(second, first, dt_s, tau_s), q_imu, self.max_angle)
                    }
                };
                [first, second]
            }
        };
        self.smoothed = Some((stamp_ns, stages));
        let q_s = stages[1];

        // A world direction seen by the real camera is `C · R_imuᵀ · d`, by the virtual one
        // `C · R_sᵀ · d`; the rotation from real to virtual is therefore `C · R_sᵀ · R_imu · Cᵀ`.
        let c = &self.config.cam_from_imu;
        let real_to_virtual = (q_s.conjugate() * q_imu).to_matrix();
        let r = *c * real_to_virtual * c.transpose();
        homography(&self.config.intrinsics, &self.k_out, &r)
    }

    /// The homography for a frame with no orientation: the zoom alone, so the framing does not
    /// jump when the IMU comes and goes.
    #[must_use]
    pub fn passthrough(&self) -> Mat3 {
        homography(&self.config.intrinsics, &self.k_out, &Mat3::IDENTITY)
    }
}

/// `K_out · R · K_in⁻¹`.
#[must_use]
pub fn homography(k_in: &Intrinsics, k_out: &Intrinsics, r: &Mat3) -> Mat3 {
    k_out.matrix() * *r * k_in.inverse()
}

/// The largest rotation, in radians, that the border uncovered by `zoom` can hide: the smaller
/// of the horizontal and vertical margins, as an angle at the focal length.
#[must_use]
pub fn max_hidden_angle(intrinsics: &Intrinsics, zoom: f64, width: u32, height: u32) -> f64 {
    let margin = |half_size: f64| half_size * (1.0 - 1.0 / zoom.max(1.0));
    let horizontal = (margin(f64::from(width) / 2.0) / intrinsics.fx).atan();
    let vertical = (margin(f64::from(height) / 2.0) / intrinsics.fy).atan();
    horizontal.min(vertical)
}

/// One step of a first-order low-pass on orientations: `prev` moves toward `current` by the
/// fraction `1 - exp(-dt/tau)` of the arc (all the way when `tau` is 0 or `dt` is large).
#[must_use]
pub fn smooth(prev: Quat, current: Quat, dt_s: f64, tau_s: f64) -> Quat {
    if tau_s <= 0.0 {
        return current;
    }
    let alpha = 1.0 - (-dt_s / tau_s).exp();
    prev.slerp(current, alpha.clamp(0.0, 1.0))
}

/// `q_s`, pulled toward `q` just enough that the rotation between them is at most `max_angle`.
#[must_use]
pub fn clamp_toward(q_s: Quat, q: Quat, max_angle: f64) -> Quat {
    let angle = (q_s.conjugate() * q).angle();
    if angle <= max_angle || angle == 0.0 {
        return q_s;
    }
    q_s.slerp(q, (angle - max_angle) / angle)
}

#[cfg(test)]
mod tests {
    use super::*;

    const K: Intrinsics = Intrinsics {
        fx: 640.0,
        fy: 640.0,
        cx: 640.0,
        cy: 360.0,
    };

    fn config() -> EisConfig {
        EisConfig {
            intrinsics: K,
            width: 1280,
            height: 720,
            zoom: 1.15,
            smoothing_tau_s: 0.5,
            smoothing_order: SmoothingOrder::First,
            cam_from_imu: Mat3::IDENTITY,
        }
    }

    impl Stabilizer {
        /// `q_s` after the last frame.
        fn virtual_camera(&self) -> Quat {
            self.smoothed.expect("a frame has been corrected").1[1]
        }
    }

    /// The signed angle about y of a rotation about y.
    fn angle_about_y(q: Quat) -> f64 {
        2.0 * q.y.atan2(q.w)
    }

    /// Where `h` sends the pixel `(u, v)`.
    fn map(h: &Mat3, u: f64, v: f64) -> (f64, f64) {
        let p = h.mul_vec([u, v, 1.0]);
        (p[0] / p[2], p[1] / p[2])
    }

    fn close(a: (f64, f64), b: (f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6
    }

    #[test]
    fn the_identity_rotation_only_zooms_about_the_principal_point() {
        let h = Stabilizer::new(config()).passthrough();
        assert!(close(map(&h, 640.0, 360.0), (640.0, 360.0)));
        // A corner moves away from the centre by the zoom.
        let (u, v) = map(&h, 0.0, 0.0);
        assert!((u - (640.0 - 640.0 * 1.15)).abs() < 1e-6, "{u}");
        assert!((v - (360.0 - 360.0 * 1.15)).abs() < 1e-6, "{v}");
    }

    #[test]
    fn the_first_frame_is_the_virtual_camera() {
        let mut s = Stabilizer::new(config());
        let q = Quat::from_axis_angle([0.0, 1.0, 0.0], 0.3);
        let h = s.correction(0, q);
        // No correction on the first frame: identical to the passthrough.
        let p = s.passthrough();
        for i in 0..3 {
            for j in 0..3 {
                assert!((h.0[i][j] - p.0[i][j]).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn a_one_degree_pitch_moves_the_principal_point_by_f_tan_one_degree() {
        // Zoom 2 so the clamp (which at zoom 1 would allow no correction at all) is far away.
        let mut s = Stabilizer::new(EisConfig {
            zoom: 2.0,
            ..config()
        });
        s.correction(0, Quat::IDENTITY);
        // The camera turned 1° about its own y axis (to the right, optical frame) in 1 ns, which
        // the smoother barely follows; the virtual camera stays put and the image must move so
        // the scene stays fixed.
        let one_degree = 1f64.to_radians();
        let h = s.correction(1, Quat::from_axis_angle([0.0, 1.0, 0.0], one_degree));
        let (u, v) = map(&h, 640.0, 360.0);
        let expected = 640.0 * 2.0 * one_degree.tan();
        assert!(((u - 640.0).abs() - expected).abs() < 1e-3, "{u}");
        assert!((v - 360.0).abs() < 1e-6, "{v}");
    }

    #[test]
    fn zoom_one_can_hide_nothing_so_the_correction_is_clamped_away() {
        let mut s = Stabilizer::new(EisConfig {
            zoom: 1.0,
            ..config()
        });
        s.correction(0, Quat::IDENTITY);
        let h = s.correction(1, Quat::from_axis_angle([0.0, 1.0, 0.0], 0.1));
        assert!(close(map(&h, 640.0, 360.0), (640.0, 360.0)));
    }

    #[test]
    fn the_imu_to_camera_rotation_changes_which_image_axis_moves() {
        // IMU yaw (about body z) is, through the REP-103 body -> optical rotation, a rotation
        // about the optical y axis: the image moves horizontally, not vertically.
        let body_from_optical = Mat3::from_rpy_deg(-90.0, 0.0, -90.0);
        let mut s = Stabilizer::new(EisConfig {
            zoom: 2.0,
            cam_from_imu: body_from_optical.transpose(),
            ..config()
        });
        s.correction(0, Quat::IDENTITY);
        let h = s.correction(1, Quat::from_axis_angle([0.0, 0.0, 1.0], 0.01));
        let (u, v) = map(&h, 640.0, 360.0);
        assert!((u - 640.0).abs() > 5.0, "{u}");
        assert!((v - 360.0).abs() < 1e-6, "{v}");
    }

    #[test]
    fn the_correction_never_exceeds_what_the_zoom_can_hide() {
        let cfg = config();
        let max = max_hidden_angle(&K, cfg.zoom, cfg.width, cfg.height);
        // Vertical margin: 360 * (1 - 1/1.15) = 46.96 px -> atan(46.96/640) = 0.0733 rad.
        assert!(
            (max - (46.956_521_739 / 640.0f64).atan()).abs() < 1e-6,
            "{max}"
        );

        for order in [SmoothingOrder::First, SmoothingOrder::Second] {
            let mut s = Stabilizer::new(EisConfig {
                smoothing_order: order,
                ..cfg
            });
            s.correction(0, Quat::IDENTITY);
            // A sudden 20° pitch: far more than the margin hides.
            let big = Quat::from_axis_angle([1.0, 0.0, 0.0], 20f64.to_radians());
            s.correction(1, big);
            let residual = (s.virtual_camera().conjugate() * big).angle();
            assert!(residual <= max + 1e-9, "{order:?}: {residual} > {max}");
        }
    }

    #[test]
    fn a_sinusoidal_shake_is_smoothed_to_a_smaller_motion() {
        // 2 Hz, ±3° about y, at 30 fps for 3 s: the virtual camera's angle from the mean must
        // vary far less than the real one's.
        let mut s = Stabilizer::new(EisConfig {
            zoom: 1.3,
            ..config()
        });
        let amplitude = 3f64.to_radians();
        let mut raw = Vec::new();
        let mut smoothed = Vec::new();
        for i in 0..90 {
            let t = i as f64 / 30.0;
            let angle = amplitude * (2.0 * std::f64::consts::PI * 2.0 * t).sin();
            let q = Quat::from_axis_angle([0.0, 1.0, 0.0], angle);
            s.correction((t * 1e9) as i64, q);
            raw.push(angle);
            smoothed.push(angle_about_y(s.virtual_camera()));
        }
        let variance = |v: &[f64]| {
            let mean = v.iter().sum::<f64>() / v.len() as f64;
            v.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / v.len() as f64
        };
        let (raw_var, smoothed_var) = (variance(&raw[30..]), variance(&smoothed[30..]));
        assert!(smoothed_var < raw_var / 10.0, "{raw_var} -> {smoothed_var}");
    }

    /// The steady-state amplitude of the virtual camera, as a fraction of the real camera's, for
    /// a 1° pitch at `hz` sampled at 30 fps (6 s, measured over the last 2).
    fn residual_ratio(order: SmoothingOrder, tau_s: f64, hz: f64) -> f64 {
        // Zoom 2 keeps the clamp out of the way.
        let mut s = Stabilizer::new(EisConfig {
            zoom: 2.0,
            smoothing_tau_s: tau_s,
            smoothing_order: order,
            ..config()
        });
        let amplitude = 1f64.to_radians();
        let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
        for i in 0..180 {
            let t = f64::from(i) / 30.0;
            let angle = amplitude * (2.0 * std::f64::consts::PI * hz * t).sin();
            s.correction(
                (t * 1e9) as i64,
                Quat::from_axis_angle([0.0, 1.0, 0.0], angle),
            );
            if i >= 120 {
                let virtual_angle = angle_about_y(s.virtual_camera());
                low = low.min(virtual_angle);
                high = high.max(virtual_angle);
            }
        }
        (high - low) / 2.0 / amplitude
    }

    #[test]
    fn each_stage_attenuates_the_gait_frequency_as_a_first_order_low_pass() {
        let gain = |tau_s: f64, hz: f64| {
            let w_tau = 2.0 * std::f64::consts::PI * hz * tau_s;
            1.0 / (1.0 + w_tau * w_tau).sqrt()
        };
        // Discretisation at 30 fps moves these by ~1-2 %; 10 % is slack, not tolerance for a
        // wrong order.
        for (order, tau_s, expected) in [
            (SmoothingOrder::First, 0.5, gain(0.5, 2.0)),
            (SmoothingOrder::Second, 0.35, gain(0.35, 2.0).powi(2)),
            (SmoothingOrder::Second, 0.25, gain(0.25, 2.0).powi(2)),
        ] {
            let ratio = residual_ratio(order, tau_s, 2.0);
            assert!(
                (ratio - expected).abs() < 0.1 * expected,
                "{order:?} tau {tau_s}: {ratio} vs {expected}"
            );
        }
        // The point of the second stage: a third of the shake for 1.4x the pan lag.
        assert!(
            residual_ratio(SmoothingOrder::Second, 0.35, 2.0)
                < residual_ratio(SmoothingOrder::First, 0.5, 2.0) / 3.0
        );
    }

    #[test]
    fn the_second_order_view_settles_without_overshoot() {
        let mut s = Stabilizer::new(EisConfig {
            zoom: 2.0,
            smoothing_order: SmoothingOrder::Second,
            ..config()
        });
        s.correction(0, Quat::IDENTITY);
        // The real camera steps 3° and stays; the virtual one must approach it monotonically.
        let target_angle = 3f64.to_radians();
        let target = Quat::from_axis_angle([0.0, 1.0, 0.0], target_angle);
        let mut previous = 0.0;
        for i in 1..=150 {
            s.correction(i64::from(i) * 33_333_333, target);
            let angle = angle_about_y(s.virtual_camera());
            assert!(angle >= previous - 1e-12, "frame {i}: {angle} < {previous}");
            assert!(
                angle <= target_angle + 1e-12,
                "frame {i}: {angle} overshoots"
            );
            previous = angle;
        }
        // 5 s is 7 time constants per stage: settled.
        assert!(
            (previous - target_angle).abs() < 1e-3 * target_angle,
            "{previous}"
        );
    }

    #[test]
    fn reset_forgets_every_stage() {
        let mut s = Stabilizer::new(EisConfig {
            zoom: 2.0,
            smoothing_order: SmoothingOrder::Second,
            ..config()
        });
        s.correction(0, Quat::IDENTITY);
        s.correction(1, Quat::from_axis_angle([0.0, 1.0, 0.0], 0.05));
        s.reset();
        // After a reset the next frame is the virtual camera again: no correction.
        let q = Quat::from_axis_angle([1.0, 0.0, 0.0], 0.2);
        let h = s.correction(2, q);
        let p = s.passthrough();
        for i in 0..3 {
            for j in 0..3 {
                assert!((h.0[i][j] - p.0[i][j]).abs() < 1e-9);
            }
        }
        assert_eq!(s.smoothed.unwrap().1, [q; 2]);
    }

    #[test]
    fn smoothing_converges_when_dt_dwarfs_the_time_constant() {
        let target = Quat::from_axis_angle([1.0, 0.0, 0.0], 0.4);
        let q = smooth(Quat::IDENTITY, target, 100.0, 0.5);
        assert!((q.conjugate() * target).angle() < 1e-9);
        // And with no time elapsed it does not move at all.
        assert_eq!(smooth(Quat::IDENTITY, target, 0.0, 0.5), Quat::IDENTITY);
        // tau 0 disables smoothing.
        assert_eq!(smooth(Quat::IDENTITY, target, 0.001, 0.0), target);
    }

    #[test]
    fn clamping_leaves_small_differences_alone_and_caps_large_ones() {
        let q = Quat::from_axis_angle([0.0, 0.0, 1.0], 0.2);
        assert_eq!(clamp_toward(Quat::IDENTITY, q, 0.3), Quat::IDENTITY);
        let clamped = clamp_toward(Quat::IDENTITY, q, 0.05);
        assert!(((clamped.conjugate() * q).angle() - 0.05).abs() < 1e-9);
    }

    #[test]
    fn the_intrinsics_inverse_matches_the_general_one() {
        let general = K.matrix().inverse().unwrap();
        let closed = K.inverse();
        for i in 0..3 {
            for j in 0..3 {
                assert!((general.0[i][j] - closed.0[i][j]).abs() < 1e-12);
            }
        }
    }
}
