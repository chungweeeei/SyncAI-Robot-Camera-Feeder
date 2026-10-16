//! The stabilizer: from the camera's orientation at a frame to the homography that warps the
//! frame into a steadier virtual camera.
//!
//! Gyro-based electronic image stabilization in the Karpenko et al. (2011) / Gyroflow mould. The
//! true orientation `q(t)` is low-passed into the virtual camera's orientation `q_s(t)`; the
//! frame is then re-projected from the real camera into the virtual one. Only rotation is
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EisConfig {
    pub intrinsics: Intrinsics,
    pub width: u32,
    pub height: u32,
    /// Output magnification, >= 1. The warp uncovers a border of up to
    /// `(1 - 1/zoom)` of each half-size; see [`max_hidden_angle`].
    pub zoom: f64,
    /// Time constant of the first-order low-pass that the virtual camera follows the real one
    /// with. Larger is steadier, and drifts further from the real framing.
    pub smoothing_tau_s: f64,
    /// The rotation taking IMU-frame vectors into the camera optical frame.
    pub cam_from_imu: Mat3,
}

/// The per-frame state of the stabilizer: the virtual camera's orientation.
#[derive(Clone, Debug)]
pub struct Stabilizer {
    config: EisConfig,
    /// (stamp, `q_s`) of the previous frame, in the IMU frame.
    smoothed: Option<(i64, Quat)>,
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
        let q_s = match self.smoothed {
            None => q_imu,
            Some((previous_stamp, previous)) => {
                let dt_s = (stamp_ns - previous_stamp).max(0) as f64 / 1e9;
                let followed = smooth(previous, q_imu, dt_s, self.config.smoothing_tau_s);
                clamp_toward(followed, q_imu, self.max_angle)
            }
        };
        self.smoothed = Some((stamp_ns, q_s));

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
            cam_from_imu: Mat3::IDENTITY,
        }
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

        let mut s = Stabilizer::new(cfg);
        s.correction(0, Quat::IDENTITY);
        // A sudden 20° pitch: far more than the margin hides.
        let big = Quat::from_axis_angle([1.0, 0.0, 0.0], 20f64.to_radians());
        s.correction(1, big);
        let (_, q_s) = s.smoothed.unwrap();
        let residual = (q_s.conjugate() * big).angle();
        assert!(residual <= max + 1e-9, "{residual} > {max}");
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
            let (_, q_s) = s.smoothed.unwrap();
            // Signed angle about y.
            smoothed.push(2.0 * q_s.y.atan2(q_s.w));
        }
        let variance = |v: &[f64]| {
            let mean = v.iter().sum::<f64>() / v.len() as f64;
            v.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / v.len() as f64
        };
        let (raw_var, smoothed_var) = (variance(&raw[30..]), variance(&smoothed[30..]));
        assert!(smoothed_var < raw_var / 10.0, "{raw_var} -> {smoothed_var}");
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
