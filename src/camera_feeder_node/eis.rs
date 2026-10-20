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
//! While the robot turns, the low-pass is shortened ([`TurnDamping`]): a turn is motion the
//! viewer means to see, and a view that trails it slides on for a second after the robot has
//! stopped.
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
/// low-pass's gain at the gait frequency, `1/√(1+(ωτ)²)` per stage (robot01 trots with its pitch,
/// the vertical image motion, at 3.8 Hz and its yaw at 1.9 Hz). Replayed on the dp1f_with_image
/// bag (`tools/eis_fit.py filters`), one stage at τ 0.5 s left 17 % of the vertical gait-band
/// shake wherever the clamp was not holding the view, two at τ 0.35 s 10.5 %, for following a
/// steady pan 0.7 s behind rather than 0.5 s. Two equal stages are critically damped, so the view
/// never overshoots and swings back when a turn ends.
///
/// Wherever the clamp *is* holding it, the real camera's shake goes straight through whatever
/// the order: turns use up the zoom's margin (11-13 % of that bag's frames, a little more the
/// longer the lag), which is why the bag's overall vertical residual only went from 1.73 px to
/// 1.64 px.
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

/// Shortens the virtual camera's low-pass while the robot turns.
///
/// A plain low-pass follows a steady turn `stages × τ` behind, which at robot01's 0.5 rad/s is
/// 20°, so every turn ran the view into the clamp (what the zoom hides, ~6°) and held it there;
/// when the turn ended, the view slid the last 6° after a robot that had already stopped, for
/// ~1.5 s at 2 × 0.35 s. Operators saw that as an image that kept flowing after the turn. Here
/// each stage's τ is divided by `1 + (excess / reference)²`, where `excess` is how far the turn
/// rate is above `deadband_rad_s`, so the view keeps up with a turn and has little left to catch
/// up when it ends.
///
/// The turn rate is the heading's (rotation about world z, see [`heading`]) through two
/// first-order stages of `rate_tau_s`, because the gait sways the heading too: on robot01 at
/// 1.9 Hz and up to ~0.4 rad/s, which unfiltered would shorten τ at every step and let the sway
/// through. The deadband keeps what the low-pass leaves of the sway (and slow drift) from
/// shortening τ while walking straight. Shortening τ costs little in a turn anyway: there the
/// clamp used to hold the view, and the real camera's shake went straight through it.
///
/// Every stage stays a first-order low-pass, only with a varying τ, so a turn's end is still
/// approached without overshoot. A feed-forward of the turn rate (zero lag in a steady turn) was
/// tried and rejected: it overshot every turn's end by 5-7°, the swing back being worse than the
/// slide.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TurnDamping {
    /// Time constant of each of the two stages the heading rate is low-passed through.
    pub rate_tau_s: f64,
    /// Turn rates up to this leave the smoothing alone.
    pub deadband_rad_s: f64,
    /// The excess over the deadband at which τ is halved; > 0.
    pub reference_rad_s: f64,
}

impl TurnDamping {
    /// The per-stage time constant to use at `turn_rate` (rad/s, either sign).
    #[must_use]
    pub fn effective_tau(&self, tau_s: f64, turn_rate: f64) -> f64 {
        let excess = (turn_rate.abs() - self.deadband_rad_s).max(0.0) / self.reference_rad_s;
        tau_s / (1.0 + excess * excess)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EisConfig {
    pub intrinsics: Intrinsics,
    pub width: u32,
    pub height: u32,
    /// Output magnification, >= 1. The warp uncovers a border of up to
    /// `(1 - 1/zoom)` of each half-size; see [`hidden_angles`].
    pub zoom: f64,
    /// Time constant of each first-order stage of the low-pass that the virtual camera follows
    /// the real one with. Larger is steadier, and drifts further from the real framing: a steady
    /// pan is followed `stages × τ` behind.
    pub smoothing_tau_s: f64,
    /// How many such stages; see [`SmoothingOrder`].
    pub smoothing_order: SmoothingOrder,
    /// `None` smooths turns like any other motion.
    pub turn_damping: Option<TurnDamping>,
    /// The rotation taking IMU-frame vectors into the camera optical frame.
    pub cam_from_imu: Mat3,
}

/// What the stabilizer carries from one frame to the next.
#[derive(Clone, Copy, Debug)]
struct State {
    stamp_ns: i64,
    /// The output of each low-pass stage, in the IMU frame. The last stage is `q_s` (before the
    /// fit to the frame); with [`SmoothingOrder::First`] both hold the same value.
    stages: [Quat; 2],
    /// The real camera's heading, for the next frame's turn rate.
    heading: f64,
    /// The heading rate (rad/s) after each of [`TurnDamping`]'s two low-pass stages.
    turn_rate: [f64; 2],
}

/// The per-frame state of the stabilizer: the virtual camera's orientation.
#[derive(Clone, Debug)]
pub struct Stabilizer {
    config: EisConfig,
    state: Option<State>,
    max_angle: f64,
    k_out: Intrinsics,
}

/// Bisection steps of [`Stabilizer::fit_to_frame`]: 2⁻²⁰ of the correction, far below a pixel.
const FIT_ITERATIONS: u32 = 20;

impl Stabilizer {
    #[must_use]
    pub fn new(config: EisConfig) -> Self {
        Self {
            max_angle: {
                let (pan, tilt) =
                    hidden_angles(&config.intrinsics, config.zoom, config.width, config.height);
                pan.min(tilt)
            },
            k_out: config.intrinsics.zoomed(config.zoom),
            config,
            state: None,
        }
    }

    /// Forgets the virtual camera, so the next frame starts it at the real orientation again.
    pub fn reset(&mut self) {
        self.state = None;
    }

    #[must_use]
    pub fn config(&self) -> &EisConfig {
        &self.config
    }

    /// The homography from this frame's pixels to the output's, given the IMU-frame orientation
    /// `q_imu` (world_from_imu) at the frame's capture time `stamp_ns`. Advances the virtual
    /// camera, so it is called once per frame, in order.
    pub fn correction(&mut self, stamp_ns: i64, q_imu: Quat) -> Mat3 {
        let heading_now = heading(q_imu);
        let state = match self.state {
            None => State {
                stamp_ns,
                stages: [q_imu; 2],
                heading: heading_now,
                turn_rate: [0.0; 2],
            },
            Some(previous) => {
                let dt_s = (stamp_ns - previous.stamp_ns).max(0) as f64 / 1e9;
                let turn_rate = match self.config.turn_damping {
                    Some(damping) if dt_s > 0.0 => {
                        let rate = wrap_angle(heading_now - previous.heading) / dt_s;
                        let first = low_pass(previous.turn_rate[0], rate, dt_s, damping.rate_tau_s);
                        [
                            first,
                            low_pass(previous.turn_rate[1], first, dt_s, damping.rate_tau_s),
                        ]
                    }
                    _ => previous.turn_rate,
                };
                let tau_s = match self.config.turn_damping {
                    Some(damping) => {
                        damping.effective_tau(self.config.smoothing_tau_s, turn_rate[1])
                    }
                    None => self.config.smoothing_tau_s,
                };
                let [first, second] = previous.stages;
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
                State {
                    stamp_ns,
                    stages: [first, second],
                    heading: heading_now,
                    turn_rate,
                }
            }
        };
        self.state = Some(state);
        let q_s = self.fit_to_frame(state.stages[1], q_imu);
        homography(
            &self.config.intrinsics,
            &self.k_out,
            &self.optical_rotation(q_s, q_imu),
        )
    }

    /// The rotation from the real camera (`q_imu`) to the virtual one (`q_s`), in the optical
    /// frame. A world direction seen by the real camera is `C · R_imuᵀ · d`, by the virtual one
    /// `C · R_sᵀ · d`; the rotation from real to virtual is therefore `C · R_sᵀ · R_imu · Cᵀ`.
    fn optical_rotation(&self, q_s: Quat, q_imu: Quat) -> Mat3 {
        let c = &self.config.cam_from_imu;
        *c * (q_s.conjugate() * q_imu).to_matrix() * c.transpose()
    }

    /// Whether every output pixel samples inside the frame when the virtual camera is `q_s`:
    /// the four corner pixels' centres land inside the frame's (the warp's own bounds), and since
    /// a homography in front of the camera maps the output rectangle onto a convex quadrilateral,
    /// so does everything between them.
    fn covers(&self, q_s: Quat, q_imu: Quat) -> bool {
        let r = self.optical_rotation(q_s, q_imu);
        // H⁻¹ = K · Rᵀ · K_out⁻¹.
        let inverse = self.config.intrinsics.matrix() * r.transpose() * self.k_out.inverse();
        let (right, bottom) = (
            f64::from(self.config.width) - 0.5,
            f64::from(self.config.height) - 0.5,
        );
        [(0.5, 0.5), (right, 0.5), (0.5, bottom), (right, bottom)]
            .iter()
            .all(|&(u, v)| {
                let [x, y, w] = inverse.mul_vec([u, v, 1.0]);
                w > 0.0
                    && (0.5 - 1e-9..=right + 1e-9).contains(&(x / w))
                    && (0.5 - 1e-9..=bottom + 1e-9).contains(&(y / w))
            })
    }

    /// `q_s`, or the nearest orientation toward `q_imu` that leaves no border in the output.
    ///
    /// The per-stage clamp is an angle, measured at the edges' midpoints ([`hidden_angles`]); a
    /// rotation about a diagonal, or a roll, uncovers a corner first, and the warp would fill it
    /// black. `q_imu` itself always covers (zoom >= 1), so the bisection has an answer.
    fn fit_to_frame(&self, q_s: Quat, q_imu: Quat) -> Quat {
        if self.covers(q_s, q_imu) {
            return q_s;
        }
        let (mut low, mut high) = (0.0, 1.0);
        for _ in 0..FIT_ITERATIONS {
            let mid = 0.5 * (low + high);
            if self.covers(q_s.slerp(q_imu, mid), q_imu) {
                high = mid;
            } else {
                low = mid;
            }
        }
        q_s.slerp(q_imu, high)
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

/// The largest pan and tilt, in radians, that the border uncovered by `zoom` can hide.
///
/// Measured where the output's edge is, not at its centre: the output edge sits at
/// `atan(half / (f · zoom))` off the axis and the frame's at `atan(half / f)`, and the difference
/// is what a rotation may use. Until 2026-10-20 this was the margin's angle at the centre,
/// `atan(half · (1 - 1/zoom) / f)`, which at fx 675 and zoom 1.3 allowed 7.0° of tilt where
/// 5.8° fit, and a clamped tilt showed a black band at the top or bottom edge.
#[must_use]
pub fn hidden_angles(intrinsics: &Intrinsics, zoom: f64, width: u32, height: u32) -> (f64, f64) {
    let zoom = zoom.max(1.0);
    let hidden = |near: f64, far: f64, f: f64| {
        let one_side = |half: f64| (half / f).atan() - (half / (f * zoom)).atan();
        one_side(near).min(one_side(far))
    };
    (
        hidden(
            intrinsics.cx,
            f64::from(width) - intrinsics.cx,
            intrinsics.fx,
        ),
        hidden(
            intrinsics.cy,
            f64::from(height) - intrinsics.cy,
            intrinsics.fy,
        ),
    )
}

/// The heading of a world_from_body orientation: its rotation about world z (the yaw of a
/// z-y-x decomposition). The IMU filter starts level with gravity, so world z is up and this is
/// the robot's turning; with a gyro-only filter (`mahony_kp` 0) it is about the IMU's first z,
/// tilted by the mount.
#[must_use]
pub fn heading(q: Quat) -> f64 {
    (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
}

/// `a` wrapped into `-π..=π`.
#[must_use]
pub fn wrap_angle(a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    a - tau * (a / tau).round()
}

/// One step of a first-order low-pass on a scalar, as [`smooth`] is on orientations.
#[must_use]
pub fn low_pass(prev: f64, current: f64, dt_s: f64, tau_s: f64) -> f64 {
    if tau_s <= 0.0 {
        return current;
    }
    prev + (current - prev) * (1.0 - (-dt_s / tau_s).exp())
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
            turn_damping: None,
            cam_from_imu: Mat3::IDENTITY,
        }
    }

    const DAMPING: TurnDamping = TurnDamping {
        rate_tau_s: 0.3,
        deadband_rad_s: 0.1,
        reference_rad_s: 0.15,
    };

    impl Stabilizer {
        /// `q_s` after the last frame.
        fn virtual_camera(&self) -> Quat {
            self.state.expect("a frame has been corrected").stages[1]
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
        let (pan, tilt) = hidden_angles(&K, cfg.zoom, cfg.width, cfg.height);
        let max = pan.min(tilt);
        // Vertical, at the edge: atan(360/640) - atan(360/(640 * 1.15)) = 0.0627 rad; the margin
        // at the centre, atan(46.96/640) = 0.0733 rad, was more than the edge can hide.
        let expected = (360.0f64 / 640.0).atan() - (360.0f64 / 736.0).atan();
        assert!((tilt - expected).abs() < 1e-9, "{tilt}");
        assert!(pan > tilt, "{pan} {tilt}");

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
        assert_eq!(s.state.unwrap().stages, [q; 2]);
    }

    /// The output pixel centres of the frame's four corners, through the inverse of `h`.
    fn corners_in_source(h: &Mat3) -> [(f64, f64); 4] {
        let inverse = h.inverse().expect("invertible");
        [(0.5, 0.5), (1279.5, 0.5), (0.5, 719.5), (1279.5, 719.5)].map(|(u, v)| map(&inverse, u, v))
    }

    #[test]
    fn no_output_pixel_is_ever_left_outside_the_frame() {
        // Rotations about diagonals and the optical axis reach a corner before an edge's midpoint,
        // which is all the angle clamp measures.
        for axis in [
            [1.0, 1.0, 0.0],
            [1.0, -1.0, 0.3],
            [0.0, 0.0, 1.0],
            [0.2, 0.9, 0.5],
        ] {
            for order in [SmoothingOrder::First, SmoothingOrder::Second] {
                let mut s = Stabilizer::new(EisConfig {
                    smoothing_order: order,
                    ..config()
                });
                s.correction(0, Quat::IDENTITY);
                for (i, degrees) in [3.0, 6.0, 10.0, 20.0].into_iter().enumerate() {
                    let q = Quat::from_axis_angle(axis, f64::to_radians(degrees));
                    let h = s.correction(i64::try_from(i + 1).unwrap(), q);
                    for (x, y) in corners_in_source(&h) {
                        assert!(
                            (0.5 - 1e-6..=1279.5 + 1e-6).contains(&x)
                                && (0.5 - 1e-6..=719.5 + 1e-6).contains(&y),
                            "{axis:?} {degrees} deg: corner at ({x}, {y})"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn a_correction_that_fits_is_not_touched_by_the_fit() {
        let mut s = Stabilizer::new(EisConfig {
            zoom: 2.0,
            ..config()
        });
        s.correction(0, Quat::IDENTITY);
        let q = Quat::from_axis_angle([1.0, 1.0, 0.0], 0.01);
        s.correction(1, q);
        let q_s = s.virtual_camera();
        assert_eq!(s.fit_to_frame(q_s, q), q_s);
    }

    #[test]
    fn turn_damping_leaves_slow_rates_alone_and_halves_tau_at_the_reference() {
        assert_eq!(DAMPING.effective_tau(0.35, 0.0), 0.35);
        assert_eq!(DAMPING.effective_tau(0.35, -0.1), 0.35);
        assert!((DAMPING.effective_tau(0.35, 0.25) - 0.175).abs() < 1e-12);
        assert!((DAMPING.effective_tau(0.35, -0.25) - 0.175).abs() < 1e-12);
        assert!(DAMPING.effective_tau(0.35, 0.5) < 0.05);
    }

    /// The virtual camera's heading through a turn about world z at `rate` rad/s for 3 s (ramped
    /// over 0.3 s at both ends), then 3 s standing, at 30 fps: (real, virtual) per frame.
    fn turn(damping: Option<TurnDamping>, rate: f64) -> Vec<(f64, f64)> {
        let mut s = Stabilizer::new(EisConfig {
            zoom: 1.3,
            smoothing_tau_s: 0.35,
            smoothing_order: SmoothingOrder::Second,
            turn_damping: damping,
            // A REP-103 body, so a turn about its z pans the image rather than rolling it.
            cam_from_imu: Mat3::from_rpy_deg(-90.0, 0.0, -90.0).transpose(),
            ..config()
        });
        let mut angle = 0.0;
        (0..180)
            .map(|i| {
                let t = f64::from(i) / 30.0;
                let ramp = (t / 0.3).min((3.0 - t) / 0.3).clamp(0.0, 1.0);
                angle += rate * ramp / 30.0;
                s.correction(
                    i64::from(i) * 33_333_333,
                    Quat::from_axis_angle([0.0, 0.0, 1.0], angle),
                );
                (angle, heading(s.virtual_camera()))
            })
            .collect()
    }

    #[test]
    fn with_turn_damping_the_view_stops_with_the_robot() {
        let travel_after_stop = |damping| {
            let headings = turn(damping, 0.5);
            let at_stop = headings[90].1;
            headings[90..]
                .iter()
                .map(|(_, v)| (v - at_stop).abs())
                .fold(0.0, f64::max)
        };
        // Without it the view trails the turn at the clamp and slides the clamp's worth after the
        // robot has stopped; with it, it is nearly there already.
        let plain = travel_after_stop(None);
        let damped = travel_after_stop(Some(DAMPING));
        assert!(plain > 4f64.to_radians(), "{}", plain.to_degrees());
        assert!(damped < 1f64.to_radians(), "{}", damped.to_degrees());
    }

    #[test]
    fn turn_damping_does_not_overshoot_the_end_of_a_turn() {
        for (real, virtual_heading) in turn(Some(DAMPING), 0.5) {
            assert!(virtual_heading <= real + 1e-9, "{virtual_heading} > {real}");
        }
    }

    #[test]
    fn turn_damping_leaves_the_gait_sway_smoothed() {
        // robot01's heading sways at 1.9 Hz; at 1.5 deg amplitude its rate peaks at 0.31 rad/s,
        // which the two rate stages bring well under the deadband.
        let sway_residual = |damping| {
            let mut s = Stabilizer::new(EisConfig {
                zoom: 2.0,
                smoothing_tau_s: 0.35,
                smoothing_order: SmoothingOrder::Second,
                turn_damping: damping,
                cam_from_imu: Mat3::from_rpy_deg(-90.0, 0.0, -90.0).transpose(),
                ..config()
            });
            let amplitude = 1.5f64.to_radians();
            let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
            for i in 0..180 {
                let t = f64::from(i) / 30.0;
                let angle = amplitude * (2.0 * std::f64::consts::PI * 1.9 * t).sin();
                s.correction(
                    (t * 1e9) as i64,
                    Quat::from_axis_angle([0.0, 0.0, 1.0], angle),
                );
                if i >= 120 {
                    low = low.min(heading(s.virtual_camera()));
                    high = high.max(heading(s.virtual_camera()));
                }
            }
            (high - low) / 2.0 / amplitude
        };
        let (plain, damped) = (sway_residual(None), sway_residual(Some(DAMPING)));
        assert!((damped - plain).abs() < 0.05 * plain, "{plain} vs {damped}");
    }

    #[test]
    fn heading_is_the_rotation_about_world_z_and_ignores_tilt() {
        let yaw = Quat::from_axis_angle([0.0, 0.0, 1.0], 0.7);
        assert!((heading(yaw) - 0.7).abs() < 1e-12);
        // A body pitched first (the lidar's mount) and then turned still heads the turn's way.
        let pitched = yaw * Quat::from_axis_angle([0.0, 1.0, 0.0], 0.25);
        assert!((heading(pitched) - 0.7).abs() < 1e-12);
        // Half a turn either way is the same angle; only the magnitude is pinned.
        assert!(
            (wrap_angle(3.0 * std::f64::consts::PI).abs() - std::f64::consts::PI).abs() < 1e-12
        );
        assert!((wrap_angle(2.0 * std::f64::consts::PI + 0.25) - 0.25).abs() < 1e-12);
        assert!((wrap_angle(-0.5) + 0.5).abs() < 1e-12);
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
