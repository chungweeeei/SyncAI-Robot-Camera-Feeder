//! IMU samples → a history of orientations, and the orientation at any instant in between.
//!
//! The subscription callback pushes every `sensor_msgs/Imu` into an [`ImuBuffer`], which
//! integrates it with a Mahony filter and keeps the last couple of seconds of orientations. The
//! warp, on the GStreamer streaming thread, asks for the orientation at a frame's capture time and
//! gets an interpolation between the two samples around it — or an extrapolation when the sample
//! after the frame has not arrived yet, which at 200 Hz against a frame that is already a few
//! milliseconds old is the common case.
//!
//! Pure: no ROS or GStreamer types, so the integration, the interpolation and the edge cases
//! (gaps, out-of-order samples, an empty buffer) are unit-tested on synthetic signals.

use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};

use super::math::Quat;

/// One IMU reading, already in the units the filter wants.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    /// Node-clock nanoseconds.
    pub stamp_ns: i64,
    pub gyro_rad_s: [f64; 3],
    /// Specific force; only its direction is used (gravity reference), so the scale matters only
    /// insofar as it must be consistent.
    pub accel_m_s2: [f64; 3],
}

/// Mahony filter gains. `kp` pulls the estimated gravity direction toward the accelerometer's,
/// `ki` learns a gyro bias from the persistent part of that error. Both zero is plain gyro
/// integration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MahonyGains {
    pub kp: f64,
    pub ki: f64,
}

/// The Mahony complementary filter (Mahony, Hamel, Pflimlin 2008), without a magnetometer:
/// heading is gyro-only and drifts freely, which is fine for stabilization — the smoother follows
/// the drift and only the *difference* between the raw and smoothed orientation reaches the image.
#[derive(Clone, Debug)]
pub struct Mahony {
    /// world_from_body.
    q: Quat,
    /// The integral feedback term (an estimated gyro bias, rad/s).
    integral: [f64; 3],
    gains: MahonyGains,
}

impl Mahony {
    #[must_use]
    pub fn new(gains: MahonyGains) -> Self {
        Self {
            q: Quat::IDENTITY,
            integral: [0.0; 3],
            gains,
        }
    }

    #[must_use]
    pub fn orientation(&self) -> Quat {
        self.q
    }

    pub fn reset(&mut self) {
        self.q = Quat::IDENTITY;
        self.integral = [0.0; 3];
    }

    /// Advances the orientation by one sample of `dt_s` seconds. `accel` is the measured
    /// specific force in the body frame (any consistent unit); `None` or a zero vector skips the
    /// gravity correction for this step.
    pub fn update(&mut self, gyro: [f64; 3], accel: Option<[f64; 3]>, dt_s: f64) -> Quat {
        let mut omega = gyro;
        if let Some(a) = accel.and_then(normalized) {
            // The estimated "up" in the body frame: world +z brought into the body.
            let v = self.q.conjugate().rotate([0.0, 0.0, 1.0]);
            // The error is the rotation that takes v onto a (a × v is the small-angle axis).
            let e = [
                a[1] * v[2] - a[2] * v[1],
                a[2] * v[0] - a[0] * v[2],
                a[0] * v[1] - a[1] * v[0],
            ];
            for i in 0..3 {
                self.integral[i] += self.gains.ki * e[i] * dt_s;
                omega[i] += self.gains.kp * e[i] + self.integral[i];
            }
        }
        let step = Quat::from_rotation_vector([omega[0] * dt_s, omega[1] * dt_s, omega[2] * dt_s]);
        self.q = (self.q * step).normalized();
        self.q
    }
}

fn normalized(v: [f64; 3]) -> Option<[f64; 3]> {
    let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    (norm > 0.0 && norm.is_finite()).then(|| [v[0] / norm, v[1] / norm, v[2] / norm])
}

/// The filter's orientation right after one sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Orientation {
    pub stamp_ns: i64,
    pub q: Quat,
}

/// What the buffer knows about the orientation at a requested instant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OrientationAt {
    /// Between two samples (or at/before the oldest one kept).
    Interpolated(Quat),
    /// After the newest sample, by `gap_ns`, integrated forward with the last gyro reading.
    Extrapolated { q: Quat, gap_ns: i64 },
    /// After the newest sample by more than the extrapolation limit: the IMU has stopped, or is
    /// too far behind to be useful.
    Stale { age_ns: i64 },
    /// No sample has ever arrived.
    Empty,
}

/// Beyond this, extrapolating with the last gyro reading is a guess rather than a prediction.
pub const MAX_EXTRAPOLATION_NS: i64 = 50_000_000;

/// A silence longer than this between two samples restarts the filter: the orientation before the
/// gap says nothing about the one after it.
const RESET_GAP_NS: i64 = 1_000_000_000;

/// Orientation history, shared between the IMU subscription (producer) and the warp (consumer).
#[derive(Clone, Debug)]
pub struct ImuBuffer {
    filter: Mahony,
    history: VecDeque<Orientation>,
    last_gyro: [f64; 3],
    capacity: usize,
}

impl ImuBuffer {
    /// `capacity` samples are kept (400 is 2 s at 200 Hz).
    #[must_use]
    pub fn new(capacity: usize, gains: MahonyGains) -> Self {
        Self {
            filter: Mahony::new(gains),
            history: VecDeque::with_capacity(capacity.max(1)),
            last_gyro: [0.0; 3],
            capacity: capacity.max(1),
        }
    }

    /// Integrates one sample. Returns `false`, storing nothing, for a sample that is not finite
    /// or not newer than the previous one (DDS can reorder best-effort samples; integrating one
    /// backwards would corrupt the orientation).
    pub fn push(&mut self, sample: ImuSample) -> bool {
        let finite = sample
            .gyro_rad_s
            .iter()
            .chain(sample.accel_m_s2.iter())
            .all(|v| v.is_finite());
        if !finite {
            return false;
        }
        let q = match self.history.back() {
            Some(previous) if sample.stamp_ns <= previous.stamp_ns => return false,
            Some(previous) => {
                let gap = sample.stamp_ns - previous.stamp_ns;
                if gap > RESET_GAP_NS {
                    self.filter.reset();
                    self.history.clear();
                    // The first sample after a gap only sets the time base.
                    self.filter.orientation()
                } else {
                    let dt_s = gap as f64 / 1e9;
                    self.filter
                        .update(sample.gyro_rad_s, Some(sample.accel_m_s2), dt_s)
                }
            }
            None => self.filter.orientation(),
        };
        if self.history.len() == self.capacity {
            self.history.pop_front();
        }
        self.history.push_back(Orientation {
            stamp_ns: sample.stamp_ns,
            q,
        });
        self.last_gyro = sample.gyro_rad_s;
        true
    }

    #[must_use]
    pub fn newest_stamp_ns(&self) -> Option<i64> {
        self.history.back().map(|o| o.stamp_ns)
    }

    /// The orientation at `t_ns`, see [`OrientationAt`].
    #[must_use]
    pub fn orientation_at(&self, t_ns: i64) -> OrientationAt {
        let (Some(oldest), Some(newest)) = (self.history.front(), self.history.back()) else {
            return OrientationAt::Empty;
        };
        if t_ns <= oldest.stamp_ns {
            return OrientationAt::Interpolated(oldest.q);
        }
        if t_ns > newest.stamp_ns {
            let gap_ns = t_ns - newest.stamp_ns;
            if gap_ns > MAX_EXTRAPOLATION_NS {
                return OrientationAt::Stale { age_ns: gap_ns };
            }
            let dt_s = gap_ns as f64 / 1e9;
            let step = Quat::from_rotation_vector([
                self.last_gyro[0] * dt_s,
                self.last_gyro[1] * dt_s,
                self.last_gyro[2] * dt_s,
            ]);
            return OrientationAt::Extrapolated {
                q: (newest.q * step).normalized(),
                gap_ns,
            };
        }
        // The first sample at or after t; it has a predecessor because t > oldest.stamp_ns.
        let after = self.history.partition_point(|o| o.stamp_ns < t_ns);
        let (a, b) = (self.history[after - 1], self.history[after]);
        let span = (b.stamp_ns - a.stamp_ns) as f64;
        let fraction = if span > 0.0 {
            (t_ns - a.stamp_ns) as f64 / span
        } else {
            1.0
        };
        OrientationAt::Interpolated(a.q.slerp(b.q, fraction))
    }
}

/// The buffer plus the condition variable the warp waits on for a sample newer than its frame.
pub type SharedImu = Arc<(Mutex<ImuBuffer>, Condvar)>;

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const HZ_200: i64 = 5_000_000;
    const GAINS: MahonyGains = MahonyGains { kp: 0.5, ki: 0.0 };
    const GYRO_ONLY: MahonyGains = MahonyGains { kp: 0.0, ki: 0.0 };

    fn yaw_rate_samples(rate: f64, count: usize) -> impl Iterator<Item = ImuSample> {
        (0..count).map(move |i| ImuSample {
            stamp_ns: i as i64 * HZ_200,
            gyro_rad_s: [0.0, 0.0, rate],
            accel_m_s2: [0.0, 0.0, 9.81],
        })
    }

    fn buffer_with(samples: impl Iterator<Item = ImuSample>) -> ImuBuffer {
        let mut buffer = ImuBuffer::new(400, GAINS);
        for s in samples {
            assert!(buffer.push(s));
        }
        buffer
    }

    #[test]
    fn a_constant_yaw_rate_integrates_to_rate_times_time() {
        // 201 samples span exactly 1 s.
        let buffer = buffer_with(yaw_rate_samples(0.5, 201));
        let OrientationAt::Interpolated(q) = buffer.orientation_at(200 * HZ_200) else {
            panic!("the newest stamp is interpolated");
        };
        assert!((q.angle() - 0.5).abs() < 1e-9, "{}", q.angle());
        // About +z, the right-hand way: x goes toward y.
        let v = q.rotate([1.0, 0.0, 0.0]);
        assert!(v[1] > 0.0 && v[2].abs() < 1e-9, "{v:?}");
    }

    #[test]
    fn between_two_samples_the_orientation_is_interpolated() {
        let buffer = buffer_with(yaw_rate_samples(1.0, 3));
        // Sample 1 is at 5 ms (0.005 rad), sample 2 at 10 ms (0.010 rad); ask for 7.5 ms.
        let OrientationAt::Interpolated(q) = buffer.orientation_at(7_500_000) else {
            panic!("inside the history");
        };
        assert!((q.angle() - 0.0075).abs() < 1e-9, "{}", q.angle());
    }

    #[test]
    fn before_the_oldest_sample_the_oldest_orientation_is_used() {
        let buffer = buffer_with(yaw_rate_samples(1.0, 3).map(|s| ImuSample {
            stamp_ns: s.stamp_ns + 1_000_000_000,
            ..s
        }));
        assert_eq!(
            buffer.orientation_at(0),
            OrientationAt::Interpolated(Quat::IDENTITY)
        );
    }

    #[test]
    fn past_the_newest_sample_the_last_gyro_rate_extrapolates() {
        let buffer = buffer_with(yaw_rate_samples(1.0, 3));
        let OrientationAt::Extrapolated { q, gap_ns } = buffer.orientation_at(12_000_000) else {
            panic!("2 ms past the newest sample");
        };
        assert_eq!(gap_ns, 2_000_000);
        assert!((q.angle() - 0.012).abs() < 1e-9, "{}", q.angle());
    }

    #[test]
    fn too_far_past_the_newest_sample_is_stale() {
        let buffer = buffer_with(yaw_rate_samples(1.0, 3));
        assert_eq!(
            buffer.orientation_at(10_000_000 + MAX_EXTRAPOLATION_NS + 1),
            OrientationAt::Stale {
                age_ns: MAX_EXTRAPOLATION_NS + 1
            }
        );
    }

    #[test]
    fn an_empty_buffer_says_so() {
        let buffer = ImuBuffer::new(400, GAINS);
        assert_eq!(buffer.orientation_at(0), OrientationAt::Empty);
        assert_eq!(buffer.newest_stamp_ns(), None);
    }

    #[test]
    fn out_of_order_and_non_finite_samples_are_rejected() {
        let mut buffer = buffer_with(yaw_rate_samples(1.0, 3));
        let stale = ImuSample {
            stamp_ns: 5_000_000,
            gyro_rad_s: [0.0; 3],
            accel_m_s2: [0.0, 0.0, 1.0],
        };
        assert!(!buffer.push(stale));
        let nan = ImuSample {
            stamp_ns: 15_000_000,
            gyro_rad_s: [f64::NAN, 0.0, 0.0],
            accel_m_s2: [0.0, 0.0, 1.0],
        };
        assert!(!buffer.push(nan));
        assert_eq!(buffer.newest_stamp_ns(), Some(10_000_000));
    }

    #[test]
    fn a_long_silence_restarts_the_filter() {
        let mut buffer = buffer_with(yaw_rate_samples(1.0, 201));
        let after_gap = ImuSample {
            stamp_ns: 200 * HZ_200 + RESET_GAP_NS + 1,
            gyro_rad_s: [0.0, 0.0, 1.0],
            accel_m_s2: [0.0, 0.0, 1.0],
        };
        assert!(buffer.push(after_gap));
        assert_eq!(
            buffer.orientation_at(after_gap.stamp_ns),
            OrientationAt::Interpolated(Quat::IDENTITY)
        );
        assert_eq!(buffer.newest_stamp_ns(), Some(after_gap.stamp_ns));
    }

    #[test]
    fn the_history_is_bounded() {
        let mut buffer = ImuBuffer::new(10, GAINS);
        for s in yaw_rate_samples(1.0, 50) {
            buffer.push(s);
        }
        assert_eq!(buffer.history.len(), 10);
        assert_eq!(buffer.history.front().unwrap().stamp_ns, 40 * HZ_200);
    }

    #[test]
    fn the_accelerometer_pulls_the_tilt_toward_gravity() {
        // The body lies on its side: gravity (the reaction force) is along body +x. With the gyro
        // silent, only the accelerometer term can move the estimate, and it must converge to an
        // orientation whose body-frame "up" is +x.
        let mut filter = Mahony::new(MahonyGains { kp: 2.0, ki: 0.0 });
        for _ in 0..4000 {
            filter.update([0.0; 3], Some([9.81, 0.0, 0.0]), 0.005);
        }
        let up_in_body = filter.orientation().conjugate().rotate([0.0, 0.0, 1.0]);
        assert!((up_in_body[0] - 1.0).abs() < 1e-3, "{up_in_body:?}");
        assert!((filter.orientation().angle() - FRAC_PI_2).abs() < 1e-3);
    }

    #[test]
    fn gyro_only_integration_ignores_the_accelerometer() {
        let mut filter = Mahony::new(GYRO_ONLY);
        for _ in 0..100 {
            filter.update([0.0; 3], Some([9.81, 0.0, 0.0]), 0.005);
        }
        assert_eq!(filter.orientation(), Quat::IDENTITY);
    }

    #[test]
    fn the_integral_term_learns_a_constant_bias() {
        // A gyro that reads a steady 0.1 rad/s about y while the accelerometer insists the body is
        // level: with ki > 0 the bias estimate grows to cancel the reading, and the orientation
        // stops drifting. Compare the drift over the last second with the first.
        let mut filter = Mahony::new(MahonyGains { kp: 1.0, ki: 0.5 });
        let drift = |filter: &mut Mahony| {
            let start = filter.orientation();
            for _ in 0..200 {
                filter.update([0.0, 0.1, 0.0], Some([0.0, 0.0, 9.81]), 0.005);
            }
            (start.conjugate() * filter.orientation()).angle()
        };
        let first_second = drift(&mut filter);
        for _ in 0..20 {
            drift(&mut filter);
        }
        let later_second = drift(&mut filter);
        assert!(
            later_second < first_second / 5.0,
            "{first_second} -> {later_second}"
        );
    }
}
