//! Capture-time stamps, as pure arithmetic over nanosecond counts.
//!
//! A frame is stamped with when it was captured, not when it reached the node: the node clock's
//! current time minus the frame's age on the pipeline clock. The age is what USB transfer, the tee
//! and the queue added, and it varies frame to frame, so stamping on arrival (what
//! `vizionsdk_ros2` does with `this->now()`) carries that jitter into every consumer.
//!
//! The two clocks are never compared directly — the pipeline clock is monotonic and the node clock
//! is ROS time — only the *difference* between two readings of the pipeline clock crosses over.

/// One frame's readings off the pipeline clock, in nanoseconds.
///
/// Named fields rather than three positional `Option<u64>` arguments, which the compiler would let
/// a caller pass in any order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FrameTiming {
    /// The pipeline clock, read when the frame arrives.
    pub clock_now: Option<u64>,
    /// The pipeline's base time (pipeline clock at running time 0).
    pub base_time: Option<u64>,
    /// The buffer's presentation timestamp, in running time.
    pub pts: Option<u64>,
}

/// The message stamp for one frame, in nanoseconds on the node clock.
///
/// `ros_now` is the node clock read at the same moment as `timing.clock_now`.
///
/// Falls back to `ros_now` when any pipeline reading is missing, so a source that does not
/// timestamp still gets a usable (arrival-time) stamp. A frame whose capture time lies in the
/// pipeline clock's future has age 0 rather than a stamp ahead of `ros_now`.
#[must_use]
pub fn capture_stamp_ns(ros_now: i64, timing: FrameTiming) -> i64 {
    let FrameTiming {
        clock_now: Some(now),
        base_time: Some(base),
        pts: Some(pts),
    } = timing
    else {
        return ros_now;
    };
    let age = now.saturating_sub(base.saturating_add(pts));
    ros_now.saturating_sub(i64::try_from(age).unwrap_or(i64::MAX))
}

/// Splits nanoseconds into `builtin_interfaces/Time`'s `(sec, nanosec)`.
///
/// Floor division, so `nanosec` is always in `0..1_000_000_000` (also for a negative input), and
/// `sec` saturates at the `i32` range rather than wrapping.
#[must_use]
pub fn to_sec_nanosec(ns: i64) -> (i32, u32) {
    const NS_PER_SEC: i64 = 1_000_000_000;
    let sec = ns.div_euclid(NS_PER_SEC);
    // rem_euclid is in 0..NS_PER_SEC, which always fits a u32
    let nanosec = u32::try_from(ns.rem_euclid(NS_PER_SEC)).unwrap_or(0);
    let sec = i32::try_from(sec).unwrap_or(if sec < 0 { i32::MIN } else { i32::MAX });
    (sec, nanosec)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ROS_NOW: i64 = 1_791_426_165_350_043_078;

    #[test]
    fn subtracts_the_frames_age() {
        // Captured at pipeline time 1_000 + 4_000 = 5_000, observed at 25_000: 20 µs old.
        assert_eq!(
            capture_stamp_ns(
                ROS_NOW,
                FrameTiming {
                    clock_now: Some(25_000),
                    base_time: Some(1_000),
                    pts: Some(4_000)
                }
            ),
            ROS_NOW - 20_000
        );
    }

    #[test]
    fn missing_readings_fall_back_to_arrival_time() {
        assert_eq!(
            capture_stamp_ns(
                ROS_NOW,
                FrameTiming {
                    clock_now: None,
                    base_time: Some(1),
                    pts: Some(1)
                }
            ),
            ROS_NOW
        );
        assert_eq!(
            capture_stamp_ns(
                ROS_NOW,
                FrameTiming {
                    clock_now: Some(1),
                    base_time: None,
                    pts: Some(1)
                }
            ),
            ROS_NOW
        );
        assert_eq!(
            capture_stamp_ns(
                ROS_NOW,
                FrameTiming {
                    clock_now: Some(1),
                    base_time: Some(1),
                    pts: None
                }
            ),
            ROS_NOW
        );
    }

    #[test]
    fn a_capture_time_in_the_future_is_age_zero() {
        assert_eq!(
            capture_stamp_ns(
                ROS_NOW,
                FrameTiming {
                    clock_now: Some(1_000),
                    base_time: Some(1_000),
                    pts: Some(5_000)
                }
            ),
            ROS_NOW
        );
    }

    #[test]
    fn huge_values_saturate_instead_of_overflowing() {
        assert_eq!(
            capture_stamp_ns(
                ROS_NOW,
                FrameTiming {
                    clock_now: Some(u64::MAX),
                    base_time: Some(0),
                    pts: Some(0)
                }
            ),
            ROS_NOW - i64::MAX
        );
        assert_eq!(
            capture_stamp_ns(
                ROS_NOW,
                FrameTiming {
                    clock_now: Some(10),
                    base_time: Some(u64::MAX),
                    pts: Some(u64::MAX)
                }
            ),
            ROS_NOW
        );
    }

    #[test]
    fn splits_into_seconds_and_nanoseconds() {
        assert_eq!(to_sec_nanosec(1_500_000_001), (1, 500_000_001));
        assert_eq!(to_sec_nanosec(0), (0, 0));
        assert_eq!(to_sec_nanosec(-1), (-1, 999_999_999));
    }

    #[test]
    fn seconds_saturate_at_the_i32_range() {
        assert_eq!(to_sec_nanosec(i64::MAX).0, i32::MAX);
        assert_eq!(to_sec_nanosec(i64::MIN).0, i32::MIN);
    }
}
