//! The rate limit down to `ros_framerate`, as pure arithmetic over PTS: which of the camera's
//! frames to keep, without touching the timestamp of any of them.
//!
//! This used to be `videorate drop-only=true`, which keeps the right frames but restamps every
//! one it keeps onto its own output grid. The grid starts at running time 0, and a kept frame
//! goes out with its slot's time: its capture time minus however far past the slot it landed. On
//! robot01 (60 → 30 fps) a frame captured at 0.410456 s went out as 0.400000 s. Every message
//! stamp was therefore early by an amount fixed for the run and anywhere from 0 to one capture
//! period (16.7 ms) from one start to the next, and the warp looked up the IMU at that instant.
//! Nothing errored; the one tell was the stamps themselves, which sat on a perfect 33.333 ms grid
//! (period jitter 0.02 ms against the camera's own ~0.3 ms).
//!
//! The node hangs this on a pad probe (`mod.rs`) that drops what [`RateLimiter::keep`] rejects, so
//! every buffer that passes is the camera's own, PTS and all.

const NS_PER_SEC: u128 = 1_000_000_000;

/// Decides, frame by frame, which frames of a `source_fps` stream to keep for `target_fps`.
///
/// Output slots lie `1 / target_fps` apart on a grid anchored at the first frame kept. A frame is
/// kept once it is due for the next slot: within half a source period of it. The tolerance is
/// what keeps the choice stable when the grid sits on the frames, as it does whenever
/// `target_fps` divides `source_fps`. With an exact comparison, capture jitter either side of a
/// slot flips the pick between two neighbouring frames, and a steady 33/33 ms turns into 17/50.
///
/// Slot times are computed from the anchor and a count rather than accumulated, so they never
/// drift off the frames, and the long-run rate is exactly `target_fps` also when it does not
/// divide `source_fps` (60 → 25 alternates 33 and 50 ms).
#[derive(Clone, Debug)]
pub struct RateLimiter {
    target_fps: u128,
    tolerance_ns: u64,
    /// The grid's first slot and how many of its slots have been filled; `None` until the first
    /// frame with a PTS.
    grid: Option<Grid>,
}

#[derive(Clone, Copy, Debug)]
struct Grid {
    anchor_ns: u64,
    /// At least 1: the anchor frame fills the first slot.
    filled: u64,
}

impl RateLimiter {
    /// Meant for `target_fps` below `source_fps`; a zero for either is taken as 1.
    #[must_use]
    pub fn new(source_fps: u32, target_fps: u32) -> Self {
        let source_period_ns = NS_PER_SEC / u128::from(source_fps.max(1));
        Self {
            target_fps: u128::from(target_fps.max(1)),
            tolerance_ns: u64::try_from(source_period_ns / 2).unwrap_or(u64::MAX),
            grid: None,
        }
    }

    /// Whether to keep the frame with presentation time `pts_ns`.
    ///
    /// A frame without a PTS is kept and leaves the grid alone: there is nothing to place it by,
    /// and letting one through beats starving the topic.
    pub fn keep(&mut self, pts_ns: Option<u64>) -> bool {
        let Some(pts) = pts_ns else {
            return true;
        };
        let reached = pts.saturating_add(self.tolerance_ns);
        if let Some(grid) = self.grid {
            let previous = self.slot_ns(grid, grid.filled - 1);
            let next = self.slot_ns(grid, grid.filled);
            if reached < next {
                // Early for the next slot. Unless it is early for the previous one as well, which
                // only a PTS that went backwards gets, the frame is dropped.
                if reached >= previous {
                    return false;
                }
            } else if reached < self.slot_ns(grid, grid.filled + 1) {
                self.grid = Some(Grid {
                    filled: grid.filled + 1,
                    ..grid
                });
                return true;
            }
            // Due for the slot after the next one too: the source skipped a whole output period
            // or more (a stall, frames lost upstream). Restarting the grid here keeps the rate;
            // filling the missed slots would let every frame through until it caught up.
        }
        self.grid = Some(Grid {
            anchor_ns: pts,
            filled: 1,
        });
        true
    }

    fn slot_ns(&self, grid: Grid, index: u64) -> u64 {
        let offset = u128::from(index) * NS_PER_SEC / self.target_fps;
        grid.anchor_ns
            .saturating_add(u64::try_from(offset).unwrap_or(u64::MAX))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The capture time of frame `index` of a `fps` stream starting at `start_ns`, the way the
    /// camera spaces them (nanosecond floor of an exact period).
    fn frame_ns(start_ns: u64, fps: u64, index: u64) -> u64 {
        start_ns + index * 1_000_000_000 / fps
    }

    /// A deterministic jitter in -3..=3 ms, several times the camera's usual (its PTS period stays
    /// within ~±0.4 ms of 16.67). What the tolerance has to cover is a frame's jitter relative to
    /// the anchor frame's, so up to 6 ms here against 8.3.
    fn jitter_ns(index: u64) -> i64 {
        ((index * 7919 % 7) as i64 - 3) * 1_000_000
    }

    fn kept_indices(limiter: &mut RateLimiter, pts: impl Iterator<Item = u64>) -> Vec<usize> {
        pts.enumerate()
            .filter(|(_, pts)| limiter.keep(Some(*pts)))
            .map(|(index, _)| index)
            .collect()
    }

    #[test]
    fn halving_keeps_every_other_frame_from_the_first() {
        let mut limiter = RateLimiter::new(60, 30);
        let pts = (0..120).map(|i| frame_ns(3_000_000_000, 60, i));
        let kept = kept_indices(&mut limiter, pts);
        assert_eq!(kept, (0..120).step_by(2).collect::<Vec<_>>());
    }

    #[test]
    fn capture_jitter_does_not_flip_the_choice() {
        // The grid sits exactly on the frames (30 divides 60), the case an exact comparison gets
        // wrong: every frame lands a few ms either side of its slot.
        let mut limiter = RateLimiter::new(60, 30);
        let pts =
            (0..600).map(|i| frame_ns(1_000_000_000, 60, i).saturating_add_signed(jitter_ns(i)));
        let kept = kept_indices(&mut limiter, pts);
        assert_eq!(kept, (0..600).step_by(2).collect::<Vec<_>>());
    }

    #[test]
    fn slots_are_computed_so_they_never_drift_off_the_frames() {
        // Thirty days of slots in. A 33_333_333 ns period accumulated instead of computed from
        // the anchor would sit 26 ms early by now, past the 8.3 ms tolerance, and keep the odd
        // frames instead.
        let mut limiter = RateLimiter::new(60, 30);
        let slots = 30 * 86_400 * 30;
        limiter.grid = Some(Grid {
            anchor_ns: 0,
            filled: slots,
        });
        let first = 2 * slots;
        let kept = kept_indices(
            &mut limiter,
            (first..first + 120).map(|i| frame_ns(0, 60, i)),
        );
        assert_eq!(kept, (0..120).step_by(2).collect::<Vec<_>>());
    }

    #[test]
    fn a_rate_that_does_not_divide_the_source_is_met_on_average() {
        let mut limiter = RateLimiter::new(60, 25);
        let pts: Vec<u64> = (0..600).map(|i| frame_ns(0, 60, i)).collect();
        let kept: Vec<u64> = pts
            .iter()
            .copied()
            .filter(|p| limiter.keep(Some(*p)))
            .collect();
        // Ten seconds of source at 25 fps.
        assert_eq!(kept.len(), 250);
        for pair in kept.windows(2) {
            let period_ms = (pair[1] - pair[0]) as f64 / 1e6;
            assert!(
                (period_ms - 33.33).abs() < 0.1 || (period_ms - 50.0).abs() < 0.1,
                "{period_ms}"
            );
        }
    }

    #[test]
    fn a_stall_restarts_the_grid_instead_of_bursting() {
        let mut limiter = RateLimiter::new(60, 30);
        let before: Vec<u64> = (0..10).map(|i| frame_ns(0, 60, i)).collect();
        // 300 ms without a frame, then the stream resumes.
        let after: Vec<u64> = (0..60).map(|i| frame_ns(450_000_000, 60, i)).collect();
        let kept: Vec<u64> = before
            .iter()
            .chain(&after)
            .copied()
            .filter(|p| limiter.keep(Some(*p)))
            .collect();
        let resumed: Vec<u64> = kept.into_iter().filter(|p| *p >= after[0]).collect();
        assert_eq!(resumed[0], after[0]);
        for pair in resumed.windows(2) {
            // Two source frames apart: 33_333_333 or _334 ns, as the floored spacing falls.
            assert!((pair[1] - pair[0]).abs_diff(33_333_333) <= 1, "{resumed:?}");
        }
    }

    #[test]
    fn a_pts_that_goes_backwards_restarts_the_grid_instead_of_stalling() {
        let mut limiter = RateLimiter::new(60, 30);
        for i in 0..60 {
            limiter.keep(Some(frame_ns(10_000_000_000, 60, i)));
        }
        let kept = kept_indices(&mut limiter, (0..10).map(|i| frame_ns(0, 60, i)));
        assert_eq!(kept, [0, 2, 4, 6, 8]);
    }

    #[test]
    fn a_frame_without_a_pts_is_kept_and_leaves_the_grid_alone() {
        let mut limiter = RateLimiter::new(60, 30);
        assert!(limiter.keep(Some(0)));
        assert!(limiter.keep(None));
        assert!(!limiter.keep(Some(frame_ns(0, 60, 1))));
        assert!(limiter.keep(Some(frame_ns(0, 60, 2))));
    }

    #[test]
    fn zero_rates_do_not_divide_by_zero() {
        // Taken as 1 → 1 fps: slots 1 s apart, 0.5 s tolerance.
        let mut limiter = RateLimiter::new(0, 0);
        assert!(limiter.keep(Some(0)));
        assert!(!limiter.keep(Some(400_000_000)));
        assert!(limiter.keep(Some(1_000_000_000)));
    }
}
