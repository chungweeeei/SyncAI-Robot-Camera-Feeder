//! The bridge between the two stabilized pipelines: the raw appsink's callback, which warps each
//! decoded frame by the IMU orientation and pushes it into the output pipeline's appsrc.
//!
//! Runs on the capture pipeline's streaming thread that delivers the decoded frames (the queue's
//! in front of `jpegdec`, or `nvv4l2decoder`'s own output thread), with the warp itself fanned
//! out across rayon's workers. Like `frame_publisher`, it must never return an error to
//! GStreamer — that would stop the capture — and must not capture either pipeline: the capture
//! pipeline owns the appsink, the appsink owns this closure, and the closure owning the pipeline
//! would be a cycle. Holding the output pipeline's *appsrc* is fine: a child element does not own
//! its parent.
//!
//! Not pure (GStreamer buffers, locks, a clock), so it is kept to glue: the orientation lookup is
//! `imu.rs`, the homography `eis.rs`, the pixels `warp.rs`, the stamp `stamp.rs`.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use gstreamer_video as gst_video;
// VideoFrameExt: plane_stride() and friends on a VideoFrameRef.
use gstreamer_video::prelude::*;
use rclrs::*;

use super::eis::Stabilizer;
use super::imu::{OrientationAt, SharedImu};
use super::math::Mat3;
use super::stamp::{self, FrameTiming};
use super::warp::{self, Plane, PlaneMut};

const LOG_THROTTLE: Duration = Duration::from_secs(5);
const STATS_THROTTLE: Duration = Duration::from_secs(30);

/// The capture stamps of the frames in flight between the two pipelines, keyed by PTS.
///
/// The capture stamp is computed here, on the capture pipeline's clock, but published from the
/// output pipeline's appsink, which runs on a different clock with a different base time. The
/// PTS is the one thing that crosses unchanged (`jpegenc`, `tee` and `queue` all keep it), so the
/// stamp travels beside the buffer in this table rather than on it.
#[derive(Debug, Default)]
pub struct StampTable {
    entries: VecDeque<(u64, i64)>,
}

impl StampTable {
    /// More than the output pipeline can hold (appsrc 2 + encoder 1 + queue 3 + appsink 2).
    const CAPACITY: usize = 16;

    pub fn insert(&mut self, pts: u64, stamp_ns: i64) {
        if self.entries.len() == Self::CAPACITY {
            self.entries.pop_front();
        }
        self.entries.push_back((pts, stamp_ns));
    }

    /// The stamp recorded for `pts`, forgetting it and everything recorded before it (a frame
    /// the output pipeline dropped is never asked for).
    pub fn take(&mut self, pts: u64) -> Option<i64> {
        let index = self.entries.iter().position(|(p, _)| *p == pts)?;
        let (_, stamp) = self.entries[index];
        self.entries.drain(..=index);
        Some(stamp)
    }
}

pub type SharedStamps = Arc<Mutex<StampTable>>;

/// How a frame's capture time is placed on the IMU timeline.
#[derive(Clone, Copy, Debug)]
pub struct WarpTiming {
    /// Added to the capture stamp (configured offset plus half the exposure).
    pub frame_time_offset_ns: i64,
    /// How long to wait for an IMU sample newer than the frame before extrapolating.
    pub max_imu_wait: Duration,
    /// The output buffers' duration (one frame at the stabilized rate).
    pub frame_duration: gst::ClockTime,
}

/// The raw appsink's callbacks. See the module docs.
pub fn raw_frame_warper(
    appsrc: gst_app::AppSrc,
    imu: SharedImu,
    mut stabilizer: Stabilizer,
    timing: WarpTiming,
    stamps: SharedStamps,
    clock: Clock,
    logger: Logger,
) -> gst_app::AppSinkCallbacks {
    let (width, height) = (stabilizer.config().width, stabilizer.config().height);
    let output_info = gst_video::VideoInfo::builder(gst_video::VideoFormat::I420, width, height)
        .build()
        .ok();
    let mut stabilizing = false;

    gst_app::AppSinkCallbacks::builder()
        .new_sample(move |sink| {
            let sample = sink.pull_sample().map_err(|_| gst::FlowError::Eos)?;
            let (Some(buffer), Some(caps)) = (sample.buffer(), sample.caps()) else {
                return Ok(gst::FlowSuccess::Ok);
            };
            let Some(output_info) = output_info.as_ref() else {
                log_error!(
                    logger.throttle(LOG_THROTTLE),
                    "[WarpBridge] no I420 layout for {width}x{height}; frames dropped"
                );
                return Ok(gst::FlowSuccess::Ok);
            };

            // The capture stamp, exactly as the passthrough path computes it.
            let ros_now = clock.now().nsec;
            let frame_timing = FrameTiming {
                clock_now: sink.clock().map(|c| c.time().nseconds()),
                base_time: sink.base_time().map(gst::ClockTime::nseconds),
                pts: buffer.pts().map(gst::ClockTime::nseconds),
            };
            let stamp_ns = stamp::capture_stamp_ns(ros_now, frame_timing);
            let frame_time_ns = stamp_ns.saturating_add(timing.frame_time_offset_ns);
            // Capture to here: the transfer, the capture queue and the decoder — the one figure
            // that tells the jpeg_codec choices apart on a running robot.
            let age_ms = ros_now.saturating_sub(stamp_ns) as f64 / 1e6;

            let waited = Instant::now();
            let orientation = orientation_at(&imu, frame_time_ns, timing.max_imu_wait);
            let wait_ms = waited.elapsed().as_secs_f64() * 1e3;

            let homography = match orientation {
                OrientationAt::Interpolated(q) | OrientationAt::Extrapolated { q, .. } => {
                    if !stabilizing {
                        log_info!(&logger, "[WarpBridge] IMU samples arriving; stabilizing");
                        stabilizing = true;
                    }
                    stabilizer.correction(frame_time_ns, q)
                }
                OrientationAt::Stale { age_ns } => {
                    log_warn!(
                        logger.throttle(LOG_THROTTLE),
                        "[WarpBridge] the newest IMU sample is {} ms older than the frame; \
                         passing frames through unstabilized",
                        age_ns / 1_000_000,
                    );
                    stabilizing = false;
                    stabilizer.reset();
                    stabilizer.passthrough()
                }
                OrientationAt::Empty => {
                    log_warn!(
                        logger.throttle(LOG_THROTTLE),
                        "[WarpBridge] no IMU samples yet; passing frames through unstabilized"
                    );
                    stabilizing = false;
                    stabilizer.reset();
                    stabilizer.passthrough()
                }
            };
            // The warp maps destination pixels back to their source.
            let inverse = homography
                .inverse()
                .or_else(|| stabilizer.passthrough().inverse())
                .unwrap_or(Mat3::IDENTITY);

            let started = Instant::now();
            let warped = match warp_frame(buffer, caps, output_info, &inverse) {
                Ok(warped) => warped,
                Err(err) => {
                    log_error!(
                        logger.throttle(LOG_THROTTLE),
                        "[WarpBridge] frame dropped: {err}"
                    );
                    return Ok(gst::FlowSuccess::Ok);
                }
            };
            let warp_ms = started.elapsed().as_secs_f64() * 1e3;

            let mut out = gst::Buffer::from_mut_slice(warped);
            if let Some(out) = out.get_mut() {
                out.set_pts(buffer.pts());
                out.set_duration(timing.frame_duration);
            }
            if let Some(pts) = buffer.pts() {
                stamps
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .insert(pts.nseconds(), stamp_ns);
            }

            match appsrc.push_buffer(out) {
                Ok(_) => log_info!(
                    logger.throttle(STATS_THROTTLE),
                    "[WarpBridge] frame age at the warp {age_ms:.1} ms, warp {warp_ms:.1} ms, \
                     IMU wait {wait_ms:.1} ms (stabilizing: {stabilizing})"
                ),
                Err(err) => log_warn!(
                    logger.throttle(LOG_THROTTLE),
                    "[WarpBridge] output pipeline refused the frame, dropped: {err}"
                ),
            }
            Ok(gst::FlowSuccess::Ok)
        })
        .build()
}

/// The orientation at `t_ns`, after waiting up to `max_wait` for a sample at or past it. No
/// wait at all while the buffer is empty: there is nothing to wait for.
fn orientation_at(imu: &SharedImu, t_ns: i64, max_wait: Duration) -> OrientationAt {
    let (lock, condvar) = &**imu;
    let mut buffer = lock.lock().unwrap_or_else(PoisonError::into_inner);
    let deadline = Instant::now() + max_wait;
    loop {
        match buffer.newest_stamp_ns() {
            None => break,
            Some(newest) if newest >= t_ns => break,
            // Already past what extrapolation would bridge: a few more milliseconds will not
            // bring the IMU back into range, so do not spend them on every frame.
            Some(newest) if t_ns - newest > super::imu::MAX_EXTRAPOLATION_NS => break,
            Some(_) => {
                let now = Instant::now();
                if now >= deadline {
                    break;
                }
                let (guard, _) = condvar
                    .wait_timeout(buffer, deadline - now)
                    .unwrap_or_else(PoisonError::into_inner);
                buffer = guard;
            }
        }
    }
    buffer.orientation_at(t_ns)
}

/// Warps the I420 frame in `buffer` (laid out per `caps`) into a fresh buffer laid out per
/// `output_info`.
fn warp_frame(
    buffer: &gst::BufferRef,
    caps: &gst::CapsRef,
    output_info: &gst_video::VideoInfo,
    inverse: &Mat3,
) -> Result<Vec<u8>, String> {
    let input_info = gst_video::VideoInfo::from_caps(caps)
        .map_err(|err| format!("raw frame has no video info: {err}"))?;
    if input_info.format() != gst_video::VideoFormat::I420 {
        return Err(format!("raw frame is {:?}, not I420", input_info.format()));
    }
    if (input_info.width(), input_info.height()) != (output_info.width(), output_info.height()) {
        return Err(format!(
            "raw frame is {}x{}, the stabilized path is configured for {}x{}",
            input_info.width(),
            input_info.height(),
            output_info.width(),
            output_info.height()
        ));
    }
    let frame = gst_video::VideoFrameRef::from_buffer_ref_readable(buffer, &input_info)
        .map_err(|err| format!("raw frame is not readable: {err}"))?;

    let src = |plane: usize| -> Result<Plane<'_>, String> {
        let data = frame
            .plane_data(plane as u32)
            .map_err(|err| format!("plane {plane} is unmapped: {err}"))?;
        let stride = usize::try_from(frame.plane_stride()[plane])
            .map_err(|_| format!("plane {plane} has a negative stride"))?;
        Ok(Plane {
            data,
            width: input_info.comp_width(plane as u8) as usize,
            height: input_info.comp_height(plane as u8) as usize,
            stride,
        })
    };
    let (src_y, src_u, src_v) = (src(0)?, src(1)?, src(2)?);

    let mut out = vec![0u8; output_info.size()];
    let [mut y, mut u, mut v] = split_planes(&mut out, output_info)?;
    warp::warp_i420([&src_y, &src_u, &src_v], [&mut y, &mut u, &mut v], inverse);
    Ok(out)
}

/// The three planes of an I420 buffer laid out per `info`, as disjoint mutable views.
fn split_planes<'a>(
    data: &'a mut [u8],
    info: &gst_video::VideoInfo,
) -> Result<[PlaneMut<'a>; 3], String> {
    let offsets = info.offset();
    let strides = info.stride();
    if offsets.len() < 3
        || strides.len() < 3
        || !(offsets[0] <= offsets[1] && offsets[1] <= offsets[2])
    {
        return Err("I420 layout does not have three ascending planes".to_string());
    }
    let (y, rest) = data.split_at_mut(offsets[1]);
    let (u, v) = rest.split_at_mut(offsets[2] - offsets[1]);
    let plane = |data: &'a mut [u8], index: usize| -> Result<PlaneMut<'a>, String> {
        Ok(PlaneMut {
            data,
            width: info.comp_width(index as u8) as usize,
            height: info.comp_height(index as u8) as usize,
            stride: usize::try_from(strides[index])
                .map_err(|_| format!("plane {index} has a negative stride"))?,
        })
    };
    Ok([plane(y, 0)?, plane(u, 1)?, plane(v, 2)?])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_stamp_table_returns_each_stamp_once_and_forgets_older_frames() {
        let mut table = StampTable::default();
        table.insert(100, 1_000);
        table.insert(200, 2_000);
        table.insert(300, 3_000);
        assert_eq!(table.take(200), Some(2_000));
        // 100 was before 200: a dropped frame, gone with it.
        assert_eq!(table.take(100), None);
        assert_eq!(table.take(200), None);
        assert_eq!(table.take(300), Some(3_000));
        assert_eq!(table.take(400), None);
    }

    #[test]
    fn the_stamp_table_is_bounded() {
        let mut table = StampTable::default();
        for i in 0..(StampTable::CAPACITY as u64 + 4) {
            table.insert(i, i as i64);
        }
        assert_eq!(table.entries.len(), StampTable::CAPACITY);
        assert_eq!(table.take(0), None);
        assert_eq!(table.take(4), Some(4));
    }
}
