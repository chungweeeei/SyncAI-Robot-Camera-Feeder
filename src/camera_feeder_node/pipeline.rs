//! The GStreamer pipeline description, as a pure function of the node's configuration.
//!
//! Kept free of GStreamer and ROS types so the exact string the node hands to `gst::parse::launch`
//! is unit-tested without a camera, a GStreamer install or a ROS graph — the same reason the
//! WebRTC worker builds its pipeline as a string in `BuildVideoPipelineString`.

/// The appsink feeding `image_raw/compressed`; the node looks it up by this name.
pub const ROS_SINK_NAME: &str = "ros";

/// Sensor controls written through `v4l2src extra-controls` when capture starts.
///
/// Ported from the WebRTC worker (`internal/config/config.go`, `internal/proc/video.go`), whose
/// defaults were measured on this camera. They are not decorative: UVC control state lives in the
/// camera firmware and survives open/close, so a process that sets nothing inherits whatever the
/// last one left behind — on this fleet that was manual exposure at 4 ms and white balance frozen
/// at 4000 K, an image both very dark and very blue.
///
/// Polarity trap, and the two controls disagree: `auto_exposure` is a menu with UVC numbering
/// (0 = auto, 1 = manual), while `white_balance_automatic` is a plain boolean (1 = automatic).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SensorControls {
    /// 0 = auto, 1 = manual.
    pub auto_exposure: i64,
    /// In 100 µs units; only written when `auto_exposure` is 1.
    pub exposure_time: i64,
    /// 1 = automatic, 0 = manual.
    pub white_balance_automatic: i64,
    /// Kelvin; only written when `white_balance_automatic` is 0.
    pub white_balance_temperature: i64,
    pub gain: i64,
    pub brightness: i64,
    /// 0 = disabled, 1 = 50 Hz, 2 = 60 Hz.
    pub power_line_frequency: i64,
}

impl Default for SensorControls {
    fn default() -> Self {
        Self {
            auto_exposure: 0,
            exposure_time: 330,
            // Manual at 5200 K, measured on the robot rather than guessed: the camera's AWB
            // settles distinctly blue under the lab's fluorescent lighting, and 5200 K sits
            // between the neutral points measured against a white wall (5040 K) and a grey carpet
            // (5410 K). Fixed also because AWB retunes as the frame fills with a coloured wall,
            // shifting the colour while nothing in the scene changed. See the worker's config.go
            // for the full sweep.
            white_balance_automatic: 0,
            white_balance_temperature: 5200,
            gain: 1,
            brightness: 16,
            // 50 Hz: the camera ships with flicker correction disabled, and mains lighting then
            // bands the image.
            power_line_frequency: 1,
        }
    }
}

/// Everything the pipeline string depends on, already validated (see `parameters.rs`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PipelineConfig {
    pub device: String,
    pub width: u32,
    pub height: u32,
    pub framerate: u32,
    pub rtp_host: String,
    pub rtp_port: u16,
    /// Rate of the ROS branch. At or above `framerate` the branch carries every frame and has no
    /// `videorate` at all.
    pub ros_framerate: u32,
    /// Replaces the whole capture head (`v4l2src ! image/jpeg,…`) when non-empty. For machines
    /// without the camera; it must produce `image/jpeg` that `rtpjpegpay` accepts (4:2:0 or 4:2:2,
    /// so pin the raw format), e.g.
    /// `videotestsrc is-live=true ! video/x-raw,format=YUY2,width=1280,height=720,framerate=60/1 ! jpegenc`.
    pub source_override: String,
    pub sensor_controls: SensorControls,
}

/// Renders `v4l2src`'s `extra-controls` string.
///
/// Order is load-bearing: each auto switch has to precede the manual value it gates, or the driver
/// rejects the manual write because the mode is still automatic. A gated value is emitted only
/// when its automatic mode is off — the driver rejects it otherwise.
#[must_use]
pub fn sensor_controls(controls: &SensorControls) -> String {
    let mut out = format!("c,auto_exposure={}", controls.auto_exposure);
    if controls.auto_exposure == 1 {
        out += &format!(",exposure_time_absolute={}", controls.exposure_time);
    }
    out += &format!(
        ",white_balance_automatic={}",
        controls.white_balance_automatic
    );
    if controls.white_balance_automatic == 0 {
        out += &format!(
            ",white_balance_temperature={}",
            controls.white_balance_temperature
        );
    }
    out += &format!(
        ",gain={},brightness={},power_line_frequency={}",
        controls.gain, controls.brightness, controls.power_line_frequency
    );
    out
}

/// A queue that drops the oldest buffer instead of blocking upstream.
///
/// One in front of each tee branch, so a slow consumer on one side (a ROS publish that blocks, an
/// RTP receiver that is not there) can never stall the capture thread or the other branch.
const LEAKY_QUEUE: &str =
    "queue max-size-buffers=3 max-size-time=0 max-size-bytes=0 leaky=downstream";

/// Builds the description handed to `gst::parse::launch`.
///
/// ```text
/// v4l2src ! image/jpeg ! tee ─┬─ queue ! rtpjpegpay ! udpsink      → WebRTC worker
///                             └─ queue [! videorate] ! appsink     → image_raw/compressed
/// ```
///
/// Nothing is decoded: the camera's JPEG bytes go to both branches untouched.
///
/// `do-timestamp` is deliberately left at its default (false): `v4l2src` then derives each
/// buffer's PTS from the V4L2 buffer's kernel timestamp, which is what `stamp.rs` turns into the
/// message stamp. With it on, the PTS would be the moment the buffer was pushed instead.
///
/// Both sinks run with `sync=false`: this is a live source, and holding a buffer until its running
/// time only adds latency.
#[must_use]
pub fn build_pipeline_string(config: &PipelineConfig) -> String {
    let source = if config.source_override.trim().is_empty() {
        format!(
            "v4l2src device=\"{}\" io-mode=2 extra-controls=\"{}\" \
             ! image/jpeg,width={},height={},framerate={}/1",
            config.device,
            sensor_controls(&config.sensor_controls),
            config.width,
            config.height,
            config.framerate,
        )
    } else {
        config.source_override.trim().to_string()
    };

    let rtp_branch = format!(
        "t. ! {LEAKY_QUEUE} ! rtpjpegpay pt=26 \
         ! udpsink host=\"{}\" port={} sync=false async=false",
        config.rtp_host, config.rtp_port,
    );

    // drop-only: videorate may only drop frames to reach the lower rate, never duplicate one.
    let rate_limit = if config.ros_framerate < config.framerate {
        format!(
            " ! videorate drop-only=true ! image/jpeg,framerate={}/1",
            config.ros_framerate
        )
    } else {
        String::new()
    };
    let ros_branch = format!(
        "t. ! {LEAKY_QUEUE}{rate_limit} \
         ! appsink name={ROS_SINK_NAME} sync=false max-buffers=2 drop=true"
    );

    format!("{source} ! tee name=t {rtp_branch} {ros_branch}")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> PipelineConfig {
        PipelineConfig {
            device: "/dev/syncai/camera0".to_string(),
            width: 1280,
            height: 720,
            framerate: 60,
            rtp_host: "127.0.0.1".to_string(),
            rtp_port: 5008,
            ros_framerate: 60,
            source_override: String::new(),
            sensor_controls: SensorControls::default(),
        }
    }

    #[test]
    fn default_controls_match_the_worker() {
        // The exact string the worker writes with its defaults, so the camera ends up in the same
        // state whichever of the two opened it last.
        assert_eq!(
            sensor_controls(&SensorControls::default()),
            "c,auto_exposure=0,white_balance_automatic=0,white_balance_temperature=5200,\
             gain=1,brightness=16,power_line_frequency=1"
        );
    }

    #[test]
    fn manual_exposure_follows_its_switch() {
        let controls = SensorControls {
            auto_exposure: 1,
            exposure_time: 100,
            ..SensorControls::default()
        };
        let out = sensor_controls(&controls);
        let switch = out.find("auto_exposure=1").unwrap();
        let value = out.find("exposure_time_absolute=100").unwrap();
        assert!(switch < value, "{out}");
    }

    #[test]
    fn automatic_white_balance_omits_the_temperature() {
        let controls = SensorControls {
            white_balance_automatic: 1,
            ..SensorControls::default()
        };
        let out = sensor_controls(&controls);
        assert!(out.contains("white_balance_automatic=1"), "{out}");
        assert!(!out.contains("white_balance_temperature"), "{out}");
    }

    #[test]
    fn auto_exposure_omits_the_exposure_time() {
        let out = sensor_controls(&SensorControls::default());
        assert!(!out.contains("exposure_time_absolute"), "{out}");
    }

    #[test]
    fn camera_pipeline_has_both_branches() {
        let out = build_pipeline_string(&config());
        assert!(
            out.starts_with("v4l2src device=\"/dev/syncai/camera0\" io-mode=2"),
            "{out}"
        );
        assert!(
            out.contains("! image/jpeg,width=1280,height=720,framerate=60/1 ! tee name=t"),
            "{out}"
        );
        assert!(
            out.contains("rtpjpegpay pt=26 ! udpsink host=\"127.0.0.1\" port=5008"),
            "{out}"
        );
        assert!(
            out.contains(&format!("appsink name={ROS_SINK_NAME}")),
            "{out}"
        );
        assert_eq!(out.matches("leaky=downstream").count(), 2, "{out}");
    }

    #[test]
    fn camera_pipeline_never_decodes_or_restamps() {
        let out = build_pipeline_string(&config());
        assert!(!out.contains("jpegdec"), "{out}");
        assert!(!out.contains("do-timestamp"), "{out}");
    }

    #[test]
    fn full_rate_ros_branch_has_no_videorate() {
        let out = build_pipeline_string(&config());
        assert!(!out.contains("videorate"), "{out}");
    }

    #[test]
    fn lower_ros_rate_inserts_videorate_on_the_ros_branch_only() {
        let out = build_pipeline_string(&PipelineConfig {
            ros_framerate: 30,
            ..config()
        });
        let ros_branch = out.rfind("t. !").unwrap();
        let videorate = out
            .find("videorate drop-only=true ! image/jpeg,framerate=30/1")
            .unwrap();
        assert!(videorate > ros_branch, "{out}");
    }

    #[test]
    fn source_override_replaces_the_capture_head() {
        let out = build_pipeline_string(&PipelineConfig {
            source_override: "  videotestsrc is-live=true ! jpegenc ".to_string(),
            ..config()
        });
        assert!(
            out.starts_with("videotestsrc is-live=true ! jpegenc ! tee name=t"),
            "{out}"
        );
        assert!(!out.contains("v4l2src"), "{out}");
        assert!(!out.contains("extra-controls"), "{out}");
    }
}
