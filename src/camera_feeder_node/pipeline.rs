//! The GStreamer pipeline description, as a pure function of the node's configuration.
//!
//! Kept free of GStreamer and ROS types so the exact string the node hands to `gst::parse::launch`
//! is unit-tested without a camera, a GStreamer install or a ROS graph — the same reason the
//! WebRTC worker builds its pipeline as a string in `BuildVideoPipelineString`.

/// The appsink feeding `image_raw/compressed`; the node looks it up by this name.
pub const ROS_SINK_NAME: &str = "ros";
/// The appsink at the end of the stabilized capture pipeline, where the decoded I420 frames come
/// out to be warped.
pub const RAW_SINK_NAME: &str = "raw";
/// The appsrc at the head of the stabilized output pipeline, where the warped frames go in.
pub const WARPED_SRC_NAME: &str = "warped";

/// The two descriptions of the stabilized path. `capture` ends in the raw appsink and `output`
/// starts at the appsrc the warp feeds; the Rust warp (`warp_bridge.rs`) is the bridge between
/// them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StabilizedPipelines {
    pub capture: String,
    pub output: String,
}

/// Default manual exposure, in 100 µs units (33 ms). Only applies to [`Exposure::Manual`].
pub const DEFAULT_EXPOSURE_TIME_100US: u32 = 330;
/// Default manual white balance, in Kelvin. See [`SensorControls::default`] for where it comes
/// from.
pub const DEFAULT_WHITE_BALANCE_KELVIN: u32 = 5200;

/// Exposure mode. The manual time exists only in manual mode, so a "manual value written while the
/// mode is still automatic" — which the driver rejects — cannot be expressed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exposure {
    Auto,
    Manual { time_100us: u32 },
}

/// White-balance mode; the temperature exists only in manual mode, as with [`Exposure`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WhiteBalance {
    Auto,
    Manual { kelvin: u32 },
}

/// Which elements decode and re-encode the JPEG frames on the stabilized path. The passthrough
/// path neither decodes nor encodes, so it has no codec.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JpegCodec {
    /// plugins-good `jpegdec` / `jpegenc` on the CPU. Runs anywhere, the Dev Container included.
    Software,
    /// The Jetson's hardware JPEG blocks through the L4T plugins: `nvv4l2decoder mjpeg=1`, whose
    /// NVMM output `nvvidconv` converts into system-memory I420, and `nvjpegenc`. Exists only
    /// where the nvidia container runtime has injected the plugins.
    Nvidia,
}

/// What the `jpeg_codec` parameter asks for; [`resolve_codec`] turns it into a [`JpegCodec`]
/// once the GStreamer registry can be asked which elements exist.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JpegCodecChoice {
    /// NVIDIA when every element of its chain is registered, software otherwise.
    Auto,
    Software,
    /// NVIDIA or a startup error: for a robot where falling back to the CPU unnoticed is worse
    /// than not starting.
    Nvidia,
}

/// The L4T factories the NVIDIA chain is built from, in chain order. Its `identity` is core and
/// always there.
pub const NVIDIA_CODEC_ELEMENTS: [&str; 3] = ["nvv4l2decoder", "nvvidconv", "nvjpegenc"];

/// The elements of [`NVIDIA_CODEC_ELEMENTS`] that `is_registered` says are missing, in chain
/// order.
#[must_use]
pub fn missing_nvidia_elements(is_registered: impl Fn(&str) -> bool) -> Vec<&'static str> {
    NVIDIA_CODEC_ELEMENTS
        .into_iter()
        .filter(|name| !is_registered(name))
        .collect()
}

/// The codec the stabilized path runs with, given the NVIDIA elements that are not registered.
///
/// # Errors
///
/// `jpeg_codec: nvidia` while an element of the chain is missing, naming the missing ones.
pub fn resolve_codec(choice: JpegCodecChoice, missing: &[&str]) -> Result<JpegCodec, String> {
    match choice {
        JpegCodecChoice::Software => Ok(JpegCodec::Software),
        JpegCodecChoice::Auto | JpegCodecChoice::Nvidia if missing.is_empty() => {
            Ok(JpegCodec::Nvidia)
        }
        JpegCodecChoice::Auto => Ok(JpegCodec::Software),
        JpegCodecChoice::Nvidia => Err(format!(
            "jpeg_codec is nvidia, but GStreamer has no {}: is the container running under the \
             nvidia runtime with NVIDIA_DRIVER_CAPABILITIES set?",
            missing.join(" / ")
        )),
    }
}

/// Mains flicker correction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerLineFrequency {
    Disabled,
    Hz50,
    Hz60,
}

impl PowerLineFrequency {
    /// The V4L2 `power_line_frequency` menu index.
    #[must_use]
    pub fn v4l2_value(self) -> u8 {
        match self {
            Self::Disabled => 0,
            Self::Hz50 => 1,
            Self::Hz60 => 2,
        }
    }
}

/// Sensor controls written through `v4l2src extra-controls` when capture starts.
///
/// Ported from the WebRTC worker (`internal/config/config.go`, `internal/proc/video.go`), whose
/// defaults were measured on this camera. They are not decorative: UVC control state lives in the
/// camera firmware and survives open/close, so a process that sets nothing inherits whatever the
/// last one left behind — on this fleet that was manual exposure at 4 ms and white balance frozen
/// at 4000 K, an image both very dark and very blue.
///
/// The modes are enums rather than the raw V4L2 numbers because the raw numbers are a trap: the
/// two switches disagree on polarity (`auto_exposure` is a UVC menu where 0 = auto, while
/// `white_balance_automatic` is a boolean where 1 = auto). The numbers appear only in
/// [`sensor_controls`], where the string is rendered, and in `parameters.rs`, where the params file
/// is parsed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SensorControls {
    pub exposure: Exposure,
    pub white_balance: WhiteBalance,
    pub gain: i64,
    pub brightness: i64,
    pub power_line_frequency: PowerLineFrequency,
}

impl Default for SensorControls {
    fn default() -> Self {
        Self {
            exposure: Exposure::Auto,
            // Manual at 5200 K, measured on the robot rather than guessed: the camera's AWB
            // settles distinctly blue under the lab's fluorescent lighting, and 5200 K sits
            // between the neutral points measured against a white wall (5040 K) and a grey carpet
            // (5410 K). Fixed also because AWB retunes as the frame fills with a coloured wall,
            // shifting the colour while nothing in the scene changed. See the worker's config.go
            // for the full sweep.
            white_balance: WhiteBalance::Manual {
                kelvin: DEFAULT_WHITE_BALANCE_KELVIN,
            },
            gain: 1,
            brightness: 16,
            // The camera ships with flicker correction disabled, and mains lighting then bands
            // the image.
            power_line_frequency: PowerLineFrequency::Hz50,
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
/// rejects the manual write because the mode is still automatic. That is why each mode and its
/// value are rendered together, switch first.
#[must_use]
pub fn sensor_controls(controls: &SensorControls) -> String {
    let exposure = match controls.exposure {
        Exposure::Auto => "auto_exposure=0".to_string(),
        Exposure::Manual { time_100us } => {
            format!("auto_exposure=1,exposure_time_absolute={time_100us}")
        }
    };
    let white_balance = match controls.white_balance {
        WhiteBalance::Auto => "white_balance_automatic=1".to_string(),
        WhiteBalance::Manual { kelvin } => {
            format!("white_balance_automatic=0,white_balance_temperature={kelvin}")
        }
    };
    format!(
        "c,{exposure},{white_balance},gain={},brightness={},power_line_frequency={}",
        controls.gain,
        controls.brightness,
        controls.power_line_frequency.v4l2_value(),
    )
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
    let source = capture_head(config);
    let rtp_branch = rtp_branch(config);
    let rate_limit = rate_limit(config);
    let ros_branch = format!("t. ! {LEAKY_QUEUE}{rate_limit} ! {}", ros_appsink());

    format!("{source} ! tee name=t {rtp_branch} {ros_branch}")
}

/// The two descriptions of the stabilized path (`stabilize: true`).
///
/// ```text
/// capture: v4l2src ! image/jpeg [! videorate] ! queue ! <decoder> ! I420 ! appsink name=raw
///                                                        │ Rust: warp by the IMU orientation
/// output:  appsrc name=warped ! <encoder> ! tee ─┬─ queue ! rtpjpegpay ! udpsink
///                                                └─ queue ! appsink name=ros
/// ```
///
/// The rate limit sits *before* the decoder, so only `ros_framerate` frames per second are
/// decoded, warped and re-encoded — and the RTP branch, downstream of the warp, carries that rate
/// too rather than `framerate`. The queue in front of the decoder moves decoding off the capture
/// thread and drops rather than blocks when the warp falls behind, as the tee queues do.
///
/// The decoder and encoder are `codec`'s; see [`decoder`] and [`encoder`]. Either way the warp
/// sees system-memory I420 and the encoder is handed system-memory I420, so nothing between the
/// two appsink/appsrc ends depends on the codec.
///
/// The appsrc is live with `do-timestamp=false`: the warp sets each buffer's PTS to the capture
/// pipeline's, so running time stays monotonic for `rtpjpegpay`. `block=false` with a leaky
/// two-buffer queue inside the appsrc: a slow encoder loses frames, never stalls the warp.
#[must_use]
pub fn build_stabilized_pipelines(
    config: &PipelineConfig,
    codec: JpegCodec,
) -> StabilizedPipelines {
    let capture = format!(
        "{}{} ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream \
         ! {} ! appsink name={RAW_SINK_NAME} sync=false max-buffers=1 drop=true",
        capture_head(config),
        rate_limit(config),
        decoder(codec),
    );
    let output = format!(
        "appsrc name={WARPED_SRC_NAME} is-live=true format=time do-timestamp=false block=false \
         max-buffers=2 max-bytes=0 leaky-type=downstream \
         caps=\"video/x-raw,format=I420,width={},height={},framerate={}/1\" \
         ! {} ! tee name=t {} t. ! {LEAKY_QUEUE} ! {}",
        config.width,
        config.height,
        config.ros_framerate,
        encoder(codec),
        rtp_branch(config),
        ros_appsink(),
    );
    StabilizedPipelines { capture, output }
}

/// JPEG in, system-memory I420 out.
///
/// Software: `jpegdec` (1.20) emits I420 for every YCbCr JPEG, 4:2:2 included (it halves the
/// chroma rows), so the `format=I420` caps are an assertion the warp relies on, not a conversion.
///
/// NVIDIA: `nvv4l2decoder` emits NVMM in the JPEG's own sampling (Y42B for this camera), which
/// the CPU cannot read, so `nvvidconv` converts it to I420 and copies it out to system memory —
/// most of what the chain still costs (measured on the AGX Orin with this camera's frames:
/// decoding to NVMM ~1 ms, the copy-out ~2.5 ms). Reading NVMM in place would need the
/// NvBufSurface C API, i.e. unsafe FFI, which this crate forbids. The full range survives: the
/// decoder tags it and the copy keeps 0–255.
///
/// The `identity` in front of `nvv4l2decoder` is load-bearing. This camera starts every stream
/// with four 0-byte buffers, flagged CORRUPTED and stamped PTS 0. `jpegdec` shrugs them off;
/// `nvv4l2decoder` fed one fails with "Failed to process frame" — a bus error, so an exit and a
/// respawn — and did on 4 of 8 starts. Dropped by their flag, 0 of 20 starts failed, and a frame
/// the UVC driver marks broken mid-stream goes the same way.
///
/// `jpegparse` also gets past the empty buffers, and must not be used for it: in 1.20 it discards
/// the upstream PTS and interpolates from the caps' framerate, seeded by the first buffer it sees
/// (an empty one, PTS 0). Every capture stamp then came out late by the time to the first real
/// frame (~145 ms) plus one frame period for every frame dropped upstream — 143 ms steady on the
/// robot at 30 fps, 7.6 s after half a minute when the warp fell behind — and the warp used the
/// IMU orientation of the wrong instant.
fn decoder(codec: JpegCodec) -> &'static str {
    match codec {
        JpegCodec::Software => "jpegdec ! video/x-raw,format=I420",
        JpegCodec::Nvidia => {
            "identity drop-buffer-flags=corrupted ! nvv4l2decoder mjpeg=1 ! nvvidconv \
             ! video/x-raw,format=I420"
        }
    }
}

/// System-memory I420 in, JPEG out. Both encoders default to quality 85 and the output was the
/// same (39.7 dB against the source on a camera frame, vs 39.6 for `jpegenc`), so `jpeg_codec`
/// does not change the image. `nvjpegenc` takes system memory directly; an `nvvidconv` to NVMM in
/// front of it only added a copy (2.5 ms vs 1.5 ms per frame).
fn encoder(codec: JpegCodec) -> &'static str {
    match codec {
        JpegCodec::Software => "jpegenc",
        JpegCodec::Nvidia => "nvjpegenc",
    }
}

/// `v4l2src ! image/jpeg,…`, or the override in its place.
fn capture_head(config: &PipelineConfig) -> String {
    if config.source_override.trim().is_empty() {
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
    }
}

fn rtp_branch(config: &PipelineConfig) -> String {
    format!(
        "t. ! {LEAKY_QUEUE} ! rtpjpegpay pt=26 \
         ! udpsink host=\"{}\" port={} sync=false async=false",
        config.rtp_host, config.rtp_port,
    )
}

/// ` ! videorate …` down to `ros_framerate`, or nothing when every frame is carried.
///
/// drop-only: videorate may only drop frames to reach the lower rate, never duplicate one.
fn rate_limit(config: &PipelineConfig) -> String {
    if config.ros_framerate < config.framerate {
        format!(
            " ! videorate drop-only=true ! image/jpeg,framerate={}/1",
            config.ros_framerate
        )
    } else {
        String::new()
    }
}

fn ros_appsink() -> String {
    format!("appsink name={ROS_SINK_NAME} sync=false max-buffers=2 drop=true")
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
            exposure: Exposure::Manual { time_100us: 100 },
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
            white_balance: WhiteBalance::Auto,
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
    fn power_line_frequency_uses_the_v4l2_menu_index() {
        for (mode, index) in [
            (PowerLineFrequency::Disabled, "0"),
            (PowerLineFrequency::Hz50, "1"),
            (PowerLineFrequency::Hz60, "2"),
        ] {
            let out = sensor_controls(&SensorControls {
                power_line_frequency: mode,
                ..SensorControls::default()
            });
            assert!(
                out.ends_with(&format!("power_line_frequency={index}")),
                "{out}"
            );
        }
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

    fn stabilized_config() -> PipelineConfig {
        PipelineConfig {
            ros_framerate: 30,
            ..config()
        }
    }

    #[test]
    fn stabilized_capture_rate_limits_before_decoding_into_the_raw_sink() {
        let StabilizedPipelines { capture, .. } =
            build_stabilized_pipelines(&stabilized_config(), JpegCodec::Software);
        assert!(
            capture.starts_with("v4l2src device=\"/dev/syncai/camera0\" io-mode=2"),
            "{capture}"
        );
        let videorate = capture
            .find("videorate drop-only=true ! image/jpeg,framerate=30/1")
            .unwrap();
        let queue = capture.find("queue max-size-buffers=2").unwrap();
        let decoder = capture.find("jpegdec ! video/x-raw,format=I420").unwrap();
        assert!(videorate < queue && queue < decoder, "{capture}");
        assert_eq!(capture.matches("leaky=downstream").count(), 1, "{capture}");
        assert!(
            capture.ends_with(&format!(
                "appsink name={RAW_SINK_NAME} sync=false max-buffers=1 drop=true"
            )),
            "{capture}"
        );
        assert!(!capture.contains("tee"), "{capture}");
        assert!(!capture.contains("do-timestamp"), "{capture}");
    }

    #[test]
    fn stabilized_output_encodes_into_both_branches_at_the_ros_rate() {
        let StabilizedPipelines { output, .. } =
            build_stabilized_pipelines(&stabilized_config(), JpegCodec::Software);
        assert!(
            output.starts_with(&format!(
                "appsrc name={WARPED_SRC_NAME} is-live=true format=time do-timestamp=false"
            )),
            "{output}"
        );
        assert!(
            output
                .contains("caps=\"video/x-raw,format=I420,width=1280,height=720,framerate=30/1\""),
            "{output}"
        );
        assert!(output.contains("! jpegenc ! tee name=t"), "{output}");
        assert!(
            output.contains("rtpjpegpay pt=26 ! udpsink host=\"127.0.0.1\" port=5008"),
            "{output}"
        );
        assert!(
            output.ends_with(&format!(
                "appsink name={ROS_SINK_NAME} sync=false max-buffers=2 drop=true"
            )),
            "{output}"
        );
        assert_eq!(output.matches("leaky=downstream").count(), 2, "{output}");
        // The whole path already runs at ros_framerate; no second rate limit.
        assert!(!output.contains("videorate"), "{output}");
    }

    #[test]
    fn stabilized_at_full_rate_decodes_every_frame() {
        let StabilizedPipelines { capture, output } =
            build_stabilized_pipelines(&config(), JpegCodec::Software);
        assert!(!capture.contains("videorate"), "{capture}");
        assert!(output.contains("framerate=60/1"), "{output}");
    }

    #[test]
    fn stabilized_capture_honours_the_source_override() {
        let StabilizedPipelines { capture, .. } = build_stabilized_pipelines(
            &PipelineConfig {
                source_override: "videotestsrc is-live=true ! jpegenc".to_string(),
                ..stabilized_config()
            },
            JpegCodec::Software,
        );
        assert!(
            capture.starts_with("videotestsrc is-live=true ! jpegenc ! videorate"),
            "{capture}"
        );
        assert!(!capture.contains("v4l2src"), "{capture}");
    }

    #[test]
    fn the_branches_are_the_same_in_both_modes() {
        // The RTP branch and the ROS appsink come from the same helpers, so a change to one mode
        // cannot silently diverge from the other.
        let passthrough = build_pipeline_string(&config());
        let branch = |s: &str| s[s.find("t. !").unwrap()..].to_string();
        for codec in [JpegCodec::Software, JpegCodec::Nvidia] {
            let StabilizedPipelines { output, .. } = build_stabilized_pipelines(&config(), codec);
            assert_eq!(branch(&passthrough), branch(&output), "{codec:?}");
        }
    }

    #[test]
    fn nvidia_capture_drops_corrupted_buffers_before_the_decoder_and_copies_out_i420() {
        let StabilizedPipelines { capture, .. } =
            build_stabilized_pipelines(&stabilized_config(), JpegCodec::Nvidia);
        let queue = capture.find("queue max-size-buffers=2").unwrap();
        let chain = capture
            .find(
                "identity drop-buffer-flags=corrupted ! nvv4l2decoder mjpeg=1 ! nvvidconv \
                 ! video/x-raw,format=I420 ! appsink",
            )
            .unwrap();
        assert!(queue < chain, "{capture}");
        // System memory: no (memory:NVMM) on the caps the warp maps.
        assert!(!capture.contains("NVMM"), "{capture}");
        assert!(!capture.contains("jpegdec"), "{capture}");
    }

    #[test]
    fn no_chain_has_a_jpegparse() {
        // jpegparse (1.20) replaces the capture PTS with one interpolated from the framerate,
        // which shifts every capture stamp; see decoder().
        let passthrough = build_pipeline_string(&config());
        assert!(!passthrough.contains("jpegparse"), "{passthrough}");
        for codec in [JpegCodec::Software, JpegCodec::Nvidia] {
            let StabilizedPipelines { capture, output } =
                build_stabilized_pipelines(&stabilized_config(), codec);
            assert!(!capture.contains("jpegparse"), "{capture}");
            assert!(!output.contains("jpegparse"), "{output}");
        }
    }

    #[test]
    fn nvidia_output_encodes_system_memory_with_nvjpegenc() {
        let StabilizedPipelines { output, .. } =
            build_stabilized_pipelines(&stabilized_config(), JpegCodec::Nvidia);
        assert!(
            output.contains("framerate=30/1\" ! nvjpegenc ! tee name=t"),
            "{output}"
        );
        assert!(!output.contains("nvvidconv"), "{output}");
        assert!(!output.contains(" jpegenc"), "{output}");
    }

    #[test]
    fn the_codec_changes_only_the_decoder_and_the_encoder() {
        let software = build_stabilized_pipelines(&stabilized_config(), JpegCodec::Software);
        let nvidia = build_stabilized_pipelines(&stabilized_config(), JpegCodec::Nvidia);
        assert_eq!(
            software
                .capture
                .replace(decoder(JpegCodec::Software), "DEC"),
            nvidia.capture.replace(decoder(JpegCodec::Nvidia), "DEC"),
        );
        assert_eq!(
            software.output.replace("! jpegenc !", "! ENC !"),
            nvidia.output.replace("! nvjpegenc !", "! ENC !"),
        );
    }

    #[test]
    fn missing_elements_are_listed_in_chain_order() {
        assert!(missing_nvidia_elements(|_| true).is_empty());
        assert_eq!(missing_nvidia_elements(|_| false), NVIDIA_CODEC_ELEMENTS);
        // A decoder without the encoder: what an L4T plugin set that is half injected looks like.
        let only_decoder = missing_nvidia_elements(|name| name == "nvv4l2decoder");
        assert_eq!(only_decoder, ["nvvidconv", "nvjpegenc"]);
    }

    #[test]
    fn auto_picks_nvidia_only_when_the_whole_chain_is_registered() {
        assert_eq!(
            resolve_codec(JpegCodecChoice::Auto, &[]),
            Ok(JpegCodec::Nvidia)
        );
        // One missing element is enough: half a hardware chain does not link.
        assert_eq!(
            resolve_codec(JpegCodecChoice::Auto, &["nvjpegenc"]),
            Ok(JpegCodec::Software)
        );
    }

    #[test]
    fn an_explicit_choice_is_honoured_or_refused() {
        assert_eq!(
            resolve_codec(JpegCodecChoice::Software, &[]),
            Ok(JpegCodec::Software)
        );
        assert_eq!(
            resolve_codec(JpegCodecChoice::Nvidia, &[]),
            Ok(JpegCodec::Nvidia)
        );
        let err =
            resolve_codec(JpegCodecChoice::Nvidia, &["nvv4l2decoder", "nvvidconv"]).unwrap_err();
        assert!(err.contains("nvv4l2decoder / nvvidconv"), "{err}");
    }
}
