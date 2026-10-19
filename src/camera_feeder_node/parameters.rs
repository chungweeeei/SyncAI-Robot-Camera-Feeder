use rclrs::*;
use std::sync::Arc;
use std::time::Duration;

use super::eis::{EisConfig, Intrinsics, SmoothingOrder};
use super::imu::MahonyGains;
use super::math::Mat3;
use super::pipeline::{
    DEFAULT_EXPOSURE_TIME_100US, DEFAULT_WHITE_BALANCE_KELVIN, Exposure, JpegCodecChoice,
    PipelineConfig, PowerLineFrequency, SensorControls, WhiteBalance,
};

const DEFAULT_DEVICE: &str = "/dev/syncai/camera0";
const DEFAULT_WIDTH: u32 = 1280;
const DEFAULT_HEIGHT: u32 = 720;
// The AR0234 offers only 60 and 120 fps at 720p — there is no 30.
const DEFAULT_FRAMERATE: u32 = 60;
const DEFAULT_RTP_HOST: &str = "127.0.0.1";
// The WebRTC worker's VIDEO_RTP_IN_PORT; the two must agree.
const DEFAULT_RTP_PORT: u16 = 5008;
const DEFAULT_FRAME_ID: &str = "camera_optical_frame";
// Half the capture rate: the ROS topic does not need what the operator's live view does, and
// halving it halves the DDS traffic and whatever records the topic. The RTP branch stays at
// `framerate`.
const DEFAULT_ROS_FRAMERATE: u32 = 30;

// Stabilization. The IMU is the lidar's (relative topic, so `<robot_id>/livox/imu`).
const DEFAULT_IMU_TOPIC: &str = "livox/imu";
/// Livox reports specific force in g; `sensor_msgs/Imu` is defined in m/s².
const DEFAULT_ACCEL_SCALE: f64 = 9.806_65;
/// The TF rotation parent = IMU (REP-103 body: x forward, y left, z up), child = optical frame
/// (x right, y down, z forward).
const DEFAULT_IMU_TO_CAMERA_RPY_DEG: [f64; 3] = [-90.0, 0.0, -90.0];
// Per stage. Two stages at 0.35 s left 10.5 % of the vertical gait shake outside turns on the
// dp1f_with_image bag and follow a steady pan 0.7 s behind; the single 0.5 s stage before them left
// 17 % with 0.5 s of lag (see SmoothingOrder).
const DEFAULT_SMOOTHING_TAU_S: f64 = 0.35;
const DEFAULT_SMOOTHING_ORDER: i64 = 2;
const DEFAULT_ZOOM: f64 = 1.15;
const MAX_ZOOM: f64 = 2.0;
const DEFAULT_MAX_IMU_WAIT_MS: f64 = 5.0;
const MAX_IMU_WAIT_MS: f64 = 50.0;
const MAX_IMU_TIME_OFFSET_MS: f64 = 500.0;
// Small on purpose: the warp only uses relative rotation, and a larger gain feeds the gait's
// fore-aft accelerations into the tilt (on the dp1f_with_image bag 0.5 added 2.0 px of vertical
// gait-band shake, 0.05 0.2 px; see imu::Mahony).
const DEFAULT_MAHONY_KP: f64 = 0.05;
const MAX_MAHONY_KP: f64 = 10.0;
const DEFAULT_JPEG_CODEC: &str = "auto";

/// The effective values the node runs on, after validation.
pub struct Config {
    pub pipeline: PipelineConfig,
    /// The launch file prefixes it with `<robot_id>/`, since TF frame names are not namespaced.
    pub frame_id: String,
    /// The camera_calibration YAML published on `camera_info`; empty publishes none.
    pub camera_info_file: String,
    /// `Some` when `stabilize` is true.
    pub stabilize: Option<StabilizeConfig>,
}

/// Everything the stabilized path needs beyond the pipeline.
#[derive(Clone, Debug, PartialEq)]
pub struct StabilizeConfig {
    pub imu_topic: String,
    /// Stamp IMU samples with `header.stamp` (the sensor's own time, which must be in the node
    /// clock's time base) rather than with the node clock at arrival.
    pub imu_use_header_stamp: bool,
    pub eis: EisConfig,
    /// Added to a frame's capture stamp to find its instant on the IMU's timeline: the configured
    /// `imu_time_offset_ms` plus half the exposure time (the stamp marks the start of exposure).
    pub frame_time_offset_ns: i64,
    /// How long the warp waits for an IMU sample newer than the frame before extrapolating.
    pub max_imu_wait: Duration,
    /// Multiplies `linear_acceleration` into m/s².
    pub accel_scale: f64,
    pub mahony: MahonyGains,
    /// Who decodes and re-encodes the frames; resolved against the GStreamer registry in
    /// `mod.rs`, since whether the NVIDIA elements exist is not a parameter.
    pub jpeg_codec: JpegCodecChoice,
}

/// The node's parameters, all of them load-time only.
///
/// Declared `read_only()`: the pipeline is built once from them, so `ros2 param set` could not
/// take effect anyway, and a rejected set says so where a silently ignored one would not. Values
/// still come from the params file and the launch file's overrides, which are applied at
/// declaration time.
///
/// The handles are kept for the node's lifetime because a parameter is undeclared when its handle
/// drops.
pub struct Parameters {
    _text: Vec<ReadOnlyParameter<Arc<str>>>,
    _integer: Vec<ReadOnlyParameter<i64>>,
    _real: Vec<ReadOnlyParameter<f64>>,
    _real_array: Vec<ReadOnlyParameter<Arc<[f64]>>>,
    _flag: Vec<ReadOnlyParameter<bool>>,
}

impl Parameters {
    pub fn declare(node: &Node) -> Result<(Self, Config), DeclarationError> {
        let text = |name: &str, default: &str, description: &str| {
            node.declare_parameter(name)
                // Turbofished: nothing downstream pins the element type
                .default(Arc::<str>::from(default))
                .description(description)
                .read_only()
        };
        let integer = |name: &str, default: i64, description: &str| {
            node.declare_parameter(name)
                .default(default)
                .description(description)
                .read_only()
        };
        let real = |name: &str, default: f64, description: &str| {
            node.declare_parameter(name)
                .default(default)
                .description(description)
                .read_only()
        };
        let real_array = |name: &str, default: &[f64], description: &str| {
            node.declare_parameter(name)
                .default(Arc::<[f64]>::from(default))
                .description(description)
                .read_only()
        };
        let flag = |name: &str, default: bool, description: &str| {
            node.declare_parameter(name)
                .default(default)
                .description(description)
                .read_only()
        };
        let device = text(
            "device",
            DEFAULT_DEVICE,
            "V4L2 capture node (a udev symlink)",
        )?;
        let width = integer("width", DEFAULT_WIDTH.into(), "capture width, in pixels")?;
        let height = integer("height", DEFAULT_HEIGHT.into(), "capture height, in pixels")?;
        let framerate = integer(
            "framerate",
            DEFAULT_FRAMERATE.into(),
            "capture rate; must be one the sensor advertises",
        )?;
        let rtp_host = text("rtp_host", DEFAULT_RTP_HOST, "RTP/JPEG destination host")?;
        let rtp_port = integer(
            "rtp_port",
            DEFAULT_RTP_PORT.into(),
            "RTP/JPEG destination port; the WebRTC worker's VIDEO_RTP_IN_PORT",
        )?;
        let ros_framerate = integer(
            "ros_framerate",
            DEFAULT_ROS_FRAMERATE.into(),
            "image_raw/compressed rate; 0 or >= framerate publishes every frame",
        )?;
        let frame_id = text(
            "frame_id",
            DEFAULT_FRAME_ID,
            "header.frame_id of the images",
        )?;
        let source_override = text(
            "source_override",
            "",
            "replaces the v4l2src head of the pipeline when set; must produce image/jpeg",
        )?;
        let camera_info_file = text(
            "camera_info_file",
            "",
            "camera_calibration YAML (ost.yaml) published on camera_info; empty = none",
        )?;
        let auto_exposure = integer(
            "auto_exposure",
            0,
            "0 = auto, 1 = manual (UVC menu numbering)",
        )?;
        let exposure_time = integer(
            "exposure_time",
            DEFAULT_EXPOSURE_TIME_100US.into(),
            "exposure in 100 us units; only applied when auto_exposure is 1",
        )?;
        let white_balance_automatic =
            integer("white_balance_automatic", 0, "1 = automatic, 0 = manual")?;
        let white_balance_temperature = integer(
            "white_balance_temperature",
            DEFAULT_WHITE_BALANCE_KELVIN.into(),
            "Kelvin; only applied when white_balance_automatic is 0",
        )?;
        let gain = integer("gain", SensorControls::default().gain, "sensor gain")?;
        let brightness = integer(
            "brightness",
            SensorControls::default().brightness,
            "sensor brightness",
        )?;
        let power_line_frequency = integer(
            "power_line_frequency",
            1,
            "flicker correction: 0 off, 1 = 50 Hz, 2 = 60 Hz",
        )?;

        let stabilize = flag(
            "stabilize",
            false,
            "warp each frame by the IMU orientation (decodes and re-encodes on the CPU)",
        )?;
        let imu_topic = text("imu_topic", DEFAULT_IMU_TOPIC, "sensor_msgs/Imu source")?;
        let imu_use_header_stamp = flag(
            "imu_use_header_stamp",
            true,
            "stamp IMU samples with header.stamp (true) or with the node clock on arrival",
        )?;
        let fx = real(
            "fx",
            0.0,
            "focal length x, px; 0 = width / 2 (about 90 deg HFOV)",
        )?;
        let fy = real("fy", 0.0, "focal length y, px; 0 = same as fx")?;
        let cx = real("cx", 0.0, "principal point x, px; 0 = image centre")?;
        let cy = real("cy", 0.0, "principal point y, px; 0 = image centre")?;
        let imu_to_camera_rpy_deg = real_array(
            "imu_to_camera_rpy_deg",
            &DEFAULT_IMU_TO_CAMERA_RPY_DEG,
            "TF rotation parent = IMU frame, child = camera optical frame, as roll pitch yaw in \
             degrees",
        )?;
        let imu_time_offset_ms = real(
            "imu_time_offset_ms",
            0.0,
            "added to each frame's capture stamp to find it on the IMU timeline",
        )?;
        let smoothing_time_constant_s = real(
            "smoothing_time_constant_s",
            DEFAULT_SMOOTHING_TAU_S,
            "time constant of each low-pass stage the virtual camera follows the real one with",
        )?;
        let smoothing_order = integer(
            "smoothing_order",
            DEFAULT_SMOOTHING_ORDER,
            "low-pass stages the virtual camera follows the real one through: 1 or 2",
        )?;
        let zoom = real(
            "zoom",
            DEFAULT_ZOOM,
            "output magnification hiding the borders the warp uncovers; 1.0 to 2.0",
        )?;
        let max_imu_wait_ms = real(
            "max_imu_wait_ms",
            DEFAULT_MAX_IMU_WAIT_MS,
            "how long a frame waits for an IMU sample newer than itself; 0 to 50",
        )?;
        let imu_accel_scale = real(
            "imu_accel_scale",
            DEFAULT_ACCEL_SCALE,
            "multiplies linear_acceleration into m/s^2 (Livox reports g)",
        )?;
        let mahony_kp = real(
            "mahony_kp",
            DEFAULT_MAHONY_KP,
            "accelerometer correction gain of the orientation filter; 0 = gyro only",
        )?;
        let jpeg_codec = text(
            "jpeg_codec",
            DEFAULT_JPEG_CODEC,
            "stabilized path's JPEG decoder/encoder: auto (NVIDIA when its elements exist), \
             nvidia or software",
        )?;

        let mut warn = |message: String| log_warn!(node.logger(), "[Parameters] {message}");

        let framerate_value = positive("framerate", framerate.get(), DEFAULT_FRAMERATE, &mut warn);
        let width_value = positive("width", width.get(), DEFAULT_WIDTH, &mut warn);
        let height_value = positive("height", height.get(), DEFAULT_HEIGHT, &mut warn);
        let ros_framerate_value = ros_rate(ros_framerate.get(), framerate_value);
        let exposure_value = exposure(auto_exposure.get(), exposure_time.get(), &mut warn);

        let stabilize_config = stabilize.get().then(|| {
            if ros_framerate_value >= framerate_value {
                warn(format!(
                    "stabilize is on with no lower ros_framerate: every frame is decoded, warped \
                     and re-encoded at {framerate_value} fps"
                ));
            }
            let intrinsics = intrinsics(
                [fx.get(), fy.get(), cx.get(), cy.get()],
                width_value,
                height_value,
                &mut warn,
            );
            let zoom = bounded("zoom", zoom.get(), 1.0, MAX_ZOOM, DEFAULT_ZOOM, &mut warn);
            let offset_ms = bounded(
                "imu_time_offset_ms",
                imu_time_offset_ms.get(),
                -MAX_IMU_TIME_OFFSET_MS,
                MAX_IMU_TIME_OFFSET_MS,
                0.0,
                &mut warn,
            );
            let wait_ms = bounded(
                "max_imu_wait_ms",
                max_imu_wait_ms.get(),
                0.0,
                MAX_IMU_WAIT_MS,
                DEFAULT_MAX_IMU_WAIT_MS,
                &mut warn,
            );
            StabilizeConfig {
                imu_topic: imu_topic.get().to_string(),
                imu_use_header_stamp: imu_use_header_stamp.get(),
                eis: EisConfig {
                    intrinsics,
                    width: width_value,
                    height: height_value,
                    zoom,
                    smoothing_tau_s: positive_real(
                        "smoothing_time_constant_s",
                        smoothing_time_constant_s.get(),
                        DEFAULT_SMOOTHING_TAU_S,
                        &mut warn,
                    ),
                    smoothing_order: smoothing_order_from(smoothing_order.get(), &mut warn),
                    cam_from_imu: cam_from_imu(&imu_to_camera_rpy_deg.get(), &mut warn),
                },
                frame_time_offset_ns: frame_time_offset_ns(offset_ms, exposure_value),
                max_imu_wait: Duration::from_secs_f64(wait_ms / 1000.0),
                accel_scale: positive_real(
                    "imu_accel_scale",
                    imu_accel_scale.get(),
                    DEFAULT_ACCEL_SCALE,
                    &mut warn,
                ),
                mahony: MahonyGains {
                    kp: bounded(
                        "mahony_kp",
                        mahony_kp.get(),
                        0.0,
                        MAX_MAHONY_KP,
                        DEFAULT_MAHONY_KP,
                        &mut warn,
                    ),
                    ki: 0.0,
                },
                jpeg_codec: jpeg_codec_choice(&jpeg_codec.get(), &mut warn),
            }
        });

        let config = Config {
            pipeline: PipelineConfig {
                device: device.get().to_string(),
                width: width_value,
                height: height_value,
                framerate: framerate_value,
                rtp_host: rtp_host.get().to_string(),
                rtp_port: port(rtp_port.get(), &mut warn),
                ros_framerate: ros_framerate_value,
                source_override: source_override.get().to_string(),
                sensor_controls: SensorControls {
                    exposure: exposure_value,
                    white_balance: white_balance(
                        white_balance_automatic.get(),
                        white_balance_temperature.get(),
                        &mut warn,
                    ),
                    gain: gain.get(),
                    brightness: brightness.get(),
                    power_line_frequency: power_line_frequency_from(
                        power_line_frequency.get(),
                        &mut warn,
                    ),
                },
            },
            frame_id: frame_id.get().to_string(),
            camera_info_file: camera_info_file.get().to_string(),
            stabilize: stabilize_config,
        };

        Ok((
            Self {
                _text: vec![
                    device,
                    rtp_host,
                    frame_id,
                    source_override,
                    camera_info_file,
                    imu_topic,
                    jpeg_codec,
                ],
                _integer: vec![
                    width,
                    height,
                    framerate,
                    rtp_port,
                    ros_framerate,
                    auto_exposure,
                    exposure_time,
                    white_balance_automatic,
                    white_balance_temperature,
                    gain,
                    brightness,
                    power_line_frequency,
                    smoothing_order,
                ],
                _real: vec![
                    fx,
                    fy,
                    cx,
                    cy,
                    imu_time_offset_ms,
                    smoothing_time_constant_s,
                    zoom,
                    max_imu_wait_ms,
                    imu_accel_scale,
                    mahony_kp,
                ],
                _real_array: vec![imu_to_camera_rpy_deg],
                _flag: vec![stabilize, imu_use_header_stamp],
            },
            config,
        ))
    }
}

/// `fx fy cx cy`, with 0 standing for the resolution-derived guess: a focal length of half the
/// width (about 90° horizontal field of view) and the principal point at the image centre. The
/// guess is for bringing the path up; stabilizing well needs a calibrated value.
fn intrinsics(
    [fx, fy, cx, cy]: [f64; 4],
    width: u32,
    height: u32,
    warn: &mut impl FnMut(String),
) -> Intrinsics {
    let (w, h) = (f64::from(width), f64::from(height));
    let fx = if fx == 0.0 {
        w / 2.0
    } else {
        positive_real("fx", fx, w / 2.0, warn)
    };
    let fy = if fy == 0.0 {
        fx
    } else {
        positive_real("fy", fy, fx, warn)
    };
    let cx = if cx == 0.0 {
        w / 2.0
    } else {
        bounded("cx", cx, 0.0, w, w / 2.0, warn)
    };
    let cy = if cy == 0.0 {
        h / 2.0
    } else {
        bounded("cy", cy, 0.0, h, h / 2.0, warn)
    };
    Intrinsics { fx, fy, cx, cy }
}

/// The rotation taking IMU-frame vectors into the optical frame: the transpose of the TF
/// rotation (parent = IMU, child = camera) the parameter spells out.
fn cam_from_imu(rpy_deg: &[f64], warn: &mut impl FnMut(String)) -> Mat3 {
    let [roll, pitch, yaw] = match rpy_deg {
        [r, p, y] if [r, p, y].iter().all(|v| v.is_finite()) => [*r, *p, *y],
        other => {
            warn(format!(
                "imu_to_camera_rpy_deg={other:?} is not three finite angles; using \
                 {DEFAULT_IMU_TO_CAMERA_RPY_DEG:?}"
            ));
            DEFAULT_IMU_TO_CAMERA_RPY_DEG
        }
    };
    Mat3::from_rpy_deg(roll, pitch, yaw).transpose()
}

/// 1 or 2 stages; anything else falls back to the default with a warning.
fn smoothing_order_from(value: i64, warn: &mut impl FnMut(String)) -> SmoothingOrder {
    match value {
        1 => SmoothingOrder::First,
        2 => SmoothingOrder::Second,
        other => {
            warn(format!(
                "smoothing_order={other} is not 1 or 2; using {DEFAULT_SMOOTHING_ORDER}"
            ));
            smoothing_order_from(DEFAULT_SMOOTHING_ORDER, warn)
        }
    }
}

/// The capture stamp marks the start of exposure; the orientation that shaped the frame is the
/// one halfway through it. In auto exposure the time is unknown and the offset parameter has to
/// absorb it.
fn frame_time_offset_ns(offset_ms: f64, exposure: Exposure) -> i64 {
    let half_exposure_ns = match exposure {
        Exposure::Auto => 0,
        Exposure::Manual { time_100us } => i64::from(time_100us) * 50_000,
    };
    (offset_ms * 1e6) as i64 + half_exposure_ns
}

fn positive_real(name: &str, value: f64, default: f64, warn: &mut impl FnMut(String)) -> f64 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        warn(format!(
            "{name}={value} is not a positive value; using {default}"
        ));
        default
    }
}

fn bounded(
    name: &str,
    value: f64,
    min: f64,
    max: f64,
    default: f64,
    warn: &mut impl FnMut(String),
) -> f64 {
    if value.is_finite() && (min..=max).contains(&value) {
        value
    } else {
        warn(format!(
            "{name}={value} is outside {min}..={max}; using {default}"
        ));
        default
    }
}

/// A size, rate, exposure time or temperature: zero or negative is meaningless for all of them,
/// and a value past `u32` is not one any sensor offers. Either falls back to the default, with a
/// warning naming the parameter.
fn positive(name: &str, value: i64, default: u32, warn: &mut impl FnMut(String)) -> u32 {
    match u32::try_from(value) {
        Ok(v) if v > 0 => v,
        _ => {
            warn(format!(
                "{name}={value} is not a positive value; using {default}"
            ));
            default
        }
    }
}

/// `auto_exposure` (UVC menu: 0 = auto, 1 = manual) plus `exposure_time`, which is read only in
/// manual mode. Any other mode value falls back to the default mode with a warning rather than
/// being written to the driver verbatim.
fn exposure(mode: i64, time_100us: i64, warn: &mut impl FnMut(String)) -> Exposure {
    match mode {
        0 => Exposure::Auto,
        1 => Exposure::Manual {
            time_100us: positive(
                "exposure_time",
                time_100us,
                DEFAULT_EXPOSURE_TIME_100US,
                warn,
            ),
        },
        _ => {
            let fallback = SensorControls::default().exposure;
            warn(format!(
                "auto_exposure={mode} is neither 0 (auto) nor 1 (manual); using {fallback:?}"
            ));
            fallback
        }
    }
}

/// `white_balance_automatic` (boolean: 1 = auto, 0 = manual — the OPPOSITE polarity of
/// `auto_exposure`) plus `white_balance_temperature`, read only in manual mode.
fn white_balance(automatic: i64, kelvin: i64, warn: &mut impl FnMut(String)) -> WhiteBalance {
    match automatic {
        1 => WhiteBalance::Auto,
        0 => WhiteBalance::Manual {
            kelvin: positive(
                "white_balance_temperature",
                kelvin,
                DEFAULT_WHITE_BALANCE_KELVIN,
                warn,
            ),
        },
        _ => {
            let fallback = SensorControls::default().white_balance;
            warn(format!(
                "white_balance_automatic={automatic} is neither 1 (auto) nor 0 (manual); \
                 using {fallback:?}"
            ));
            fallback
        }
    }
}

fn power_line_frequency_from(value: i64, warn: &mut impl FnMut(String)) -> PowerLineFrequency {
    match value {
        0 => PowerLineFrequency::Disabled,
        1 => PowerLineFrequency::Hz50,
        2 => PowerLineFrequency::Hz60,
        _ => {
            let fallback = SensorControls::default().power_line_frequency;
            warn(format!(
                "power_line_frequency={value} is not 0, 1 or 2; using {fallback:?}"
            ));
            fallback
        }
    }
}

/// `jpeg_codec`, case and surrounding whitespace ignored. Anything else falls back to auto with a
/// warning: auto can only ever pick a chain that exists, so it is the one fallback that cannot
/// stop the camera.
fn jpeg_codec_choice(value: &str, warn: &mut impl FnMut(String)) -> JpegCodecChoice {
    match value.trim().to_ascii_lowercase().as_str() {
        "auto" => JpegCodecChoice::Auto,
        "nvidia" => JpegCodecChoice::Nvidia,
        "software" => JpegCodecChoice::Software,
        _ => {
            warn(format!(
                "jpeg_codec=\"{value}\" is not auto, nvidia or software; using auto"
            ));
            JpegCodecChoice::Auto
        }
    }
}

fn port(value: i64, warn: &mut impl FnMut(String)) -> u16 {
    match u16::try_from(value) {
        Ok(v) if v > 0 => v,
        _ => {
            warn(format!(
                "rtp_port={value} is not a UDP port; using {DEFAULT_RTP_PORT}"
            ));
            DEFAULT_RTP_PORT
        }
    }
}

/// `ros_framerate`: 0 — and anything that is not a lower positive rate — means the ROS branch
/// carries every captured frame.
fn ros_rate(value: i64, framerate: u32) -> u32 {
    match u32::try_from(value) {
        Ok(v) if v > 0 && v < framerate => v,
        _ => framerate,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ignore(_: String) {}

    #[test]
    fn positive_values_pass_through() {
        assert_eq!(positive("width", 640, DEFAULT_WIDTH, &mut ignore), 640);
    }

    #[test]
    fn non_positive_and_oversized_values_fall_back_with_a_warning() {
        for value in [0, -1, i64::from(u32::MAX) + 1] {
            let mut warnings = Vec::new();
            let got = positive("width", value, DEFAULT_WIDTH, &mut |m| warnings.push(m));
            assert_eq!(got, DEFAULT_WIDTH);
            assert_eq!(warnings.len(), 1);
            assert!(warnings[0].contains("width"), "{}", warnings[0]);
        }
    }

    #[test]
    fn ports_outside_the_udp_range_fall_back() {
        assert_eq!(port(5010, &mut ignore), 5010);
        assert_eq!(port(0, &mut ignore), DEFAULT_RTP_PORT);
        assert_eq!(port(65_536, &mut ignore), DEFAULT_RTP_PORT);
        assert_eq!(port(-1, &mut ignore), DEFAULT_RTP_PORT);
    }

    #[test]
    fn exposure_follows_the_uvc_menu_numbering() {
        assert_eq!(exposure(0, 999, &mut ignore), Exposure::Auto);
        assert_eq!(
            exposure(1, 100, &mut ignore),
            Exposure::Manual { time_100us: 100 }
        );
        assert_eq!(
            exposure(1, 0, &mut ignore),
            Exposure::Manual {
                time_100us: DEFAULT_EXPOSURE_TIME_100US
            }
        );
    }

    #[test]
    fn white_balance_has_the_opposite_polarity() {
        assert_eq!(white_balance(1, 999, &mut ignore), WhiteBalance::Auto);
        assert_eq!(
            white_balance(0, 4000, &mut ignore),
            WhiteBalance::Manual { kelvin: 4000 }
        );
    }

    #[test]
    fn unknown_modes_fall_back_to_the_defaults_with_a_warning() {
        let defaults = SensorControls::default();
        let mut warnings = Vec::new();
        let mut warn = |m| warnings.push(m);
        assert_eq!(exposure(7, 100, &mut warn), defaults.exposure);
        assert_eq!(white_balance(2, 100, &mut warn), defaults.white_balance);
        assert_eq!(
            power_line_frequency_from(3, &mut warn),
            defaults.power_line_frequency
        );
        assert_eq!(warnings.len(), 3, "{warnings:?}");
    }

    #[test]
    fn param_file_defaults_parse_to_sensor_controls_default() {
        // The raw numbers declared as parameter defaults must mean SensorControls::default().
        let defaults = SensorControls::default();
        assert_eq!(
            exposure(0, DEFAULT_EXPOSURE_TIME_100US.into(), &mut ignore),
            defaults.exposure
        );
        assert_eq!(
            white_balance(0, DEFAULT_WHITE_BALANCE_KELVIN.into(), &mut ignore),
            defaults.white_balance
        );
        assert_eq!(
            power_line_frequency_from(1, &mut ignore),
            defaults.power_line_frequency
        );
    }

    #[test]
    fn ros_rate_defaults_to_every_frame() {
        assert_eq!(ros_rate(0, 60), 60);
        assert_eq!(ros_rate(-5, 60), 60);
        assert_eq!(ros_rate(60, 60), 60);
        assert_eq!(ros_rate(120, 60), 60);
        assert_eq!(ros_rate(30, 60), 30);
    }

    #[test]
    fn zero_intrinsics_mean_the_resolution_derived_guess() {
        let k = intrinsics([0.0; 4], 1280, 720, &mut ignore);
        assert_eq!(
            k,
            Intrinsics {
                fx: 640.0,
                fy: 640.0,
                cx: 640.0,
                cy: 360.0
            }
        );
        // fy follows a given fx.
        let k = intrinsics([900.0, 0.0, 0.0, 0.0], 1280, 720, &mut ignore);
        assert_eq!(k.fy, 900.0);
    }

    #[test]
    fn bad_intrinsics_fall_back_with_a_warning() {
        let mut warnings = Vec::new();
        let k = intrinsics([-1.0, f64::NAN, 5000.0, 100.0], 1280, 720, &mut |m| {
            warnings.push(m)
        });
        assert_eq!(k.fx, 640.0);
        assert_eq!(k.fy, 640.0);
        assert_eq!(k.cx, 640.0);
        assert_eq!(k.cy, 100.0);
        assert_eq!(warnings.len(), 3, "{warnings:?}");
    }

    #[test]
    fn the_imu_to_camera_rotation_takes_body_forward_to_optical_z() {
        let c = cam_from_imu(&DEFAULT_IMU_TO_CAMERA_RPY_DEG, &mut ignore);
        let forward = c.mul_vec([1.0, 0.0, 0.0]);
        assert!((forward[2] - 1.0).abs() < 1e-9, "{forward:?}");
        // Body "up" is optical -y.
        let up = c.mul_vec([0.0, 0.0, 1.0]);
        assert!((up[1] + 1.0).abs() < 1e-9, "{up:?}");
    }

    #[test]
    fn a_malformed_rotation_falls_back_to_the_default() {
        let mut warnings = Vec::new();
        let c = cam_from_imu(&[1.0, 2.0], &mut |m| warnings.push(m));
        assert_eq!(c, cam_from_imu(&DEFAULT_IMU_TO_CAMERA_RPY_DEG, &mut ignore));
        assert_eq!(warnings.len(), 1);
        let c = cam_from_imu(&[0.0, f64::INFINITY, 0.0], &mut |m| warnings.push(m));
        assert_eq!(c, cam_from_imu(&DEFAULT_IMU_TO_CAMERA_RPY_DEG, &mut ignore));
        assert_eq!(warnings.len(), 2);
    }

    #[test]
    fn the_frame_time_offset_adds_half_the_manual_exposure() {
        assert_eq!(frame_time_offset_ns(0.0, Exposure::Auto), 0);
        assert_eq!(frame_time_offset_ns(-12.5, Exposure::Auto), -12_500_000);
        // 330 x 100 us = 33 ms of exposure: half is 16.5 ms.
        assert_eq!(
            frame_time_offset_ns(0.0, Exposure::Manual { time_100us: 330 }),
            16_500_000
        );
    }

    #[test]
    fn jpeg_codec_names_are_case_insensitive_and_unknown_ones_mean_auto() {
        assert_eq!(
            jpeg_codec_choice("auto", &mut ignore),
            JpegCodecChoice::Auto
        );
        assert_eq!(
            jpeg_codec_choice(" NVIDIA ", &mut ignore),
            JpegCodecChoice::Nvidia
        );
        assert_eq!(
            jpeg_codec_choice("Software", &mut ignore),
            JpegCodecChoice::Software
        );
        let mut warnings = Vec::new();
        assert_eq!(
            jpeg_codec_choice("nvjpeg", &mut |m| warnings.push(m)),
            JpegCodecChoice::Auto
        );
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].contains("jpeg_codec"), "{}", warnings[0]);
    }

    #[test]
    fn the_smoothing_order_is_one_or_two_stages() {
        assert_eq!(smoothing_order_from(1, &mut ignore), SmoothingOrder::First);
        assert_eq!(smoothing_order_from(2, &mut ignore), SmoothingOrder::Second);
        let mut warnings = Vec::new();
        assert_eq!(
            smoothing_order_from(3, &mut |m| warnings.push(m)),
            SmoothingOrder::Second
        );
        assert_eq!(
            smoothing_order_from(0, &mut |m| warnings.push(m)),
            SmoothingOrder::Second
        );
        assert_eq!(warnings.len(), 2, "{warnings:?}");
        assert!(warnings[0].contains("smoothing_order"), "{}", warnings[0]);
    }

    #[test]
    fn bounded_values_fall_back_outside_their_range() {
        assert_eq!(bounded("zoom", 1.3, 1.0, 2.0, 1.15, &mut ignore), 1.3);
        assert_eq!(bounded("zoom", 0.9, 1.0, 2.0, 1.15, &mut ignore), 1.15);
        assert_eq!(bounded("zoom", 2.5, 1.0, 2.0, 1.15, &mut ignore), 1.15);
        assert_eq!(bounded("zoom", f64::NAN, 1.0, 2.0, 1.15, &mut ignore), 1.15);
        assert_eq!(positive_real("tau", 0.0, 0.5, &mut ignore), 0.5);
        assert_eq!(positive_real("tau", 0.2, 0.5, &mut ignore), 0.2);
    }
}
