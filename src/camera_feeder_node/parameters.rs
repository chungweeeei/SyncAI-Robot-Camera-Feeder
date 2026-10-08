use rclrs::*;
use std::sync::Arc;

use super::pipeline::{
    DEFAULT_EXPOSURE_TIME_100US, DEFAULT_WHITE_BALANCE_KELVIN, Exposure, PipelineConfig,
    PowerLineFrequency, SensorControls, WhiteBalance,
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

/// The effective values the node runs on, after validation.
pub struct Config {
    pub pipeline: PipelineConfig,
    /// The launch file prefixes it with `<robot_id>/`, since TF frame names are not namespaced.
    pub frame_id: String,
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

        let mut warn = |message: String| log_warn!(node.logger(), "[Parameters] {message}");

        let framerate_value = positive("framerate", framerate.get(), DEFAULT_FRAMERATE, &mut warn);
        let config = Config {
            pipeline: PipelineConfig {
                device: device.get().to_string(),
                width: positive("width", width.get(), DEFAULT_WIDTH, &mut warn),
                height: positive("height", height.get(), DEFAULT_HEIGHT, &mut warn),
                framerate: framerate_value,
                rtp_host: rtp_host.get().to_string(),
                rtp_port: port(rtp_port.get(), &mut warn),
                ros_framerate: ros_rate(ros_framerate.get(), framerate_value),
                source_override: source_override.get().to_string(),
                sensor_controls: SensorControls {
                    exposure: exposure(auto_exposure.get(), exposure_time.get(), &mut warn),
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
        };

        Ok((
            Self {
                _text: vec![device, rtp_host, frame_id, source_override],
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
                ],
            },
            config,
        ))
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
}
