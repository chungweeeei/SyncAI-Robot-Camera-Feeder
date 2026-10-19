mod bus;
mod camera_info;
mod eis;
mod imu;
mod math;
mod parameters;
mod pipeline;
mod rate_limit;
mod stamp;
mod warp;
mod warp_bridge;

use std::error::Error;
use std::sync::{Arc, Condvar, Mutex, PoisonError};
use std::time::Duration;

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use rclrs::*;
// Aliased: `rclrs::*` also exports a `Time` (the clock reading), and the message type must not be
// mistaken for it.
use ros_env::builtin_interfaces::msg::Time as TimeMsg;
use ros_env::sensor_msgs::msg::{CameraInfo, CompressedImage, Imu};
use ros_env::std_msgs::msg::Header;

use bus::BusWatch;
use eis::Stabilizer;
use imu::{ImuBuffer, ImuSample, SharedImu};
use parameters::{Parameters, StabilizeConfig};
use pipeline::{JpegCodec, JpegCodecChoice, PipelineConfig};
use rate_limit::RateLimiter;
use stamp::FrameTiming;
use warp_bridge::{SharedStamps, WarpTiming};

/// Two seconds of IMU history at 200 Hz.
const IMU_HISTORY: usize = 400;

/// The sole opener of the camera: one capture pipeline whose MJPEG stream is teed to the WebRTC
/// worker as RTP/JPEG and to ROS as `sensor_msgs/CompressedImage` on the relative topic
/// `image_raw/compressed` (so `<robot_id>/image_raw/compressed`). See PROPOSAL.md.
///
/// With a `camera_info_file`, the calibration is published once on `camera_info`, latched
/// (reliable + transient local), rather than beside every image: it never changes while the node
/// runs. Only on the passthrough path: see `camera_info_publisher`.
///
/// With `stabilize` on, the capture pipeline instead ends in a raw appsink, each decoded frame is
/// warped by the IMU orientation at its capture time (`warp_bridge.rs`), and a second, output
/// pipeline encodes the warped frames and carries the same two branches. Decoding and encoding
/// run on the Jetson's hardware JPEG blocks when the L4T plugins are there (`jpeg_codec`), on the
/// CPU otherwise; the warp is on the CPU either way.
///
/// # Threading
///
/// No rclrs worker or timer. The one executor callback is the IMU subscription, present only
/// with `stabilize` on; it runs on the executor thread `main.rs` spins and does nothing but push
/// the sample into the buffer shared with the warp. Frames are published straight from an
/// appsink's GStreamer streaming thread (an rclrs `Publisher` is an `Arc` and `publish` takes
/// `&self`); the warp runs on the capture pipeline's streaming thread, fanned out across rayon's
/// workers. Each pipeline's bus is watched on a thread of its own. The executor otherwise only
/// spins so the node exists in the graph and serves its parameters.
pub struct CameraFeederNode {
    // Drop::drop takes the pipelines to NULL first; the fields then drop in declaration order, so
    // the bus watchers are joined before the last pipeline references and the node go away.
    _bus_watches: Vec<BusWatch>,
    _imu_subscription: Option<Subscription<Imu>>,
    /// Held, never published to again: the latched sample lives in this writer, so dropping it
    /// would take the calibration away from every subscriber that joins later.
    _camera_info: Option<Publisher<CameraInfo>>,
    /// The capture pipeline: the one that opens the camera.
    pipeline: gst::Pipeline,
    /// The stabilized path's output pipeline (`appsrc ! jpegenc|nvjpegenc ! tee …`).
    output_pipeline: Option<gst::Pipeline>,
    _parameters: Parameters,
    node: Node,
}

impl CameraFeederNode {
    pub fn new(node: Node) -> Result<Self, Box<dyn Error>> {
        let (parameters, config) = Parameters::declare(&node)?;

        // SensorData QoS, the same as vizionsdk_ros2's publisher of this topic, so existing
        // subscribers keep matching. A best-effort publisher cannot satisfy a RELIABLE
        // subscriber: subscribe with SensorData QoS too.
        let publisher: Publisher<CompressedImage> =
            node.create_publisher("image_raw/compressed".keep_last(5).best_effort().volatile())?;
        let camera_info = camera_info_publisher(&node, &config)?;

        let (pipeline, output_pipeline, imu_subscription) = match &config.stabilize {
            None => {
                let description = pipeline::build_pipeline_string(&config.pipeline);
                log_info!(node.logger(), "[CameraFeederNode] pipeline: {description}");
                let pipeline = parse(&description)?;
                limit_rate(&pipeline, &config.pipeline)?;
                appsink(&pipeline, pipeline::ROS_SINK_NAME)?.set_callbacks(frame_publisher(
                    publisher,
                    node.get_clock(),
                    node.logger().clone(),
                    config.frame_id,
                    StampSource::PipelineClock,
                ));
                (pipeline, None, None)
            }
            Some(stabilize) => {
                let codec = jpeg_codec(stabilize.jpeg_codec, node.logger())?;
                let descriptions = pipeline::build_stabilized_pipelines(&config.pipeline, codec);
                log_info!(
                    node.logger(),
                    "[CameraFeederNode] capture pipeline: {}",
                    descriptions.capture
                );
                log_info!(
                    node.logger(),
                    "[CameraFeederNode] output pipeline: {}",
                    descriptions.output
                );
                let capture = parse(&descriptions.capture)?;
                limit_rate(&capture, &config.pipeline)?;
                let output = parse(&descriptions.output)?;
                let appsrc = output
                    .by_name(pipeline::WARPED_SRC_NAME)
                    .and_then(|element| element.downcast::<gst_app::AppSrc>().ok())
                    .ok_or("the output pipeline has no appsrc for the warped frames")?;

                let stamps: SharedStamps = Arc::default();
                appsink(&output, pipeline::ROS_SINK_NAME)?.set_callbacks(frame_publisher(
                    publisher,
                    node.get_clock(),
                    node.logger().clone(),
                    config.frame_id,
                    StampSource::Table(Arc::clone(&stamps)),
                ));

                let imu: SharedImu = Arc::new((
                    Mutex::new(ImuBuffer::new(IMU_HISTORY, stabilize.mahony)),
                    Condvar::new(),
                ));
                appsink(&capture, pipeline::RAW_SINK_NAME)?.set_callbacks(
                    warp_bridge::raw_frame_warper(
                        appsrc,
                        Arc::clone(&imu),
                        Stabilizer::new(stabilize.eis),
                        WarpTiming {
                            frame_time_offset_ns: stabilize.frame_time_offset_ns,
                            max_imu_wait: stabilize.max_imu_wait,
                            frame_duration: gst::ClockTime::from_nseconds(
                                1_000_000_000 / u64::from(config.pipeline.ros_framerate.max(1)),
                            ),
                        },
                        stamps,
                        node.get_clock(),
                        node.logger().clone(),
                    ),
                );
                let subscription = imu_subscription(&node, stabilize, imu)?;
                (capture, Some(output), Some(subscription))
            }
        };

        // Before PLAYING, so an error during the state change is already being watched for.
        let mut bus_watches = Vec::new();
        for p in output_pipeline.iter().chain(std::iter::once(&pipeline)) {
            bus_watches.push(BusWatch::start(p.clone(), node.logger().clone())?);
        }

        // The output pipeline first: it must be ready to take frames when the capture starts.
        if let Some(output) = &output_pipeline {
            start(output, node.logger())?;
        }
        if let Err(err) = start(&pipeline, node.logger()) {
            if let Some(output) = &output_pipeline {
                stop(output, node.logger());
            }
            return Err(err);
        }

        match &config.stabilize {
            None => log_info!(
                node.logger(),
                "[CameraFeederNode] streaming {}x{}@{} to {}:{} (RTP/JPEG) and image_raw/compressed @{}",
                config.pipeline.width,
                config.pipeline.height,
                config.pipeline.framerate,
                config.pipeline.rtp_host,
                config.pipeline.rtp_port,
                config.pipeline.ros_framerate,
            ),
            Some(stabilize) => log_info!(
                node.logger(),
                "[CameraFeederNode] streaming {}x{}@{}, stabilized by {} (zoom {}, {} x tau {} s) \
                 at {} fps to {}:{} (RTP/JPEG) and image_raw/compressed",
                config.pipeline.width,
                config.pipeline.height,
                config.pipeline.framerate,
                stabilize.imu_topic,
                stabilize.eis.zoom,
                stabilize.eis.smoothing_order.stages(),
                stabilize.eis.smoothing_tau_s,
                config.pipeline.ros_framerate,
                config.pipeline.rtp_host,
                config.pipeline.rtp_port,
            ),
        }

        Ok(Self {
            _bus_watches: bus_watches,
            _imu_subscription: imu_subscription,
            _camera_info: camera_info,
            pipeline,
            output_pipeline,
            _parameters: parameters,
            node,
        })
    }
}

impl Drop for CameraFeederNode {
    fn drop(&mut self) {
        // Runs only on an orderly shutdown — rclrs installs no SIGINT handler, so Ctrl-C and
        // SIGTERM end the process without it, and the kernel closes the device instead.
        // Capture first (it holds the device and feeds the other), then the output.
        stop(&self.pipeline, self.node.logger());
        if let Some(output) = &self.output_pipeline {
            stop(output, self.node.logger());
        }
    }
}

/// The stabilized path's codec: `choice` resolved against the elements the GStreamer registry
/// actually has. Asked of the registry rather than configured, because the same params file runs
/// on the robot (nvidia runtime, L4T plugins injected) and in the Dev Container (neither).
fn jpeg_codec(choice: JpegCodecChoice, logger: &Logger) -> Result<JpegCodec, Box<dyn Error>> {
    let missing =
        pipeline::missing_nvidia_elements(|name| gst::ElementFactory::find(name).is_some());
    let codec = pipeline::resolve_codec(choice, &missing)
        .map_err(|err| format!("[CameraFeederNode] {err}"))?;
    if choice == JpegCodecChoice::Auto && codec == JpegCodec::Software {
        log_info!(
            logger,
            "[CameraFeederNode] jpeg_codec auto: no {} in this GStreamer, so frames are decoded \
             and encoded on the CPU",
            missing.join(" / ")
        );
    } else {
        log_info!(
            logger,
            "[CameraFeederNode] jpeg_codec {choice:?}: decoding and encoding with {codec:?}"
        );
    }
    Ok(codec)
}

fn parse(description: &str) -> Result<gst::Pipeline, Box<dyn Error>> {
    Ok(gst::parse::launch(description)?
        .downcast::<gst::Pipeline>()
        .map_err(|_| "the pipeline description did not produce a gst::Pipeline")?)
}

/// Drops frames down to `ros_framerate` at the pipeline's rate-limit element, by PTS and without
/// restamping the ones it keeps (`rate_limit.rs` has why not `videorate`). Nothing to do at full
/// rate, where the element does not exist.
///
/// A sink-pad probe, so a dropped frame never enters the element, nor the queue and decoder after
/// it on the stabilized path. The probe runs on the one streaming thread that pushes into the
/// element, so the lock is never contended; it is there because a probe must be `Fn + Sync`.
///
/// A rejected frame is returned as `Handled`, not `Drop`. gstreamer-rs (0.24.5, and 0.25.4 the
/// same) takes the buffer out of the probe info before calling the probe and, on `Drop`, frees it
/// and leaves the info's data NULL; GStreamer 1.20's chain path then unrefs that NULL itself, a
/// `gst_mini_object_unref: assertion 'mini_object != NULL' failed` CRITICAL for every dropped
/// frame (30 a second on robot01). `Handled` tells the core the probe consumed the buffer, which
/// is what the binding did, and the flow return stays OK.
fn limit_rate(pipeline: &gst::Pipeline, config: &PipelineConfig) -> Result<(), Box<dyn Error>> {
    if !pipeline::is_rate_limited(config) {
        return Ok(());
    }
    let pad = pipeline
        .by_name(pipeline::RATE_LIMIT_NAME)
        .and_then(|element| element.static_pad("sink"))
        .ok_or("the pipeline has no rate-limit element")?;
    let limiter = Mutex::new(RateLimiter::new(config.framerate, config.ros_framerate));
    pad.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        let pts = info
            .buffer()
            .and_then(|buffer| buffer.pts())
            .map(gst::ClockTime::nseconds);
        if limiter
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .keep(pts)
        {
            gst::PadProbeReturn::Ok
        } else {
            gst::PadProbeReturn::Handled
        }
    })
    .ok_or("the rate-limit probe could not be added")?;
    Ok(())
}

fn appsink(pipeline: &gst::Pipeline, name: &str) -> Result<gst_app::AppSink, Box<dyn Error>> {
    pipeline
        .by_name(name)
        .and_then(|element| element.downcast::<gst_app::AppSink>().ok())
        .ok_or_else(|| format!("the pipeline has no appsink named {name}").into())
}

/// PLAYING, or back to NULL so the device is released before the process exits for respawn.
fn start(pipeline: &gst::Pipeline, logger: &Logger) -> Result<(), Box<dyn Error>> {
    if let Err(err) = pipeline.set_state(gst::State::Playing) {
        stop(pipeline, logger);
        return Err(format!(
            "[CameraFeederNode] pipeline {} failed to start: {err}",
            pipeline.name()
        )
        .into());
    }
    Ok(())
}

fn stop(pipeline: &gst::Pipeline, logger: &Logger) {
    if let Err(err) = pipeline.set_state(gst::State::Null) {
        log_error!(
            logger,
            "[CameraFeederNode] pipeline {} did not reach NULL: {err}",
            pipeline.name()
        );
    }
}

/// The IMU subscription: every sample goes into the shared buffer, and the warp waiting for one
/// is woken.
fn imu_subscription(
    node: &Node,
    config: &StabilizeConfig,
    imu: SharedImu,
) -> Result<Subscription<Imu>, RclrsError> {
    let use_header_stamp = config.imu_use_header_stamp;
    let accel_scale = config.accel_scale;
    let clock = node.get_clock();
    let logger = node.logger().clone();
    // SensorData QoS: compatible with a reliable or a best-effort publisher (the Livox driver's
    // is best-effort).
    node.create_subscription(
        config.imu_topic.as_str().sensor_data_qos(),
        move |msg: Imu| {
            let stamp_ns = if use_header_stamp {
                i64::from(msg.header.stamp.sec) * 1_000_000_000
                    + i64::from(msg.header.stamp.nanosec)
            } else {
                clock.now().nsec
            };
            let sample = ImuSample {
                stamp_ns,
                gyro_rad_s: [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                accel_m_s2: [
                    msg.linear_acceleration.x * accel_scale,
                    msg.linear_acceleration.y * accel_scale,
                    msg.linear_acceleration.z * accel_scale,
                ],
            };
            let (buffer, condvar) = &*imu;
            let accepted = buffer
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .push(sample);
            if accepted {
                condvar.notify_all();
            } else {
                log_warn!(
                    logger.throttle(Duration::from_secs(5)),
                    "[ImuSubscription] dropped a sample that is not newer than the previous one \
                     (or not finite); stamp {stamp_ns}"
                );
            }
        },
    )
}

/// Publishes the calibration once, latched, and returns the publisher that holds it; `None` (with
/// the reason logged) when there is no calibration that describes the published image.
///
/// Latched rather than beside every frame, because the calibration is fixed for the node's life.
/// The cost is on the subscriber side: only a TRANSIENT_LOCAL subscriber gets the sample after it
/// went out (a volatile one matches and receives nothing, so it looks like a missing calibration),
/// and nothing pairs an image with it by stamp — the header stamp is the node's start. A bag
/// replay delivers it only with a QoS override marking the topic transient local.
///
/// None of the failures stops the node: the camera still serves the WebRTC view and the image
/// topic without a calibration, and an exit would only respawn into the same file.
///
/// With `stabilize` on there is none by design: every frame is warped by a rotation that changes
/// per frame and magnified by `zoom`, so no fixed K and D describe the published image, and a
/// consumer projecting points with the lens's would misplace them.
fn camera_info_publisher(
    node: &Node,
    config: &parameters::Config,
) -> Result<Option<Publisher<CameraInfo>>, RclrsError> {
    let path = config.camera_info_file.as_str();
    if path.is_empty() {
        log_info!(
            node.logger(),
            "[CameraFeederNode] no camera_info_file; camera_info is not published"
        );
        return Ok(None);
    }
    if config.stabilize.is_some() {
        log_warn!(
            node.logger(),
            "[CameraFeederNode] stabilize is on, so the published frames are warped and {path} \
             does not describe them; camera_info is not published"
        );
        return Ok(None);
    }
    let parsed = std::fs::read_to_string(path)
        .map_err(|err| err.to_string())
        .and_then(|text| camera_info::parse(&text, config.pipeline.width, config.pipeline.height));
    let calibration = match parsed {
        Ok(calibration) => calibration,
        Err(err) => {
            log_warn!(
                node.logger(),
                "[CameraFeederNode] {path}: {err}; camera_info is not published"
            );
            return Ok(None);
        }
    };
    log_info!(
        node.logger(),
        "[CameraFeederNode] camera_info from {path}: {} fx {} fy {} cx {} cy {}",
        calibration.distortion_model,
        calibration.k[0],
        calibration.k[4],
        calibration.k[2],
        calibration.k[5],
    );

    // Reliable as well as transient local: DDS hands the stored sample to a late joiner only over
    // a reliable connection. Depth 1 because there is only ever the one sample.
    let publisher: Publisher<CameraInfo> =
        node.create_publisher("camera_info".keep_last(1).reliable().transient_local())?;
    let (sec, nanosec) = stamp::to_sec_nanosec(node.get_clock().now().nsec);
    let message = CameraInfo {
        header: Header {
            stamp: TimeMsg { sec, nanosec },
            frame_id: config.frame_id.clone(),
        },
        width: calibration.width,
        height: calibration.height,
        distortion_model: calibration.distortion_model,
        d: calibration.d,
        k: calibration.k,
        r: calibration.r,
        p: calibration.p,
        ..Default::default()
    };
    if let Err(err) = publisher.publish(message) {
        // Nothing would ever retry it, so a failure here means no calibration for this run.
        log_warn!(
            node.logger(),
            "[CameraFeederNode] camera_info publish failed: {err}; camera_info is not published"
        );
        return Ok(None);
    }
    Ok(Some(publisher))
}

/// Where the ROS appsink's callback takes a frame's capture stamp from.
enum StampSource {
    /// Computed here from the pipeline clock: the appsink is on the capture pipeline.
    PipelineClock,
    /// Looked up by PTS: the appsink is on the output pipeline, and the warp recorded the stamp
    /// it computed on the capture pipeline.
    Table(SharedStamps),
}

/// The appsink callbacks: each sample becomes one `CompressedImage`, published on the streaming
/// thread.
///
/// The JPEG bytes are copied once, into the message; nothing is decoded. A publish error is logged
/// (throttled) and the frame dropped — returning an error to GStreamer would stop the whole
/// pipeline, the RTP branch with it.
///
/// The pipeline clock and base time are read off the appsink itself, which the pipeline hands both
/// to when it goes PLAYING. The closure must not capture the pipeline: the pipeline owns the
/// appsink and the appsink owns this closure, so a captured pipeline would be a reference cycle
/// that is never freed.
fn frame_publisher(
    publisher: Publisher<CompressedImage>,
    clock: Clock,
    logger: Logger,
    frame_id: String,
    stamp_source: StampSource,
) -> gst_app::AppSinkCallbacks {
    gst_app::AppSinkCallbacks::builder()
        .new_sample(move |sink| {
            let sample = sink.pull_sample().map_err(|_| gst::FlowError::Eos)?;
            let Some(buffer) = sample.buffer() else {
                return Ok(gst::FlowSuccess::Ok);
            };
            let map = buffer.map_readable().map_err(|_| gst::FlowError::Error)?;

            // Read both clocks back to back, so the age measured on the pipeline clock and the
            // node time it is subtracted from describe the same instant.
            let ros_now = clock.now().nsec;
            let stamp_ns = match &stamp_source {
                StampSource::PipelineClock => {
                    let timing = FrameTiming {
                        clock_now: sink.clock().map(|c| c.time().nseconds()),
                        base_time: sink.base_time().map(gst::ClockTime::nseconds),
                        pts: buffer.pts().map(gst::ClockTime::nseconds),
                    };
                    stamp::capture_stamp_ns(ros_now, timing)
                }
                StampSource::Table(table) => buffer
                    .pts()
                    .and_then(|pts| {
                        table
                            .lock()
                            .unwrap_or_else(PoisonError::into_inner)
                            .take(pts.nseconds())
                    })
                    .unwrap_or_else(|| {
                        log_warn!(
                            logger.throttle(Duration::from_secs(5)),
                            "[CameraFeederNode] no capture stamp recorded for this frame; \
                             stamped on arrival"
                        );
                        ros_now
                    }),
            };
            let (sec, nanosec) = stamp::to_sec_nanosec(stamp_ns);

            let message = CompressedImage {
                header: Header {
                    stamp: TimeMsg { sec, nanosec },
                    frame_id: frame_id.clone(),
                },
                format: "jpeg".to_string(),
                data: map.as_slice().to_vec(),
            };

            match publisher.publish(message) {
                Ok(()) => log_info!(
                    logger.once(),
                    "[CameraFeederNode] first frame published ({} bytes)",
                    map.len(),
                ),
                Err(err) => log_warn!(
                    logger.throttle(Duration::from_secs(5)),
                    "[CameraFeederNode] publish failed, frame dropped: {err}",
                ),
            }
            Ok(gst::FlowSuccess::Ok)
        })
        .build()
}
