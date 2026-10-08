mod bus;
mod parameters;
mod pipeline;
mod stamp;

use std::error::Error;
use std::time::Duration;

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use rclrs::*;
// Aliased: `rclrs::*` also exports a `Time` (the clock reading), and the message type must not be
// mistaken for it.
use ros_env::builtin_interfaces::msg::Time as TimeMsg;
use ros_env::sensor_msgs::msg::CompressedImage;
use ros_env::std_msgs::msg::Header;

use bus::BusWatch;
use parameters::Parameters;

/// The sole opener of the camera: one capture pipeline whose MJPEG stream is teed to the WebRTC
/// worker as RTP/JPEG and to ROS as `sensor_msgs/CompressedImage` on the relative topic
/// `image_raw/compressed` (so `<robot_id>/image_raw/compressed`). See PROPOSAL.md.
///
/// # Threading
///
/// No rclrs worker or timer: the node has nothing to react to on the executor. Frames are
/// published straight from the appsink's GStreamer streaming thread (an rclrs `Publisher` is an
/// `Arc` and `publish` takes `&self`), and the bus is watched on a thread of its own. The executor
/// only spins so the node exists in the graph and serves its parameters.
pub struct CameraFeederNode {
    // Drop::drop takes the pipeline to NULL first; the fields then drop in declaration order, so
    // the bus watcher is joined before the last pipeline reference and the node go away.
    _bus_watch: BusWatch,
    pipeline: gst::Pipeline,
    _parameters: Parameters,
    _node: Node,
}

impl CameraFeederNode {
    pub fn new(node: Node) -> Result<Self, Box<dyn Error>> {
        let (parameters, config) = Parameters::declare(&node)?;

        // SensorData QoS, the same as vizionsdk_ros2's publisher of this topic, so existing
        // subscribers keep matching. A best-effort publisher cannot satisfy a RELIABLE
        // subscriber: subscribe with SensorData QoS too.
        let publisher: Publisher<CompressedImage> =
            node.create_publisher("image_raw/compressed".keep_last(5).best_effort().volatile())?;

        let description = pipeline::build_pipeline_string(&config.pipeline);
        log_info!(node.logger(), "[CameraFeederNode] pipeline: {description}");
        let pipeline = gst::parse::launch(&description)?
            .downcast::<gst::Pipeline>()
            .map_err(|_| "the pipeline description did not produce a gst::Pipeline")?;

        let appsink = pipeline
            .by_name(pipeline::ROS_SINK_NAME)
            .and_then(|element| element.downcast::<gst_app::AppSink>().ok())
            .ok_or("the pipeline has no appsink for the ROS branch")?;
        appsink.set_callbacks(frame_publisher(
            publisher,
            node.get_clock(),
            node.logger().clone(),
            pipeline.clone(),
            config.frame_id,
        ));

        // Before PLAYING, so an error during the state change is already being watched for.
        let bus_watch = BusWatch::start(pipeline.clone(), node.logger().clone())?;

        if let Err(err) = pipeline.set_state(gst::State::Playing) {
            // Back to NULL so the device is released before the process exits for respawn.
            let _ = pipeline.set_state(gst::State::Null);
            return Err(format!("[CameraFeederNode] pipeline failed to start: {err}").into());
        }

        log_info!(
            node.logger(),
            "[CameraFeederNode] streaming {}x{}@{} to {}:{} (RTP/JPEG) and image_raw/compressed @{}",
            config.pipeline.width,
            config.pipeline.height,
            config.pipeline.framerate,
            config.pipeline.rtp_host,
            config.pipeline.rtp_port,
            config.pipeline.ros_framerate,
        );

        Ok(Self {
            _bus_watch: bus_watch,
            pipeline,
            _parameters: parameters,
            _node: node,
        })
    }
}

impl Drop for CameraFeederNode {
    fn drop(&mut self) {
        // Runs only on an orderly shutdown — rclrs installs no SIGINT handler, so Ctrl-C and
        // SIGTERM end the process without it, and the kernel closes the device instead.
        let _ = self.pipeline.set_state(gst::State::Null);
    }
}

/// The appsink callbacks: each sample becomes one `CompressedImage`, published on the streaming
/// thread.
///
/// The JPEG bytes are copied once, into the message; nothing is decoded. A publish error is logged
/// (throttled) and the frame dropped — returning an error to GStreamer would stop the whole
/// pipeline, the RTP branch with it.
fn frame_publisher(
    publisher: Publisher<CompressedImage>,
    clock: Clock,
    logger: Logger,
    pipeline: gst::Pipeline,
    frame_id: String,
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
            let pipeline_now = pipeline.clock().map(|c| c.time().nseconds());
            let stamp_ns = stamp::capture_stamp_ns(
                ros_now,
                pipeline_now,
                pipeline.base_time().map(gst::ClockTime::nseconds),
                buffer.pts().map(gst::ClockTime::nseconds),
            );
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
