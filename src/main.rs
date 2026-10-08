mod camera_feeder_node;

use rclrs::*;

use camera_feeder_node::CameraFeederNode;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // GStreamer before ROS: a missing or broken GStreamer install is the more likely failure on a
    // fresh image, and it should stop the process before the node shows up in the graph.
    gstreamer::init()?;

    // rclrs inverts the rclcpp order: the executor comes first, and the node is created from it.
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("syncai_camera_feeder")?;
    log_info!(
        node.logger(),
        "[CameraFeederNode] GStreamer runtime {}",
        gstreamer::version_string()
    );
    let _camera_feeder = CameraFeederNode::new(node)?;

    // No SIGINT handler: rclrs does not install one, so Ctrl-C ends the process without running
    // Drop. In scripts, stop a backgrounded node with SIGTERM.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
