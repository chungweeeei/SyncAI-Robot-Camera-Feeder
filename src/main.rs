use rclrs::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // GStreamer before ROS: a missing or broken GStreamer install is the more likely failure on a
    // fresh image, and it should stop the process before the node shows up in the graph.
    gstreamer::init()?;

    // rclrs inverts the rclcpp order: the executor comes first, and the node is created from it.
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("syncai_camera_feeder")?;

    // Environment check only, until the pipeline lands: proves rclrs and GStreamer link and run in
    // the same process. The runtime version is what the robot container actually loaded, which is
    // the one that matters for the v1_20 feature pin in Cargo.toml.
    log_info!(
        node.logger(),
        "[CameraFeederNode] started; GStreamer runtime {}",
        gstreamer::version_string()
    );

    // No SIGINT handler: rclrs does not install one, so Ctrl-C ends the process without running
    // Drop. In scripts, stop a backgrounded node with SIGTERM.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
