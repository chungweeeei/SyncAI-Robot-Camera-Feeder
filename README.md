# syncai_camera_feeder (Rust)

The ROS 2 package `syncai_camera_feeder`, written with [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)
(`rclrs` 0.8) and GStreamer, plus a Docker environment for developing it on its own.

It is the single owner of `/dev/syncai/camera0`, so the WebRTC worker and ROS can stream the same
camera at the same time:

```
/dev/syncai/camera0 ── v4l2src (MJPEG 1280x720@60) ── tee ─┬─ rtpjpegpay ─ udpsink 127.0.0.1:5008 ──► WebRTC worker
                                                           └─ appsink ─► <robot_id>/image_raw/compressed ──► ros2 bag record
```

The design, the alternatives that were rejected (v4l2loopback, shm, the worker as owner) and the
changes it needs in the other repos are in [PROPOSAL.md](PROPOSAL.md) (Chinese).

**Status:** implemented and verified in the Dev Container with a `videotestsrc` stand-in (topic at
60 Hz and at a reduced `ros_framerate`, RTP/JPEG decoded by a separate receiver, exit-and-respawn on
a pipeline error). Not yet run against the real camera on the robot.

## The node

| Direction | Name | Type / format | Notes |
|---|---|---|---|
| Publish | `image_raw/compressed` | `sensor_msgs/CompressedImage`, `format: jpeg` | SensorData QoS (best effort, keep last 5), as `vizionsdk_ros2` published it |
| Send | `udp://127.0.0.1:5008` | RTP/JPEG (RFC 2435), payload type 26 | for the WebRTC worker's `rtpjpeg` source |

- **Nothing is decoded.** The camera's JPEG bytes go to both branches untouched, each behind its
  own leaky queue so neither consumer can stall the capture or the other.
- **Stamps are capture time**, not arrival time: node clock now minus the frame's age on the
  pipeline clock (`src/camera_feeder_node/stamp.rs`).
- **ISP controls** (exposure, white balance, gain, flicker) are written once at capture start with
  the WebRTC worker's measured values; see `params/camera_feeder_params.yaml`.
- **On any pipeline error the node exits** and the launch file respawns it.

Parameters (all read-only) are documented in
[`params/camera_feeder_params.yaml`](params/camera_feeder_params.yaml).

**This repo is itself a single colcon package** (`package.xml` / `Cargo.toml` live at the root),
laid out like SyncAI-Robot-State and SyncAI-Robot-Driver-Manager, and is meant to be pulled into
SyncAI-Robot-Workspace with vcstool as `src/syncai_camera_feeder`.

## Development environment

Open the repo in the Dev Container (`.devcontainer/`): ROS 2 Humble, Rust 1.85, the ros2_rust
underlay (built from source on the first image build, which is slow), GStreamer 1.20 with its
development headers, and `v4l-utils`. Build and test commands are in [CLAUDE.md](CLAUDE.md).

```bash
cd /workspace && colcon build --symlink-install
ros2 launch syncai_camera_feeder camera_feeder.launch.py
```

There is no camera in the Dev Container; see CLAUDE.md for the `videotestsrc` stand-in and for
passing the real device through on the robot.
