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

**Status: environment scaffold.** The node starts, initialises GStreamer and logs its version;
the pipeline is not implemented yet.

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
