# syncai_camera_feeder (Rust)

The ROS 2 package `syncai_camera_feeder`, written with [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)
(`rclrs` 0.8) and GStreamer, plus a Docker environment for developing it on its own.

It is the single owner of `/dev/syncai/camera0`, so the WebRTC worker and ROS can stream the same
camera at the same time:

```
/dev/syncai/camera0 ── v4l2src (MJPEG 1280x720@60) ── tee ─┬─ rtpjpegpay ─ udpsink 127.0.0.1:5008 ──► WebRTC worker
                                                           └─ appsink ─► <robot_id>/image_raw/compressed ──► ros2 bag record
```

With `stabilize: true` the same two branches are fed by a gyro-stabilized image instead
(electronic image stabilization from the lidar's IMU, Karpenko-style: rotation only, CPU warp).
On a Jetson the JPEG decode and re-encode run on the hardware JPEG blocks (`jpeg_codec: auto`);
elsewhere they fall back to `jpegdec` / `jpegenc`:

```
v4l2src (MJPEG @60) ── rate limit @30 ── nvv4l2decoder ─ nvvidconv (I420) ── appsink ─┐
                                                 <robot_id>/livox/imu ─┤ warp by the orientation at capture time (Rust)
                       appsrc ── nvjpegenc ── tee ─┬─ rtpjpegpay ─ udpsink ◄───────────┘
                                                   └─ appsink ─► image_raw/compressed
```

The design, the alternatives that were rejected (v4l2loopback, shm, the worker as owner) and the
changes it needs in the other repos are in [PROPOSAL.md](PROPOSAL.md) (Chinese).

**Status:** implemented and verified in the Dev Container with a `videotestsrc` stand-in (topic at
30 Hz by default and 60 Hz with `ros_framerate:=0`, RTP/JPEG decoded by a separate receiver, exit-and-respawn on
a pipeline error). Not yet run against the real camera on the robot.

## The node

| Direction | Name | Type / format | Notes |
|---|---|---|---|
| Publish | `image_raw/compressed` | `sensor_msgs/CompressedImage`, `format: jpeg` | 30 Hz (`ros_framerate`); SensorData QoS (best effort, keep last 5), as `vizionsdk_ros2` published it |
| Publish | `camera_info` | `sensor_msgs/CameraInfo` | Once at start, latched (reliable, transient local, depth 1); only with a valid `camera_info_file` and `stabilize` off |
| Send | `udp://127.0.0.1:5008` | RTP/JPEG (RFC 2435), payload type 26 | 60 Hz (`framerate`), for the WebRTC worker's `rtpjpeg` source |

- **Nothing is decoded** (by default). The camera's JPEG bytes go to both branches untouched, each
  behind its own leaky queue so neither consumer can stall the capture or the other.
- **Stabilization** (`stabilize: true`, off by default) decodes, warps and re-encodes every frame
  at `ros_framerate`, so both branches then run at 30 fps. The warp is on the CPU; the JPEG codec
  is the Jetson's hardware when the L4T GStreamer plugins are present (`jpeg_codec`; in robot01
  at 30 fps the node went from ~2.5 to ~1.9 cores and frames reach ROS ~35 ms after capture
  instead of ~53) and the CPU otherwise. It needs the
  camera intrinsics and the IMU-to-camera rotation calibrated; the parameters file explains each
  knob. Without IMU samples the frames pass through (zoomed, unwarped) and a throttled warning
  says so.
- **Stamps are capture time**, not arrival time: node clock now minus the frame's age on the
  pipeline clock (`src/camera_feeder_node/stamp.rs`); with stabilization on, the stamp computed
  on the capture pipeline travels to the output pipeline's appsink keyed by PTS. The rate limit
  down to `ros_framerate` drops frames by PTS and restamps none
  (`src/camera_feeder_node/rate_limit.rs`); `videorate`, which did it until 2026-10-16, put
  every stamp up to 16.7 ms early, so stamps in bags recorded before then are biased.
- **`camera_info`** comes from the YAML `camera_calibration` writes (`ost.yaml`), read once at
  start. The launch file looks for it at `~/robot_ws/config/calibration/<robot_id>/camera0.yaml`
  (`camera_info_file:=` overrides it), since intrinsics belong to one physical camera. A missing
  or malformed file, or one calibrated at another resolution, publishes no `camera_info` and
  warns; so does `stabilize: true`, whose warped frames no fixed K describes. The images go on
  in every case. It is **published once and latched**, not beside every frame: subscribe with
  `TRANSIENT_LOCAL` durability (a volatile subscriber that joins later receives nothing), and do
  not pair it with images by stamp — its stamp is the node's start. `ros2 bag record` picks the
  durability up from the publisher; `ros2 bag play` on Humble needs a QoS override marking the
  topic transient local for late joiners to get it.
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
