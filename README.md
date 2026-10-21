# syncai_camera_feeder (Rust)

The ROS 2 package `syncai_camera_feeder`, written with [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)
(`rclrs` 0.8) and GStreamer, plus a Docker environment for developing it on its own.

It is the single owner of `/dev/syncai/camera0`, so the WebRTC worker and ROS can stream the same
camera at the same time:

```
/dev/syncai/camera0 ── v4l2src (MJPEG 1280x720@60) ── tee ─┬─ rtpjpegpay ─ udpsink 127.0.0.1:5008 ──► WebRTC worker
                                                           └─ appsink ─► <robot_id>/image_raw/compressed ──► ros2 bag record
```

With `stabilize: true` the WebRTC branch is fed by a gyro-stabilized image instead
(electronic image stabilization from the lidar's IMU, Karpenko-style: rotation only, CPU warp),
while `image_raw/compressed` stays the camera's own JPEG, teed off before the decoder, so that
`camera_info` and the camera's TF still describe it. The warped frames also go to ROS, on
`image_stabilized/compressed`. On a Jetson the JPEG decode and re-encode run on the hardware JPEG
blocks (`jpeg_codec: auto`); elsewhere they fall back to `jpegdec` / `jpegenc`:

```
v4l2src (MJPEG @60) ── rate limit @30 ── tee ─┬─ appsink ─► image_raw/compressed (+ camera_info)
                                              └─ nvv4l2decoder ─ nvvidconv (I420) ── appsink ─┐
                                                        <robot_id>/livox/imu ─┤ warp by the orientation at capture time (Rust)
                       appsrc ── nvjpegenc ── tee ─┬─ rtpjpegpay ─ udpsink ◄──────────────────┘
                                                   └─ appsink ─► image_stabilized/compressed
```

The design, the alternatives that were rejected (v4l2loopback, shm, the worker as owner) and the
changes it needs in the other repos are in [PROPOSAL.md](PROPOSAL.md) (Chinese).

**Status:** implemented and verified in the Dev Container with a `videotestsrc` stand-in (topic at
30 Hz by default and 60 Hz with `ros_framerate:=0`, RTP/JPEG decoded by a separate receiver, exit-and-respawn on
a pipeline error). Not yet run against the real camera on the robot.

## The node

| Direction | Name | Type / format | Notes |
|---|---|---|---|
| Publish | `image_raw/compressed` | `sensor_msgs/CompressedImage`, `format: jpeg` | 30 Hz (`ros_framerate`); SensorData QoS (best effort, keep last 5), as `vizionsdk_ros2` published it. The camera's own JPEG in both modes |
| Publish | `image_stabilized/compressed` | `sensor_msgs/CompressedImage`, `format: jpeg` | Only with `stabilize` on: the warped, zoomed frames the WebRTC worker gets, at `ros_framerate`, same QoS. No `camera_info` describes them |
| Publish | `camera_info` | `sensor_msgs/CameraInfo` | Once at start, latched (reliable, transient local, depth 1); only with a valid `camera_info_file`. Describes `image_raw/compressed` |
| Send | `udp://127.0.0.1:5008` | RTP/JPEG (RFC 2435), payload type 26 | 60 Hz (`framerate`), for the WebRTC worker's `rtpjpeg` source; stabilized at `ros_framerate` with `stabilize` on |
| Send | each of `rtp_extra_clients` | the same RTP/JPEG packets | Empty by default. For another GStreamer receiver beside the worker: the branch becomes `multiudpsink`, payloaded once; see below |

- **Nothing is decoded** (by default). The camera's JPEG bytes go to both branches untouched, each
  behind its own leaky queue so neither consumer can stall the capture or the other.
- **Stabilization** (`stabilize: true`, off by default) decodes, warps and re-encodes every frame
  at `ros_framerate` for the WebRTC view, which then runs at 30 fps, and for
  `image_stabilized/compressed`. `image_raw/compressed` is still the camera's own JPEG: a warped
  frame is a virtual camera rotated away from the physical one by a different few degrees every
  frame and magnified by `zoom`, so a consumer that projects lidar points into the image with
  `camera_info` and TF (the semantic mapping) needs the unwarped one. Until 2026-10-20 both
  branches carried the warped image, so bags recorded with `stabilize` on before then have no
  usable frames for that. The warp is on the CPU; the JPEG codec
  is the Jetson's hardware when the L4T GStreamer plugins are present (`jpeg_codec`; in robot01
  at 30 fps the node went from ~2.5 to ~1.9 cores and stabilized frames reach ROS ~35 ms after
  capture instead of ~53) and the CPU otherwise. It needs the
  camera intrinsics and the IMU-to-camera rotation calibrated; the parameters file explains each
  knob. Without IMU samples the frames pass through (zoomed, unwarped) and a throttled warning
  says so. The virtual camera follows the real one through two cascaded first-order low-pass
  stages (`smoothing_order`, `smoothing_time_constant_s`), which leave ~40 % less of the walking
  shake than one stage did wherever turns have not used up the zoom's margin. While the robot
  turns, the stages are shortened (`turn_damping_*`), so the view keeps up with the turn instead
  of sliding on for a second after the robot has stopped; and the warp is fitted so that no
  output pixel falls outside the frame (no black corners). The orientation
  comes from a Mahony filter started level with gravity whose accelerometer gain (`mahony_kp`) is
  kept small, so the gait's surge is not read as tilt.
  `tools/eis_fit.py` fits `imu_time_offset_ms` and compares these settings offline, from a rosbag
  (see CLAUDE.md).
- **Stamps are capture time**, not arrival time: node clock now minus the frame's age on the
  pipeline clock (`src/camera_feeder_node/stamp.rs`); for `image_stabilized/compressed`, the
  stamp computed on the capture pipeline travels to the output pipeline's appsink keyed by PTS.
  The rate limit
  down to `ros_framerate` drops frames by PTS and restamps none
  (`src/camera_feeder_node/rate_limit.rs`); `videorate`, which did it until 2026-10-16, put
  every stamp up to 16.7 ms early, so stamps in bags recorded before then are biased.
- **`camera_info`** comes from the YAML `camera_calibration` writes (`ost.yaml`), read once at
  start. The launch file looks for it at `~/robot_ws/config/calibration/<robot_id>/camera0.yaml`
  (`camera_info_file:=` overrides it), since intrinsics belong to one physical camera. A missing
  or malformed file, or one calibrated at another resolution, publishes no `camera_info` and
  warns. The images go on in every case. `stabilize` does not change it, since it describes
  `image_raw/compressed`, which is unwarped in both modes. It is **published once and latched**,
  not beside every frame: subscribe with
  `TRANSIENT_LOCAL` durability (a volatile subscriber that joins later receives nothing), and do
  not pair it with images by stamp — its stamp is the node's start. `ros2 bag record` picks the
  durability up from the publisher; `ros2 bag play` on Humble needs a QoS override marking the
  topic transient local for late joiners to get it.
- **ISP controls** (exposure, white balance, gain, flicker) are written once at capture start with
  the WebRTC worker's measured values; see `params/camera_feeder_params.yaml`.
- **On any pipeline error the node exits** and the launch file respawns it.
- **Another GStreamer receiver** gets the RTP stream through `rtp_extra_clients`
  (`"127.0.0.1:5010"`, comma-separated for more), never by opening the camera or by binding the
  worker's 5008 (unicast: a second socket there takes packets from the worker instead of copying
  them). Raw RTP carries no SDP, so the receiver states the caps:
  ```bash
  gst-launch-1.0 udpsrc port=5010 \
      caps="application/x-rtp,media=video,encoding-name=JPEG,payload=26,clock-rate=90000" \
    ! rtpjitterbuffer latency=50 ! rtpjpegdepay ! jpegdec ! videoconvert ! autovideosink
  ```
  In another container it needs host networking to be reachable at `127.0.0.1`. There is no RTSP
  server here; one (MediaMTX, `gst-rtsp-server`) would be such a receiver.

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
