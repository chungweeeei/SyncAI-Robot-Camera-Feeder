#!/usr/bin/env python3
"""Offline fitting and evaluation of the camera feeder's gyro stabilization, from a rosbag.

Run it inside the robot container, which has rosbag2_py, OpenCV and NumPy (the host has no bag
reader), on a bag under the workspace so the container can see it:

    python3 ~/robot_ws/src/syncai_camera_feeder/tools/eis_fit.py filters  ~/robot_ws/record/<bag>
    python3 ~/robot_ws/src/syncai_camera_feeder/tools/eis_fit.py offset   ~/robot_ws/record/<bag>
    python3 ~/robot_ws/src/syncai_camera_feeder/tools/eis_fit.py residual ~/robot_ws/record/<bag>

filters   IMU only. Re-runs the orientation filter and the virtual-camera low-pass over the bag's
          IMU at the frame rate and prints how much gait-band shake (1-6 Hz by default) each
          smoothing setting leaves in the picture, and how much each Mahony gain injects. Any bag
          with the lidar IMU will do, including ones whose image stamps are wrong.
offset    Fits imu_time_offset_ms: the frame-to-frame image shift measured by phase correlation
          against the shift the gyro predicts, swept over the offset. Needs raw frames stamped
          with the capture's own time, i.e. a bag recorded after 2026-10-16 17:50 (ece6437):
          earlier frames went through videorate, which stamped them 0-16.7 ms early by an amount
          fixed per recording, and a fit on them measures that. image_raw/compressed is raw in
          either mode since 2026-10-20; before that, only with stabilize off.
residual  The gait-band image shake of a recording, from its frames alone (and, when the bag has
          the IMU, what the unstabilized camera did over the same time): the before/after number
          of an on-robot A/B. Pass --stabilized to measure image_stabilized/compressed, the
          warped frames, so the zoomed focal length converts pixels to degrees. A bag recorded
          with stabilize on since 2026-10-20 carries both topics: run it once with and once
          without --stabilized.

The orientation filter and the low-pass below mirror src/camera_feeder_node/imu.rs (Mahony,
ImuBuffer) and eis.rs (Stabilizer) line for line; keep them in step when either changes. Every
value comes from the feeder's params file (default: the one in this package), interpreted the way
parameters.rs does, so a fit runs on the values the node would.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np
import yaml

DEFAULT_PARAMS = Path(__file__).resolve().parent.parent / "params" / "camera_feeder_params.yaml"

# imu.rs RESET_GAP_NS: a silence longer than this restarts the filter.
RESET_GAP_NS = 1_000_000_000
IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
# Phase correlation runs on frames decoded at a quarter of the resolution (fast, and sub-pixel
# precise enough), so its shifts are multiplied back up by this.
DECODE_SCALE = 4

Quat = tuple[float, float, float, float]  # w, x, y, z; world_from_body, as in math.rs
IDENTITY: Quat = (1.0, 0.0, 0.0, 0.0)


# --- Configuration (parameters.rs) -------------------------------------------------------------


@dataclass
class Config:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    zoom: float
    cam_from_imu: np.ndarray  # 3x3, IMU-frame vectors into the optical frame
    smoothing_order: int
    smoothing_tau_s: float
    turn_rate_tau_s: float
    turn_deadband_rad_s: float
    turn_reference_rad_s: float  # 0: no turn damping
    mahony_kp: float
    accel_scale: float
    frame_rate: float  # what the stabilized path runs at: ros_framerate
    frame_time_offset_ns: int  # imu_time_offset_ms plus half a manual exposure
    imu_time_offset_ms: float
    half_exposure_ms: float
    use_header_stamp: bool

    @property
    def hidden_angles(self) -> tuple[float, float]:
        """eis.rs hidden_angles: the (pan, tilt) the zoom's border can hide, in radians, measured
        at the frame's edges."""

        def hidden(near: float, far: float, f: float) -> float:
            def one_side(half: float) -> float:
                return math.atan(half / f) - math.atan(half / (f * max(self.zoom, 1.0)))

            return min(one_side(near), one_side(far))

        return (
            hidden(self.cx, self.width - self.cx, self.fx),
            hidden(self.cy, self.height - self.cy, self.fy),
        )

    def covers(self, real_to_virtual: np.ndarray) -> bool:
        """Stabilizer::covers: whether every output pixel samples inside the frame, given the
        rotation (IMU frame, virtual_from_real) between the real and the virtual camera."""
        k = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])
        k_out_inv = np.array(
            [
                [1 / (self.fx * self.zoom), 0, -self.cx / (self.fx * self.zoom)],
                [0, 1 / (self.fy * self.zoom), -self.cy / (self.fy * self.zoom)],
                [0, 0, 1.0],
            ]
        )
        r = self.cam_from_imu @ real_to_virtual @ self.cam_from_imu.T
        inverse = k @ r.T @ k_out_inv
        right, bottom = self.width - 0.5, self.height - 0.5
        for u, v in ((0.5, 0.5), (right, 0.5), (0.5, bottom), (right, bottom)):
            x, y, w = inverse @ np.array([u, v, 1.0])
            if w <= 0.0 or not (0.5 - 1e-9 <= x / w <= right + 1e-9):
                return False
            if not (0.5 - 1e-9 <= y / w <= bottom + 1e-9):
                return False
        return True


def rpy_deg_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """math.rs Mat3::from_rpy_deg: Rz(yaw) * Ry(pitch) * Rx(roll)."""
    sr, cr = math.sin(math.radians(roll)), math.cos(math.radians(roll))
    sp, cp = math.sin(math.radians(pitch)), math.cos(math.radians(pitch))
    sy, cy = math.sin(math.radians(yaw)), math.cos(math.radians(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def load_config(path: Path) -> Config:
    params = yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
    width = int(params.get("width", 1280))
    height = int(params.get("height", 720))
    # 0 means the resolution-derived guess, exactly as parameters.rs::intrinsics.
    fx = float(params.get("fx", 0.0)) or width / 2.0
    fy = float(params.get("fy", 0.0)) or fx
    cx = float(params.get("cx", 0.0)) or width / 2.0
    cy = float(params.get("cy", 0.0)) or height / 2.0
    framerate = int(params.get("framerate", 60))
    ros_framerate = int(params.get("ros_framerate", 30))
    if ros_framerate <= 0 or ros_framerate > framerate:
        ros_framerate = framerate
    offset_ms = float(params.get("imu_time_offset_ms", 0.0))
    # auto_exposure 1 is manual (UVC menu); only then is half the exposure added.
    manual_exposure = int(params.get("auto_exposure", 0)) == 1
    half_exposure_ms = int(params.get("exposure_time", 0)) * 0.05 if manual_exposure else 0.0
    return Config(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        zoom=float(params.get("zoom", 1.15)),
        cam_from_imu=rpy_deg_matrix(*params.get("imu_to_camera_rpy_deg", [-90.0, 0.0, -90.0])).T,
        smoothing_order=int(params.get("smoothing_order", 2)),
        smoothing_tau_s=float(params.get("smoothing_time_constant_s", 0.35)),
        turn_rate_tau_s=float(params.get("turn_rate_time_constant_s", 0.3)),
        turn_deadband_rad_s=float(params.get("turn_damping_deadband_rad_s", 0.1)),
        turn_reference_rad_s=float(params.get("turn_damping_reference_rad_s", 0.15)),
        mahony_kp=float(params.get("mahony_kp", 0.05)),
        accel_scale=float(params.get("imu_accel_scale", 9.80665)),
        frame_rate=float(ros_framerate),
        frame_time_offset_ns=int((offset_ms + half_exposure_ms) * 1e6),
        imu_time_offset_ms=offset_ms,
        half_exposure_ms=half_exposure_ms,
        use_header_stamp=bool(params.get("imu_use_header_stamp", True)),
    )


# --- Quaternions (math.rs) ---------------------------------------------------------------------


def q_mul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_conj(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def q_normalized(q: Quat) -> Quat:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n == 0.0 or not math.isfinite(n):
        return IDENTITY
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_from_axis_angle(axis: tuple[float, float, float], angle: float) -> Quat:
    n = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    if n == 0.0:
        return IDENTITY
    s = math.sin(angle / 2.0) / n
    return (math.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def q_angle(q: Quat) -> float:
    return 2.0 * math.atan2(math.sqrt(q[1] ** 2 + q[2] ** 2 + q[3] ** 2), abs(q[0]))


def q_slerp(a: Quat, b: Quat, t: float) -> Quat:
    dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
    if dot < 0.0:
        b = (-b[0], -b[1], -b[2], -b[3])
        dot = -dot
    if dot > 0.9995:
        wa, wb = 1.0 - t, t
    else:
        theta = math.acos(min(max(dot, -1.0), 1.0))
        sin_theta = math.sin(theta)
        wa, wb = math.sin((1.0 - t) * theta) / sin_theta, math.sin(t * theta) / sin_theta
    return q_normalized(
        (wa * a[0] + wb * b[0], wa * a[1] + wb * b[1], wa * a[2] + wb * b[2], wa * a[3] + wb * b[3])
    )


def quats_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=1,
    )


def quats_conj(q: np.ndarray) -> np.ndarray:
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def quats_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q.T
    return np.stack(
        [
            np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], axis=1),
            np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], axis=1),
            np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], axis=1),
        ],
        axis=1,
    )


def quats_rotation_vector(q: np.ndarray) -> np.ndarray:
    """Axis times angle of each rotation, the shorter way round."""
    q = np.where(q[:, :1] < 0.0, -q, q)
    v = q[:, 1:]
    sin_half = np.linalg.norm(v, axis=1)
    angle = 2.0 * np.arctan2(sin_half, q[:, 0])
    scale = np.where(sin_half > 1e-12, angle / np.maximum(sin_half, 1e-300), 2.0)
    return v * scale[:, None]


# --- Orientation filter (imu.rs) ---------------------------------------------------------------


@dataclass
class ImuData:
    stamps_ns: np.ndarray  # int64
    gyro: np.ndarray  # N x 3, rad/s
    accel: np.ndarray  # N x 3, m/s^2 (already scaled)


def align_to_gravity(accel: tuple[float, float, float]) -> Quat | None:
    n = math.sqrt(accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2)
    if n == 0.0 or not math.isfinite(n):
        return None
    up = (accel[0] / n, accel[1] / n, accel[2] / n)
    axis = (up[1], -up[0], 0.0)
    sin = math.hypot(axis[0], axis[1])
    if sin > 1e-12:
        return q_from_axis_angle(axis, math.atan2(sin, up[2]))
    return IDENTITY if up[2] > 0.0 else q_from_axis_angle((1.0, 0.0, 0.0), math.pi)


def integrate(imu: ImuData, kp: float) -> tuple[np.ndarray, np.ndarray]:
    """ImuBuffer::push over every sample: the orientation history (stamps, N x 4 quaternions)."""

    def restart(accel: tuple[float, float, float]) -> Quat:
        aligned = align_to_gravity(accel) if kp > 0.0 else None
        return aligned or IDENTITY

    stamps: list[int] = []
    quats: list[Quat] = []
    q = IDENTITY
    previous: int | None = None
    for t, (gx, gy, gz), (ax, ay, az) in zip(
        imu.stamps_ns.tolist(), imu.gyro.tolist(), imu.accel.tolist()
    ):
        if previous is None or t - previous > RESET_GAP_NS:
            q = restart((ax, ay, az))
        elif t <= previous:
            continue
        else:
            dt = (t - previous) / 1e9
            n = math.sqrt(ax * ax + ay * ay + az * az)
            if n > 0.0 and math.isfinite(n):
                ax, ay, az = ax / n, ay / n, az / n
                w, x, y, z = q
                # The estimated "up" in the body frame: world +z brought into the body.
                vx, vy, vz = 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)
                gx += kp * (ay * vz - az * vy)
                gy += kp * (az * vx - ax * vz)
                gz += kp * (ax * vy - ay * vx)
            step = q_from_axis_angle((gx, gy, gz), math.sqrt(gx * gx + gy * gy + gz * gz) * dt)
            q = q_normalized(q_mul(q, step))
        stamps.append(t)
        quats.append(q)
        previous = t
    return np.asarray(stamps, dtype=np.int64), np.asarray(quats)


def orientation_at(stamps: np.ndarray, quats: np.ndarray, t_ns: np.ndarray) -> np.ndarray:
    """ImuBuffer::orientation_at, vectorized: interpolated between the samples around each time.

    5 ms apart the samples are within the slerp's nlerp branch, so nlerp is what both use. Times
    outside the history are clamped to its ends (the node would extrapolate a few ms instead).
    """
    after = np.clip(np.searchsorted(stamps, t_ns, side="left"), 1, len(stamps) - 1)
    t0, t1 = stamps[after - 1], stamps[after]
    fraction = np.clip((t_ns - t0) / np.maximum(t1 - t0, 1), 0.0, 1.0)[:, None]
    qa, qb = quats[after - 1], quats[after]
    qb = np.where(np.sum(qa * qb, axis=1, keepdims=True) < 0.0, -qb, qb)
    q = qa * (1.0 - fraction) + qb * fraction
    return q / np.linalg.norm(q, axis=1, keepdims=True)


# --- Virtual camera (eis.rs) -------------------------------------------------------------------


def smooth(previous: Quat, current: Quat, dt_s: float, tau_s: float) -> Quat:
    if tau_s <= 0.0:
        return current
    alpha = 1.0 - math.exp(-dt_s / tau_s)
    return q_slerp(previous, current, min(max(alpha, 0.0), 1.0))


def clamp_toward(q_s: Quat, q: Quat, max_angle: float) -> Quat:
    angle = q_angle(q_mul(q_conj(q_s), q))
    if angle <= max_angle or angle == 0.0:
        return q_s
    return q_slerp(q_s, q, (angle - max_angle) / angle)


def wrap_angle(a: float) -> float:
    return math.remainder(a, 2.0 * math.pi)


def heading(q: Quat) -> float:
    """eis.rs heading: the rotation about world z of a world_from_body orientation."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def low_pass(previous: float, current: float, dt_s: float, tau_s: float) -> float:
    if tau_s <= 0.0:
        return current
    return previous + (current - previous) * (1.0 - math.exp(-dt_s / tau_s))


def effective_tau(config: Config, tau_s: float, turn_rate: float, reference: float) -> float:
    """TurnDamping::effective_tau."""
    excess = max(abs(turn_rate) - config.turn_deadband_rad_s, 0.0) / reference
    return tau_s / (1.0 + excess * excess)


# eis.rs FIT_ITERATIONS.
FIT_ITERATIONS = 20


def fit_to_frame(config: Config, q_s: Quat, q: Quat) -> tuple[Quat, bool]:
    """Stabilizer::fit_to_frame: q_s, or the nearest orientation toward q that leaves no border;
    and whether it had to move."""

    def covers(candidate: Quat) -> bool:
        return config.covers(quats_matrix(np.asarray([q_mul(q_conj(candidate), q)]))[0])

    if covers(q_s):
        return q_s, False
    low, high = 0.0, 1.0
    for _ in range(FIT_ITERATIONS):
        mid = 0.5 * (low + high)
        if covers(q_slerp(q_s, q, mid)):
            high = mid
        else:
            low = mid
    return q_slerp(q_s, q, high), True


def virtual_camera(
    frame_ns: np.ndarray,
    frame_q: np.ndarray,
    config: Config,
    order: int,
    tau_s: float,
    turn_reference: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Stabilizer::correction over every frame: the virtual camera's orientations, and whether
    the clamp or the fit held it at each frame (the zoom's margin was used up, so the real
    camera's motion went straight through). turn_reference 0 is no turn damping. frame_q must
    be levelled with gravity, as the node's filter is, for the heading to be the robot's turn."""
    max_angle = min(config.hidden_angles)
    out: list[Quat] = []
    clamped: list[bool] = []
    first = second = IDENTITY
    turn_rate = [0.0, 0.0]
    previous: int | None = None
    previous_heading = 0.0
    for t, (w, x, y, z) in zip(frame_ns.tolist(), frame_q.tolist()):
        q = (w, x, y, z)
        heading_now = heading(q)
        if previous is None:
            first = second = q
            turn_rate = [0.0, 0.0]
        else:
            dt = max(t - previous, 0) / 1e9
            tau = tau_s
            if turn_reference > 0.0:
                if dt > 0.0:
                    rate = wrap_angle(heading_now - previous_heading) / dt
                    turn_rate[0] = low_pass(turn_rate[0], rate, dt, config.turn_rate_tau_s)
                    turn_rate[1] = low_pass(turn_rate[1], turn_rate[0], dt, config.turn_rate_tau_s)
                tau = effective_tau(config, tau_s, turn_rate[1], turn_reference)
            first = clamp_toward(smooth(first, q, dt, tau), q, max_angle)
            if order == 1:
                second = first
            else:
                second = clamp_toward(smooth(second, first, dt, tau), q, max_angle)
        q_s, fitted = fit_to_frame(config, second, q)
        clamped.append(
            previous is not None
            and (fitted or q_angle(q_mul(q_conj(second), q)) >= max_angle * (1.0 - 1e-6))
        )
        out.append(q_s)
        previous, previous_heading = t, heading_now
    return np.asarray(out), np.asarray(clamped)


def turn_slide_deg(config: Config, order: int, tau_s: float, turn_reference: float) -> float:
    """How far the view still turns after the robot stops, on a synthetic 0.5 rad/s, 3 s turn
    (ramped over 0.3 s at both ends) of a levelled camera at the frame rate: the slide a viewer
    sees as the image flowing on after the turn."""
    fs = config.frame_rate
    frames = np.arange(int(6 * fs), dtype=np.int64) * int(1e9 / fs)
    level = q_from_axis_angle((0.0, 1.0, 0.0), 0.25)  # the lidar's mount pitch
    quats, angle = [], 0.0
    for i in range(len(frames)):
        t = i / fs
        angle += 0.5 * min(max(min(t / 0.3, (3.0 - t) / 0.3), 0.0), 1.0) / fs
        quats.append(q_mul(q_from_axis_angle((0.0, 0.0, 1.0), angle), level))
    virtual, _ = virtual_camera(frames, np.asarray(quats), config, order, tau_s, turn_reference)
    headings = np.unwrap([heading(tuple(q)) for q in virtual.tolist()])
    stop = int(3 * fs)
    return float(np.degrees(np.abs(headings[stop:] - headings[stop]).max()))


# --- Signal helpers ----------------------------------------------------------------------------


def band_integral(rate: np.ndarray, fs: float, band: tuple[float, float]) -> np.ndarray:
    """The band-limited integral of a uniformly sampled rate (unit/s -> unit).

    Integrating in the frequency domain, inside the band only, keeps turns (large and slow) and
    sensor drift out of a measure meant for the gait's shake.
    """
    spectrum = np.fft.rfft(rate - rate.mean())
    freqs = np.fft.rfftfreq(len(rate), 1.0 / fs)
    inside = (freqs >= band[0]) & (freqs <= band[1])
    integral = np.zeros_like(spectrum)
    integral[inside] = spectrum[inside] / (2j * np.pi * freqs[inside])
    return np.fft.irfft(integral, len(rate))


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values**2))) if len(values) else float("nan")


def band_integral_rms(rate: np.ndarray, fs: float, band: tuple[float, float]) -> float:
    return rms(band_integral(rate, fs, band)) if len(rate) >= 4 else float("nan")


def camera_rates(q: np.ndarray, cam_from_imu: np.ndarray, fs: float) -> np.ndarray:
    """Per-frame angular rate (rad/s) of a sequence of orientations, about the optical axes:
    x (the image moves vertically), y (horizontally), z (it rolls)."""
    delta = quats_mul(quats_conj(q[:-1]), q[1:])
    return quats_rotation_vector(delta) @ cam_from_imu.T * fs


def shake_px(
    rates: np.ndarray, fs: float, band: tuple[float, float], fx: float, fy: float
) -> tuple[float, float, float]:
    """(vertical px, horizontal px, roll deg) RMS in the band."""
    vertical = band_integral_rms(rates[:, 0], fs, band) * fy
    horizontal = band_integral_rms(rates[:, 1], fs, band) * fx
    roll = math.degrees(band_integral_rms(rates[:, 2], fs, band))
    return vertical, horizontal, roll


def dilate(mask: np.ndarray, frames: int) -> np.ndarray:
    """`mask`, widened by `frames` on each side."""
    if frames <= 0 or not mask.any():
        return mask
    return np.convolve(mask.astype(float), np.ones(2 * frames + 1), mode="same") > 0.0


def peak_frequency(rate: np.ndarray, fs: float, band: tuple[float, float]) -> float:
    """Welch-averaged spectral peak of a signal inside a band (10 s Hann segments)."""
    n = int(10 * fs)
    if len(rate) < n:
        n = len(rate)
    window = np.hanning(n)
    power = np.zeros(n // 2 + 1)
    for start in range(0, len(rate) - n + 1, n // 2):
        segment = rate[start : start + n]
        power += np.abs(np.fft.rfft((segment - segment.mean()) * window)) ** 2
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    inside = (freqs >= band[0]) & (freqs <= band[1])
    return float(freqs[inside][np.argmax(power[inside])])


# --- Bag access --------------------------------------------------------------------------------


def read_bag(path: Path, topics: list[str]) -> Iterator[tuple[str, object, int]]:
    """(topic, message, receive time ns) for the given topics, in bag order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    info = yaml.safe_load((path / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    # A FILE-compressed bag (dp1f_with_image is zstd) needs the compression reader, which
    # decompresses each file next to the bag as it goes.
    reader = (
        rosbag2_py.SequentialCompressionReader()
        if info.get("compression_format")
        else rosbag2_py.SequentialReader()
    )
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(path), storage_id=info.get("storage_identifier", "sqlite3")
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    classes = {t: get_message(types[t]) for t in topics if t in types}
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield topic, deserialize_message(data, classes[topic]), t


def bag_topics(path: Path) -> dict[str, str]:
    info = yaml.safe_load((path / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    return {
        entry["topic_metadata"]["name"]: entry["topic_metadata"]["type"]
        for entry in info["topics_with_message_count"]
    }


def pick_topic(topics: dict[str, str], given: str | None, msg_type: str, suffix: str) -> str | None:
    if given:
        return given
    matches = [name for name, kind in topics.items() if kind == msg_type and name.endswith(suffix)]
    return matches[0] if len(matches) == 1 else None


def stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def read_imu(path: Path, topic: str, config: Config, window: tuple[float, float] | None) -> ImuData:
    stamps, gyro, accel = [], [], []
    for _, msg, t_recv in read_bag(path, [topic]):
        stamps.append(stamp_ns(msg.header) if config.use_header_stamp else t_recv)
        gyro.append((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
        a = msg.linear_acceleration
        accel.append((a.x * config.accel_scale, a.y * config.accel_scale, a.z * config.accel_scale))
    if not stamps:
        sys.exit(f"no messages on {topic}")
    imu = ImuData(np.asarray(stamps, dtype=np.int64), np.asarray(gyro), np.asarray(accel))
    if window is not None:
        start = imu.stamps_ns[0] + int(window[0] * 1e9)
        keep = (imu.stamps_ns >= start) & (imu.stamps_ns < start + int(window[1] * 1e9))
        imu = ImuData(imu.stamps_ns[keep], imu.gyro[keep], imu.accel[keep])
    return imu


@dataclass
class FramePairs:
    t0_ns: np.ndarray
    t1_ns: np.ndarray
    shift: np.ndarray  # N x 2, full-resolution px of content displacement from frame 0 to frame 1
    response: np.ndarray


def frame_pairs(path: Path, topic: str, window: tuple[float, float] | None) -> FramePairs:
    """Phase correlation between consecutive frames, on the central 80 % at a quarter of the
    resolution. The shift is how far the content moved from one frame to the next."""
    import cv2

    t0s, t1s, shifts, responses = [], [], [], []
    previous: tuple[int, np.ndarray] | None = None
    hann = None
    first_ns: int | None = None
    for count, (_, msg, _) in enumerate(read_bag(path, [topic])):
        t = stamp_ns(msg.header)
        first_ns = t if first_ns is None else first_ns
        if window is not None and not (window[0] <= (t - first_ns) / 1e9 < window[0] + window[1]):
            continue
        jpeg = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(jpeg, cv2.IMREAD_REDUCED_GRAYSCALE_4)
        if image is None:
            previous = None
            continue
        h, w = image.shape
        crop = np.float32(image[h // 10 : h - h // 10, w // 10 : w - w // 10])
        if hann is None:
            hann = cv2.createHanningWindow((crop.shape[1], crop.shape[0]), cv2.CV_32F)
        if previous is not None and t > previous[0]:
            (dx, dy), response = cv2.phaseCorrelate(previous[1], crop, hann)
            t0s.append(previous[0])
            t1s.append(t)
            shifts.append((dx * DECODE_SCALE, dy * DECODE_SCALE))
            responses.append(response)
        previous = (t, crop)
        if count % 2000 == 0:
            print(f"  ... {count} frames", file=sys.stderr)
    if not t0s:
        sys.exit(f"no consecutive frames on {topic}")
    return FramePairs(
        np.asarray(t0s, dtype=np.int64),
        np.asarray(t1s, dtype=np.int64),
        np.asarray(shifts),
        np.asarray(responses),
    )


def predicted_shift(
    stamps: np.ndarray, quats: np.ndarray, t0: np.ndarray, t1: np.ndarray, config: Config
) -> np.ndarray:
    """The content displacement at the principal point that the IMU's rotation between t0 and
    t1 predicts: a direction d in the first frame's optical axes is C R(q1)^T R(q0) C^T d in the
    second's (the same chain as eis.rs's real-to-virtual rotation)."""
    q0 = orientation_at(stamps, quats, t0)
    q1 = orientation_at(stamps, quats, t1)
    rotation = quats_matrix(quats_mul(quats_conj(q1), q0))
    c = config.cam_from_imu
    centre = np.einsum("ij,njk,k->ni", c, rotation, c.T[:, 2])
    return np.stack(
        [config.fx * centre[:, 0] / centre[:, 2], config.fy * centre[:, 1] / centre[:, 2]], axis=1
    )


# --- Subcommands -------------------------------------------------------------------------------


def run_filters(args: argparse.Namespace, config: Config) -> None:
    topics = bag_topics(args.bag)
    imu_topic = pick_topic(topics, args.imu_topic, "sensor_msgs/msg/Imu", "livox/imu")
    if imu_topic is None:
        sys.exit(f"pass --imu-topic; candidates: {topics}")
    imu = read_imu(args.bag, imu_topic, config, args.window)
    span_s = (imu.stamps_ns[-1] - imu.stamps_ns[0]) / 1e9
    imu_hz = (len(imu.stamps_ns) - 1) / span_s
    fs = config.frame_rate
    frames = np.arange(imu.stamps_ns[0], imu.stamps_ns[-1], int(1e9 / fs), dtype=np.int64)
    band = tuple(args.band)
    fx_out, fy_out = config.fx * config.zoom, config.fy * config.zoom

    camera_gyro = imu.gyro @ config.cam_from_imu.T
    print(f"{imu_topic}: {len(imu.stamps_ns)} samples over {span_s:.1f} s ({imu_hz:.1f} Hz)")
    print(
        f"gait: spectral peak {peak_frequency(camera_gyro[:, 0], imu_hz, (0.8, 6.0)):.2f} Hz about "
        f"the optical x axis (vertical image motion), "
        f"{peak_frequency(camera_gyro[:, 1], imu_hz, (0.8, 6.0)):.2f} Hz about y"
    )
    print(
        f"frames simulated at {fs:g} Hz; output focal {fx_out:.0f} x {fy_out:.0f} px (zoom "
        f"{config.zoom:g}); the zoom hides {math.degrees(config.hidden_angles[0]):.1f} deg of "
        f"pan and {math.degrees(config.hidden_angles[1]):.1f} deg of tilt; band "
        f"{band[0]:g}-{band[1]:g} Hz"
    )

    stamps, quats = integrate(imu, 0.0)
    # Level the gyro-only history with the first sample's gravity, as the node's filter starts:
    # the turn damping reads the heading about world z. A fixed world rotation changes no
    # relative rotation, so nothing else below sees it.
    level = align_to_gravity(tuple(imu.accel[0])) or IDENTITY
    quats = quats_mul(np.tile(np.asarray(level), (len(quats), 1)), quats)
    real = orientation_at(stamps, quats, frames)
    raw_rates = camera_rates(real, config.cam_from_imu, fs)
    real_heading = np.unwrap([heading(tuple(q)) for q in real.tolist()])
    # Turning: the heading rate over 1 s above 0.15 rad/s; straight: below 0.05 (per interval).
    turn_rate = np.convolve(np.diff(real_heading) * fs, np.ones(int(fs)) / int(fs), mode="same")
    turning, straight = np.abs(turn_rate) > 0.15, np.abs(turn_rate) < 0.05

    def shake(rates: np.ndarray, mask: np.ndarray) -> tuple[float, float]:
        vertical = band_integral(rates[:, 0], fs, band) * fy_out
        horizontal = band_integral(rates[:, 1], fs, band) * fx_out
        return rms(vertical[mask]), rms(horizontal[mask])

    print(
        f"\nsmoothing (gyro-only orientation, so the low-pass alone is compared), deadband "
        f"{config.turn_deadband_rad_s:g} rad/s and rate tau {config.turn_rate_tau_s:g} s from the "
        f"params.\n{100 * straight.mean():.0f} % of the bag walks straight (heading rate < 0.05 "
        f"rad/s over 1 s), {100 * turning.mean():.0f} % turns (> 0.15). 'held': frames where the "
        "clamp held the view,\nso the real camera's shake went straight through. 'behind': how far "
        "the view's heading trails the robot's\nwhile turning (median / p95). 'slide': how far the "
        "view still turns after a synthetic 0.5 rad/s turn stops."
    )
    print(
        "  order  tau s  turn ref  held    behind deg   slide deg   straight v / h px   "
        "turning v / h px"
    )
    sv, sh = shake(raw_rates, straight)
    tv, th = shake(raw_rates, turning)
    print(f"  raw{'':55s}{sv:8.2f} / {sh:5.2f}{tv:12.2f} / {th:5.2f}")
    for order, tau, reference in args.configs:
        virtual, clamped = virtual_camera(frames, real, config, order, tau, reference)
        rates = camera_rates(virtual, config.cam_from_imu, fs)
        behind = np.degrees(
            np.abs(np.unwrap([heading(tuple(q)) for q in virtual.tolist()]) - real_heading)
        )[1:][turning]
        sv, sh = shake(rates, straight)
        tv, th = shake(rates, turning)
        reference_text = f"{reference:8.2f}" if reference > 0.0 else "     off"
        print(
            f"  {order:5d}  {tau:5.2f}  {reference_text}  {100 * clamped.mean():4.1f} %  "
            f"{np.median(behind):4.1f} / {np.percentile(behind, 95):4.1f}  "
            f"{turn_slide_deg(config, order, tau, reference):9.1f}  "
            f"{sv:8.2f} / {sh:5.2f}{tv:12.2f} / {th:5.2f}"
        )

    print(
        "\nmahony kp: rotation the accelerometer term adds to the gyro's, frame to frame, in the\n"
        "same band (the warp only sees relative rotation, so this is the shake the filter puts in)"
    )
    print("  kp      vertical px  horizontal px  roll deg")
    for kp in args.kps:
        if kp == 0.0:
            continue
        k_stamps, k_quats = integrate(imu, kp)
        filtered = orientation_at(k_stamps, k_quats, frames)
        # Compare increments, not orientations: a filter that started level and the gyro-only one
        # that started at the identity differ by a fixed world rotation, which the warp never sees
        # but which, seen from the moving body, would look like motion at the gait frequency.
        added = camera_rates(filtered, config.cam_from_imu, fs) - raw_rates
        v, h, roll = shake_px(added, fs, band, fx_out, fy_out)
        print(f"  {kp:6.3f}  {v:11.2f}  {h:13.2f}  {roll:8.3f}")


def trimmed_rms(residuals: np.ndarray, keep: float = 0.9) -> float:
    """RMS of the smallest `keep` fraction: phase correlation fails on a few pairs (blur, a
    textureless wall, something walking through), and those should not pick the offset."""
    ordered = np.sort(residuals)
    return float(np.sqrt(np.mean(ordered[: max(int(len(ordered) * keep), 1)] ** 2)))


def sweep(
    pairs: FramePairs,
    mask: np.ndarray,
    stamps: np.ndarray,
    quats: np.ndarray,
    config: Config,
    grid_ms: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """(trimmed residual px, fitted scale) of the measured shifts against the predicted ones,
    for each offset added to the frame stamps."""
    measured = pairs.shift[mask]
    residuals, scales = [], []
    for offset_ms in grid_ms:
        offset = int(offset_ms * 1e6)
        t0, t1 = pairs.t0_ns[mask] + offset, pairs.t1_ns[mask] + offset
        predicted = predicted_shift(stamps, quats, t0, t1, config)
        scale = float(np.sum(measured * predicted) / max(np.sum(predicted * predicted), 1e-12))
        residuals.append(trimmed_rms(np.linalg.norm(measured - scale * predicted, axis=1)))
        scales.append(scale)
    return np.asarray(residuals), np.asarray(scales)


def run_offset(args: argparse.Namespace, config: Config) -> None:
    topics = bag_topics(args.bag)
    imu_topic = pick_topic(topics, args.imu_topic, "sensor_msgs/msg/Imu", "livox/imu")
    image_topic = pick_topic(topics, args.image_topic, IMAGE_TYPE, "image_raw/compressed")
    if imu_topic is None or image_topic is None:
        sys.exit(f"pass --imu-topic / --image-topic; candidates: {topics}")
    imu = read_imu(args.bag, imu_topic, config, None)
    # Gyro only: the offset is a matter of timing, and the accelerometer adds nothing to it.
    stamps, quats = integrate(imu, 0.0)
    print(f"decoding {image_topic} ...", file=sys.stderr)
    pairs = frame_pairs(args.bag, image_topic, args.window)
    interval_ms = (pairs.t1_ns - pairs.t0_ns) / 1e6
    nominal_ms = 1000.0 / config.frame_rate
    mask = (
        (pairs.response >= args.min_response)
        & (np.abs(interval_ms - nominal_ms) < 0.5 * nominal_ms)
        & (np.linalg.norm(pairs.shift, axis=1) < args.max_shift)
    )
    print(
        f"{len(pairs.t0_ns)} consecutive frame pairs, {int(mask.sum())} usable (response >= "
        f"{args.min_response}, one frame period apart, shift < {args.max_shift} px)"
    )
    if mask.sum() < 50:
        sys.exit("too few usable pairs: record more motion (turns, walking) in a textured scene")

    grid = np.arange(args.range[0], args.range[1] + 0.5 * args.step, args.step)
    residuals, scales = sweep(pairs, mask, stamps, quats, config, grid)
    best = int(np.argmin(residuals))
    flat = grid[residuals <= residuals[best] * 1.05]
    print("\n  offset ms  residual px  scale")
    for i in range(0, len(grid), max(int(round(5 / args.step)), 1)):
        marker = "  <" if i == best else ""
        print(f"  {grid[i]:+9.1f}  {residuals[i]:11.3f}  {scales[i]:5.3f}{marker}")
    print(
        f"\nbest {grid[best]:+.1f} ms: residual {residuals[best]:.3f} px, within 5 % from "
        f"{flat.min():+.1f} to {flat.max():+.1f} ms; fitted scale {scales[best]:.3f} "
        f"(fx {config.fx:g} would be ~{config.fx * scales[best]:.0f} px)"
    )
    if scales[best] < 0.5:
        print(
            "warning: the fitted scale is far from 1; check imu_to_camera_rpy_deg before "
            "trusting the offset"
        )

    print("\nper window (offset that window alone prefers):")
    start = pairs.t0_ns[0]
    window_ns = int(args.window_s * 1e9)
    for w0 in range(0, int(pairs.t0_ns[-1] - start) + 1, window_ns):
        in_window = mask & (pairs.t0_ns - start >= w0) & (pairs.t0_ns - start < w0 + window_ns)
        if in_window.sum() < 60:
            continue
        r, _ = sweep(pairs, in_window, stamps, quats, config, grid)
        print(
            f"  {w0 / 1e9:6.0f}-{(w0 + window_ns) / 1e9:<6.0f}s  "
            f"{grid[int(np.argmin(r))]:+6.1f} ms  ({int(in_window.sum())} pairs)"
        )

    param = grid[best] - config.half_exposure_ms
    note = (
        f"minus half the manual exposure ({config.half_exposure_ms:g} ms), which the node "
        "adds itself"
        if config.half_exposure_ms
        else "auto exposure: the node adds no half-exposure term"
    )
    print(
        f"\n=> imu_time_offset_ms: {param:.1f}   ({note}; the params file has "
        f"{config.imu_time_offset_ms:g})"
    )


def run_residual(args: argparse.Namespace, config: Config) -> None:
    topics = bag_topics(args.bag)
    # The warped frames have had a topic of their own since 2026-10-20; before that they were on
    # image_raw/compressed, so a stabilized bag from then needs --image-topic.
    suffix = "image_stabilized/compressed" if args.stabilized else "image_raw/compressed"
    image_topic = pick_topic(topics, args.image_topic, IMAGE_TYPE, suffix)
    if image_topic is None:
        sys.exit(f"pass --image-topic; candidates: {topics}")
    fx = config.fx * (config.zoom if args.stabilized else 1.0)
    fy = config.fy * (config.zoom if args.stabilized else 1.0)
    print(f"decoding {image_topic} ...", file=sys.stderr)
    pairs = frame_pairs(args.bag, image_topic, args.window)
    usable = pairs.response >= args.min_response
    shift = np.where(usable[:, None], pairs.shift, 0.0)
    # Accumulate the shifts into a position, resample it on a uniform grid, and measure that as a
    # rate, so dropped frames do not distort the band.
    fs = config.frame_rate
    times = np.concatenate([[pairs.t0_ns[0]], pairs.t1_ns]).astype(np.float64)
    position = np.vstack([[0.0, 0.0], np.cumsum(shift, axis=0)])
    grid = np.arange(times[0], times[-1], 1e9 / fs)
    rates = np.stack(
        [np.diff(np.interp(grid, times, position[:, k])) * fs for k in range(2)], axis=1
    )
    band = tuple(args.band)
    vertical = band_integral_rms(rates[:, 1], fs, band)
    horizontal = band_integral_rms(rates[:, 0], fs, band)
    print(
        f"{len(pairs.t0_ns)} frame pairs over {(times[-1] - times[0]) / 1e9:.1f} s, "
        f"{100 * (1 - usable.mean()):.1f} % below response {args.min_response} (counted as still)"
    )
    kind = "stabilized" if args.stabilized else "raw"
    print(f"band {band[0]:g}-{band[1]:g} Hz, focal {fx:.0f} x {fy:.0f} px ({kind})")
    print(f"  image:  vertical {vertical:6.2f} px ({math.degrees(vertical / fy):.3f} deg)   "
          f"horizontal {horizontal:6.2f} px ({math.degrees(horizontal / fx):.3f} deg)")

    imu_topic = pick_topic(topics, args.imu_topic, "sensor_msgs/msg/Imu", "livox/imu")
    if imu_topic is not None:
        imu = read_imu(args.bag, imu_topic, config, None)
        stamps, quats = integrate(imu, 0.0)
        real = orientation_at(stamps, quats, grid.astype(np.int64) + config.frame_time_offset_ns)
        v, h, _ = shake_px(camera_rates(real, config.cam_from_imu, fs), fs, band, fx, fy)
        print(f"  camera: vertical {v:6.2f} px ({math.degrees(v / fy):.3f} deg)   "
              f"horizontal {h:6.2f} px ({math.degrees(h / fx):.3f} deg)   "
              "(IMU: what the camera itself did)")


def smoothing_config(text: str) -> tuple[int, float, float]:
    """ORDER:TAU[:TURN_REF], TURN_REF being turn_damping_reference_rad_s; without it (or with 0)
    turns are smoothed like any other motion, as they were until 2026-10-20."""
    order, tau, *rest = text.split(":")
    if order not in ("1", "2") or len(rest) > 1:
        raise argparse.ArgumentTypeError(f"expected ORDER:TAU[:TURN_REF], order 1 or 2: {text}")
    return int(order), float(tau), float(rest[0]) if rest else 0.0


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--params", type=Path, default=DEFAULT_PARAMS, help="camera_feeder params YAML"
    )
    parser.add_argument(
        "--imu-topic", help="default: the one sensor_msgs/Imu topic ending in livox/imu"
    )
    parser.add_argument(
        "--image-topic",
        help="default: the one CompressedImage topic ending in image_raw/compressed (in "
        "image_stabilized/compressed for residual --stabilized)",
    )
    parser.add_argument(
        "--window",
        type=float,
        nargs=2,
        metavar=("START_S", "DURATION_S"),
        help="only this slice of the bag",
    )
    parser.add_argument(
        "--band", type=float, nargs=2, default=[1.0, 6.0], metavar=("LOW_HZ", "HIGH_HZ")
    )
    commands = parser.add_subparsers(dest="command", required=True)

    filters = commands.add_parser(
        "filters", help="compare smoothing settings and Mahony gains (IMU only)"
    )
    filters.add_argument("bag", type=Path)
    filters.add_argument(
        "--configs",
        type=smoothing_config,
        nargs="+",
        default=[(2, 0.35, 0.0), (2, 0.35, 0.3), (2, 0.35, 0.2), (2, 0.35, 0.15), (2, 0.35, 0.1)],
        metavar="ORDER:TAU[:TURN_REF]",
    )
    filters.add_argument("--kps", type=float, nargs="+", default=[0.5, 0.2, 0.05, 0.02])

    offset = commands.add_parser(
        "offset", help="fit imu_time_offset_ms (raw frames, post-2026-10-16 stamps)"
    )
    offset.add_argument("bag", type=Path)
    offset.add_argument(
        "--range", type=float, nargs=2, default=[-40.0, 60.0], metavar=("FROM_MS", "TO_MS")
    )
    offset.add_argument("--step", type=float, default=1.0, help="ms")
    offset.add_argument("--min-response", type=float, default=0.05)
    offset.add_argument(
        "--max-shift", type=float, default=60.0, help="px; larger shifts are unreliable"
    )
    offset.add_argument(
        "--window-s", type=float, default=30.0, help="length of the per-window fits"
    )

    residual = commands.add_parser(
        "residual", help="gait-band shake measured in a recording's frames"
    )
    residual.add_argument("bag", type=Path)
    residual.add_argument(
        "--stabilized",
        action="store_true",
        help="measure image_stabilized/compressed, the zoomed, warped output",
    )
    residual.add_argument("--min-response", type=float, default=0.05)

    args = parser.parse_args()
    config = load_config(args.params)
    commands_by_name = {"filters": run_filters, "offset": run_offset, "residual": run_residual}
    commands_by_name[args.command](args, config)


if __name__ == "__main__":
    main()
