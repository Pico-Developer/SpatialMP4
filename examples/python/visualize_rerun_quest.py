"""Visualize a SpatialMP4 capture in Rerun with RGB, depth, head pose,
controller pose, hand tracking, and controller input streams.

Usage::

    python visualize_rerun_full.py path/to/spatial.mp4

Design notes
------------
* This file's poses follow OpenXR / Quest convention: world frame is
  ``RIGHT_HAND_Y_UP`` (X-right, Y-up, Z-back-out-of-page) and the camera
  extrinsics map IMU → OpenCV image frame (X-right, Y-down, Z-forward).
  We feed Rerun those frames directly — no cyclic permutation.
* The new ``head`` / ``left_hand`` / ``right_hand`` / ``*_controller`` /
  ``*_controller_input`` timed-metadata tracks introduced by the live writer
  RFC are read up-front, and the per-frame ``rgb_frame.pose`` /
  ``depth_frame.pose`` (which the C++ reader does *not* yet populate for
  files that store head pose in its own ``head`` track) are recovered with a
  binary-search lookup against the head track.
* The ``load_rgbd`` densification path produces all-zero depth on these
  captures (depth_extrinsics ≈ identity for live-writer captures), so we
  decode native 320×320 TOF depth via ``load_depth`` and log it under its
  own ``world/depth_camera`` Pinhole, separate from the 1280×1280 RGB
  Pinhole at ``world/camera``.
"""

from __future__ import annotations

import bisect
import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import typer
from scipy.spatial.transform import Rotation

import spatialmp4 as sm


def load_sidecar_depth_poses(video_file: str) -> List[Optional[np.ndarray]]:
    """Resolve the per-frame ``local_from_depth_eye`` poses from the spool
    sidecar (``<session>/depth/frames.jsonl``) and return one 4×4 pose per
    depth frame in the file, indexed in capture order.

    The Godot live writer drops these poses into the mp4 depth_extrinsics
    descriptor as identity (writer bug — only one extrinsic is stored, but
    OpenXR depth provides a per-frame ``local_from_depth_eye``), so we go
    back to the sidecar to recover them.

    Returns an empty list if the sidecar is unavailable.
    """
    session_dir = Path(video_file).with_suffix("")
    sidecar = session_dir / "depth" / "frames.jsonl"
    if not sidecar.exists():
        return []
    poses: List[Optional[np.ndarray]] = []
    with sidecar.open() as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
                lfde = rec["metadata"]["local_from_depth_eye"]
                pos = lfde["position"]
                rot = lfde["rotation"]
                mat = np.eye(4, dtype=np.float64)
                mat[:3, :3] = Rotation.from_quat(
                    [rot["x"], rot["y"], rot["z"], rot["w"]]
                ).as_matrix()
                mat[:3, 3] = [pos["x"], pos["y"], pos["z"]]
                poses.append(mat)
            except (json.JSONDecodeError, KeyError, TypeError):
                poses.append(None)
    return poses


def load_sidecar_camera_calibration(
    video_file: str, eye: str = "left"
) -> Optional[Dict[str, object]]:
    """Load ``<session>/<eye>_camera_characteristics.json`` and return the
    Camera2 → Unity-head-from-camera transform plus a 3×3 intrinsic matrix.

    Ports ``_openquest_head_from_camera`` from godot_depth_rgb_align.py:
    Camera2's ``lens_pose_translation/rotation`` is in Android-sensor frame
    and needs a particular conversion (negate quaternion X/Y, transpose,
    right-multiply by diag(1,-1,-1)) before it can be composed with the
    Godot head pose. The mp4 ``rgb_extrinsics`` descriptor does *not* go
    through this transform, which is why depth/rgb alignment computed off
    of mp4's value is wrong.
    """
    session_dir = Path(video_file).with_suffix("")
    path = session_dir / f"{eye}_camera_characteristics.json"
    if not path.exists():
        return None
    char = json.loads(path.read_text())
    translation = char.get("lens_pose_translation") or [0.0, 0.0, 0.0]
    raw_rotation = char.get("lens_pose_rotation") or [0.0, 0.0, 0.0, 1.0]
    intrinsics = char.get("lens_intrinsic_calibration") or []
    if len(intrinsics) < 4:
        return None
    fx, fy, cx, cy = (float(v) for v in intrinsics[:4])

    translation_unity = np.array(
        [translation[0], translation[1], -translation[2]], dtype=np.float64
    )
    converted_quat = np.array(
        [-raw_rotation[0], -raw_rotation[1], raw_rotation[2], raw_rotation[3]],
        dtype=np.float64,
    )
    converted_quat /= max(np.linalg.norm(converted_quat), 1e-12)
    rotation_unity = (
        Rotation.from_quat(converted_quat).as_matrix().T @ np.diag([1.0, -1.0, -1.0])
    )
    head_from_camera_unity = np.eye(4, dtype=np.float64)
    head_from_camera_unity[:3, :3] = rotation_unity
    head_from_camera_unity[:3, 3] = translation_unity
    K_rgb = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64
    )
    width = int((char.get("sensor_active_array_size") or {}).get("width", 0))
    height = int((char.get("sensor_active_array_size") or {}).get("height", 0))
    return {
        "head_from_camera_unity": head_from_camera_unity,
        "K_rgb": K_rgb,
        "width": width,
        "height": height,
    }


# Godot (right-handed Y-up Z-back) ↔ Unity (left-handed Y-up Z-forward).
# Both spaces share X-right and Y-up axes, only Z is negated, so the
# similarity transform conv⋅T⋅conv (with conv = diag(1,1,-1)) toggles
# between the two conventions.
UNITY_FROM_GODOT_3 = np.diag([1.0, 1.0, -1.0])
UNITY_FROM_GODOT_4 = np.diag([1.0, 1.0, -1.0, 1.0])


def godot_transform_to_unity(transform_godot: np.ndarray) -> np.ndarray:
    return UNITY_FROM_GODOT_4 @ transform_godot @ UNITY_FROM_GODOT_4


def project_pts_unity_to_rgb(
    pts_unity: np.ndarray,
    local_from_rgb_unity: np.ndarray,
    K_rgb: np.ndarray,
    rgb_w: int,
    rgb_h: int,
    splat_radius: int = 3,
) -> np.ndarray:
    """Project Unity-world points into the RGB image using the reference
    script's Y-UP projection formula (``v = cy - fy * Y / Z``)."""
    if pts_unity.size == 0:
        return np.zeros((rgb_h, rgb_w), dtype=np.float32)
    rgb_from_world = np.linalg.inv(local_from_rgb_unity)
    h = np.concatenate(
        [pts_unity, np.ones((len(pts_unity), 1))], axis=1
    )
    cam = (rgb_from_world @ h.T).T[:, :3]
    valid = np.all(np.isfinite(cam), axis=1) & (cam[:, 2] > 1e-3)
    cam = cam[valid]
    if cam.size == 0:
        return np.zeros((rgb_h, rgb_w), dtype=np.float32)
    fx, fy = K_rgb[0, 0], K_rgb[1, 1]
    cx, cy = K_rgb[0, 2], K_rgb[1, 2]
    u_f = fx * cam[:, 0] / cam[:, 2] + cx
    v_f = cy - fy * cam[:, 1] / cam[:, 2]      # Y-UP
    u = np.rint(u_f).astype(np.int64)
    v = np.rint(v_f).astype(np.int64)
    inside = (u >= 0) & (u < rgb_w) & (v >= 0) & (v < rgb_h)
    u = u[inside]; v = v[inside]; z = cam[inside, 2]
    if z.size == 0:
        return np.zeros((rgb_h, rgb_w), dtype=np.float32)
    INF = np.float64(1e9)
    buf = np.full((rgb_h, rgb_w), INF, dtype=np.float64)
    flat = buf.reshape(-1)
    for dv in range(-splat_radius, splat_radius + 1):
        for du in range(-splat_radius, splat_radius + 1):
            uu = u + du; vv = v + dv
            ok = (uu >= 0) & (uu < rgb_w) & (vv >= 0) & (vv < rgb_h)
            if not ok.any():
                continue
            idx = vv[ok].astype(np.int64) * rgb_w + uu[ok].astype(np.int64)
            np.minimum.at(flat, idx, z[ok])
    return np.where(buf < INF, buf, 0.0).astype(np.float32)


def load_sidecar_depth_metadata(
    video_file: str,
) -> List[Optional[Dict[str, object]]]:
    """Load per-frame depth metadata records (``inverse_projection_view``,
    ``near_z``, ``far_z``, ``local_from_depth_eye``, …) from the spool
    sidecar so the visualizer can use the same direct pixel→world matrix
    the Godot live writer used to produce the depth in the first place."""
    session_dir = Path(video_file).with_suffix("")
    sidecar = session_dir / "depth" / "frames.jsonl"
    if not sidecar.exists():
        return []
    records: List[Optional[Dict[str, object]]] = []
    with sidecar.open() as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
                md = rec.get("metadata") or {}
                records.append(md)
            except json.JSONDecodeError:
                records.append(None)
    return records


def unproject_depth_via_ipv(
    depth: np.ndarray,
    inverse_projection_view: np.ndarray,
    near_z: float,
    far_z: Optional[float],
    depth_min: float,
    depth_max: float,
) -> np.ndarray:
    """Unproject a (H, W) linear-depth image (metres) directly into world
    coordinates using the per-frame OpenXR ``depth_inverse_projection_view``
    matrix exposed by the Godot live writer. Returns (N, 3) world points.

    Pipeline (mirrors ``godot_depth_rgb_align.unproject_depth_to_unity``):

    1. Map linear-metres depth back to the [0, 1] ``window_depth`` the
       OpenXR runtime hands the Godot live writer. The reference's
       ``linearize_window_depth`` computes
       ``linear = x_param / (2*window_depth - 1 + y_param)``, so we invert
       to ``window_depth = (x_param/linear - y_param + 1) / 2``.
    2. Build clip-space ``(u_ndc, v_ndc, 2*window_depth - 1, 1)``.
    3. Apply the per-frame ``inverse_projection_view`` (4×4 Godot matrix)
       and perspective-divide to get Godot world coordinates.

    Note: we stop here in Godot frame; downstream paths apply the
    Godot-vs-rest-of-pipeline conversion themselves.
    """
    if far_z is None or not np.isfinite(far_z) or far_z < near_z:
        x_param = -2.0 * near_z
        y_param = -1.0
    else:
        x_param = -2.0 * far_z * near_z / (far_z - near_z)
        y_param = -(far_z + near_z) / (far_z - near_z)

    H, W = depth.shape
    valid_depth = (depth > depth_min) & (depth < depth_max) & np.isfinite(depth)
    if not valid_depth.any():
        return np.zeros((0, 3), dtype=np.float64)

    # window_depth ∈ [0, 1] (inverts linearize_window_depth)
    with np.errstate(divide="ignore", invalid="ignore"):
        window_depth = (x_param / depth - y_param + 1.0) / 2.0
    # For depths outside the projection's well-defined range the formula
    # blows up; mask those out instead of feeding NaN to the matmul.
    window_depth_finite = np.isfinite(window_depth)

    rows, cols = np.indices((H, W), dtype=np.float64)
    clip = np.stack(
        [
            2.0 * ((cols + 0.5) / W) - 1.0,
            2.0 * ((rows + 0.5) / H) - 1.0,
            2.0 * np.where(window_depth_finite, window_depth, 0.0) - 1.0,
            np.ones_like(depth, dtype=np.float64),
        ],
        axis=-1,
    ).reshape(-1, 4)
    h = (inverse_projection_view @ clip.T).T
    w = h[:, 3]
    valid_w = np.isfinite(w) & (np.abs(w) > 1e-12)
    pts = np.full((h.shape[0], 3), np.nan, dtype=np.float64)
    pts[valid_w] = h[valid_w, :3] / w[valid_w, None]

    valid = (
        valid_w
        & valid_depth.reshape(-1)
        & window_depth_finite.reshape(-1)
        & np.all(np.isfinite(pts), axis=1)
    )
    return pts[valid]


def project_world_to_rgb(
    pts_world: np.ndarray,
    T_W_Srgb: np.ndarray,
    K_rgb: sm.CameraIntrinsics,
    rgb_w: int,
    rgb_h: int,
    splat_radius: int = 3,
) -> np.ndarray:
    """Project world points into the RGB camera frame (T_W_Srgb) and rasterise
    each one as a small splat. Returns a (rgb_h, rgb_w) float32 depth image
    with the smallest (nearest) depth per pixel, zeros where empty."""
    if pts_world.size == 0:
        return np.zeros((rgb_h, rgb_w), dtype=np.float32)
    T_Srgb_W = np.linalg.inv(T_W_Srgb)
    cam = (T_Srgb_W[:3, :3] @ pts_world.T).T + T_Srgb_W[:3, 3]
    z = cam[:, 2]
    in_front = z > 1e-3
    if not in_front.any():
        return np.zeros((rgb_h, rgb_w), dtype=np.float32)
    cam = cam[in_front]
    z = z[in_front]
    fx, fy = float(K_rgb.fx), float(K_rgb.fy)
    cx, cy = float(K_rgb.cx), float(K_rgb.cy)
    u = (cam[:, 0] * fx / z + cx).astype(np.int32)
    v = (cam[:, 1] * fy / z + cy).astype(np.int32)
    inside = (u >= 0) & (u < rgb_w) & (v >= 0) & (v < rgb_h)
    if not inside.any():
        return np.zeros((rgb_h, rgb_w), dtype=np.float32)
    u, v, z = u[inside], v[inside], z[inside]
    INF = np.float64(1e9)
    buf = np.full((rgb_h, rgb_w), INF, dtype=np.float64)
    flat = buf.reshape(-1)
    for dv in range(-splat_radius, splat_radius + 1):
        for du in range(-splat_radius, splat_radius + 1):
            uu = u + du
            vv = v + dv
            ok = (uu >= 0) & (uu < rgb_w) & (vv >= 0) & (vv < rgb_h)
            if not ok.any():
                continue
            idx = vv[ok].astype(np.int64) * rgb_w + uu[ok].astype(np.int64)
            np.minimum.at(flat, idx, z[ok])
    out = np.where(buf < INF, buf, 0.0).astype(np.float32)
    return out


# ---------------------------------------------------------------------------
# Rerun compatibility shims
# ---------------------------------------------------------------------------


def set_time_seconds(name: str, seconds: float) -> None:
    """rerun 0.23 uses ``set_time_seconds`` while 0.30+ replaced it with
    ``set_time(timeline, duration=...)``. Support both."""
    if hasattr(rr, "set_time_seconds"):
        rr.set_time_seconds(name, seconds)  # type: ignore[attr-defined]
    else:
        rr.set_time(name, duration=float(seconds))


def log_transform3d(path: str, translation, rotation_xyzw, axis_len: float = 0.0) -> None:
    """Log a Transform3D; on older rerun SDKs that lack the ``axis_length``
    keyword we emit a small Arrows3D triad as a child entity instead."""
    transform_kwargs = {
        "translation": translation,
        "rotation": rr.Quaternion(xyzw=rotation_xyzw),
    }
    if axis_len > 0:
        try:
            rr.log(path, rr.Transform3D(axis_length=axis_len, **transform_kwargs))
            return
        except TypeError:
            pass

    rr.log(path, rr.Transform3D(**transform_kwargs))
    if axis_len > 0:
        rr.log(
            f"{path}/axes",
            rr.Arrows3D(
                vectors=[[axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]],
                colors=[[255, 60, 60], [60, 255, 60], [60, 60, 255]],
            ),
        )


# ---------------------------------------------------------------------------
# Pose math
# ---------------------------------------------------------------------------


def ensure_right_handed_rotation(rotation_matrix: np.ndarray) -> np.ndarray:
    """Project a 3×3 matrix back onto SO(3) so Rerun's Transform3D parser
    accepts it (the C++-side quaternions are usually fine but extrinsics
    matrices may drift)."""
    r = np.array(rotation_matrix, dtype=np.float64, copy=True)
    u, _, vh = np.linalg.svd(r)
    corrected = u @ vh
    if np.linalg.det(corrected) < 0:
        u[:, -1] *= -1
        corrected = u @ vh
    return corrected


def pose_frame_to_matrix(pose: sm.PoseFrame) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    rot = Rotation.from_quat([pose.qx, pose.qy, pose.qz, pose.qw]).as_matrix()
    mat[:3, :3] = rot
    mat[:3, 3] = [pose.x, pose.y, pose.z]
    return mat


# ---------------------------------------------------------------------------
# Head-pose lookup (binary-search by timestamp)
# ---------------------------------------------------------------------------


class PoseLookup:
    """Sorted-by-timestamp head pose track with nearest-neighbour lookup."""

    def __init__(self, frames: Sequence[sm.PoseFrame]) -> None:
        valid = [f for f in frames if f.timestamp > 0]
        valid.sort(key=lambda f: f.timestamp)
        self._ts = [f.timestamp for f in valid]
        self._frames = valid

    def __len__(self) -> int:
        return len(self._frames)

    def empty(self) -> bool:
        return len(self._frames) == 0

    def nearest(self, t: float) -> Optional[sm.PoseFrame]:
        if not self._frames:
            return None
        idx = bisect.bisect_left(self._ts, t)
        candidates = []
        if idx < len(self._ts):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        best = min(candidates, key=lambda i: abs(self._ts[i] - t))
        return self._frames[best]

    def trajectory_xyz(self) -> np.ndarray:
        return np.array(
            [[f.x, f.y, f.z] for f in self._frames], dtype=np.float64
        )


# ---------------------------------------------------------------------------
# OpenXR hand-skeleton metadata (XR_EXT_hand_tracking, 26 joints)
# ---------------------------------------------------------------------------


XR_HAND_JOINT_NAMES: Tuple[str, ...] = (
    "PALM",
    "WRIST",
    "THUMB_METACARPAL", "THUMB_PROXIMAL", "THUMB_DISTAL", "THUMB_TIP",
    "INDEX_METACARPAL", "INDEX_PROXIMAL", "INDEX_INTERMEDIATE",
    "INDEX_DISTAL", "INDEX_TIP",
    "MIDDLE_METACARPAL", "MIDDLE_PROXIMAL", "MIDDLE_INTERMEDIATE",
    "MIDDLE_DISTAL", "MIDDLE_TIP",
    "RING_METACARPAL", "RING_PROXIMAL", "RING_INTERMEDIATE",
    "RING_DISTAL", "RING_TIP",
    "LITTLE_METACARPAL", "LITTLE_PROXIMAL", "LITTLE_INTERMEDIATE",
    "LITTLE_DISTAL", "LITTLE_TIP",
)


# (parent_id, child_id) pairs for each finger chain, rooted at WRIST(1).
XR_HAND_BONES: Tuple[Tuple[int, int], ...] = (
    # Thumb
    (1, 2), (2, 3), (3, 4), (4, 5),
    # Index
    (1, 6), (6, 7), (7, 8), (8, 9), (9, 10),
    # Middle
    (1, 11), (11, 12), (12, 13), (13, 14), (14, 15),
    # Ring
    (1, 16), (16, 17), (17, 18), (18, 19), (19, 20),
    # Little
    (1, 21), (21, 22), (22, 23), (23, 24), (24, 25),
)


HAND_COLORS: Dict[str, Tuple[int, int, int]] = {
    "left_hand": (80, 180, 255),
    "right_hand": (255, 160, 80),
}


CONTROLLER_COLORS: Dict[str, Tuple[int, int, int]] = {
    "left_controller": (120, 220, 255),
    "right_controller": (255, 200, 120),
}


CONTROLLER_INPUT_BITS: Tuple[Tuple[str, int], ...] = (
    ("TRIGGER_CLICK", 0), ("TRIGGER_TOUCH", 1),
    ("GRIP_CLICK", 2),
    ("THUMBSTICK_CLICK", 3), ("THUMBSTICK_TOUCH", 4),
    ("A_CLICK", 5), ("A_TOUCH", 6),
    ("B_CLICK", 7), ("B_TOUCH", 8),
    ("X_CLICK", 9), ("X_TOUCH", 10),
    ("Y_CLICK", 11), ("Y_TOUCH", 12),
    ("MENU_CLICK", 13), ("SYSTEM_CLICK", 14),
    ("THUMBREST_TOUCH", 15),
    ("TRACKPAD_CLICK", 16), ("TRACKPAD_TOUCH", 17),
)


# ---------------------------------------------------------------------------
# Per-track loggers
# ---------------------------------------------------------------------------


def log_hand_frame(track_id: str, hand_frame: sm.HandJointsFrame) -> None:
    """Log hand joints (points + bones) directly in world coordinates."""
    joints = hand_frame.joints
    if not joints:
        return

    positions = np.array(
        [[j.x, j.y, j.z] for j in joints], dtype=np.float64
    )
    joint_ids = np.array([int(j.joint_id) for j in joints], dtype=np.int32)
    radii = np.array(
        [max(float(j.radius_m), 0.004) for j in joints], dtype=np.float32
    )

    color = HAND_COLORS.get(track_id, (200, 200, 200))
    labels = [
        XR_HAND_JOINT_NAMES[i] if 0 <= i < len(XR_HAND_JOINT_NAMES) else f"J{i}"
        for i in joint_ids
    ]

    rr.log(
        f"world/hands/{track_id}/joints",
        rr.Points3D(
            positions=positions,
            radii=radii,
            colors=[color] * len(joints),
            labels=labels,
            show_labels=False,
        ),
    )

    id_to_idx = {int(jid): i for i, jid in enumerate(joint_ids)}
    strips = [
        np.stack([positions[id_to_idx[p]], positions[id_to_idx[c]]], axis=0)
        for p, c in XR_HAND_BONES
        if p in id_to_idx and c in id_to_idx
    ]
    if strips:
        rr.log(
            f"world/hands/{track_id}/bones",
            rr.LineStrips3D(
                strips=strips, colors=[color] * len(strips), radii=0.0025
            ),
        )


def log_rigid_pose(track_id: str, pose: sm.PoseFrame, axis_len: float = 0.08) -> None:
    mat = pose_frame_to_matrix(pose)
    mat[:3, :3] = ensure_right_handed_rotation(mat[:3, :3])
    translation = mat[:3, 3]
    quat = Rotation.from_matrix(mat[:3, :3]).as_quat()

    color = CONTROLLER_COLORS.get(track_id, (220, 220, 220))
    log_transform3d(f"world/rigid/{track_id}", translation, quat, axis_len)
    rr.log(
        f"world/rigid/{track_id}/marker",
        rr.Points3D(positions=[translation], colors=[color], radii=[0.015]),
    )


def format_controller_input(frame: sm.ControllerInputFrame) -> str:
    side = "left" if frame.controller == 0 else "right"
    pkt = "SNAPSHOT" if frame.packet_type == 1 else "EVENT"

    def bits(mask: int) -> str:
        return ", ".join(
            name for name, bit in CONTROLLER_INPUT_BITS if (mask >> bit) & 1
        )

    return "\n".join([
        f"[{pkt}] {side} controller @ {frame.timestamp:.3f}s",
        f"pressed: {bits(frame.pressed_mask) or '-'}",
        f"touched: {bits(frame.touched_mask) or '-'}",
        f"changed: {bits(frame.changed_mask) or '-'}",
        f"trigger={frame.trigger_value:.2f} grip={frame.grip_value:.2f}",
        f"thumbstick=({frame.thumbstick_x:+.2f}, {frame.thumbstick_y:+.2f})",
        f"trackpad =({frame.trackpad_x:+.2f}, {frame.trackpad_y:+.2f})",
    ])


# ---------------------------------------------------------------------------
# Track discovery
# ---------------------------------------------------------------------------


def collect_tracks(reader: sm.Reader) -> Dict[str, List[str]]:
    rigid: List[str] = []
    hands: List[str] = []
    inputs: List[str] = []
    for tid in reader.list_timed_metadata_tracks():
        if reader.get_rigid_pose_frames(tid):
            rigid.append(tid)
            continue
        if reader.get_hand_joint_frames(tid):
            hands.append(tid)
            continue
        if reader.get_controller_input_frames(tid):
            inputs.append(tid)
    return {"rigid_pose": rigid, "hand_joints": hands, "controller_input": inputs}


def log_all_rigid_pose_tracks(reader: sm.Reader, track_ids: Sequence[str]) -> None:
    for tid in track_ids:
        if tid == "head":
            # head pose is rendered as world/camera + world/trajectory below.
            continue
        for pose in reader.get_rigid_pose_frames(tid):
            if pose.timestamp <= 0:
                continue
            set_time_seconds("time", pose.timestamp)
            log_rigid_pose(tid, pose)


def log_all_hand_tracks(reader: sm.Reader, track_ids: Sequence[str]) -> None:
    for tid in track_ids:
        for frame in reader.get_hand_joint_frames(tid):
            if frame.timestamp <= 0:
                continue
            set_time_seconds("time", frame.timestamp)
            log_hand_frame(tid, frame)


def log_hand_frame_head_relative(
    track_id: str,
    hand_frame: sm.HandJointsFrame,
    head_lookup: "PoseLookup",
) -> None:
    """Log hand joints in the head's local frame so the user can see hand
    motion relative to the head (independent of where the user walked)."""
    joints = hand_frame.joints
    if not joints:
        return
    head = head_lookup.nearest(hand_frame.timestamp)
    if head is None:
        return

    T_W_H = pose_frame_to_matrix(head)
    R = T_W_H[:3, :3]
    t = T_W_H[:3, 3]
    R_T = R.T  # rotation inverse for an SE(3)
    # X_H = R^T (X_W - t)
    positions_world = np.array(
        [[j.x, j.y, j.z] for j in joints], dtype=np.float64
    )
    positions_head = (positions_world - t) @ R  # equivalent to R^T @ (p-t) for each row

    joint_ids = np.array([int(j.joint_id) for j in joints], dtype=np.int32)
    radii = np.array(
        [max(float(j.radius_m), 0.004) for j in joints], dtype=np.float32
    )
    color = HAND_COLORS.get(track_id, (200, 200, 200))
    labels = [
        XR_HAND_JOINT_NAMES[i] if 0 <= i < len(XR_HAND_JOINT_NAMES) else f"J{i}"
        for i in joint_ids
    ]

    rr.log(
        f"head_relative/hands/{track_id}/joints",
        rr.Points3D(
            positions=positions_head,
            radii=radii,
            colors=[color] * len(joints),
            labels=labels,
            show_labels=False,
        ),
    )

    id_to_idx = {int(jid): i for i, jid in enumerate(joint_ids)}
    strips = [
        np.stack(
            [positions_head[id_to_idx[p]], positions_head[id_to_idx[c]]],
            axis=0,
        )
        for p, c in XR_HAND_BONES
        if p in id_to_idx and c in id_to_idx
    ]
    if strips:
        rr.log(
            f"head_relative/hands/{track_id}/bones",
            rr.LineStrips3D(
                strips=strips, colors=[color] * len(strips), radii=0.0025
            ),
        )


def log_all_hand_tracks_head_relative(
    reader: sm.Reader,
    track_ids: Sequence[str],
    head_lookup: "PoseLookup",
) -> None:
    for tid in track_ids:
        for frame in reader.get_hand_joint_frames(tid):
            if frame.timestamp <= 0:
                continue
            set_time_seconds("time", frame.timestamp)
            log_hand_frame_head_relative(tid, frame, head_lookup)


def log_all_controller_input_tracks(
    reader: sm.Reader, track_ids: Sequence[str]
) -> None:
    for tid in track_ids:
        for frame in reader.get_controller_input_frames(tid):
            if frame.timestamp <= 0:
                continue
            set_time_seconds("time", frame.timestamp)
            rr.log(
                f"controller_input/{tid}",
                rr.TextLog(format_controller_input(frame)),
            )
            rr.log(f"plots/{tid}/trigger", rr.Scalars(float(frame.trigger_value)))
            rr.log(f"plots/{tid}/grip", rr.Scalars(float(frame.grip_value)))
            rr.log(f"plots/{tid}/thumbstick_x", rr.Scalars(float(frame.thumbstick_x)))
            rr.log(f"plots/{tid}/thumbstick_y", rr.Scalars(float(frame.thumbstick_y)))


# ---------------------------------------------------------------------------
# Head trajectory
# ---------------------------------------------------------------------------


def log_head_pose_track(head_lookup: PoseLookup) -> None:
    """Log the entire head trajectory (static line strip) and the per-frame
    head transform on the shared ``time`` timeline so the user can scrub it
    in 3D."""

    if head_lookup.empty():
        return

    traj = head_lookup.trajectory_xyz()
    rr.log(
        "world/trajectory/head",
        rr.LineStrips3D(
            strips=[traj],
            colors=[[255, 215, 0]],
            radii=0.02,
        ),
        static=True,
    )
    rr.log(
        "world/trajectory/head/points",
        rr.Points3D(positions=traj, colors=[[255, 120, 0]], radii=0.008),
        static=True,
    )
    rr.log(
        "world/trajectory/head/start",
        rr.Points3D(
            positions=traj[:1],
            colors=[[0, 255, 0]],
            radii=[0.05],
            labels=["start"],
        ),
        static=True,
    )
    rr.log(
        "world/trajectory/head/end",
        rr.Points3D(
            positions=traj[-1:],
            colors=[[255, 0, 0]],
            radii=[0.05],
            labels=["end"],
        ),
        static=True,
    )

    for f in head_lookup._frames:
        set_time_seconds("time", f.timestamp)
        log_rigid_pose("head", f, axis_len=0.1)

        # Head gaze arrow: OpenXR head looks down its local -Z axis, so the
        # world-frame gaze direction is -R[:, 2] of the head rotation matrix.
        mat = pose_frame_to_matrix(f)
        forward = -mat[:3, 2]
        origin = mat[:3, 3]
        rr.log(
            "world/rigid/head/gaze",
            rr.Arrows3D(
                origins=[origin],
                vectors=[forward * 0.5],
                colors=[[255, 80, 200]],
                radii=0.015,
            ),
        )


def log_floor_grid(
    half_extent_x: float, half_extent_z: float, y: float = 0.0, step: float = 1.0
) -> None:
    """Draw a horizontal grid at world Y = y so the user has a ground
    reference. Half-extents are in metres, padded to the nearest ``step``."""
    nx = int(np.ceil(half_extent_x / step)) + 1
    nz = int(np.ceil(half_extent_z / step)) + 1
    xs = np.arange(-nx, nx + 1) * step
    zs = np.arange(-nz, nz + 1) * step

    strips: List[np.ndarray] = []
    for x in xs:
        strips.append(
            np.array([[x, y, zs[0]], [x, y, zs[-1]]], dtype=np.float64)
        )
    for z in zs:
        strips.append(
            np.array([[xs[0], y, z], [xs[-1], y, z]], dtype=np.float64)
        )
    rr.log(
        "world/floor",
        rr.LineStrips3D(strips=strips, colors=[[80, 80, 80]] * len(strips), radii=0.002),
        static=True,
    )


# ---------------------------------------------------------------------------
# Depth-to-3D point cloud
# ---------------------------------------------------------------------------


def project_depth_to_points(
    depth: np.ndarray, K: sm.CameraIntrinsics, stride: int = 2
) -> np.ndarray:
    """Unproject a (H, W) float32 depth map into the camera's local frame.
    Camera frame convention: X-right, Y-down, Z-forward (OpenCV / RDF). The
    returned (N, 3) array holds points with z > 0 only."""
    h, w = depth.shape
    fx, fy, cx, cy = float(K.fx), float(K.fy), float(K.cx), float(K.cy)
    if stride > 1:
        d = depth[::stride, ::stride]
    else:
        d = depth
    rows, cols = np.indices(d.shape, dtype=np.float32)
    cols = cols * stride
    rows = rows * stride
    mask = d > 0
    z = d[mask]
    x = (cols[mask] - cx) / fx * z
    y = (rows[mask] - cy) / fy * z
    return np.stack([x, y, z], axis=-1)


def depth_colormap_jet(values: np.ndarray, lo: float, hi: float) -> np.ndarray:
    """Map a (N,) float depth array to RGB uint8 via OpenCV's JET colormap."""
    clipped = np.clip((values - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    gray = (clipped * 255).astype(np.uint8)
    rgb = cv2.applyColorMap(gray.reshape(-1, 1), cv2.COLORMAP_JET)
    return cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB).reshape(-1, 3)


def warp_depth_to_rgb(
    depth: np.ndarray,
    K_d: sm.CameraIntrinsics,
    K_rgb: sm.CameraIntrinsics,
    T_rgb_d: np.ndarray,
    rgb_w: int,
    rgb_h: int,
) -> np.ndarray:
    """Project a (Hd, Wd) float depth (depth-camera frame) onto a (rgb_h,
    rgb_w) image at the RGB camera's intrinsics, using ``T_rgb_d`` to move
    points from the depth camera frame into the rgb camera frame. Output is
    a float32 metric depth image; pixels with no valid contribution stay 0.
    Each warped sample is splatted to a small block so that neighboring depth
    pixels (spaced ~fx_rgb/fx_depth apart in the warped image) don't leave
    visible gaps, and a Z-buffer ensures the nearest depth wins per pixel."""
    Hd, Wd = depth.shape
    fx_d, fy_d = float(K_d.fx), float(K_d.fy)
    cx_d, cy_d = float(K_d.cx), float(K_d.cy)
    fx_r, fy_r = float(K_rgb.fx), float(K_rgb.fy)
    cx_r, cy_r = float(K_rgb.cx), float(K_rgb.cy)

    v_d, u_d = np.indices((Hd, Wd), dtype=np.float32)
    mask = depth > 0
    if not mask.any():
        return np.zeros((rgb_h, rgb_w), dtype=np.float32)

    z = depth[mask]
    x = (u_d[mask] - cx_d) / fx_d * z
    y = (v_d[mask] - cy_d) / fy_d * z
    pts_d = np.stack([x, y, z], axis=-1)

    R = T_rgb_d[:3, :3]
    t = T_rgb_d[:3, 3]
    pts_r = pts_d @ R.T + t

    z_r = pts_r[:, 2]
    in_front = z_r > 1e-3
    if not in_front.any():
        return np.zeros((rgb_h, rgb_w), dtype=np.float32)

    pts_r = pts_r[in_front]
    z_r = z_r[in_front]

    u = (pts_r[:, 0] * fx_r / z_r + cx_r).astype(np.int32)
    v = (pts_r[:, 1] * fy_r / z_r + cy_r).astype(np.int32)
    inside = (u >= 0) & (u < rgb_w) & (v >= 0) & (v < rgb_h)
    u, v, z_r = u[inside], v[inside], z_r[inside]
    if z_r.size == 0:
        return np.zeros((rgb_h, rgb_w), dtype=np.float32)

    # Splat each warped sample as a small block ~ (fx_r / fx_d) wide so the
    # output is dense enough to read against the RGB background. Use a
    # per-pixel z-buffer that keeps the smallest (nearest) depth.
    rad_u = max(int(np.ceil(fx_r / fx_d / 2.0)), 1)
    rad_v = max(int(np.ceil(fy_r / fy_d / 2.0)), 1)

    INF = np.float32(1e9)
    zbuf = np.full((rgb_h, rgb_w), INF, dtype=np.float32)
    for dv in range(-rad_v, rad_v + 1):
        for du in range(-rad_u, rad_u + 1):
            uu = u + du
            vv = v + dv
            ok = (uu >= 0) & (uu < rgb_w) & (vv >= 0) & (vv < rgb_h)
            if not ok.any():
                continue
            flat = (vv[ok].astype(np.int64) * rgb_w + uu[ok].astype(np.int64))
            zz = z_r[ok].astype(np.float32)
            # Sort by descending z so the smallest (nearest) is written last.
            order = np.argsort(-zz)
            zbuf.reshape(-1)[flat[order]] = zz[order]

    out = np.where(zbuf < INF, zbuf, np.float32(0.0))
    return out


def blend_depth_on_rgb(
    rgb_bgr: np.ndarray,
    depth_aligned: np.ndarray,
    depth_min: float,
    depth_max: float,
    alpha: float = 0.55,
) -> np.ndarray:
    """Composite a JET-colored depth heatmap over an RGB (BGR) image, only
    where depth is valid. Returns a BGR image of the same size as RGB."""
    H, W = rgb_bgr.shape[:2]
    if depth_aligned.shape[:2] != (H, W):
        return rgb_bgr
    mask = depth_aligned > 0
    if not mask.any():
        return rgb_bgr
    norm = np.clip(
        (depth_aligned - depth_min) / max(depth_max - depth_min, 1e-6),
        0.0,
        1.0,
    )
    gray = (norm * 255).astype(np.uint8)
    heat = cv2.applyColorMap(gray, cv2.COLORMAP_JET)  # BGR
    blended = cv2.addWeighted(rgb_bgr, 1.0 - alpha, heat, alpha, 0.0)
    out = rgb_bgr.copy()
    out[mask] = blended[mask]
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(
    video_file: str,
    depth_only: bool = False,
    rgb_only: bool = False,
    topk: Optional[int] = typer.Option(
        None, help="Limit RGB / depth visualization to the first K frames."
    ),
    depth_min: float = typer.Option(0.1, help="Min valid depth in meters."),
    depth_max: float = typer.Option(15.0, help="Max valid depth in meters."),
    depth_edge_filter: bool = typer.Option(
        False, help="Drop large-gradient depth pixels (Sobel-based denoiser)."
    ),
    depth_pc_stride: int = typer.Option(
        2, help="Subsample stride for the per-frame depth point cloud."
    ),
    world_coord: str = typer.Option(
        "RUB",
        help="Rerun world ViewCoordinates label (RUB matches OpenXR Quest; "
        "try LUF if your file follows SpatialMP4's documented "
        "X-left/Y-up/Z-forward convention).",
    ),
    camera_coord: str = typer.Option(
        "RDF",
        help="Rerun camera ViewCoordinates label (RDF = OpenCV image axes).",
    ),
    no_floor: bool = typer.Option(False, help="Disable floor grid."),
    spawn: bool = typer.Option(True, help="Spawn the rerun viewer."),
    save_rrd: Optional[str] = typer.Option(
        None, help="Optional path to save an .rrd file instead of streaming."
    ),
):
    """Visualize a SpatialMP4 file in Rerun with all available timed tracks."""

    reader = sm.Reader(video_file)

    if depth_only:
        reader.set_read_mode(sm.ReadMode.DEPTH_ONLY)
    elif rgb_only:
        reader.set_read_mode(sm.ReadMode.RGB_ONLY)
    else:
        # We *won't* call load_rgbd / load_both; we manually iterate depth
        # frames and look up the matching RGB frame + head pose ourselves.
        reader.set_read_mode(sm.ReadMode.DEPTH_ONLY)

    if not rgb_only and not reader.has_depth():
        typer.echo(typer.style("No depth track found, switching to RGB_ONLY", fg=typer.colors.YELLOW))
        rgb_only = True
        reader.set_read_mode(sm.ReadMode.RGB_ONLY)

    tracks = collect_tracks(reader)
    typer.echo("Timed-metadata tracks:")
    for kind, ids in tracks.items():
        typer.echo(f"  {kind}: {ids}")

    # ----- head pose track ---------------------------------------------------
    head_lookup = PoseLookup(
        reader.get_rigid_pose_frames("head")
        if "head" in tracks["rigid_pose"]
        else reader.get_pose_frames()
    )
    if head_lookup.empty():
        typer.echo(typer.style("No head pose data found in input file", fg=typer.colors.RED))
        raise typer.Exit(code=1)
    typer.echo(f"Head pose samples: {len(head_lookup)}")

    blueprint = rrb.Horizontal(
        rrb.Vertical(
            # World 3D view: head trajectory + cameras + hands in absolute
            # OpenXR coords. DepthImage is logged outside `world` (under
            # `depth2d/`) so rerun's automatic Pinhole+DepthImage backproject
            # never fires here.
            rrb.Spatial3DView(name="3D World", origin="world"),
            rrb.TextLogView(name="Controller Input", origin="controller_input"),
            row_shares=[7, 3],
        ),
        rrb.Vertical(
            rrb.Tabs(
                rrb.Spatial2DView(
                    name="RGB",
                    origin="world/camera/image",
                    contents="world/camera/image/rgb",
                ),
                rrb.Spatial2DView(
                    name="Depth",
                    origin="depth2d",
                    contents="depth2d/depth",
                ),
                # Pre-blended RGB + JET-colored depth (logged as a regular
                # Image so we don't trigger rerun's automatic 3D Pinhole
                # backproject again).
                rrb.Spatial2DView(
                    name="RGB+Depth",
                    origin="rgb_depth_overlay",
                    contents="rgb_depth_overlay/image",
                ),
            ),
            # Hand motion relative to the head (head pose subtracted out).
            rrb.Spatial3DView(name="3D Hand (head-relative)", origin="head_relative"),
            rrb.TimeSeriesView(name="Inputs", origin="plots"),
            name="2D + head-relative",
            row_shares=[3, 3, 2],
        ),
        column_shares=[2, 1],
    )

    if save_rrd is not None:
        rr.init(f"spatialmp4_{os.path.basename(video_file)}", spawn=False)
        rr.save(save_rrd)
    else:
        rr.init(f"spatialmp4_{os.path.basename(video_file)}", spawn=spawn)
    rr.send_blueprint(blueprint)

    # ---- Coordinate system -------------------------------------------------
    # Default: captures follow OpenXR Quest convention — world is X-right /
    # Y-up / Z-back-out-of-page (RUB) and the camera local frame from the rgb
    # extrinsics is OpenCV-style X-right / Y-down / Z-forward (RDF).
    try:
        world_view = getattr(rr.ViewCoordinates, world_coord.upper())
    except AttributeError as exc:
        raise typer.BadParameter(f"unknown world ViewCoordinates: {world_coord}") from exc
    try:
        camera_xyz = getattr(rr.ViewCoordinates, camera_coord.upper())
    except AttributeError as exc:
        raise typer.BadParameter(f"unknown camera ViewCoordinates: {camera_coord}") from exc

    rr.log("world", world_view, static=True)
    rr.log(
        "world/xyz",
        rr.Arrows3D(
            vectors=[[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]],
            colors=[[255, 60, 60], [60, 255, 60], [60, 60, 255]],
            labels=["X+", "Y+", "Z+"],
        ),
        static=True,
    )

    # Floor grid keyed off the head trajectory extents.
    if not no_floor:
        traj = head_lookup.trajectory_xyz()
        max_xz = float(np.max(np.abs(np.concatenate([traj[:, 0], traj[:, 2]]))))
        log_floor_grid(
            half_extent_x=max(max_xz, 2.0),
            half_extent_z=max(max_xz, 2.0),
            y=float(traj[:, 1].min()) - 1.2,  # roughly floor height
            step=1.0,
        )

    # ---- Cameras / intrinsics ---------------------------------------------
    rgb_w, rgb_h = reader.get_rgb_width(), reader.get_rgb_height()
    K_rgb = reader.get_rgb_intrinsics_left()
    T_I_Srgb = reader.get_rgb_extrinsics_left().as_se3()

    depth_w, depth_h = reader.get_depth_width(), reader.get_depth_height()
    K_d_raw = reader.get_depth_intrinsics()
    T_I_Sd = reader.get_depth_extrinsics().as_se3()

    # Per-frame depth metadata from the spool sidecar — gives us the
    # OpenXR ``inverse_projection_view`` matrix, ``near_z``, ``far_z`` and
    # ``local_from_depth_eye`` that the Godot live writer used to render
    # the depth. The inverse_projection_view matrix is the cleanest route
    # to world coordinates: it encodes the exact pinhole + Y-direction +
    # pose used at capture time, bypassing every guess about row order,
    # principal-point convention or sensor mount tilt.
    # Sidecar Camera2 calibration → proper Camera2-to-Unity head-from-rgb
    # transform. Reproduces what ``godot_depth_rgb_align._openquest_head_from_camera``
    # builds. The mp4's own ``rgb_extrinsics`` descriptor skips this
    # conversion, so the depth↔rgb alignment computed from it is biased by
    # ~10° about the X axis. When the sidecar is present we use it; otherwise
    # we fall back to mp4's value.
    sidecar_rgb_calib = load_sidecar_camera_calibration(video_file, "left")
    if sidecar_rgb_calib is not None:
        typer.echo(
            typer.style(
                "Loaded Camera2 lens_pose from "
                "left_camera_characteristics.json; using its "
                "_openquest_head_from_camera-style conversion for the rgb "
                "camera pose (replaces mp4's rgb_extrinsics).",
                fg=typer.colors.GREEN,
            )
        )

    sidecar_metadata = load_sidecar_depth_metadata(video_file)
    if sidecar_metadata:
        typer.echo(
            typer.style(
                f"Loaded {len(sidecar_metadata)} per-frame depth metadata "
                "records from depth/frames.jsonl — will unproject depth via "
                "OpenXR depth_inverse_projection_view for the RGB overlay "
                "and the 3D point cloud.",
                fg=typer.colors.GREEN,
            )
        )

    # Quest OpenXR Environment Depth ships row 0 at the bottom of the image
    # (OpenGL texture convention). The inverse_projection_view handles this
    # natively, but the 2D Depth tab + rerun's Pinhole+DepthImage
    # auto-projection expect row 0 at the top, so we flip + mirror cy only
    # for those visualization paths.
    flip_depth_y = True
    K_d = sm.CameraIntrinsics()
    K_d.fx = K_d_raw.fx
    K_d.fy = K_d_raw.fy
    K_d.cx = K_d_raw.cx
    K_d.cy = (depth_h - 1) - K_d_raw.cy if flip_depth_y else K_d_raw.cy

    # ---- Depth extrinsic sanity check / fallback ---------------------------
    # The live writer ships depth_extrinsics = identity for these captures
    # (writer-side bug; OpenXR Environment Depth provides a per-frame
    # local_from_depth_eye but it's not being written into the mp4
    # depth-extrinsic descriptor). Without that, we substitute the rgb
    # extrinsic's ROTATION (which carries the IMU->OpenCV camera axis flip
    # + the ~10° mount tilt) but ZERO TRANSLATION so the depth sits at the
    # IMU/cyclopean origin — closer to the actual OpenXR depth-eye position
    # than the rgb lens, which is offset ~6 cm forward of the IMU.
    if (
        not rgb_only
        and np.allclose(T_I_Sd[:3, :3], np.eye(3))
        and np.allclose(T_I_Sd[:3, 3], 0.0)
    ):
        T_I_Sd = T_I_Srgb.copy()
        T_I_Sd[:3, 3] = 0.0  # depth eye at IMU, not at the rgb lens
        typer.echo(
            typer.style(
                "Detected identity depth_extrinsics; using rgb-extrinsics "
                "rotation with zero translation so depth eye sits at IMU "
                "(closer to the actual OpenXR depth-eye location than the "
                "rgb lens). Proper alignment requires the writer to record "
                "the per-frame local_from_depth_eye pose.",
                fg=typer.colors.YELLOW,
            )
        )

    typer.echo(
        f"RGB {rgb_w}x{rgb_h} @ {reader.get_rgb_fps():.1f}fps  "
        f"Depth {depth_w}x{depth_h} @ {reader.get_depth_fps():.1f}fps  "
        f"duration={reader.get_duration():.2f}s"
    )

    # ---- Static intrinsics on both pinholes -------------------------------
    rr.log(
        "world/camera/image",
        rr.Pinhole(
            resolution=[rgb_w, rgb_h],
            focal_length=[float(K_rgb.fx), float(K_rgb.fy)],
            principal_point=[float(K_rgb.cx), float(K_rgb.cy)],
            camera_xyz=camera_xyz,
        ),
        static=True,
    )
    if not rgb_only:
        rr.log(
            "world/depth_camera/image",
            rr.Pinhole(
                resolution=[depth_w, depth_h],
                focal_length=[float(K_d.fx), float(K_d.fy)],
                principal_point=[float(K_d.cx), float(K_d.cy)],
                camera_xyz=camera_xyz,
            ),
            static=True,
        )
        # Separate non-Pinhole image entity for the 2D Depth tab — avoids the
        # automatic 3D backprojection that double-flips with our T_I_Sd
        # compensation.
        # (Pinhole is not logged here, so no auto-projection occurs.)

    # ---- Timed-metadata tracks --------------------------------------------
    log_head_pose_track(head_lookup)
    log_all_rigid_pose_tracks(reader, tracks["rigid_pose"])
    log_all_hand_tracks(reader, tracks["hand_joints"])
    log_all_hand_tracks_head_relative(reader, tracks["hand_joints"], head_lookup)
    log_all_controller_input_tracks(reader, tracks["controller_input"])

    # Static origin / axes for the head-relative view (head sits at origin,
    # looking down -Z per OpenXR convention).
    rr.log("head_relative", rr.ViewCoordinates.RUB, static=True)
    rr.log(
        "head_relative/origin",
        rr.Arrows3D(
            origins=[[0, 0, 0]] * 4,
            vectors=[
                [0.2, 0, 0],   # +X (right)
                [0, 0.2, 0],   # +Y (up)
                [0, 0, 0.2],   # +Z (back)
                [0, 0, -0.3],  # head forward (-Z)
            ],
            colors=[[255, 60, 60], [60, 255, 60], [60, 60, 255], [255, 80, 200]],
            labels=["X", "Y", "Z", "gaze"],
        ),
        static=True,
    )
    rr.log(
        "head_relative/head_marker",
        rr.Points3D(
            positions=[[0, 0, 0]], colors=[[255, 255, 255]], radii=[0.04]
        ),
        static=True,
    )

    # ---- Depth frames ------------------------------------------------------
    # Pass 1: log depth-only entities (depth camera frustum, native 2D depth,
    # 3D point cloud) AND cache (timestamp, depth_320x320, T_W_Sd) so the
    # RGB pass can blend a depth overlay onto each RGB frame with the proper
    # per-frame depth-eye pose.
    sidecar_poses = load_sidecar_depth_poses(video_file)
    if sidecar_poses:
        typer.echo(
            typer.style(
                f"Loaded {len(sidecar_poses)} per-frame depth poses from "
                "depth/frames.jsonl sidecar — using these instead of the "
                "rgb-extrinsic proxy for the depth camera transform.",
                fg=typer.colors.GREEN,
            )
        )

    # The sidecar's ``local_from_depth_eye`` expects depth-cam-local points
    # in Quest3 / OpenXR RUB convention (X-right, Y-up, Z-back-out-of-page).
    # Our backprojection produces OpenCV-style points (X-right, Y-down,
    # Z-forward). The bridge is a Y AND Z flip = diag(1, -1, -1), the same
    # convention the rgb extrinsics already use to map OpenCV-RGB-cam to IMU.
    OPENCV_TO_DEPTHEYE = np.diag([1.0, -1.0, -1.0, 1.0])

    # Cache (ts, depth_raw_for_ipv, T_W_Sd, ipv_matrix, near_z, far_z) per
    # depth frame. depth_raw_for_ipv is the unflipped native depth so the
    # ipv-based unprojection (used for the RGB overlay AND the explicit 3D
    # point cloud) works correctly.
    depth_cache: List[Tuple[float, np.ndarray, np.ndarray, Optional[np.ndarray], float, Optional[float]]] = []
    if not rgb_only:
        reader.set_read_mode(sm.ReadMode.DEPTH_ONLY)
        reader.reset()
        processed = 0
        depth_idx = -1
        while reader.has_next():
            df = reader.load_depth()
            depth_idx += 1
            ts = df.timestamp
            depth_raw = df.depth.copy()  # unflipped, native row order

            T_W_Sd: Optional[np.ndarray] = None
            if depth_idx < len(sidecar_poses) and sidecar_poses[depth_idx] is not None:
                T_W_Sd = sidecar_poses[depth_idx] @ OPENCV_TO_DEPTHEYE
            else:
                head_pose = head_lookup.nearest(ts)
                if head_pose is None:
                    continue
                T_W_H = pose_frame_to_matrix(head_pose)
                T_W_Sd = T_W_H @ T_I_Sd
            T_W_Sd[:3, :3] = ensure_right_handed_rotation(T_W_Sd[:3, :3])

            depth_raw[(depth_raw < depth_min) | (depth_raw > depth_max)] = 0
            if depth_edge_filter:
                depth_uint16 = (depth_raw * 1000).astype(np.uint16)
                sobelx = cv2.Sobel(depth_uint16, cv2.CV_32F, 1, 0, ksize=3)
                sobely = cv2.Sobel(depth_uint16, cv2.CV_32F, 0, 1, ksize=3)
                grad_mag = np.sqrt(sobelx ** 2 + sobely ** 2)
                depth_raw[grad_mag > 500] = 0

            # Extract inverse_projection_view + near/far from sidecar md
            ipv = None
            near_z = depth_min
            far_z: Optional[float] = None
            if depth_idx < len(sidecar_metadata) and sidecar_metadata[depth_idx] is not None:
                md = sidecar_metadata[depth_idx] or {}
                ipv_cols = md.get("depth_inverse_projection_view_columns")
                if isinstance(ipv_cols, list) and len(ipv_cols) == 4:
                    ipv = np.asarray(ipv_cols, dtype=np.float64).T
                near_z = float(md.get("near_z", depth_min))
                far_z_val = md.get("far_z")
                far_z = float(far_z_val) if far_z_val is not None else None

            # Depth flipped for rerun's 2D Depth tab (visual orientation).
            depth_for_2d = depth_raw[::-1, :].copy() if flip_depth_y else depth_raw

            depth_cache.append(
                (ts, depth_raw, T_W_Sd, ipv, near_z, far_z)
            )

            set_time_seconds("time", ts)
            log_transform3d(
                "world/depth_camera",
                T_W_Sd[:3, 3],
                Rotation.from_matrix(T_W_Sd[:3, :3]).as_quat(),
                axis_len=0.0,
            )
            # 2D Depth tab: log the Y-flipped depth on a path that has NO
            # Pinhole parent, so rerun does NOT auto-backproject it into the
            # 3D world view (that auto-projection uses the camera_xyz +
            # Transform3D chain which doesn't reproduce the OpenXR
            # projection, and would conflict with our explicit point cloud).
            try:
                rr.log(
                    "depth2d/depth",
                    rr.DepthImage(
                        depth_for_2d,
                        meter=1.0,
                        depth_range=(depth_min, depth_max),
                    ),
                )
            except TypeError:
                rr.log(
                    "depth2d/depth",
                    rr.DepthImage(depth_for_2d, meter=1.0),
                )

            # 3D world point cloud: unproject via the per-frame OpenXR
            # inverse_projection_view matrix when available, falling back to
            # the rgb-extrinsic + standard pinhole pipeline otherwise.
            if ipv is not None:
                pts_world = unproject_depth_via_ipv(
                    depth_raw, ipv, near_z, far_z, depth_min, depth_max
                )
            else:
                pts_cam = project_depth_to_points(depth_raw, K_d, stride=depth_pc_stride)
                pts_world = (T_W_Sd[:3, :3] @ pts_cam.T).T + T_W_Sd[:3, 3]
            if pts_world.size:
                # Stride along the flat point list to control density.
                if depth_pc_stride > 1:
                    pts_world = pts_world[::depth_pc_stride]
                # Colour by camera-local Z (distance from depth eye) so the
                # gradient reads naturally as "blue=near, red=far".
                T_Sd_W = np.linalg.inv(T_W_Sd)
                z_local = (T_Sd_W[:3, :3] @ pts_world.T).T[:, 2] + T_Sd_W[2, 3]
                colors = depth_colormap_jet(np.abs(z_local), depth_min, depth_max)
                rr.log(
                    "world/depth_pointcloud",
                    rr.Points3D(positions=pts_world, colors=colors, radii=0.012),
                )

            processed += 1
            if topk is not None and processed >= topk:
                break

    depth_ts = np.array([entry[0] for entry in depth_cache], dtype=np.float64)

    # ---- RGB frames -------------------------------------------------------
    if not depth_only:
        reader.set_read_mode(sm.ReadMode.RGB_ONLY)
        reader.reset()
        processed = 0
        last_overlay_idx = -1
        while reader.has_next():
            frame_rgb = reader.load_rgb()
            ts = frame_rgb.timestamp
            head_pose = head_lookup.nearest(ts)
            if head_pose is None:
                continue
            T_W_H = pose_frame_to_matrix(head_pose)
            T_W_Srgb = T_W_H @ T_I_Srgb
            T_W_Srgb[:3, :3] = ensure_right_handed_rotation(T_W_Srgb[:3, :3])

            set_time_seconds("time", ts)
            log_transform3d(
                "world/camera",
                T_W_Srgb[:3, 3],
                Rotation.from_matrix(T_W_Srgb[:3, :3]).as_quat(),
                axis_len=0.0,
            )
            rgb_bgr = frame_rgb.left_rgb
            rr.log(
                "world/camera/image/rgb",
                rr.Image(rgb_bgr, color_model="BGR").compress(jpeg_quality=85),
            )

            # Find nearest cached depth frame and composite a colored
            # depth overlay onto the RGB image (logged as a plain Image so
            # no automatic 3D backprojection fires). Use a tighter
            # visualization range than the (potentially wide) depth filter
            # so close objects don't all collapse to black-blue.
            if depth_ts.size:
                idx = int(np.argmin(np.abs(depth_ts - ts)))
                # Only emit one overlay per unique depth frame (depth runs at
                # ~24 fps while RGB is 50 fps; logging an overlay per RGB
                # frame would 2x the .rrd size for zero new information).
                if (
                    idx != last_overlay_idx
                    and abs(depth_ts[idx] - ts) < 0.1
                ):
                    last_overlay_idx = idx
                    (
                        _depth_ts_entry,
                        depth_native,
                        T_W_Sd_for_warp,
                        ipv_entry,
                        near_z_entry,
                        far_z_entry,
                    ) = depth_cache[idx]
                    if ipv_entry is not None:
                        # Reference path: unproject directly to world via the
                        # per-frame OpenXR inverse_projection_view matrix
                        # (Godot world), convert to Unity, then project into
                        # the RGB camera via the Camera2-→-Unity composed
                        # pose. Matches godot_depth_rgb_align.py exactly.
                        pts_world_godot = unproject_depth_via_ipv(
                            depth_native,
                            ipv_entry,
                            near_z_entry,
                            far_z_entry,
                            depth_min,
                            depth_max,
                        )
                        if sidecar_rgb_calib is not None:
                            head_pose_at_rgb = head_lookup.nearest(ts)
                            T_W_H_godot = pose_frame_to_matrix(head_pose_at_rgb)
                            T_W_H_unity = godot_transform_to_unity(T_W_H_godot)
                            T_W_Srgb_unity = (
                                T_W_H_unity
                                @ sidecar_rgb_calib["head_from_camera_unity"]
                            )
                            pts_unity = (
                                UNITY_FROM_GODOT_3 @ pts_world_godot.T
                            ).T
                            depth_on_rgb = project_pts_unity_to_rgb(
                                pts_unity,
                                T_W_Srgb_unity,
                                sidecar_rgb_calib["K_rgb"],
                                rgb_w,
                                rgb_h,
                            )
                        else:
                            depth_on_rgb = project_world_to_rgb(
                                pts_world_godot, T_W_Srgb, K_rgb, rgb_w, rgb_h
                            )
                    else:
                        T_rgb_d = np.linalg.inv(T_W_Srgb) @ T_W_Sd_for_warp
                        depth_on_rgb = warp_depth_to_rgb(
                            depth_native, K_d, K_rgb, T_rgb_d, rgb_w, rgb_h
                        )
                    overlay = blend_depth_on_rgb(
                        rgb_bgr,
                        depth_on_rgb,
                        depth_min,
                        depth_max,
                        alpha=0.7,
                    )
                    rr.log(
                        "rgb_depth_overlay/image",
                        rr.Image(overlay, color_model="BGR").compress(
                            jpeg_quality=70
                        ),
                    )

            processed += 1
            if topk is not None and processed >= topk:
                break

    if save_rrd is not None:
        typer.echo(f"Saved rerun recording to {save_rrd}")


if __name__ == "__main__":
    typer.run(main)
