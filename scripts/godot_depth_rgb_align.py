#!/usr/bin/env python3
"""Project Godot Quest Environment Depth samples into a Camera2 RGB frame.

The capture streams do not share an image plane: Environment Depth is rendered
from an OpenXR depth-eye view while RGB is produced by a physical Camera2 lens.
This utility reconstructs local-space points from the depth view, transforms
them into the selected RGB lens view, and z-buffers them in RGB pixel space.
"""

import argparse
import bisect
import json
import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


class AlignmentError(RuntimeError):
    pass


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    rotation_xyzw: np.ndarray


@dataclass(frozen=True)
class DepthFrame:
    timestamp_ns: int
    path: Path
    width: int
    height: int
    metadata: Dict[str, Any]


@dataclass(frozen=True)
class RgbFrame:
    timestamp_ns: int
    camera_sensor_timestamp_ns: Optional[int]
    path: Path
    width: int
    height: int
    planes: List[Dict[str, Any]]


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    intrinsic: np.ndarray
    head_from_camera_unity: np.ndarray
    distortion: np.ndarray


UNITY_FROM_GODOT = np.diag([1.0, 1.0, -1.0])


def _load_head_poses(session_dir: Path) -> List[Dict[str, Any]]:
    """Resolve head-pose records for the session.

    Prefers the legacy `poses/head.jsonl` sidecar when present (older
    captures and runs with `save_head_pose_sidecar=true`). Otherwise reads
    the mp4 `mett:head` rigid_pose track directly via read_mett_pose; the
    session's `manifest.json` carries the Godot-ticks-µs anchor we need to
    fold mp4 PTS back into the absolute domain `interpolate_head_pose`
    expects (matching what head.jsonl used to do bit-for-bit).
    """
    sidecar = session_dir / "poses" / "head.jsonl"
    if sidecar.exists():
        return _read_jsonl(sidecar)

    # Locate the mp4 the session was packed into. The Godot capture app writes
    # both <session_dir>/<id>.mp4 and <parent>/<id>.mp4; check both spots.
    session_id = session_dir.name
    candidates = [
        session_dir.parent / f"{session_id}.mp4",
        session_dir / f"{session_id}.mp4",
    ]
    mp4_path = next((p for p in candidates if p.is_file()), None)
    if mp4_path is None:
        raise AlignmentError(
            f"No head.jsonl sidecar and no <session>/<id>.mp4 found at "
            f"any of: {[str(p) for p in candidates]}"
        )

    manifest_path = session_dir / "manifest.json"
    session_start_us = 0
    if manifest_path.exists():
        try:
            manifest = json.loads(manifest_path.read_text())
            session_start_us = int(manifest.get("session_start_ticks_us", 0))
        except (json.JSONDecodeError, ValueError):
            session_start_us = 0

    # Import lazily so installations that don't keep read_mett_pose.py adjacent
    # still trigger a sensible error message at first use.
    try:
        from read_mett_pose import read_head_poses  # type: ignore
    except ImportError as exc:  # pragma: no cover
        raise AlignmentError(
            "Cannot resolve head pose: head.jsonl is missing and "
            "read_mett_pose.py (SpatialMP4/scripts) is not importable: %s" % exc
        )
    return read_head_poses(mp4_path, session_start_godot_ticks_us=session_start_us)


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        raise AlignmentError("Missing required input: {}".format(path))
    lines = path.read_text(encoding="utf-8").splitlines()
    records = []
    for index, line in enumerate(lines):
            if line.strip():
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError as error:
                    if index == len(lines) - 1:
                        break
                    raise AlignmentError("Malformed JSONL record in {}: {}".format(path, error)) from error
    return records


def _normalize_quaternion(q: Sequence[float]) -> np.ndarray:
    value = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(value)
    if not np.isfinite(norm) or norm <= 0.0:
        raise AlignmentError("Invalid zero or non-finite quaternion")
    return value / norm


def quaternion_to_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = _normalize_quaternion(q)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quaternion_slerp(first: Sequence[float], second: Sequence[float], alpha: float) -> np.ndarray:
    q0 = _normalize_quaternion(first)
    q1 = _normalize_quaternion(second)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return _normalize_quaternion(q0 + alpha * (q1 - q0))
    theta_0 = math.acos(min(1.0, max(-1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    return (
        math.sin((1.0 - alpha) * theta_0) / sin_theta_0 * q0
        + math.sin(alpha * theta_0) / sin_theta_0 * q1
    )


def pose_matrix(pose: Pose) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_matrix(pose.rotation_xyzw)
    matrix[:3, 3] = pose.position
    return matrix


def godot_transform_to_unity(transform: np.ndarray) -> np.ndarray:
    conversion = np.eye(4, dtype=np.float64)
    conversion[:3, :3] = UNITY_FROM_GODOT
    return conversion @ transform @ conversion


def _pose_from_record(record: Dict[str, Any]) -> Pose:
    position = record["position"]
    rotation = record["rotation"]
    return Pose(
        position=np.array([position["x"], position["y"], position["z"]], dtype=np.float64),
        rotation_xyzw=np.array(
            [rotation["x"], rotation["y"], rotation["z"], rotation["w"]], dtype=np.float64
        ),
    )


def interpolate_head_pose(records: List[Dict[str, Any]], timestamp_ns: int, max_gap_ns: int) -> Pose:
    valid = [record for record in records if record.get("tracking_valid", True)]
    if not valid:
        raise AlignmentError("No tracked head poses are available")
    valid.sort(key=lambda value: int(value["timestamp_ns"]))
    times = [int(value["timestamp_ns"]) for value in valid]
    index = bisect.bisect_left(times, timestamp_ns)
    if index < len(times) and times[index] == timestamp_ns:
        return _pose_from_record(valid[index])
    if index == 0 or index == len(times):
        raise AlignmentError("RGB timestamp is outside the recorded head-pose interval")
    before = valid[index - 1]
    after = valid[index]
    t0 = int(before["timestamp_ns"])
    t1 = int(after["timestamp_ns"])
    if timestamp_ns - t0 > max_gap_ns or t1 - timestamp_ns > max_gap_ns:
        raise AlignmentError("No sufficiently close head pose bracketing the RGB frame")
    alpha = float(timestamp_ns - t0) / float(t1 - t0)
    pose0 = _pose_from_record(before)
    pose1 = _pose_from_record(after)
    return Pose(
        position=(1.0 - alpha) * pose0.position + alpha * pose1.position,
        rotation_xyzw=quaternion_slerp(pose0.rotation_xyzw, pose1.rotation_xyzw, alpha),
    )


def _camera_dimensions(metadata: Dict[str, Any]) -> Tuple[int, int]:
    size = metadata.get("sensor_active_array_size") or metadata.get("sensor", {}).get("activeArraySize")
    if not size:
        size = metadata.get("sensor_pixel_array_size") or metadata.get("sensor", {}).get("pixelArraySize")
    if not size:
        raise AlignmentError("Camera metadata does not include an active or pixel array size")
    if "right" in size:
        return int(size["right"]) - int(size.get("left", 0)), int(size["bottom"]) - int(size.get("top", 0))
    return int(size["width"]), int(size["height"])


def _camera_intrinsic(metadata: Dict[str, Any]) -> np.ndarray:
    if isinstance(metadata.get("intrinsics"), dict):
        intrinsics = metadata["intrinsics"]
        values = [intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"]]
    else:
        values = metadata.get("lens_intrinsic_calibration") or []
    if len(values) < 4:
        raise AlignmentError("Camera metadata does not include calibrated intrinsics")
    fx, fy, cx, cy = [float(value) for value in values[:4]]
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


def _openquest_head_from_camera(metadata: Dict[str, Any]) -> np.ndarray:
    """Port metaquest-3d-reconstruction's Camera2-to-Unity pose conversion."""
    if isinstance(metadata.get("pose"), dict):
        translation = metadata["pose"].get("translation") or [0.0, 0.0, 0.0]
        raw_rotation = metadata["pose"].get("rotation") or [0.0, 0.0, 0.0, 1.0]
    else:
        translation = metadata.get("lens_pose_translation") or [0.0, 0.0, 0.0]
        raw_rotation = metadata.get("lens_pose_rotation") or [0.0, 0.0, 0.0, 1.0]
    translation_unity = np.array([translation[0], translation[1], -translation[2]], dtype=np.float64)
    converted = [-raw_rotation[0], -raw_rotation[1], raw_rotation[2], raw_rotation[3]]
    rotation_unity = quaternion_to_matrix(converted).T @ np.diag([1.0, -1.0, -1.0])
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation_unity
    matrix[:3, 3] = translation_unity
    return matrix


def load_camera_calibration(session_dir: Path, eye: str, ignore_distortion: bool) -> CameraCalibration:
    path = session_dir / "{}_camera_characteristics.json".format(eye)
    if not path.exists():
        raise AlignmentError("Missing RGB camera calibration: {}".format(path))
    metadata = json.loads(path.read_text(encoding="utf-8"))
    distortion = np.asarray(metadata.get("lens_distortion", metadata.get("distortion", [])), dtype=np.float64)
    if distortion.size and np.any(np.abs(distortion) > 1e-10) and not ignore_distortion:
        raise AlignmentError(
            "RGB lens distortion is nonzero. The Camera2 model must be explicitly calibrated; "
            "rerun with --ignore-distortion only for diagnostic output."
        )
    width, height = _camera_dimensions(metadata)
    return CameraCalibration(
        width=width,
        height=height,
        intrinsic=_camera_intrinsic(metadata),
        head_from_camera_unity=_openquest_head_from_camera(metadata),
        distortion=distortion,
    )


def load_depth_frames(session_dir: Path, eye: str) -> List[DepthFrame]:
    index_path = session_dir / "depth" / "frames.jsonl"
    if not index_path.exists():
        index_path = session_dir / "depth_frames.jsonl"
    frames = []
    for record in _read_jsonl(index_path):
        if record.get("eye") != eye:
            continue
        path = Path(record["image_path"])
        if not path.is_absolute():
            path = session_dir / path
        frames.append(
            DepthFrame(
                timestamp_ns=int(record["timestamp_ns"]),
                path=path,
                width=int(record["width"]),
                height=int(record["height"]),
                metadata=record.get("metadata") or {},
            )
        )
    if not frames:
        raise AlignmentError("No {} depth frames were found".format(eye))
    return sorted(frames, key=lambda frame: frame.timestamp_ns)


def load_rgb_frames(session_dir: Path, eye: str) -> List[RgbFrame]:
    frames = []
    for record in _read_jsonl(session_dir / "{}_camera_frames.jsonl".format(eye)):
        path = session_dir / record["raw_path"]
        frames.append(
            RgbFrame(
                timestamp_ns=int(record["timestamp_ns"]),
                camera_sensor_timestamp_ns=(
                    int(record["camera_sensor_timestamp_ns"])
                    if record.get("camera_sensor_timestamp_ns") is not None
                    else None
                ),
                path=path,
                width=int(record["width"]),
                height=int(record["height"]),
                planes=record.get("planes") or [],
            )
        )
    if not frames:
        raise AlignmentError("No {} RGB frames were found".format(eye))
    return sorted(frames, key=lambda frame: frame.timestamp_ns)


def nearest_rgb_frame(
    frames: List[RgbFrame], depth_frame: DepthFrame, max_delta_ns: int
) -> Tuple[RgbFrame, str, int]:
    runtime_timestamp = depth_frame.metadata.get("runtime_display_time_ns")
    sensor_frames = [frame for frame in frames if frame.camera_sensor_timestamp_ns is not None]
    if runtime_timestamp is not None and sensor_frames:
        timestamp_ns = int(runtime_timestamp)
        selected = min(sensor_frames, key=lambda frame: abs(int(frame.camera_sensor_timestamp_ns) - timestamp_ns))
        delta = abs(int(selected.camera_sensor_timestamp_ns) - timestamp_ns)
        domain = "openxr_runtime_display_time_vs_camera_sensor_time"
    else:
        timestamp_ns = depth_frame.timestamp_ns
        selected = min(frames, key=lambda frame: abs(frame.timestamp_ns - timestamp_ns))
        delta = abs(selected.timestamp_ns - timestamp_ns)
        domain = "godot_callback_ticks"
    if delta > max_delta_ns:
        raise AlignmentError(
            "Nearest RGB frame differs from depth by {:.3f} ms, exceeding the allowed {:.3f} ms".format(
                delta / 1e6, max_delta_ns / 1e6
            )
        )
    return selected, domain, delta


def read_window_depth(frame: DepthFrame, storage_override: Optional[str] = None) -> np.ndarray:
    raw = frame.path.read_bytes()
    storage = storage_override or frame.metadata.get("sample_storage")
    if storage == "u16_unorm_le":
        expected = frame.width * frame.height * 2
        if len(raw) != expected:
            raise AlignmentError("Depth byte size does not match u16 image dimensions")
        return np.frombuffer(raw, dtype="<u2").reshape(frame.height, frame.width).astype(np.float64) / 65535.0
    if storage == "f16_le":
        dtype = "<f2"
    elif storage == "f32_le":
        dtype = "<f4"
    else:
        raise AlignmentError("Unsupported Environment Depth sample storage: {}".format(storage))
    expected = frame.width * frame.height * np.dtype(dtype).itemsize
    if len(raw) != expected:
        raise AlignmentError("Depth byte size does not match {} image dimensions".format(storage))
    return np.frombuffer(raw, dtype=dtype).reshape(frame.height, frame.width).astype(np.float64)


def linearize_window_depth(depth: np.ndarray, near_z: float, far_z: Optional[float]) -> np.ndarray:
    if not np.isfinite(near_z) or near_z <= 0.0:
        raise AlignmentError("Invalid Environment Depth near plane")
    if far_z is None or not np.isfinite(far_z) or far_z < near_z:
        x, y = -2.0 * near_z, -1.0
    elif far_z == near_z:
        raise AlignmentError("Environment Depth near/far planes are equal")
    else:
        x = -2.0 * far_z * near_z / (far_z - near_z)
        y = -(far_z + near_z) / (far_z - near_z)
    denominator = depth * 2.0 - 1.0 + y
    return np.divide(x, denominator, out=np.full_like(depth, np.inf), where=denominator != 0.0)


def _local_from_depth_eye_unity(metadata: Dict[str, Any]) -> np.ndarray:
    raw_pose = metadata.get("local_from_depth_eye")
    if not isinstance(raw_pose, dict):
        raise AlignmentError("Depth frame is missing local_from_depth_eye")
    return godot_transform_to_unity(pose_matrix(_pose_from_record(raw_pose)))


def unproject_depth_to_unity(
    frame: DepthFrame,
    window_depth: np.ndarray,
    min_depth_m: float,
    max_depth_m: float,
    flip_depth_y: bool = False,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    metadata = frame.metadata
    rows, columns = np.indices((frame.height, frame.width), dtype=np.float64)
    sampled_rows = frame.height - 1.0 - rows if flip_depth_y else rows
    inverse_columns = metadata.get("depth_inverse_projection_view_columns")
    finite_sample = np.isfinite(window_depth) & (window_depth >= 0.0) & (window_depth <= 1.0)
    eye_from_local = _local_from_depth_eye_unity(metadata)
    eye_origin = eye_from_local[:3, 3]

    if isinstance(inverse_columns, list) and len(inverse_columns) == 4:
        inverse_projection_view = np.asarray(inverse_columns, dtype=np.float64).T
        clip = np.stack(
            [
                2.0 * ((columns + 0.5) / frame.width) - 1.0,
                2.0 * ((sampled_rows + 0.5) / frame.height) - 1.0,
                2.0 * window_depth - 1.0,
                np.ones_like(window_depth),
            ],
            axis=-1,
        ).reshape(-1, 4)
        local_h_godot = (inverse_projection_view @ clip.T).T
        valid_w = np.isfinite(local_h_godot[:, 3]) & (np.abs(local_h_godot[:, 3]) > 1e-12)
        local_godot = np.full((clip.shape[0], 3), np.nan, dtype=np.float64)
        local_godot[valid_w] = local_h_godot[valid_w, :3] / local_h_godot[valid_w, 3:4]
        local_unity = (UNITY_FROM_GODOT @ local_godot.T).T
        valid = finite_sample.reshape(-1) & valid_w & np.all(np.isfinite(local_unity), axis=1)
        distances = np.linalg.norm(local_unity - eye_origin, axis=1)
        valid &= (distances >= min_depth_m) & (distances <= max_depth_m)
        return local_unity[valid], {
            "unprojection": "depth_inverse_projection_view",
            "input_pixel_count": int(frame.width * frame.height),
            "valid_depth_point_count": int(np.count_nonzero(valid)),
        }

    fov = metadata.get("fov_tangent") or {}
    required = ("left", "right", "top", "bottom")
    if not all(key in fov for key in required):
        raise AlignmentError("Depth frame lacks inverse projection-view matrix and FOV fallback metadata")
    near_z = float(metadata["near_z"])
    far_value = metadata.get("far_z")
    far_z = float(far_value) if far_value is not None else None
    linear_z = linearize_window_depth(window_depth, near_z, far_z)
    fx = frame.width / (float(fov["left"]) + float(fov["right"]))
    fy = frame.height / (float(fov["top"]) + float(fov["bottom"]))
    cx = frame.width * float(fov["right"]) / (float(fov["left"]) + float(fov["right"]))
    cy = frame.height * float(fov["top"]) / (float(fov["top"]) + float(fov["bottom"]))
    camera_points = np.stack(
        [
            (columns + 0.5 - cx) * linear_z / fx,
            (cy - (sampled_rows + 0.5)) * linear_z / fy,
            linear_z,
        ],
        axis=-1,
    ).reshape(-1, 3)
    valid = finite_sample.reshape(-1) & np.all(np.isfinite(camera_points), axis=1)
    valid &= (linear_z.reshape(-1) >= min_depth_m) & (linear_z.reshape(-1) <= max_depth_m)
    depth_from_camera = eye_from_local
    local_unity = (depth_from_camera[:3, :3] @ camera_points.T).T + depth_from_camera[:3, 3]
    return local_unity[valid], {
        "unprojection": "fov_and_local_from_depth_eye",
        "input_pixel_count": int(frame.width * frame.height),
        "valid_depth_point_count": int(np.count_nonzero(valid)),
    }


def project_to_rgb(
    local_unity_points: np.ndarray,
    local_from_rgb_unity: np.ndarray,
    intrinsic: np.ndarray,
    width: int,
    height: int,
    splat_radius: int = 0,
) -> Tuple[np.ndarray, int]:
    rgb_from_local = np.linalg.inv(local_from_rgb_unity)
    points_h = np.concatenate([local_unity_points, np.ones((len(local_unity_points), 1))], axis=1)
    camera = (rgb_from_local @ points_h.T).T[:, :3]
    valid = np.all(np.isfinite(camera), axis=1) & (camera[:, 2] > 0.0)
    camera = camera[valid]
    if camera.size == 0:
        return np.zeros((height, width), dtype=np.float32), 0
    u = intrinsic[0, 0] * camera[:, 0] / camera[:, 2] + intrinsic[0, 2]
    v = intrinsic[1, 2] - intrinsic[1, 1] * camera[:, 1] / camera[:, 2]
    u = np.rint(u).astype(np.int64)
    v = np.rint(v).astype(np.int64)
    output = np.full((height, width), np.inf, dtype=np.float64)
    flat = output.ravel()
    projected = 0
    for dy in range(-splat_radius, splat_radius + 1):
        for dx in range(-splat_radius, splat_radius + 1):
            x = u + dx
            y = v + dy
            inside = (x >= 0) & (x < width) & (y >= 0) & (y < height)
            if dx == 0 and dy == 0:
                projected = int(np.count_nonzero(inside))
            indices = y[inside] * width + x[inside]
            np.minimum.at(flat, indices, camera[inside, 2])
    output[~np.isfinite(output)] = 0.0
    return output.astype(np.float32), projected


def _plane_array(raw: np.ndarray, plane: Dict[str, Any], width: int, height: int) -> np.ndarray:
    offset = int(plane["offset"])
    row_stride = int(plane["row_stride"])
    pixel_stride = int(plane["pixel_stride"])
    positions = (
        offset
        + np.arange(height, dtype=np.int64)[:, None] * row_stride
        + np.arange(width, dtype=np.int64)[None, :] * pixel_stride
    )
    if int(positions.max()) >= len(raw):
        raise AlignmentError("YUV plane metadata extends beyond the captured payload")
    return raw[positions]


def read_yuv420_rgb(frame: RgbFrame) -> np.ndarray:
    if len(frame.planes) != 3:
        raise AlignmentError("RGB frame does not include three YUV_420_888 plane descriptions")
    payload = np.frombuffer(frame.path.read_bytes(), dtype=np.uint8)
    y = _plane_array(payload, frame.planes[0], frame.width, frame.height).astype(np.float64)
    chroma_width = (frame.width + 1) // 2
    chroma_height = (frame.height + 1) // 2
    u = _plane_array(payload, frame.planes[1], chroma_width, chroma_height).astype(np.float64)
    v = _plane_array(payload, frame.planes[2], chroma_width, chroma_height).astype(np.float64)
    u = np.repeat(np.repeat(u, 2, axis=0), 2, axis=1)[: frame.height, : frame.width]
    v = np.repeat(np.repeat(v, 2, axis=0), 2, axis=1)[: frame.height, : frame.width]
    c = y - 16.0
    d = u - 128.0
    e = v - 128.0
    rgb = np.stack(
        [
            (298.082 * c + 408.583 * e) / 256.0,
            (298.082 * c - 100.291 * d - 208.120 * e) / 256.0,
            (298.082 * c + 516.412 * d) / 256.0,
        ],
        axis=-1,
    )
    return np.clip(np.rint(rgb), 0, 255).astype(np.uint8)


def depth_heatmap(depth: np.ndarray) -> np.ndarray:
    valid = depth > 0.0
    image = np.zeros(depth.shape + (3,), dtype=np.uint8)
    if not np.any(valid):
        return image
    low, high = np.percentile(depth[valid], [2, 98])
    if high <= low:
        high = low + 1.0
    t = np.clip((depth - low) / (high - low), 0.0, 1.0)
    anchors = np.array(
        [[20, 30, 120], [20, 180, 220], [60, 220, 80], [250, 210, 40], [220, 35, 25]],
        dtype=np.float64,
    )
    position = t * (len(anchors) - 1)
    left = np.minimum(position.astype(np.int64), len(anchors) - 2)
    fraction = position - left
    colors = anchors[left] * (1.0 - fraction[..., None]) + anchors[left + 1] * fraction[..., None]
    image[valid] = np.rint(colors[valid]).astype(np.uint8)
    return image


def render_panel(rgb: np.ndarray, aligned_depth: np.ndarray) -> np.ndarray:
    heatmap = depth_heatmap(aligned_depth)
    valid = aligned_depth > 0.0
    overlay = rgb.copy()
    overlay[valid] = np.rint(rgb[valid] * 0.45 + heatmap[valid] * 0.55).astype(np.uint8)
    return np.concatenate([rgb, heatmap, overlay], axis=1)


def write_png(path: Path, image: np.ndarray) -> None:
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise AlignmentError("PNG writer expects an HxWx3 uint8 image")
    height, width = image.shape[:2]

    def chunk(name: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + name + data + struct.pack(">I", zlib.crc32(name + data) & 0xFFFFFFFF)

    rows = b"".join(b"\0" + image[row].tobytes() for row in range(height))
    payload = b"\x89PNG\r\n\x1a\n"
    payload += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    payload += chunk(b"IDAT", zlib.compress(rows, level=6))
    payload += chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def align_frame(
    session_dir: Path,
    eye: str,
    output_png: Path,
    depth_index: int,
    max_pair_delta_ns: int,
    max_pose_gap_ns: int,
    min_depth_m: float,
    max_depth_m: float,
    splat_radius: int,
    flip_depth_y: bool,
    storage_override: Optional[str],
    ignore_distortion: bool,
) -> Dict[str, Any]:
    calibration = load_camera_calibration(session_dir, eye, ignore_distortion)
    depth_frames = load_depth_frames(session_dir, eye)
    if depth_index < 0 or depth_index >= len(depth_frames):
        raise AlignmentError("Depth index is outside the available frame range")
    depth_frame = depth_frames[depth_index]
    rgb_frame, pairing_domain, pair_delta_ns = nearest_rgb_frame(
        load_rgb_frames(session_dir, eye), depth_frame, max_pair_delta_ns
    )
    head_pose = interpolate_head_pose(
        _load_head_poses(session_dir), rgb_frame.timestamp_ns, max_pose_gap_ns
    )
    rgb = read_yuv420_rgb(rgb_frame)
    if rgb_frame.width != calibration.width or rgb_frame.height != calibration.height:
        raise AlignmentError("RGB capture dimensions differ from Camera2 calibration dimensions")
    window_depth = read_window_depth(depth_frame, storage_override)
    local_points, unprojection_report = unproject_depth_to_unity(
        depth_frame, window_depth, min_depth_m, max_depth_m, flip_depth_y
    )
    local_from_head_unity = godot_transform_to_unity(pose_matrix(head_pose))
    local_from_rgb_unity = local_from_head_unity @ calibration.head_from_camera_unity
    aligned_depth, projected_count = project_to_rgb(
        local_points,
        local_from_rgb_unity,
        calibration.intrinsic,
        calibration.width,
        calibration.height,
        splat_radius,
    )
    if projected_count == 0:
        raise AlignmentError(
            "No valid depth point projects into the RGB view. "
            "Verify that head pose is tracked in the same XR local space as Environment Depth."
        )
    write_png(output_png, render_panel(rgb, aligned_depth))
    npy_path = output_png.with_name(output_png.stem + "_aligned_depth.npy")
    np.save(npy_path, aligned_depth)
    report = {
        "session_dir": str(session_dir),
        "eye": eye,
        "depth_index": depth_index,
        "depth_timestamp_ns": depth_frame.timestamp_ns,
        "rgb_timestamp_ns": rgb_frame.timestamp_ns,
        "rgb_depth_delta_ms": pair_delta_ns / 1e6,
        "rgb_depth_pairing_domain": pairing_domain,
        "rgb_camera_sensor_timestamp_ns": rgb_frame.camera_sensor_timestamp_ns,
        "depth_timestamp_source": depth_frame.metadata.get("timestamp_source", ""),
        "depth_runtime_display_time_ns": depth_frame.metadata.get("runtime_display_time_ns"),
        "storage": storage_override or depth_frame.metadata.get("sample_storage"),
        "storage_overridden": storage_override is not None,
        "flip_depth_y": flip_depth_y,
        "min_depth_m": min_depth_m,
        "max_depth_m": max_depth_m,
        "splat_radius": splat_radius,
        "projected_center_pixel_count": projected_count,
        "output_nonzero_pixel_count": int(np.count_nonzero(aligned_depth)),
        "output_png": str(output_png),
        "output_aligned_depth_npy": str(npy_path),
        "camera_pose_convention": "OpenQuestCapture Camera2-to-Unity composed with Godot head pose",
        "distortion_ignored": bool(ignore_distortion and calibration.distortion.size),
        "timestamp_warning": (
            "Depth/RGB pairing fell back to async callback delivery time; capture raw runtime timestamps "
            "for motion-accurate matching."
            if pairing_domain == "godot_callback_ticks"
            else "Depth/RGB pairing uses runtime display and Camera2 sensor timestamps; RGB pose is still "
            "interpolated from game-thread head samples."
        ),
    }
    report.update(unprojection_report)
    report_path = output_png.with_name(output_png.stem + "_report.json")
    report["output_report_json"] = str(report_path)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="Output three-panel PNG: RGB, aligned depth, overlay.")
    parser.add_argument("--eye", choices=("left", "right"), default="left")
    parser.add_argument("--depth-index", type=int, default=0)
    parser.add_argument("--max-pair-delta-ms", type=float, default=50.0)
    parser.add_argument("--max-pose-gap-ms", type=float, default=50.0)
    parser.add_argument("--min-depth-m", type=float, default=0.2)
    parser.add_argument("--max-depth-m", type=float, default=20.0)
    parser.add_argument("--splat-radius", type=int, default=2)
    parser.add_argument("--flip-depth-y", action="store_true")
    parser.add_argument(
        "--sample-storage-override",
        choices=("u16_unorm_le", "f16_le", "f32_le"),
        help="Use only for older captures with incorrect depth sample_storage metadata.",
    )
    parser.add_argument(
        "--ignore-distortion",
        action="store_true",
        help="Permit diagnostic pinhole rendering when Camera2 reports nonzero distortion.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        report = align_frame(
            session_dir=args.session_dir,
            eye=args.eye,
            output_png=args.output,
            depth_index=args.depth_index,
            max_pair_delta_ns=int(args.max_pair_delta_ms * 1e6),
            max_pose_gap_ns=int(args.max_pose_gap_ms * 1e6),
            min_depth_m=args.min_depth_m,
            max_depth_m=args.max_depth_m,
            splat_radius=max(0, args.splat_radius),
            flip_depth_y=args.flip_depth_y,
            storage_override=args.sample_storage_override,
            ignore_distortion=args.ignore_distortion,
        )
    except (AlignmentError, OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        print("alignment failed: {}".format(error))
        return 1
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
