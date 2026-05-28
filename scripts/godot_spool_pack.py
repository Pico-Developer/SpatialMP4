#!/usr/bin/env python3
"""Pack a Godot Quest capture spool into SpatialMP4-ready packet inputs.

This does not mux the final MP4. It produces the exact stream payloads and
metadata a SpatialMP4 muxer needs:

- RGB left/right raw frame pairs with timestamp pairing metadata.
- Metric depth frames, or OpenXR window-depth frames with reconstruction
  metadata, converted to raw1-compatible uint16 millimeters.
- Head pose packets encoded as 7 little-endian doubles.
- Controller and hand pose sidecars copied through unchanged.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


TIME_BASE_HZ = 1_000_000
DEFAULT_PAIR_DELTA_NS = 10_000_000


class PackError(RuntimeError):
    pass


@dataclass(frozen=True)
class TimestampedFile:
    timestamp_ns: int
    path: Path
    metadata: Dict[str, Any]


@dataclass(frozen=True)
class Pair:
    timestamp_ns: int
    left: TimestampedFile
    right: TimestampedFile
    delta_ns: int


@dataclass(frozen=True)
class DepthFrame:
    timestamp_ns: int
    eye: str
    path: Path
    width: int
    height: int
    metadata: Dict[str, Any]


def pack_session(
    session_dir: Path,
    output_dir: Path,
    *,
    max_pair_delta_ns: int = DEFAULT_PAIR_DELTA_NS,
    depth_input_format: str = "auto",
    allow_partial: bool = False,
    make_zip: bool = False,
) -> Dict[str, Any]:
    session_dir = session_dir.resolve()
    output_dir = output_dir.resolve()

    if not session_dir.exists():
        raise PackError(f"Session directory does not exist: {session_dir}")
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)

    manifest = _read_json_if_exists(session_dir / "manifest.json") or {}
    session_start_unix_us = int(manifest.get("session_start_unix_us", 0))

    rgb_pairs = _pair_rgb_frames(session_dir, max_pair_delta_ns)
    depth_frames = _load_depth_frames(session_dir)
    camera_metadata = _load_camera_metadata(session_dir, depth_frames)
    head_records = _read_jsonl(session_dir / "poses" / "head.jsonl")
    controller_records = _read_jsonl(session_dir / "poses" / "controllers.jsonl")
    hand_records = _read_jsonl(session_dir / "poses" / "hands.jsonl")

    if not allow_partial:
        if not rgb_pairs:
            raise PackError("Missing required paired RGB frames in left_camera_raw/right_camera_raw")
        if not depth_frames:
            raise PackError("Missing required depth frames")
        if not head_records:
            raise PackError("Missing required head pose samples at poses/head.jsonl")
        if not controller_records and not hand_records:
            raise PackError("Missing required controller or hand pose samples")
    elif not head_records:
        raise PackError("Missing required head pose samples at poses/head.jsonl")

    rgb_report = _write_rgb_packet_inputs(output_dir, rgb_pairs)
    depth_report = _write_depth_packet_inputs(
        session_dir,
        output_dir,
        depth_frames,
        depth_input_format,
    )
    pose_report = _write_pose_packets(output_dir, head_records)
    sidecar_report = _copy_pose_sidecars(session_dir, output_dir)

    package_manifest = {
        "schema": "spatialmp4.packet_inputs.v1",
        "source_schema": manifest.get("schema", "unknown"),
        "source_session": str(session_dir),
        "session_start_unix_us": session_start_unix_us,
        "time_base_hz": TIME_BASE_HZ,
        "streams": {
            "rgb": rgb_report,
            "depth": depth_report,
            "head_pose": pose_report,
            "sidecars": sidecar_report,
        },
        "camera_metadata": camera_metadata,
        "spatialmp4_contract": {
            "rgb": {
                "target_codec": "hevc",
                "layout": "side_by_side_stereo",
                "note": "Muxer must convert paired YUV frames into one HEVC frame per pair.",
            },
            "depth": {
                "target_codec_tag": "raw1",
                "payload": "uint16 little-endian depth in millimeters",
            },
            "head_pose": {
                "target_codec_tag": "mett",
                "mime_type": "application/pose",
                "payload": "7 little-endian float64 values: x,y,z,qx,qy,qz,qw",
            },
        },
    }
    package_manifest["muxer_inputs"] = _write_muxer_inputs(output_dir, package_manifest)

    _write_json(output_dir / "package_manifest.json", package_manifest)
    validation = validate_packet_package(output_dir, strict=not allow_partial)
    package_manifest["validation"] = validation
    _write_json(output_dir / "package_manifest.json", package_manifest)

    if make_zip:
        archive = shutil.make_archive(str(output_dir), "zip", output_dir)
        package_manifest["archive"] = archive
        _write_json(output_dir / "package_manifest.json", package_manifest)

    return package_manifest


def validate_packet_package(packet_dir: Path, *, strict: bool = True) -> Dict[str, Any]:
    packet_dir = packet_dir.resolve()
    manifest_path = packet_dir / "package_manifest.json"
    manifest = _read_json_if_exists(manifest_path)
    if manifest is None:
        raise PackError(f"Missing package_manifest.json in {packet_dir}")
    if manifest.get("schema") != "spatialmp4.packet_inputs.v1":
        raise PackError(f"Unsupported packet package schema: {manifest.get('schema')}")

    streams = manifest.get("streams") or {}
    rgb_summary = _validate_rgb_packet_stream(packet_dir, streams.get("rgb") or {}, strict)
    depth_summary = _validate_depth_packet_stream(packet_dir, streams.get("depth") or {}, strict)
    pose_summary = _validate_head_pose_packet_stream(packet_dir, streams.get("head_pose") or {}, strict)
    sidecar_summary = _validate_sidecars(packet_dir, streams.get("sidecars") or {}, strict)
    camera_summary = _validate_camera_metadata(manifest.get("camera_metadata") or {}, strict)
    muxer_summary = _validate_muxer_inputs(packet_dir, manifest.get("muxer_inputs") or {}, strict)

    return {
        "valid": True,
        "rgb": rgb_summary,
        "depth": depth_summary,
        "head_pose": pose_summary,
        "sidecars": sidecar_summary,
        "camera_metadata": camera_summary,
        "muxer_inputs": muxer_summary,
    }


def _write_muxer_inputs(output_dir: Path, manifest: Dict[str, Any]) -> Dict[str, Any]:
    muxer_dir = output_dir / "muxer"
    muxer_dir.mkdir(parents=True)

    camera_metadata = manifest["camera_metadata"]
    session_start_unix_us = int(manifest.get("session_start_unix_us", 0))
    streams = manifest.get("streams") or {}

    rgb_icam = _pack_icam([camera_metadata["left"], camera_metadata["right"]])
    rgb_ecam = _pack_ecam([camera_metadata["left"], camera_metadata["right"]])
    rgb_dstr = _pack_dstr([camera_metadata["left"], camera_metadata["right"]])

    depth_camera = camera_metadata.get("depth")
    if not depth_camera:
        raise PackError(
            "Missing depth camera calibration. Provide depth_camera_characteristics.json "
            "or depth FOV metadata from the OpenXR depth provider."
        )
    depth_icam = _pack_icam([depth_camera])
    depth_ecam = _pack_ecam([depth_camera])
    # The current FFmpeg reader patch reads two sets of brown coefficients even
    # for raw depth. Preserve that width to avoid a short dstr payload.
    depth_dstr = _pack_dstr([depth_camera, _zero_distortion_camera(depth_camera)])

    files = {
        "rgb_icam": ("muxer/rgb_icam.bin", rgb_icam),
        "rgb_ecam": ("muxer/rgb_ecam.bin", rgb_ecam),
        "rgb_dstr": ("muxer/rgb_dstr.bin", rgb_dstr),
        "depth_icam": ("muxer/depth_icam.bin", depth_icam),
        "depth_ecam": ("muxer/depth_ecam.bin", depth_ecam),
        "depth_dstr": ("muxer/depth_dstr.bin", depth_dstr),
    }
    file_report: Dict[str, Any] = {}
    for key, (rel_path, payload) in files.items():
        path = output_dir / rel_path
        path.write_bytes(payload)
        file_report[key] = {"path": rel_path, "bytes": len(payload)}

    metadata = {
        "schema": "spatialmp4.muxer_inputs.v1",
        "time_base_hz": TIME_BASE_HZ,
        "track_base_time": session_start_unix_us,
        "rgb": {
            "target_codec": "hevc",
            "layout": "side_by_side_stereo",
            "cam_count": 2,
            "camera_model": "pinhole",
            "distortion_model": _stream_distortion_model([camera_metadata["left"], camera_metadata["right"]]),
            "side_data": {
                "AV_PKT_DATA_ICAM": file_report["rgb_icam"],
                "AV_PKT_DATA_ECAM": file_report["rgb_ecam"],
                "AV_PKT_DATA_DISTORTION_COEFFICIENTS": file_report["rgb_dstr"],
            },
            "input_index": streams.get("rgb", {}).get("index"),
        },
        "depth": {
            "target_codec_tag": "raw1",
            "cam_count": 1,
            "camera_model": "pinhole",
            "distortion_model": _stream_distortion_model([depth_camera]),
            "depth_data_precision": "dtmm",
            "data_accuracy": 2,
            "depth_legal_range": 1000,
            "side_data": {
                "AV_PKT_DATA_ICAM": file_report["depth_icam"],
                "AV_PKT_DATA_ECAM": file_report["depth_ecam"],
                "AV_PKT_DATA_DISTORTION_COEFFICIENTS": file_report["depth_dstr"],
            },
            "input_index": streams.get("depth", {}).get("index"),
        },
        "head_pose": {
            "target_codec_tag": "mett",
            "mime_type": "application/pose",
            "pose_coordinate": 1,
            "data_accuracy": 2,
            "pose_position": "head",
            "payload_file": streams.get("head_pose", {}).get("payload_file"),
            "input_index": streams.get("head_pose", {}).get("index"),
        },
    }
    _write_json(muxer_dir / "metadata.json", metadata)
    return {
        "metadata": "muxer/metadata.json",
        "side_data": file_report,
    }


def _pack_icam(cameras: List[Dict[str, Any]]) -> bytes:
    values: List[float] = []
    for camera in cameras:
        intrinsics = camera.get("intrinsics") or {}
        values.extend([
            float(intrinsics.get("fx", 0.0)),
            float(intrinsics.get("fy", 0.0)),
            float(intrinsics.get("cx", 0.0)),
            float(intrinsics.get("cy", 0.0)),
        ])
    return struct.pack("<" + "d" * len(values), *values)


def _pack_ecam(cameras: List[Dict[str, Any]]) -> bytes:
    values: List[float] = []
    for camera in cameras:
        extrinsics = list(camera.get("extrinsics_row_major_3x4") or [])
        if len(extrinsics) != 12:
            extrinsics = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        values.extend(float(value) for value in extrinsics[:12])
    return struct.pack("<" + "d" * len(values), *values)


def _pack_dstr(cameras: List[Dict[str, Any]]) -> bytes:
    model = _stream_distortion_model(cameras)
    coeff_count = _distortion_coeff_count(model)
    values: List[float] = []
    for camera in cameras:
        coeffs = [float(value) for value in (camera.get("distortion") or [])]
        coeffs = (coeffs + [0.0] * coeff_count)[:coeff_count]
        values.extend(coeffs)
    return struct.pack("<" + "d" * len(values), *values)


def _distortion_coeff_count(model: str) -> int:
    if model == "equidistant":
        return 4
    if model == "equiDis62":
        return 8
    return 5


def _stream_distortion_model(cameras: List[Dict[str, Any]]) -> str:
    for camera in cameras:
        model = camera.get("distortion_model")
        if model:
            return str(model)
    return "brown"


def _zero_distortion_camera(camera: Dict[str, Any]) -> Dict[str, Any]:
    clone = dict(camera)
    clone["distortion"] = []
    return clone


def _validate_rgb_packet_stream(packet_dir: Path, stream: Dict[str, Any], strict: bool) -> Dict[str, Any]:
    index_rel = stream.get("index")
    if not index_rel:
        if strict:
            raise PackError("RGB stream index is missing")
        return {"frame_count": 0}

    rows = _read_jsonl(packet_dir / index_rel)
    expected_count = int(stream.get("frame_count", len(rows)))
    if len(rows) != expected_count:
        raise PackError(f"RGB index has {len(rows)} rows, manifest says {expected_count}")
    if strict and not rows:
        raise PackError("RGB stream has no frame pairs")

    prev_ts = -1
    max_pair_delta_ns = 0
    for row in rows:
        timestamp_ns = int(row["timestamp_ns"])
        if timestamp_ns < prev_ts:
            raise PackError("RGB timestamps are not monotonic")
        prev_ts = timestamp_ns
        max_pair_delta_ns = max(max_pair_delta_ns, int(row.get("pair_delta_ns", 0)))

        for side in ("left", "right"):
            payload_path = packet_dir / row[side]
            if not payload_path.exists():
                raise PackError(f"Missing RGB {side} payload: {payload_path}")
            metadata = row.get(f"{side}_metadata") or {}
            if metadata:
                _validate_yuv420_planes(payload_path, metadata)

    return {
        "frame_count": len(rows),
        "max_pair_delta_ns": max_pair_delta_ns,
    }


def _validate_yuv420_planes(payload_path: Path, metadata: Dict[str, Any]) -> None:
    planes = metadata.get("planes") or []
    if len(planes) != 3:
        raise PackError(f"{payload_path} metadata must contain 3 YUV planes")

    file_size = payload_path.stat().st_size
    for plane in planes:
        offset = int(plane.get("offset", -1))
        size = int(plane.get("size", -1))
        row_stride = int(plane.get("row_stride", 0))
        pixel_stride = int(plane.get("pixel_stride", 0))
        if offset < 0 or size < 0:
            raise PackError(f"{payload_path} plane has invalid offset/size")
        if offset + size > file_size:
            raise PackError(f"{payload_path} plane exceeds payload size")
        if row_stride <= 0 or pixel_stride <= 0:
            raise PackError(f"{payload_path} plane has invalid stride metadata")


def _validate_depth_packet_stream(packet_dir: Path, stream: Dict[str, Any], strict: bool) -> Dict[str, Any]:
    index_rel = stream.get("index")
    if not index_rel:
        if strict:
            raise PackError("Depth stream index is missing")
        return {"frame_count": 0}

    rows = _read_jsonl(packet_dir / index_rel)
    expected_count = int(stream.get("frame_count", len(rows)))
    if len(rows) != expected_count:
        raise PackError(f"Depth index has {len(rows)} rows, manifest says {expected_count}")
    if strict and not rows:
        raise PackError("Depth stream has no frames")

    prev_ts = -1
    dimensions = set()
    for row in rows:
        timestamp_ns = int(row["timestamp_ns"])
        if timestamp_ns < prev_ts:
            raise PackError("Depth timestamps are not monotonic")
        prev_ts = timestamp_ns

        width = int(row["width"])
        height = int(row["height"])
        payload_path = packet_dir / row["path"]
        if not payload_path.exists():
            raise PackError(f"Missing depth payload: {payload_path}")
        expected_size = width * height * 2
        actual_size = payload_path.stat().st_size
        if actual_size != expected_size:
            raise PackError(
                f"Depth payload size mismatch for {payload_path}: "
                f"expected {expected_size}, got {actual_size}"
            )
        dimensions.add((width, height))

    return {"frame_count": len(rows), "dimensions": sorted(dimensions)}


def _validate_head_pose_packet_stream(packet_dir: Path, stream: Dict[str, Any], strict: bool) -> Dict[str, Any]:
    payload_rel = stream.get("payload_file")
    index_rel = stream.get("index")
    if not payload_rel or not index_rel:
        if strict:
            raise PackError("Head pose payload or index is missing")
        return {"packet_count": 0}

    rows = _read_jsonl(packet_dir / index_rel)
    expected_count = int(stream.get("packet_count", len(rows)))
    if len(rows) != expected_count:
        raise PackError(f"Head pose index has {len(rows)} rows, manifest says {expected_count}")
    if strict and not rows:
        raise PackError("Head pose stream has no packets")

    payload_path = packet_dir / payload_rel
    if not payload_path.exists():
        raise PackError(f"Missing head pose payload: {payload_path}")
    payload_size = payload_path.stat().st_size
    expected_payload_size = len(rows) * 7 * 8
    if payload_size != expected_payload_size:
        raise PackError(
            f"Head pose payload size mismatch: expected {expected_payload_size}, got {payload_size}"
        )

    prev_ts = -1
    with payload_path.open("rb") as payload:
        for row in rows:
            timestamp_ns = int(row["timestamp_ns"])
            if timestamp_ns < prev_ts:
                raise PackError("Head pose timestamps are not monotonic")
            prev_ts = timestamp_ns

            offset = int(row["offset"])
            size = int(row["size"])
            if size != 7 * 8 or offset + size > payload_size:
                raise PackError("Invalid head pose packet offset/size")
            payload.seek(offset)
            values = struct.unpack("<7d", payload.read(size))
            if not all(math.isfinite(v) for v in values):
                raise PackError("Head pose packet contains non-finite values")
            q_norm = math.sqrt(sum(v * v for v in values[3:7]))
            if strict and q_norm == 0.0:
                raise PackError("Head pose quaternion has zero norm")

    return {"packet_count": len(rows), "payload_bytes": payload_size}


def _validate_sidecars(packet_dir: Path, sidecars: Dict[str, Any], strict: bool) -> Dict[str, Any]:
    if strict and not sidecars:
        raise PackError("Missing controller or hand pose samples")

    result: Dict[str, Any] = {}
    total_records = 0
    for name, info in sidecars.items():
        path = packet_dir / info["path"]
        if not path.exists():
            raise PackError(f"Missing sidecar: {path}")
        records = sum(1 for line in path.open("r", encoding="utf-8") if line.strip())
        if int(info.get("records", records)) != records:
            raise PackError(f"Sidecar record count mismatch for {path}")
        result[name] = {"records": records}
        total_records += records
    if strict and total_records == 0:
        raise PackError("Missing required controller or hand pose samples")
    return result


def _validate_camera_metadata(metadata: Dict[str, Any], strict: bool) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for eye in ("left", "right", "depth"):
        eye_metadata = metadata.get(eye) or {}
        intrinsics = eye_metadata.get("intrinsics") or {}
        extrinsics = eye_metadata.get("extrinsics_row_major_3x4") or []
        has_intrinsics = all(float(intrinsics.get(k, 0.0)) != 0.0 for k in ("fx", "fy"))
        has_extrinsics = len(extrinsics) == 12
        if strict and not has_intrinsics:
            raise PackError(f"Missing {eye} camera intrinsics")
        if strict and not has_extrinsics:
            raise PackError(f"Missing {eye} camera extrinsics")
        result[eye] = {
            "has_intrinsics": has_intrinsics,
            "has_extrinsics": has_extrinsics,
            "distortion_model": eye_metadata.get("distortion_model", ""),
        }
    return result


def _validate_muxer_inputs(packet_dir: Path, muxer_inputs: Dict[str, Any], strict: bool) -> Dict[str, Any]:
    metadata_rel = muxer_inputs.get("metadata")
    if not metadata_rel:
        if strict:
            raise PackError("Muxer metadata is missing")
        return {"has_metadata": False}

    metadata_path = packet_dir / metadata_rel
    metadata = _read_json_if_exists(metadata_path)
    if metadata is None:
        raise PackError(f"Missing muxer metadata: {metadata_path}")
    if metadata.get("schema") != "spatialmp4.muxer_inputs.v1":
        raise PackError(f"Unsupported muxer metadata schema: {metadata.get('schema')}")

    side_data = muxer_inputs.get("side_data") or {}
    expected_sizes = {
        "rgb_icam": 2 * 4 * 8,
        "rgb_ecam": 2 * 12 * 8,
        "rgb_dstr": 2 * _distortion_coeff_count(metadata["rgb"].get("distortion_model", "brown")) * 8,
        "depth_icam": 1 * 4 * 8,
        "depth_ecam": 1 * 12 * 8,
        # See _write_muxer_inputs: depth dstr preserves two coefficient sets.
        "depth_dstr": 2 * _distortion_coeff_count(metadata["depth"].get("distortion_model", "brown")) * 8,
    }

    result: Dict[str, Any] = {"has_metadata": True, "side_data": {}}
    for name, expected_size in expected_sizes.items():
        info = side_data.get(name)
        if not info:
            raise PackError(f"Missing muxer side data entry: {name}")
        path = packet_dir / info["path"]
        if not path.exists():
            raise PackError(f"Missing muxer side data file: {path}")
        actual_size = path.stat().st_size
        if actual_size != expected_size:
            raise PackError(
                f"Muxer side data size mismatch for {name}: expected {expected_size}, got {actual_size}"
            )
        if int(info.get("bytes", actual_size)) != actual_size:
            raise PackError(f"Muxer side data byte count mismatch for {name}")
        result["side_data"][name] = {"bytes": actual_size}

    if strict:
        if metadata.get("track_base_time", 0) <= 0:
            raise PackError("Muxer track_base_time is missing")
        if metadata["rgb"].get("cam_count") != 2:
            raise PackError("Muxer RGB cam_count must be 2")
        if metadata["depth"].get("cam_count") != 1:
            raise PackError("Muxer depth cam_count must be 1")

    return result


def _load_camera_metadata(session_dir: Path, depth_frames: List[DepthFrame]) -> Dict[str, Any]:
    left = _read_json_if_exists(session_dir / "left_camera_characteristics.json")
    right = _read_json_if_exists(session_dir / "right_camera_characteristics.json")
    depth = _read_json_if_exists(session_dir / "depth_camera_characteristics.json")
    if depth:
        depth_metadata = _normalize_camera_metadata(depth, "depth")
        depth_metadata["calibration_source"] = "depth_camera_characteristics.json"
    else:
        depth_metadata = _derive_depth_camera_metadata(depth_frames)
    return {
        "left": _normalize_camera_metadata(left or {}, "left"),
        "right": _normalize_camera_metadata(right or {}, "right"),
        "depth": depth_metadata,
    }


def _derive_depth_camera_metadata(depth_frames: List[DepthFrame]) -> Optional[Dict[str, Any]]:
    frame = next((item for item in depth_frames if item.eye == "left"), None)
    if frame is None:
        return None
    fov = frame.metadata.get("fov_tangent")
    if not isinstance(fov, dict):
        return None
    left = float(fov["left"])
    right = float(fov["right"])
    top = float(fov["top"])
    bottom = float(fov["bottom"])
    if left + right <= 0.0 or top + bottom <= 0.0:
        raise PackError("Invalid depth FOV tangent metadata")
    return {
        "eye": "depth",
        "camera_id": "openxr_environment_depth_left",
        "intrinsics": {
            "fx": frame.width / (left + right),
            "fy": frame.height / (top + bottom),
            "cx": frame.width * right / (left + right),
            "cy": frame.height * top / (top + bottom),
            "skew": 0.0,
        },
        "distortion_model": "",
        "distortion": [],
        "extrinsics_row_major_3x4": [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ],
        "calibration_source": "openxr_depth_fov",
        "extrinsics_status": "identity_until_head_to_depth_calibration_is_supplied",
        "raw": frame.metadata,
    }


def _normalize_camera_metadata(metadata: Dict[str, Any], eye: str) -> Dict[str, Any]:
    intrinsics = _extract_intrinsics(metadata)
    distortion = metadata.get("lens_distortion", metadata.get("distortion", []))
    extrinsics = _extract_camera_extrinsics(metadata)
    return {
        "eye": eye,
        "camera_id": metadata.get("camera_id", metadata.get("cameraId", "")),
        "intrinsics": intrinsics,
        "distortion_model": "brown" if distortion else "",
        "distortion": distortion,
        "extrinsics_row_major_3x4": extrinsics,
        "raw": metadata,
    }


def _extract_intrinsics(metadata: Dict[str, Any]) -> Dict[str, float]:
    if isinstance(metadata.get("intrinsics"), dict):
        values = metadata["intrinsics"]
        return {
            "fx": float(values.get("fx", 0.0)),
            "fy": float(values.get("fy", 0.0)),
            "cx": float(values.get("cx", 0.0)),
            "cy": float(values.get("cy", 0.0)),
            "skew": float(values.get("skew", 0.0)),
        }

    values = metadata.get("lens_intrinsic_calibration") or []
    if len(values) >= 5:
        return {
            "fx": float(values[0]),
            "fy": float(values[1]),
            "cx": float(values[2]),
            "cy": float(values[3]),
            "skew": float(values[4]),
        }

    return {"fx": 0.0, "fy": 0.0, "cx": 0.0, "cy": 0.0, "skew": 0.0}


def _extract_camera_extrinsics(metadata: Dict[str, Any]) -> List[float]:
    pose = metadata.get("pose")
    if isinstance(pose, dict):
        translation = pose.get("translation") or [0.0, 0.0, 0.0]
        rotation = pose.get("rotation") or [0.0, 0.0, 0.0, 1.0]
    else:
        translation = metadata.get("lens_pose_translation") or [0.0, 0.0, 0.0]
        rotation = metadata.get("lens_pose_rotation") or [0.0, 0.0, 0.0, 1.0]

    matrix = _quat_translation_to_3x4(rotation, translation)
    return [float(v) for row in matrix for v in row]


def _quat_translation_to_3x4(
    quaternion_xyzw: Iterable[float],
    translation_xyz: Iterable[float],
) -> List[List[float]]:
    q = [float(v) for v in quaternion_xyzw]
    t = [float(v) for v in translation_xyz]
    if len(q) < 4:
        q = [0.0, 0.0, 0.0, 1.0]
    if len(t) < 3:
        t = [0.0, 0.0, 0.0]

    x, y, z, w = q[:4]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), t[0]],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), t[1]],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), t[2]],
    ]


def _pair_rgb_frames(session_dir: Path, max_delta_ns: int) -> List[Pair]:
    left_frames = _collect_rgb_frames(session_dir, "left")
    right_frames = _collect_rgb_frames(session_dir, "right")

    pairs: List[Pair] = []
    right_idx = 0
    for left in left_frames:
        best_idx = -1
        best_delta = max_delta_ns + 1
        for idx in range(right_idx, len(right_frames)):
            delta = abs(right_frames[idx].timestamp_ns - left.timestamp_ns)
            if delta < best_delta:
                best_idx = idx
                best_delta = delta
            if right_frames[idx].timestamp_ns > left.timestamp_ns and delta > best_delta:
                break
        if best_idx >= 0 and best_delta <= max_delta_ns:
            right = right_frames[best_idx]
            right_idx = best_idx + 1
            pairs.append(Pair(min(left.timestamp_ns, right.timestamp_ns), left, right, best_delta))

    return pairs


def _collect_rgb_frames(session_dir: Path, eye: str) -> List[TimestampedFile]:
    index_path = session_dir / f"{eye}_camera_frames.jsonl"
    if index_path.exists():
        frames: List[TimestampedFile] = []
        for record in _read_jsonl(index_path):
            raw_path = Path(record.get("raw_path") or record.get("path") or "")
            if not raw_path.is_absolute():
                raw_path = session_dir / raw_path
            frames.append(
                TimestampedFile(
                    timestamp_ns=int(record["timestamp_ns"]),
                    path=raw_path,
                    metadata=record,
                )
            )
        return sorted(frames, key=lambda item: item.timestamp_ns)

    return _collect_timestamped_files(session_dir / f"{eye}_camera_raw", {".yuv", ".bin"})


def _collect_timestamped_files(directory: Path, suffixes: set) -> List[TimestampedFile]:
    if not directory.exists():
        return []

    files: List[TimestampedFile] = []
    for path in sorted(directory.iterdir()):
        if path.suffix.lower() not in suffixes:
            continue
        try:
            timestamp = int(path.stem)
        except ValueError:
            continue
        files.append(TimestampedFile(timestamp, path, {}))
    return sorted(files, key=lambda item: item.timestamp_ns)


def _write_rgb_packet_inputs(output_dir: Path, pairs: List[Pair]) -> Dict[str, Any]:
    rgb_dir = output_dir / "rgb"
    left_dir = rgb_dir / "left_yuv"
    right_dir = rgb_dir / "right_yuv"
    left_dir.mkdir(parents=True)
    right_dir.mkdir(parents=True)

    index_path = rgb_dir / "stereo_pairs.jsonl"
    with index_path.open("w", encoding="utf-8") as index:
        for frame_id, pair in enumerate(pairs):
            left_name = f"{frame_id:06d}_{pair.left.timestamp_ns}.yuv"
            right_name = f"{frame_id:06d}_{pair.right.timestamp_ns}.yuv"
            shutil.copy2(pair.left.path, left_dir / left_name)
            shutil.copy2(pair.right.path, right_dir / right_name)
            index.write(
                json.dumps(
                    {
                        "frame_id": frame_id,
                        "timestamp_ns": pair.timestamp_ns,
                        "pts_us": pair.timestamp_ns // 1000,
                        "left": f"rgb/left_yuv/{left_name}",
                        "right": f"rgb/right_yuv/{right_name}",
                        "left_metadata": _rgb_frame_metadata(pair.left),
                        "right_metadata": _rgb_frame_metadata(pair.right),
                        "pair_delta_ns": pair.delta_ns,
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )

    return {
        "frame_count": len(pairs),
        "index": "rgb/stereo_pairs.jsonl",
        "left_payload_dir": "rgb/left_yuv",
        "right_payload_dir": "rgb/right_yuv",
        "payload": "paired raw YUV_420_888 planes; muxer must encode side-by-side HEVC",
    }


def _rgb_frame_metadata(frame: TimestampedFile) -> Dict[str, Any]:
    if not frame.metadata:
        return {"timestamp_ns": frame.timestamp_ns, "format": "unknown"}

    metadata = dict(frame.metadata)
    metadata.pop("raw_path", None)
    metadata.pop("path", None)
    return metadata


def _load_depth_frames(session_dir: Path) -> List[DepthFrame]:
    frames_jsonl = session_dir / "depth" / "frames.jsonl"
    if frames_jsonl.exists():
        frames: List[DepthFrame] = []
        for record in _read_jsonl(frames_jsonl):
            image_path = Path(record["image_path"])
            if not image_path.is_absolute():
                image_path = session_dir / image_path
            frames.append(
                DepthFrame(
                    timestamp_ns=int(record["timestamp_ns"]),
                    eye=str(record.get("eye", "left")),
                    path=image_path,
                    width=int(record["width"]),
                    height=int(record["height"]),
                    metadata=dict(record.get("metadata", {})),
                )
            )
        return frames

    return _load_quest_reality_depth_frames(session_dir)


def _load_quest_reality_depth_frames(session_dir: Path) -> List[DepthFrame]:
    descriptors = _read_depth_descriptors(session_dir / "left_depth_descriptors.csv")
    frames: List[DepthFrame] = []
    for item in _collect_timestamped_files(session_dir / "left_depth", {".raw"}):
        descriptor = descriptors.get(str(item.timestamp_ns))
        if not descriptor:
            raise PackError(f"Missing left depth descriptor for {item.path.name}")
        frames.append(
            DepthFrame(
                timestamp_ns=_timestamp_stem_to_ns(item.timestamp_ns),
                eye="left",
                path=item.path,
                width=int(descriptor.get("width", 0)),
                height=int(descriptor.get("height", 0)),
                metadata={
                    "source": "quest_reality_capture",
                    "depth_encoding": "openxr_window_depth_normalized",
                    "sample_storage": "f32_le",
                    "near_z": float(descriptor["near_z"]),
                    "far_z": float(descriptor["far_z"]),
                    "fov_tangent": {
                        "left": float(descriptor["fov_left_angle_tangent"]),
                        "right": float(descriptor["fov_right_angle_tangent"]),
                        "top": float(descriptor["fov_top_angle_tangent"]),
                        "bottom": float(descriptor["fov_down_angle_tangent"]),
                    },
                },
            )
        )
    return frames


def _timestamp_stem_to_ns(value: int) -> int:
    # QuestRealityCapture filenames are Unix milliseconds; Godot plugin filenames
    # are nanoseconds. Values below year-scale nanoseconds are treated as ms.
    if value < 10_000_000_000_000:
        return value * 1_000_000
    return value


def _read_depth_descriptors(path: Path) -> Dict[str, Dict[str, str]]:
    if not path.exists():
        return {}
    result: Dict[str, Dict[str, str]] = {}
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            result[str(row["timestamp_ms"])] = row
    return result


def _write_depth_packet_inputs(
    session_dir: Path,
    output_dir: Path,
    frames: List[DepthFrame],
    depth_input_format: str,
) -> Dict[str, Any]:
    depth_dir = output_dir / "depth" / "raw1_u16_mm"
    depth_dir.mkdir(parents=True)
    index_path = output_dir / "depth" / "depth_frames.jsonl"

    written = 0
    skipped = 0
    formats: Dict[str, int] = {}
    with index_path.open("w", encoding="utf-8") as index:
        for frame_id, frame in enumerate(frames):
            if frame.eye != "left":
                skipped += 1
                continue

            raw = frame.path.read_bytes()
            detected = _resolve_depth_format(depth_input_format, frame, len(raw))
            converted = _convert_depth_to_u16_mm(raw, detected, frame)
            out_name = f"{frame_id:06d}_{frame.timestamp_ns}.raw"
            (depth_dir / out_name).write_bytes(converted)
            formats[detected] = formats.get(detected, 0) + 1
            written += 1
            index.write(
                json.dumps(
                    {
                        "frame_id": frame_id,
                        "timestamp_ns": frame.timestamp_ns,
                        "pts_us": frame.timestamp_ns // 1000,
                        "eye": frame.eye,
                        "path": f"depth/raw1_u16_mm/{out_name}",
                        "width": frame.width,
                        "height": frame.height,
                        "input_format": detected,
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )

    return {
        "frame_count": written,
        "skipped_non_left_frames": skipped,
        "index": "depth/depth_frames.jsonl",
        "payload_dir": "depth/raw1_u16_mm",
        "payload": "uint16 little-endian millimeters",
        "input_formats": formats,
    }


def _resolve_depth_format(mode: str, frame: DepthFrame, byte_count: int) -> str:
    depth_encoding = frame.metadata.get("depth_encoding")
    if depth_encoding == "openxr_window_depth_normalized":
        _validate_openxr_conversion_metadata(frame)
        return "openxr_window_depth_normalized"

    metadata_unit = frame.metadata.get("depth_unit")
    if metadata_unit in {"f32_m", "u16_mm", "u8_raw"}:
        return str(metadata_unit)

    if frame.metadata.get("has_depth_projection_view") or frame.metadata.get("source") == "XR_META_environment_depth":
        raise PackError(
            "OpenXR depth without near/far or inverse projection-view metadata "
            "cannot be converted to SpatialMP4 raw1: "
            f"{frame.path}"
        )

    if mode != "auto":
        return mode

    pixel_count = frame.width * frame.height
    if byte_count == pixel_count * 4:
        return "f32_m"
    if byte_count == pixel_count * 2:
        return "u16_mm"
    if byte_count == pixel_count:
        return "u8_raw"

    raise PackError(
        f"Cannot infer depth format for {frame.path}: "
        f"{byte_count} bytes for {frame.width}x{frame.height}"
    )


def _convert_depth_to_u16_mm(raw: bytes, depth_format: str, frame: DepthFrame) -> bytes:
    width = frame.width
    height = frame.height
    pixel_count = width * height
    if depth_format == "u16_mm":
        expected = pixel_count * 2
        if len(raw) != expected:
            raise PackError(f"Expected {expected} bytes for u16 depth, got {len(raw)}")
        return raw

    if depth_format == "f32_m":
        expected = pixel_count * 4
        if len(raw) != expected:
            raise PackError(f"Expected {expected} bytes for f32 depth, got {len(raw)}")
        out = bytearray(pixel_count * 2)
        offset = 0
        for (meters,) in struct.iter_unpack("<f", raw):
            if math.isfinite(meters) and meters > 0.0:
                mm = min(65535, max(0, int(round(meters * 1000.0))))
            else:
                mm = 0
            struct.pack_into("<H", out, offset, mm)
            offset += 2
        return bytes(out)

    if depth_format == "u8_raw":
        expected = pixel_count
        if len(raw) != expected:
            raise PackError(f"Expected {expected} bytes for u8 depth, got {len(raw)}")
        out = bytearray(pixel_count * 2)
        for idx, value in enumerate(raw):
            struct.pack_into("<H", out, idx * 2, value)
        return bytes(out)

    if depth_format == "openxr_window_depth_normalized":
        return _convert_openxr_window_depth_to_u16_mm(raw, frame)

    raise PackError(f"Unsupported depth format: {depth_format}")


def _validate_openxr_conversion_metadata(frame: DepthFrame) -> None:
    storage = frame.metadata.get("sample_storage")
    if storage not in {"u16_unorm_le", "f16_le", "f32_le"}:
        raise PackError(f"Unsupported OpenXR depth sample storage for {frame.path}: {storage}")

    has_planes = "near_z" in frame.metadata and "far_z" in frame.metadata
    inverse_columns = frame.metadata.get("depth_inverse_projection_view_columns")
    has_inverse_projection_view = _is_matrix_columns(inverse_columns)
    if not has_planes and not has_inverse_projection_view:
        raise PackError(
            "OpenXR depth without near/far or inverse projection-view metadata "
            f"cannot be converted to SpatialMP4 raw1: {frame.path}"
        )


def _is_matrix_columns(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 4
        and all(isinstance(column, list) and len(column) == 4 for column in value)
    )


def _read_openxr_depth_samples(raw: bytes, frame: DepthFrame) -> Iterable[float]:
    storage = frame.metadata["sample_storage"]
    bytes_per_pixel = {"u16_unorm_le": 2, "f16_le": 2, "f32_le": 4}[storage]
    expected = frame.width * frame.height * bytes_per_pixel
    if len(raw) != expected:
        raise PackError(f"Expected {expected} bytes for {storage} OpenXR depth, got {len(raw)}")
    if storage == "u16_unorm_le":
        return (value[0] / 65535.0 for value in struct.iter_unpack("<H", raw))
    sample_format = "<e" if storage == "f16_le" else "<f"
    return (value[0] for value in struct.iter_unpack(sample_format, raw))


def _convert_openxr_window_depth_to_u16_mm(raw: bytes, frame: DepthFrame) -> bytes:
    samples = _read_openxr_depth_samples(raw, frame)
    inverse_columns = frame.metadata.get("depth_inverse_projection_view_columns")
    inverse_rows = None
    if _is_matrix_columns(inverse_columns):
        inverse_rows = [
            [float(inverse_columns[column][row]) for column in range(4)]
            for row in range(4)
        ]

    near_z = frame.metadata.get("near_z")
    far_z = frame.metadata.get("far_z")
    out = bytearray(frame.width * frame.height * 2)
    for index, window_depth in enumerate(samples):
        if not math.isfinite(window_depth) or window_depth < 0.0 or window_depth > 1.0:
            meters = 0.0
        elif inverse_rows is not None:
            x = index % frame.width
            y = index // frame.width
            clip = (
                2.0 * ((x + 0.5) / frame.width) - 1.0,
                2.0 * ((y + 0.5) / frame.height) - 1.0,
                2.0 * window_depth - 1.0,
                1.0,
            )
            homogeneous_w = sum(inverse_rows[3][axis] * clip[axis] for axis in range(4))
            meters = 1.0 / homogeneous_w if homogeneous_w > 0.0 else 0.0
        else:
            meters = _openxr_window_depth_to_linear_z(window_depth, float(near_z), float(far_z))

        mm = min(65535, max(0, int(round(meters * 1000.0)))) if math.isfinite(meters) else 0
        struct.pack_into("<H", out, index * 2, mm)
    return bytes(out)


def _openxr_window_depth_to_linear_z(window_depth: float, near_z: float, far_z: float) -> float:
    if not math.isfinite(near_z) or near_z <= 0.0:
        raise PackError(f"Invalid OpenXR near_z: {near_z}")
    if not math.isfinite(far_z) or far_z < near_z:
        x = -2.0 * near_z
        y = -1.0
    elif far_z == near_z:
        raise PackError(f"Invalid OpenXR near_z/far_z pair: {near_z}, {far_z}")
    else:
        x = -2.0 * far_z * near_z / (far_z - near_z)
        y = -(far_z + near_z) / (far_z - near_z)
    denominator = 2.0 * window_depth - 1.0 + y
    return x / denominator if denominator != 0.0 else 0.0


def _write_pose_packets(output_dir: Path, head_records: List[Dict[str, Any]]) -> Dict[str, Any]:
    pose_dir = output_dir / "pose"
    pose_dir.mkdir(parents=True)
    packet_path = pose_dir / "head_pose_mett.bin"
    index_path = pose_dir / "head_pose_packets.jsonl"

    offset = 0
    with packet_path.open("wb") as packet_file, index_path.open("w", encoding="utf-8") as index:
        for frame_id, record in enumerate(head_records):
            timestamp_ns = int(record["timestamp_ns"])
            payload = _pose_record_to_mett_payload(record)
            packet_file.write(payload)
            index.write(
                json.dumps(
                    {
                        "frame_id": frame_id,
                        "timestamp_ns": timestamp_ns,
                        "pts_us": timestamp_ns // 1000,
                        "offset": offset,
                        "size": len(payload),
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
            offset += len(payload)

    return {
        "packet_count": len(head_records),
        "payload_file": "pose/head_pose_mett.bin",
        "index": "pose/head_pose_packets.jsonl",
        "packet_size": 7 * 8,
    }


def _pose_record_to_mett_payload(record: Dict[str, Any]) -> bytes:
    position = record.get("position") or {}
    rotation = record.get("rotation") or {}
    values = (
        float(position.get("x", 0.0)),
        float(position.get("y", 0.0)),
        float(position.get("z", 0.0)),
        float(rotation.get("x", 0.0)),
        float(rotation.get("y", 0.0)),
        float(rotation.get("z", 0.0)),
        float(rotation.get("w", 1.0)),
    )
    return struct.pack("<7d", *values)


def _copy_pose_sidecars(session_dir: Path, output_dir: Path) -> Dict[str, Any]:
    source_pose_dir = session_dir / "poses"
    sidecar_dir = output_dir / "sidecars"
    sidecar_dir.mkdir(parents=True)
    copied: Dict[str, Any] = {}

    for name in ("controllers.jsonl", "hands.jsonl"):
        source = source_pose_dir / name
        if source.exists():
            target = sidecar_dir / name
            shutil.copy2(source, target)
            copied[name] = {
                "path": f"sidecars/{name}",
                "records": sum(1 for line in target.open("r", encoding="utf-8") if line.strip()),
            }

    return copied


def _read_json_if_exists(path: Path) -> Optional[Dict[str, Any]]:
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    records = []
    with path.open("r", encoding="utf-8") as f:
        lines = f.readlines()
        last_nonempty_line = -1
        for idx, line in enumerate(lines):
            if line.strip():
                last_nonempty_line = idx
        for idx, line in enumerate(lines):
            line = line.strip()
            if line:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError as error:
                    if idx == last_nonempty_line:
                        continue
                    raise PackError(f"Invalid JSONL record in {path}:{idx + 1}: {error}") from error
    return records


def _write_json(path: Path, value: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(value, f, indent=2, sort_keys=True)
        f.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path, nargs="?", help="Godot Quest capture spool directory")
    parser.add_argument("output_dir", type=Path, nargs="?", help="Output packet directory")
    parser.add_argument(
        "--max-pair-delta-ms",
        type=float,
        default=10.0,
        help="Maximum RGB left/right timestamp delta in milliseconds",
    )
    parser.add_argument(
        "--depth-input-format",
        choices=("auto", "f32_m", "u16_mm", "u8_raw"),
        default="auto",
        help="Depth input format before conversion to SpatialMP4 raw1 uint16 millimeters",
    )
    parser.add_argument(
        "--allow-partial",
        action="store_true",
        help="Allow missing RGB, depth, or controller/hand sidecar data.",
    )
    parser.add_argument(
        "--validate-packet",
        type=Path,
        help="Validate an existing packet package instead of packing a session.",
    )
    parser.add_argument("--zip", action="store_true", help="Also create output_dir.zip")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.validate_packet:
        validation = validate_packet_package(args.validate_packet, strict=not args.allow_partial)
        print(json.dumps(validation, indent=2))
        return 0

    if args.session_dir is None or args.output_dir is None:
        raise PackError("session_dir and output_dir are required unless --validate-packet is used")

    package = pack_session(
        args.session_dir,
        args.output_dir,
        max_pair_delta_ns=int(args.max_pair_delta_ms * 1_000_000),
        depth_input_format=args.depth_input_format,
        allow_partial=args.allow_partial,
        make_zip=args.zip,
    )
    print(json.dumps({
        "output": str(args.output_dir.resolve()),
        "rgb_frames": package["streams"]["rgb"]["frame_count"],
        "depth_frames": package["streams"]["depth"]["frame_count"],
        "head_pose_packets": package["streams"]["head_pose"]["packet_count"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
