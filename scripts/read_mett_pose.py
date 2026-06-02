"""Read SpatialMP4 timed-metadata rigid-pose tracks (head / controllers) out of
a finalized mp4 -- the replacement for the old `poses/head.jsonl` sidecar.

Why this exists:
    The Godot Quest Capture stack used to mirror every head-pose sample into a
    `poses/head.jsonl` file alongside the mp4 because nothing downstream knew
    how to read the SpatialMP4 `mett:head` rigid-pose track. With the muxer
    refactor that track is now first-class -- a 56-byte payload per packet
    (7 little-endian doubles: pos_xyz + quat_xyzw) at the OpenXR predicted-
    display-time PTS, written by the patched FFmpeg + libspatialmp4_writer.
    This module decodes those packets back into the same record shape the
    JSONL sidecar produced, so callers that previously did
    `_read_jsonl(session_dir / "poses" / "head.jsonl")` can switch to
    `read_head_poses(session_mp4_path)` without changing anything else.

Why ffmpeg + ffprobe (not PyAV):
    The host project already shells out to the patched FFmpeg in several
    places (verify_depth_ffv1.py, godot_depth_rgb_align.py); adding PyAV
    would mean another build dependency that's painful to install on macOS.
    The CLI pipeline below uses only `ffprobe` and `ffmpeg`, which the pixi
    env / patched build script already produce.

Timestamp math (do not skip):
    The mp4 track's PTS is in microseconds, zero-relative (the live writer
    subtracts `first_pts_us_writer_` from every packet's absolute Godot-ticks
    µs before muxing so the container starts at 0). The writer also stamps
    `track_base_godot_ticks_us` into per-stream metadata, but the patched mov
    demuxer currently only surfaces it as `timebase` on the RGB / depth
    streams -- mett tracks come back with just the handler_name. So this
    module defaults to **session-relative ns** (`pts_us * 1000`), and exposes
    a `session_start_godot_ticks_us` parameter for callers that need the
    legacy absolute Godot-ticks-ns domain the head.jsonl sidecar used:

        timestamp_ns = pts_us * 1000                          # default
        timestamp_ns = (pts_us + start_godot_ticks_us) * 1000 # absolute mode

    The anchor lives in `manifest.json` / `android_timebase.json` -- the
    helper deliberately doesn't read those files itself so the mp4 stays the
    only mandatory input.
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Iterable


# Magic numbers that mirror the writer's PackPose / AddTimedMetadataStream
POSE_PACKET_BYTES = 56       # 7 little-endian doubles
_POSE_STRUCT = struct.Struct("<7d")
HEAD_HANDLER_SUFFIX = ":head"
RIGID_POSE_KIND = "rigid_pose"


class PoseTrackNotFound(RuntimeError):
    """Raised when a SpatialMP4 mp4 lacks the requested rigid-pose mett track."""


def _ffprobe_json(args: list[str]) -> dict[str, Any]:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-of", "json", *args],
        capture_output=True, text=True, check=True,
    )
    return json.loads(out.stdout)


def _find_rigid_pose_stream(mp4_path: os.PathLike | str, suffix: str) -> dict[str, Any]:
    """Return the stream dict whose handler_name ends with `suffix`."""
    streams = _ffprobe_json(["-show_streams", str(mp4_path)]).get("streams", [])
    for s in streams:
        handler = (s.get("tags") or {}).get("handler_name", "")
        if handler.endswith(suffix) and RIGID_POSE_KIND in handler:
            return s
    raise PoseTrackNotFound(
        f"no rigid_pose mett track ending with {suffix!r} in {mp4_path}"
    )


def _stream_packets(mp4_path: os.PathLike | str, stream_index: int) -> tuple[list[int], bytes]:
    """Return parallel (pts_us_list, raw_bytes_blob) for one track.

    We do two ffmpeg invocations:
      * `ffprobe -show_packets` for the PTS list (small JSON).
      * `ffmpeg -c copy -f data` for the concatenated payload (one big blob
        we slice by `POSE_PACKET_BYTES`).
    Each `mett` rigid-pose packet is fixed-size, so slicing by index gives us
    the same ordering as the PTS list.
    """
    pkts = _ffprobe_json([
        "-select_streams", str(stream_index),
        "-show_entries", "packet=pts,size",
        str(mp4_path),
    ]).get("packets", [])
    pts_us: list[int] = []
    for pkt in pkts:
        size = int(pkt.get("size", 0))
        if size != POSE_PACKET_BYTES:
            raise ValueError(
                f"rigid_pose packet in stream {stream_index} has size {size}, "
                f"expected {POSE_PACKET_BYTES}"
            )
        # PTS can be 'N/A' on the first packet of malformed captures; treat as 0.
        raw_pts = pkt.get("pts")
        pts_us.append(int(raw_pts) if raw_pts not in (None, "N/A") else 0)

    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as tf:
        raw_path = tf.name
    try:
        subprocess.run(
            ["ffmpeg", "-v", "error", "-y", "-i", str(mp4_path),
             "-map", f"0:{stream_index}", "-c", "copy", "-f", "data", raw_path],
            check=True,
        )
        blob = Path(raw_path).read_bytes()
    finally:
        try:
            os.unlink(raw_path)
        except OSError:
            pass

    expected = POSE_PACKET_BYTES * len(pts_us)
    if len(blob) != expected:
        raise ValueError(
            f"mett:rigid_pose blob length {len(blob)} != expected {expected} "
            f"({len(pts_us)} packets x {POSE_PACKET_BYTES} bytes)"
        )
    return pts_us, blob


def read_rigid_pose_track(
    mp4_path: os.PathLike | str,
    handler_suffix: str = HEAD_HANDLER_SUFFIX,
    source_label: str | None = None,
    session_start_godot_ticks_us: int = 0,
) -> list[dict[str, Any]]:
    """Decode a rigid_pose mett track into the same record shape head.jsonl used.

    Args:
        mp4_path: absolute path to the finalized SpatialMP4 .mp4.
        handler_suffix: identifies which rigid_pose track to read. Defaults to
            ":head" (the head-pose track); use ":left_controller" or
            ":right_controller" for controllers.
        source_label: value written into each record's `source` field. Defaults
            to the suffix stripped of its leading colon (`"head"`,
            `"left_controller"`, ...).
        session_start_godot_ticks_us: optional anchor (Godot ticks µs at
            session start) used to convert the mp4's session-relative PTS
            back into the absolute Godot-ticks-ns domain the head.jsonl
            sidecar carried. Pass 0 (default) for session-relative timestamps;
            pass the value from `manifest.json["session_start_ticks_us"]` /
            `android_timebase.json["session_start_godot_ticks_us"]` to mimic
            the legacy sidecar exactly.

    Returns:
        List of records ordered by PTS:
            { timestamp_ns: int,
              source: str,
              tracking_valid: True,
              position: {x, y, z}: float,
              rotation: {x, y, z, w}: float }

        `tracking_valid` is always True because the writer only emits a
        rigid_pose packet when its caller asserts trackingValid=true; invalid
        samples never reach the mp4.
    """
    mp4 = Path(mp4_path)
    if not mp4.is_file():
        raise FileNotFoundError(mp4)

    stream = _find_rigid_pose_stream(mp4, handler_suffix)
    stream_index = int(stream["index"])

    pts_us, blob = _stream_packets(mp4, stream_index)

    label = source_label if source_label is not None else handler_suffix.lstrip(":")
    records: list[dict[str, Any]] = []
    for i, pts in enumerate(pts_us):
        off = i * POSE_PACKET_BYTES
        px, py, pz, qx, qy, qz, qw = _POSE_STRUCT.unpack_from(blob, off)
        records.append({
            "timestamp_ns": (pts + session_start_godot_ticks_us) * 1000,
            "source": label,
            "tracking_valid": True,
            "position": {"x": px, "y": py, "z": pz},
            "rotation": {"x": qx, "y": qy, "z": qz, "w": qw},
        })
    return records


def read_head_poses(
    mp4_path: os.PathLike | str,
    session_start_godot_ticks_us: int = 0,
) -> list[dict[str, Any]]:
    """Shorthand for [read_rigid_pose_track] with the head track."""
    return read_rigid_pose_track(
        mp4_path, HEAD_HANDLER_SUFFIX, "head",
        session_start_godot_ticks_us=session_start_godot_ticks_us,
    )


def iter_rigid_pose_tracks(
    mp4_path: os.PathLike | str,
    session_start_godot_ticks_us: int = 0,
) -> Iterable[tuple[str, list[dict[str, Any]]]]:
    """Yield (source_label, records) for every rigid_pose track in the mp4.

    Lets callers handle the head + both controllers without re-probing -- the
    handler_name suffix (without the leading colon) is the yielded label, so
    a consumer can do
        { label: records for label, records in iter_rigid_pose_tracks(mp4) }
    and end up with a dict keyed by `"head"` / `"left_controller"` /
    `"right_controller"`.
    """
    streams = _ffprobe_json(["-show_streams", str(mp4_path)]).get("streams", [])
    for s in streams:
        handler = (s.get("tags") or {}).get("handler_name", "")
        if RIGID_POSE_KIND not in handler:
            continue
        label = handler.rsplit(":", 1)[-1]
        yield label, read_rigid_pose_track(
            mp4_path, f":{label}", label,
            session_start_godot_ticks_us=session_start_godot_ticks_us,
        )


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Dump SpatialMP4 mett:head pose track as JSONL.")
    ap.add_argument("mp4")
    ap.add_argument("--suffix", default=HEAD_HANDLER_SUFFIX,
                    help="handler_name suffix to match (default: :head)")
    args = ap.parse_args()
    for rec in read_rigid_pose_track(args.mp4, args.suffix):
        print(json.dumps(rec))
