import importlib.util
import json
import os
import struct
import subprocess
import sys
from pathlib import Path

import pytest


def load_packer_module():
    script = Path(__file__).resolve().parents[2] / "scripts" / "godot_spool_pack.py"
    spec = importlib.util.spec_from_file_location("godot_spool_pack", script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def write_jsonl(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )


def openxr_window_depth(meters, near_z=0.1, far_z=10.0):
    ndc = (far_z + near_z) / (far_z - near_z) - (2.0 * far_z * near_z) / ((far_z - near_z) * meters)
    return (ndc + 1.0) / 2.0


def inverse_projection_columns_for_z(near_z=0.1, far_z=10.0):
    a = -(far_z + near_z) / (far_z - near_z)
    b = -(2.0 * far_z * near_z) / (far_z - near_z)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 1.0 / b],
        [0.0, 0.0, 0.0, a / b],
    ]


def make_spool(root):
    write_json(
        root / "manifest.json",
        {
            "schema": "spatialmp4.quest_capture.spool.v1",
            "session_start_unix_us": 1_700_000_000_000_000,
        },
    )
    metadata = {
        "camera_id": "0",
        "lens_intrinsic_calibration": [100.0, 101.0, 50.0, 51.0, 0.0],
        "lens_pose_translation": [1.0, 2.0, 3.0],
        "lens_pose_rotation": [0.0, 0.0, 0.0, 1.0],
        "lens_distortion": [0.1, 0.2, 0.3, 0.4],
    }
    write_json(root / "left_camera_characteristics.json", metadata)
    write_json(root / "right_camera_characteristics.json", metadata)
    write_json(
        root / "depth_camera_characteristics.json",
        {
            "camera_id": "depth",
            "intrinsics": {"fx": 80.0, "fy": 81.0, "cx": 1.0, "cy": 1.0, "skew": 0.0},
            "pose": {"translation": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0, 1.0]},
        },
    )

    (root / "left_camera_raw").mkdir()
    (root / "right_camera_raw").mkdir()
    (root / "left_camera_raw" / "1000000000.yuv").write_bytes(b"leftuvuv")
    (root / "right_camera_raw" / "1001000000.yuv").write_bytes(b"rghtuvuv")
    write_jsonl(
        root / "left_camera_frames.jsonl",
        [
            {
                "timestamp_ns": 1_000_000_000,
                "eye": "left",
                "raw_path": "left_camera_raw/1000000000.yuv",
                "width": 2,
                "height": 2,
                "format": 35,
                "encoding": "YUV_420_888_planes",
                "planes": [
                    {"index": 0, "row_stride": 2, "pixel_stride": 1, "offset": 0, "size": 4},
                    {"index": 1, "row_stride": 1, "pixel_stride": 2, "offset": 4, "size": 2},
                    {"index": 2, "row_stride": 1, "pixel_stride": 2, "offset": 6, "size": 2},
                ],
            }
        ],
    )
    write_jsonl(
        root / "right_camera_frames.jsonl",
        [
            {
                "timestamp_ns": 1_001_000_000,
                "eye": "right",
                "raw_path": "right_camera_raw/1001000000.yuv",
                "width": 2,
                "height": 2,
                "format": 35,
                "encoding": "YUV_420_888_planes",
                "planes": [
                    {"index": 0, "row_stride": 2, "pixel_stride": 1, "offset": 0, "size": 4},
                    {"index": 1, "row_stride": 1, "pixel_stride": 2, "offset": 4, "size": 2},
                    {"index": 2, "row_stride": 1, "pixel_stride": 2, "offset": 6, "size": 2},
                ],
            }
        ],
    )

    depth_path = root / "depth" / "left_1000000000.bin"
    depth_path.parent.mkdir()
    depth_path.write_bytes(struct.pack("<4f", 0.1, 1.2, 70.0, -1.0))
    write_jsonl(
        root / "depth" / "frames.jsonl",
        [
            {
                "timestamp_ns": 1_000_000_000,
                "eye": "left",
                "image_path": str(depth_path),
                "width": 2,
                "height": 2,
                "metadata": {"depth_unit": "f32_m"},
            }
        ],
    )

    write_jsonl(
        root / "poses" / "head.jsonl",
        [
            {
                "timestamp_ns": 1_000_000_000,
                "source": "head",
                "tracking_valid": True,
                "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                "rotation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.4},
            }
        ],
    )
    write_jsonl(
        root / "poses" / "controllers.jsonl",
        [{"timestamp_ns": 1_000_000_000, "source": "left_controller"}],
    )
    write_jsonl(
        root / "poses" / "hands.jsonl",
        [{"timestamp_ns": 1_000_000_000, "hand": "left", "joints": []}],
    )


def make_muxable_rgb_spool(root):
    make_spool(root)
    (root / "depth_camera_characteristics.json").unlink()
    depth_path = root / "depth" / "left_1000000000.bin"
    depth_path.write_bytes(
        struct.pack("<4e", *(openxr_window_depth(depth) for depth in (0.1, 1.0, 2.0, 10.0)))
    )
    write_jsonl(
        root / "depth" / "frames.jsonl",
        [
            {
                "timestamp_ns": 1_000_000_000,
                "eye": "left",
                "image_path": "depth/left_1000000000.bin",
                "width": 2,
                "height": 2,
                "metadata": {
                    "depth_encoding": "openxr_window_depth_normalized",
                    "sample_storage": "f16_le",
                    "depth_inverse_projection_view_columns": inverse_projection_columns_for_z(),
                    "fov_tangent": {"left": 1.0, "right": 1.0, "top": 1.0, "bottom": 1.0},
                },
            }
        ],
    )
    width = height = 64
    y_size = width * height
    chroma_size = (width // 2) * (height // 2)
    planes = [
        {"index": 0, "row_stride": width, "pixel_stride": 1, "offset": 0, "size": y_size},
        {"index": 1, "row_stride": width // 2, "pixel_stride": 1, "offset": y_size, "size": chroma_size},
        {
            "index": 2,
            "row_stride": width // 2,
            "pixel_stride": 1,
            "offset": y_size + chroma_size,
            "size": chroma_size,
        },
    ]
    left_records = []
    right_records = []
    for frame_id, timestamp_ns in enumerate((1_000_000_000, 1_033_333_000, 1_066_666_000)):
        left_name = f"{timestamp_ns}.yuv"
        right_name = f"{timestamp_ns + 1_000_000}.yuv"
        left_payload = bytes([48 + 8 * frame_id]) * y_size + bytes([128]) * (2 * chroma_size)
        right_payload = bytes([96 + 8 * frame_id]) * y_size + bytes([128]) * (2 * chroma_size)
        (root / "left_camera_raw" / left_name).write_bytes(left_payload)
        (root / "right_camera_raw" / right_name).write_bytes(right_payload)
        left_records.append(
            {
                "timestamp_ns": timestamp_ns,
                "eye": "left",
                "raw_path": f"left_camera_raw/{left_name}",
                "width": width,
                "height": height,
                "format": 35,
                "encoding": "YUV_420_888_planes",
                "planes": planes,
            }
        )
        right_records.append(
            {
                "timestamp_ns": timestamp_ns + 1_000_000,
                "eye": "right",
                "raw_path": f"right_camera_raw/{right_name}",
                "width": width,
                "height": height,
                "format": 35,
                "encoding": "YUV_420_888_planes",
                "planes": planes,
            }
        )
    write_jsonl(root / "left_camera_frames.jsonl", left_records)
    write_jsonl(root / "right_camera_frames.jsonl", right_records)


def test_pack_session_outputs_spatialmp4_payload_contract(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)

    manifest = packer.pack_session(session, output)

    assert manifest["schema"] == "spatialmp4.packet_inputs.v1"
    assert manifest["streams"]["rgb"]["frame_count"] == 1
    assert manifest["streams"]["depth"]["frame_count"] == 1
    assert manifest["streams"]["head_pose"]["packet_count"] == 1
    assert manifest["streams"]["sidecars"]["controllers.jsonl"]["records"] == 1
    assert manifest["camera_metadata"]["left"]["intrinsics"]["fx"] == 100.0
    assert manifest["muxer_inputs"]["side_data"]["rgb_icam"]["bytes"] == 64
    assert manifest["validation"]["muxer_inputs"]["side_data"]["rgb_dstr"]["bytes"] == 80

    depth_index = (output / "depth" / "depth_frames.jsonl").read_text().strip()
    depth_record = json.loads(depth_index)
    depth_values = struct.unpack(
        "<4H",
        (output / depth_record["path"]).read_bytes(),
    )
    assert depth_values == (100, 1200, 65535, 0)

    pose_payload = (output / "pose" / "head_pose_mett.bin").read_bytes()
    pose_values = struct.unpack("<7d", pose_payload)
    assert pose_values == (1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.4)

    rgb_pair = json.loads((output / "rgb" / "stereo_pairs.jsonl").read_text())
    assert rgb_pair["pair_delta_ns"] == 1_000_000
    assert rgb_pair["left_metadata"]["width"] == 2
    assert rgb_pair["left_metadata"]["planes"][0]["row_stride"] == 2
    assert (output / rgb_pair["left"]).read_bytes() == b"leftuvuv"
    assert (output / rgb_pair["right"]).read_bytes() == b"rghtuvuv"

    muxer_metadata = json.loads((output / "muxer" / "metadata.json").read_text())
    assert muxer_metadata["schema"] == "spatialmp4.muxer_inputs.v1"
    assert muxer_metadata["rgb"]["cam_count"] == 2
    assert muxer_metadata["depth"]["target_codec_tag"] == "raw1"
    assert struct.unpack("<8d", (output / "muxer" / "rgb_icam.bin").read_bytes())[:4] == (
        100.0,
        101.0,
        50.0,
        51.0,
    )
    assert struct.unpack("<4d", (output / "muxer" / "depth_icam.bin").read_bytes()) == (
        80.0,
        81.0,
        1.0,
        1.0,
    )


def test_pack_session_skips_trailing_partial_jsonl_record(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)
    for rel_path in (
        "left_camera_frames.jsonl",
        "depth/frames.jsonl",
        "poses/head.jsonl",
        "poses/controllers.jsonl",
    ):
        with (session / rel_path).open("a", encoding="utf-8") as f:
            f.write('{"timestamp_ns":')

    manifest = packer.pack_session(session, output)

    assert manifest["streams"]["rgb"]["frame_count"] == 1
    assert manifest["streams"]["depth"]["frame_count"] == 1
    assert manifest["streams"]["head_pose"]["packet_count"] == 1


def test_pack_session_requires_controller_or_hand_sidecar(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)
    (session / "poses" / "controllers.jsonl").unlink()
    (session / "poses" / "hands.jsonl").unlink()

    with pytest.raises(packer.PackError, match="controller or hand"):
        packer.pack_session(session, output)


def test_pack_session_rejects_empty_controller_and_hand_sidecars(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)
    (session / "poses" / "controllers.jsonl").write_text("", encoding="utf-8")
    (session / "poses" / "hands.jsonl").write_text("", encoding="utf-8")

    with pytest.raises(packer.PackError, match="controller or hand pose samples"):
        packer.pack_session(session, output)


def test_pack_session_converts_godot_openxr_depth_with_inverse_projection(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)
    (session / "depth_camera_characteristics.json").unlink()

    depth_path = session / "depth" / "left_1000000000.bin"
    depth_path.write_bytes(
        struct.pack("<4e", *(openxr_window_depth(depth) for depth in (0.1, 1.0, 2.0, 10.0)))
    )
    write_jsonl(
        session / "depth" / "frames.jsonl",
        [
            {
                "timestamp_ns": 1_000_000_000,
                "eye": "left",
                "image_path": "depth/left_1000000000.bin",
                "width": 2,
                "height": 2,
                "metadata": {
                    "depth_encoding": "openxr_window_depth_normalized",
                    "sample_storage": "f16_le",
                    "depth_inverse_projection_view_columns": inverse_projection_columns_for_z(),
                    "fov_tangent": {"left": 1.0, "right": 1.0, "top": 1.0, "bottom": 1.0},
                },
            }
        ],
    )

    manifest = packer.pack_session(session, output)
    depth_record = json.loads((output / "depth" / "depth_frames.jsonl").read_text().strip())
    depth_values = struct.unpack("<4H", (output / depth_record["path"]).read_bytes())
    expected = (100, 1000, 2000, 10000)
    assert all(abs(actual - reference) <= 30 for actual, reference in zip(depth_values, expected))
    assert manifest["camera_metadata"]["depth"]["calibration_source"] == "openxr_depth_fov"
    assert struct.unpack("<4d", (output / "muxer" / "depth_icam.bin").read_bytes()) == (
        1.0,
        1.0,
        1.0,
        1.0,
    )


def test_pack_session_converts_quest_reality_openxr_depth_with_near_far(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)
    (session / "depth" / "frames.jsonl").unlink()
    (session / "depth_camera_characteristics.json").unlink()
    (session / "left_depth").mkdir()
    (session / "left_depth" / "1000.raw").write_bytes(
        struct.pack("<4f", *(openxr_window_depth(depth) for depth in (0.1, 1.0, 2.0, 10.0)))
    )
    (session / "left_depth_descriptors.csv").write_text(
        "timestamp_ms,near_z,far_z,width,height,fov_left_angle_tangent,fov_right_angle_tangent,"
        "fov_top_angle_tangent,fov_down_angle_tangent\n"
        "1000,0.1,10.0,2,2,1.0,1.0,1.0,1.0\n",
        encoding="utf-8",
    )

    manifest = packer.pack_session(session, output)
    depth_record = json.loads((output / "depth" / "depth_frames.jsonl").read_text().strip())
    depth_values = struct.unpack("<4H", (output / depth_record["path"]).read_bytes())
    assert depth_values == (100, 1000, 2000, 10000)
    assert manifest["camera_metadata"]["depth"]["intrinsics"]["fx"] == 1.0
    assert struct.unpack("<4d", (output / "muxer" / "depth_icam.bin").read_bytes()) == (
        1.0,
        1.0,
        1.0,
        1.0,
    )


def test_pack_session_converts_godot_d16_unorm_openxr_depth(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)
    (session / "depth_camera_characteristics.json").unlink()

    depths = (0.1, 1.0, 2.0, 10.0)
    normalized = [min(65535, max(0, round(openxr_window_depth(depth) * 65535))) for depth in depths]
    (session / "depth" / "left_1000000000.bin").write_bytes(struct.pack("<4H", *normalized))
    write_jsonl(
        session / "depth" / "frames.jsonl",
        [
            {
                "timestamp_ns": 1_000_000_000,
                "eye": "left",
                "image_path": "depth/left_1000000000.bin",
                "width": 2,
                "height": 2,
                "metadata": {
                    "depth_encoding": "openxr_window_depth_normalized",
                    "sample_storage": "u16_unorm_le",
                    "depth_inverse_projection_view_columns": inverse_projection_columns_for_z(),
                    "fov_tangent": {"left": 1.0, "right": 1.0, "top": 1.0, "bottom": 1.0},
                },
            }
        ],
    )

    packer.pack_session(session, output)
    depth_record = json.loads((output / "depth" / "depth_frames.jsonl").read_text().strip())
    depth_values = struct.unpack("<4H", (output / depth_record["path"]).read_bytes())
    expected = (100, 1000, 2000, 10000)
    assert all(abs(actual - reference) <= 30 for actual, reference in zip(depth_values, expected))


def test_pack_session_rejects_legacy_unconverted_openxr_depth(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)

    write_jsonl(
        session / "depth" / "frames.jsonl",
        [
            {
                "timestamp_ns": 1_000_000_000,
                "eye": "left",
                "image_path": "depth/left_1000000000.bin",
                "width": 2,
                "height": 2,
                "metadata": {"image_format": 12, "has_depth_projection_view": True},
            }
        ],
    )

    with pytest.raises(packer.PackError, match="without near/far or inverse projection-view"):
        packer.pack_session(session, output)


def test_validate_packet_package_rejects_corrupt_depth_payload(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)
    packer.pack_session(session, output)

    depth_record = json.loads((output / "depth" / "depth_frames.jsonl").read_text().strip())
    (output / depth_record["path"]).write_bytes(b"\x00")

    with pytest.raises(packer.PackError, match="Depth payload size mismatch"):
        packer.validate_packet_package(output)


def test_validate_packet_package_rejects_corrupt_muxer_side_data(tmp_path):
    packer = load_packer_module()
    session = tmp_path / "session"
    output = tmp_path / "packet"
    session.mkdir()
    make_spool(session)
    packer.pack_session(session, output)

    (output / "muxer" / "rgb_icam.bin").write_bytes(b"\x00")

    with pytest.raises(packer.PackError, match="Muxer side data size mismatch"):
        packer.validate_packet_package(output)


def test_patched_muxer_writes_spatialmp4_streams_and_metadata(tmp_path):
    muxer_path = os.environ.get("SPATIALMP4_PACKET_MUXER")
    ffprobe_path = os.environ.get("SPATIALMP4_FFPROBE")
    reader_smoke_path = os.environ.get("SPATIALMP4_READER_SMOKE")
    if not muxer_path or not ffprobe_path:
        pytest.skip("Set SPATIALMP4_PACKET_MUXER and SPATIALMP4_FFPROBE for patched FFmpeg integration test")

    packer = load_packer_module()
    session = tmp_path / "session"
    packet = tmp_path / "packet"
    output_mp4 = tmp_path / "capture.mp4"
    session.mkdir()
    make_muxable_rgb_spool(session)
    packer.pack_session(session, packet)
    depth_record = json.loads((packet / "depth" / "depth_frames.jsonl").read_text().strip())
    depth_values = struct.unpack("<4H", (packet / depth_record["path"]).read_bytes())
    assert all(abs(actual - reference) <= 30 for actual, reference in zip(depth_values, (100, 1000, 2000, 10000)))

    result = subprocess.run(
        [muxer_path, str(packet), str(output_mp4)],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "3 RGB packets" in result.stdout
    assert output_mp4.stat().st_size > 0

    probe_result = subprocess.run(
        [ffprobe_path, "-v", "error", "-show_streams", "-of", "json", str(output_mp4)],
        capture_output=True,
        text=True,
        check=True,
    )
    streams = json.loads(probe_result.stdout)["streams"]
    rgb_stream = next(stream for stream in streams if stream["codec_name"] == "hevc")
    depth_stream = next(
        stream
        for stream in streams
        if stream.get("codec_tag_string") == "raw1" or "icam_0" in stream.get("tags", {})
    )
    pose_stream = next(stream for stream in streams if stream.get("codec_tag_string") == "mett")

    assert rgb_stream["width"] == 128
    assert rgb_stream["height"] == 64
    assert rgb_stream["tags"]["icam_0"]
    assert rgb_stream["tags"]["icam_1"]
    assert rgb_stream["tags"]["timebase"] == "1700000000000000"
    assert depth_stream["tags"]["icam_0"]
    assert depth_stream["tags"]["timebase"] == "1700000000000000"
    assert depth_stream["r_frame_rate"] != "1/0"
    assert depth_stream["avg_frame_rate"] != "0/0"
    assert pose_stream["codec_type"] == "data"
    assert pose_stream["nb_frames"] == "1"

    if reader_smoke_path:
        reader_result = subprocess.run(
            [reader_smoke_path, str(output_mp4)],
            capture_output=True,
            text=True,
            check=True,
        )
        assert "reader loaded RGB" in reader_result.stdout
