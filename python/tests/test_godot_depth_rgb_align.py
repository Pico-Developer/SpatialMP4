import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest


def load_alignment_module():
    script = Path(__file__).resolve().parents[2] / "scripts" / "godot_depth_rgb_align.py"
    spec = importlib.util.spec_from_file_location("godot_depth_rgb_align", script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


align = load_alignment_module()


def test_jsonl_reader_ignores_truncated_final_capture_record(tmp_path):
    path = tmp_path / "records.jsonl"
    path.write_text('{"timestamp_ns": 1}\n{"timestamp_ns":', encoding="utf-8")
    assert align._read_jsonl(path) == [{"timestamp_ns": 1}]


def test_linearizes_environment_window_depth():
    result = align.linearize_window_depth(np.array([0.0, 0.5]), 0.1, None)
    assert result[0] == pytest.approx(0.1)
    assert result[1] == pytest.approx(0.2)


def test_projection_z_buffers_nearest_point():
    points = np.array([[0.0, 0.0, 2.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    intrinsic = np.array([[100.0, 0.0, 1.0], [0.0, 100.0, 1.0], [0.0, 0.0, 1.0]])
    depth, count = align.project_to_rgb(points, np.eye(4), intrinsic, 3, 3)
    assert count == 2
    assert depth[1, 1] == pytest.approx(1.0)


def test_rgb_pairing_prefers_runtime_and_camera_sensor_time(tmp_path):
    depth_frame = align.DepthFrame(
        timestamp_ns=1_000,
        path=tmp_path / "depth",
        width=1,
        height=1,
        metadata={"runtime_display_time_ns": 10_000},
    )
    callback_nearest = align.RgbFrame(1_001, 30_000, tmp_path / "a", 1, 1, [])
    sensor_nearest = align.RgbFrame(2_000, 10_002, tmp_path / "b", 1, 1, [])
    selected, domain, delta = align.nearest_rgb_frame([callback_nearest, sensor_nearest], depth_frame, 100)
    assert selected == sensor_nearest
    assert domain == "openxr_runtime_display_time_vs_camera_sensor_time"
    assert delta == 2


def test_unprojects_with_inverse_projection_view_in_local_space(tmp_path):
    metadata = {
        "local_from_depth_eye": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "depth_inverse_projection_view_columns": np.eye(4).T.tolist(),
    }
    frame = align.DepthFrame(0, tmp_path / "unused", 1, 1, metadata)
    points, report = align.unproject_depth_to_unity(
        frame, np.array([[0.5]], dtype=np.float64), min_depth_m=0.0, max_depth_m=2.0
    )
    assert report["unprojection"] == "depth_inverse_projection_view"
    assert points.shape == (1, 3)
    assert points[0].tolist() == pytest.approx([0.0, 0.0, 0.0])


def test_refuses_unmodelled_rgb_distortion(tmp_path):
    metadata = {
        "sensor_active_array_size": {"left": 0, "top": 0, "right": 2, "bottom": 2},
        "lens_intrinsic_calibration": [1.0, 1.0, 1.0, 1.0, 0.0],
        "lens_pose_translation": [0.0, 0.0, 0.0],
        "lens_pose_rotation": [0.0, 0.0, 0.0, 1.0],
        "lens_distortion": [0.1],
    }
    (tmp_path / "left_camera_characteristics.json").write_text(json.dumps(metadata), encoding="utf-8")
    with pytest.raises(align.AlignmentError, match="distortion"):
        align.load_camera_calibration(tmp_path, "left", ignore_distortion=False)
