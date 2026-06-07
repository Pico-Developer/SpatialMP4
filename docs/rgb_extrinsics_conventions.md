# RGB Extrinsics: Per-Device Conventions

Status: Documentation of current behavior — do **not** change the
`rgb_extrinsics` descriptor name or layout without a coordinated reader
update. Field name is kept for historical / cross-device compatibility
(Pico, etc.) even though its exact semantics vary by capture device.

## Why this doc exists

The `rgb_extrinsics` field in a SpatialMP4 file is **not** a single,
device-agnostic "RGB camera-to-IMU" transform. Different capture chains
write different things into it because each device's underlying API
returns the lens pose in its own convention. Consumers that blindly use
`Reader::GetRgbExtrinsicsLeft()` will get **silently misaligned** depth/RGB
overlays, point clouds, and 3D reconstructions whenever the device-specific
convention does not match what they assumed.

This document captures what each writer puts into `rgb_extrinsics` and
what conversion the consumer must apply before composing it with head /
depth poses.

## Field layout

`rgb_extrinsics` is a 4×4 `cv::Matx44d`:

```
[ R(3x3)   t(3x1) ]
[   0       1     ]
```

- `t` is the translation in metres
- `R` is a rotation matrix
- Together they describe a 6-DoF SE(3) transform

The semantic meaning of `R` and `t` is **device-specific** (see below).

## Per-device conventions

### Pico (legacy capture chain)

`rgb_extrinsics` is a **clean** RGB-camera-frame → IMU SE(3) transform:

- `t` is the RGB lens position **in the IMU frame** (metres)
- `R` rotates a vector from the RGB camera local frame (OpenCV-style:
  X-right, Y-down, Z-forward) into the IMU frame (X-right, Y-up, Z-back
  matches OpenXR / Quest convention)
- Composition rule: `T_W_S = T_W_H @ rgb_extrinsics` gives the world-to-
  RGB-camera-frame transform directly, where `T_W_H` is the head pose
  from the `head` timed-metadata track

Consumers can use this value directly. No additional conversion needed.

### Quest 3 (Godot live writer)

`rgb_extrinsics` is the **raw Android Camera2 `lens_pose_translation` and
`lens_pose_rotation`**, converted to a 4×4 matrix by the naïve formula:

```
R = quaternion_to_matrix(lens_pose_rotation)   # standard SO(3) formula
t = lens_pose_translation                      # copied verbatim
```

This is **not** a usable RGB-camera-to-IMU transform on its own. The
Android Camera2 `LENS_POSE_ROTATION` field does not describe a standard
SO(3) rotation between OpenCV-style camera frame and the IMU — it
describes the lens orientation in a Camera2-internal reference frame
whose Y-axis interpretation depends on the device's
`SENSOR_INFO_ACTIVE_ARRAY_ORIENTATION` and whose direction relative to
the IMU has to be reversed.

Quest 3 consumers must run the following conversion (port of
`_openquest_head_from_camera` in
[`scripts/godot_depth_rgb_align.py`](../scripts/godot_depth_rgb_align.py)
and originally from the
[Meta Quest 3D Reconstruction](https://github.com/t-34400/QuestRealityCapture/)
project) on the raw quaternion + translation:

```python
def quest_camera2_to_head_unity(t_raw, q_raw):
    """Convert raw Camera2 lens_pose to the head-from-camera SE(3) used by
    OpenXR depth pose composition."""
    # 1. Negate translation Z: Android "back of device" Z → Unity / OpenXR
    #    "forward" Z
    t_unity = (t_raw[0], t_raw[1], -t_raw[2])

    # 2. Negate quaternion X and Y components: reflect the rotation axis
    #    through the X-Y plane (Quest-3-specific axis remap)
    qx, qy, qz, qw = q_raw
    converted = (-qx, -qy, qz, qw)

    # 3. Transpose the rotation matrix: invert the rotation (Camera2's
    #    quaternion describes "from IMU to camera", we need "from camera
    #    to head/world")
    R_converted = quaternion_to_matrix(converted)
    R_unity = R_converted.T @ np.diag([1.0, -1.0, -1.0])

    # 4. (diag(1, -1, -1) right-multiplication is the OpenCV → OpenXR axis
    #    flip baked into the final rotation)

    T = np.eye(4)
    T[:3, :3] = R_unity
    T[:3, 3]  = t_unity
    return T
```

The result is the proper `head_from_camera_unity` transform. Compose
with the head pose (also converted Godot → Unity via
`diag(1, 1, -1)` basis change) and apply the inverse to depth-camera
points to get RGB pixel projections that align with the captured RGB
frames.

Without these 4 steps, the effective RGB camera tilt is wrong by
**roughly 15°** (Camera2 reports the camera looking ~10° up from
horizontal; the true Quest 3 mount is ~5° down). This bias is enough to
visibly misalign depth point clouds, overlaid heatmaps, and any RGBD
fusion driven by `rgb_extrinsics`.

#### Where to find the raw data for Quest 3

For Quest 3 captures produced by `GodotQuestCapture`, the raw Camera2
data is also written out as a sidecar:

```
<session>/left_camera_characteristics.json    # left eye
<session>/right_camera_characteristics.json   # right eye
```

Each contains the original `lens_pose_translation`, `lens_pose_rotation`,
`lens_intrinsic_calibration`, and the
`sensor_info_active_array_orientation` needed to disambiguate the Camera2
sensor frame. Consumers should prefer these sidecar values over the
naïvely-converted mp4 `rgb_extrinsics` whenever the sidecar is present.

A reference implementation that performs the proper conversion is
[`scripts/godot_depth_rgb_align.py`](../scripts/godot_depth_rgb_align.py),
specifically the `_openquest_head_from_camera` helper.

The visualizer example
[`examples/python/visualize_rerun_full.py`](../examples/python/visualize_rerun_full.py)
demonstrates loading the sidecar, applying the conversion, and using the
proper transform to overlay depth on RGB.

## Decision tree for consumers

```
                   ┌────────────────────────────────┐
                   │ Reading rgb_extrinsics from    │
                   │ a SpatialMP4 file              │
                   └──────────────┬─────────────────┘
                                  │
                ┌─────────────────┴─────────────────┐
                │ What writer produced this file?   │
                └─────────────────┬─────────────────┘
                                  │
              ┌───────────────────┼──────────────────────┐
              ▼                   ▼                      ▼
           ┌──────┐         ┌──────────┐         ┌──────────────┐
           │ Pico │         │ Quest 3  │         │  Unknown /   │
           │legacy│         │ (Godot   │         │  3rd party   │
           │      │         │ live)    │         │              │
           └───┬──┘         └────┬─────┘         └──────┬───────┘
               │                 │                      │
               │ Use as-is.      │ Convert via the      │ Treat as
               │ Already an      │ 4-step               │ Pico-style
               │ RGB→IMU SE(3).  │ Camera2-to-Unity     │ unless the
               │                 │ transform on raw     │ writer
               │                 │ sidecar values.      │ documents
               │                 │                      │ otherwise.
               ▼                 ▼                      ▼
        ┌──────────────────────────────────────────────────────┐
        │ T_W_S = T_W_H @ rgb_extrinsics_effective             │
        │                                                       │
        │ Project depth points or compose with depth_extrinsics │
        │ for warping / pointcloud generation.                  │
        └──────────────────────────────────────────────────────┘
```

## Detecting the writer

There is currently no `writer` field in the mp4 metadata that
unambiguously identifies the source device. Heuristics consumers can use:

- The presence of `<session>/depth/frames.jsonl` and
  `<session>/left_camera_characteristics.json` sidecar files alongside
  the mp4 strongly indicates a Godot Quest 3 capture (GodotQuestCapture).
  If these sidecars exist, treat `rgb_extrinsics` as raw Camera2 and use
  the sidecar values + conversion.
- A Pico capture pipeline does not produce these sidecars and writes
  `rgb_extrinsics` as a ready-to-use SE(3).
- If neither sidecar nor known origin is available, fall back to Pico
  semantics and accept the ~15° bias risk for Quest 3 captures, or
  document the assumption in the consumer.

A future writer should set a `capture_source` / `rgb_extrinsics_convention`
key in the mp4 stream metadata (`hdlr` box or a dedicated descriptor)
so consumers can dispatch deterministically.

## TODO / Future direction

1. Add a `rgb_extrinsics_convention` string to the mp4 metadata
   ("pico_se3" / "quest_camera2_raw") so consumers can branch without
   sniffing for sidecars.
2. Provide a `Reader::GetRgbExtrinsicsLeftConverted()` helper in
   `spatialmp4` that applies the per-convention conversion automatically
   based on the metadata flag.
3. Mirror this doc into the Python bindings README.
4. Cross-reference from `Reader::GetRgbExtrinsicsLeft()` doxygen comments
   so consumers see the convention warning at the API surface.

## See also

- [`scripts/godot_depth_rgb_align.py`](../scripts/godot_depth_rgb_align.py)
  — full reference pipeline for Quest 3 captures
- [`examples/python/visualize_rerun_full.py`](../examples/python/visualize_rerun_full.py)
  — visualizer that consumes both conventions
- [`docs/godot_quest3_capture_rfc.md`](godot_quest3_capture_rfc.md)
  — writer-side RFC describing how Quest 3 data is produced
- [Meta Quest 3D Reconstruction](https://github.com/t-34400/QuestRealityCapture)
  — origin of the Camera2-to-Unity conversion
