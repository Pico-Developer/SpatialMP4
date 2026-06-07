# RFC: Godot Quest 3 Capture to SpatialMP4

Status: Prototype implemented; Quest 3 RGB/depth/pose sample validated
Date: 2026-05-26

## Summary

Build a Godot 4 application for Quest 3 / Quest 3S that captures:

- stereo passthrough RGB frames,
- environment depth frames,
- head pose,
- controller pose and/or hand pose,
- camera calibration and timing metadata,

and packages the result as a SpatialMP4-compatible recording.

The practical split is:

1. Godot app captures and spools synchronized raw session data.
2. A SpatialMP4 packer converts metric-depth spool data into MP4 streams matching the current reader.
3. Once the packer is stable, the same path can run on-device with Android MediaCodec and the patched FFmpeg muxer.

## Sources

Current codebase evidence:

- `QuestRealityCapture` already logs HMD/controller poses, stereo passthrough YUV images, Camera2 metadata, and depth maps.
- `SpatialMP4` currently reads RGB as HEVC side-by-side stereo, depth as raw `raw1` uint16 millimeter frames, and head pose as `mett` packets containing 7 doubles.

External API references:

- Godot XR setup: https://docs.godotengine.org/en/stable/tutorials/xr/setting_up_xr.html
- Godot XRCamera3D: https://docs.godotengine.org/en/stable/classes/class_xrcamera3d.html
- Godot XRHandTracker: https://docs.godotengine.org/en/stable/classes/class_xrhandtracker.html
- Godot Android plugin v2: https://docs.godotengine.org/en/stable/tutorials/platform/android/android_plugin.html
- Godot OpenXR Vendors Meta Environment Depth: https://godotvr.github.io/godot_openxr_vendors/manual/meta/environment_depth.html
- Godot OpenXR Vendors Meta depth implementation: https://github.com/GodotVR/godot_openxr_vendors/blob/master/plugin/src/main/cpp/extensions/openxr_meta_environment_depth_extension.cpp
- Godot OpenXR Vendors `4.2.0-stable` release used by the prototype: https://github.com/GodotVR/godot_openxr_vendors/releases/tag/4.2.0-stable
- OpenXR `XrEnvironmentDepthImageMETA`: https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrEnvironmentDepthImageMETA.html
- Godot OpenXR Vendors Meta Hand Tracking: https://godotvr.github.io/godot_openxr_vendors/manual/meta/hand_tracking.html
- Meta Passthrough Camera API overview: https://developers.meta.com/horizon/documentation/unity/unity-pca-overview/
- Android Camera2 API: https://developer.android.com/reference/android/hardware/camera2/package-summary

## Goals

- Capture Quest 3 / 3S RGB using the official Passthrough Camera API surface, which is Android Camera2 with Meta vendor tags.
- Capture environment depth through Meta OpenXR environment depth.
- Capture head pose and controller/hand poses from OpenXR/Godot.
- Preserve monotonic timestamps, Unix base time, camera intrinsics, camera extrinsics, distortion, and frame dimensions.
- Produce MP4 files that the current `SpatialML::Reader` can load without changing its RGB/depth/head-pose assumptions.
- Define a forward-compatible extension for controller and hand-joint tracks.

## Non-Goals

- Do not use MediaProjection. It records the user's composited view and UI, not unobstructed passthrough RGB.
- Do not treat Godot's passthrough composition layer as RGB camera access. It is for rendering, not frame extraction.
- Do not require every MVP recording to be muxed on-device. Offline packing is acceptable for the first verified version.

## Current SpatialMP4 Compatibility Contract

The current reader classifies streams by codec name and metadata:

- RGB stream: codec name `hevc`. It expects stereo side-by-side: stream width is `2 * per_eye_width`, height is per-eye height.
- Depth stream: codec name `none` plus metadata key `icam_0`. It expects every depth packet to be a keyframe.
- Pose stream: codec name `none` without `icam_0`. It loads all packets as head pose.

Payload requirements:

- RGB packet payload: HEVC video frame. Decoded frame is split into left/right halves.
- Depth packet payload: `uint16` little-endian depth image, one channel, unit millimeters. Reader converts to meters with scale `0.001`.
- Head pose packet payload: 7 little-endian `double` values in this exact order:

```text
x, y, z, qx, qy, qz, qw
```

Required metadata:

- RGB: `icam_0`, `icam_1`, `ecam_0`, `ecam_1`, `distortion_model`, `distortion_params_0`, `distortion_params_1`, `timebase`.
- Depth: `icam_0`, `ecam_0`, `distortion_model`, `distortion_params_0`, `timebase`.
- Pose: `pose_coordinate`, `data_accuracy`, `pose_position=head`, `mime_type=application/pose`, `track_base_time`.

Important mismatch with `QuestRealityCapture`:

- Existing Unity capture writes RGB as per-camera raw `YUV_420_888`, not HEVC side-by-side.
- Existing Unity depth writes float32 normalized window-depth samples with per-frame `near_z`/`far_z`, while SpatialMP4 expects uint16 millimeter linear Z depth.
- Existing SpatialMP4 reader has one primary pose stream; controller/hand pose must be sidecar for MVP or added as a reader extension.

## Proposed Architecture

```text
Godot scene
  CaptureApp.gd
    PoseSampler.gd
    DepthSampler.gd
    Android QuestCapturePlugin singleton
    SessionSpoolWriter.gd

Android plugin v2 / Kotlin
  QuestCapturePlugin
    Permission bridge
    Camera2 enumerator
    Camera2 capture sessions
    ImageReader YUV frame writer or callback
    Camera metadata exporter

OpenXR Vendors plugin / GDExtension
  Environment depth access
  Hand tracking access

SpatialMP4 packer
  Reads session spool
  Converts YUV -> RGB/NV12
  Builds side-by-side stereo
  Encodes HEVC
  Converts float/meter depth -> uint16/mm raw1
  Writes head pose mett packets
  Optionally writes controller/hand extension tracks
```

## Godot Project Design

Required scene nodes:

- `XROrigin3D`
- `XRCamera3D`
- `XRController3D` for `/user/hand/left`
- `XRController3D` for `/user/hand/right`
- Optional `XRNode3D` for `/user/hand_tracker/left`
- Optional `XRNode3D` for `/user/hand_tracker/right`

Startup:

1. Find and initialize `OpenXR`.
2. Set `get_viewport().use_xr = true`.
3. Confirm Quest standalone export uses the Compatibility renderer.
4. Enable Godot OpenXR Vendors plugin Meta features:
   - passthrough; this was required for environment-depth provider startup on the tested Quest 3,
   - environment depth,
   - hand tracking,
   - simultaneous hands/controllers if the app needs both on Quest 3 / Quest Pro.

Pose capture options:

- MVP: sample `XRCamera3D.global_transform`, `XRController3D.global_transform`, and `XRHandTracker` joints each process tick.
- Production: expose a native OpenXR GDExtension method that samples poses at the RGB/depth frame timestamp. This avoids the documented `XRCamera3D` few-millisecond lag relative to render-thread HMD tracking.

## RGB Capture

Use a Godot Android plugin v2 written in Kotlin/Java. Godot's JavaClassWrapper is acceptable for low-frequency calls, but frame capture must stay inside the Android plugin to avoid GDScript/JNI copies.

The plugin must:

1. Declare permissions:

```xml
<uses-permission android:name="android.permission.CAMERA" />
<uses-permission android:name="horizonos.permission.HEADSET_CAMERA" />
```

2. Request runtime camera permission.
3. Use `CameraManager.getCameraIdList()`.
4. Filter Camera2 devices using Meta vendor tags:

```text
com.meta.extra_metadata.camera_source == 0
com.meta.extra_metadata.position == 0 for left camera
com.meta.extra_metadata.position == 1 for right camera
```

These vendor-tag values are `byte[]` in the tested Camera2 implementation, not
`Integer`. Locate the native `CameraCharacteristics.Key` in
`characteristics.keys`, then read its first byte. Constructing an `Integer`
key silently excludes both passthrough cameras.

5. Query Camera2 metadata:

- `LENS_INTRINSIC_CALIBRATION` -> `fx, fy, cx, cy, skew`
- `LENS_DISTORTION`
- `LENS_POSE_TRANSLATION`
- `LENS_POSE_ROTATION`
- sensor pixel/active/physical size
- timestamp source

6. Create one `ImageReader` per eye with `YUV_420_888`.
7. For each image, record:

- `camera_id`
- `eye`
- `android_sensor_timestamp_ns`
- app monotonic timestamp if available
- Unix timestamp derived from session base
- width, height, format
- plane row/pixel strides
- Y/U/V payload

MVP spool format can preserve QuestRealityCapture-style files:

```text
session/
  manifest.json
  left_camera_characteristics.json
  right_camera_characteristics.json
  left_camera_image_format.json
  right_camera_image_format.json
  left_camera_frames.jsonl
  right_camera_frames.jsonl
  android_timebase.json
  left_camera_raw/<mono_ns>.yuv
  right_camera_raw/<mono_ns>.yuv
```

Each `*_camera_frames.jsonl` record contains the frame timestamp, raw path, width, height, Android image format, and per-plane layout:

```json
{
  "timestamp_ns": 1000000000,
  "eye": "left",
  "raw_path": "left_camera_raw/1000000000.yuv",
  "width": 1280,
  "height": 960,
  "format": 35,
  "encoding": "YUV_420_888_planes",
  "timestamp_domain": "godot_ticks_ns",
  "camera_sensor_timestamp_ns": 123456789000000,
  "capture_elapsed_realtime_ns": 123456790000000,
  "capture_unix_time_ms": 1760000000000,
  "camera_timestamp_source": 1,
  "planes": [
    {"index": 0, "row_stride": 1280, "pixel_stride": 1, "offset": 0, "size": 1228800},
    {"index": 1, "row_stride": 1280, "pixel_stride": 2, "offset": 1228800, "size": 614400},
    {"index": 2, "row_stride": 1280, "pixel_stride": 2, "offset": 1843200, "size": 614400}
  ]
}
```

`timestamp_ns` is in Godot `Time.get_ticks_usec()` nanosecond domain, not raw Android sensor time. The Android plugin receives the Godot session timebase through `configureSessionWithTime(...)` and writes `android_timebase.json`, so RGB timestamps can be matched against pose samples written by `PoseSampler.gd`.

The packer pairs left/right frames by nearest timestamp, converts each to RGB or NV12, lays them out side-by-side, and encodes HEVC.

## Depth Capture

MVP:

- Use Godot OpenXR Vendors `OpenXRMetaEnvironmentDepthExtensionWrapper` with the tested `4.2.0-stable` addon (`OpenXRMetaEnvironmentDepthExtension` in the newer v5 layout).
- Call `start_environment_depth()` after the OpenXR session begins.
- Request CPU data with `get_environment_depth_map_async(callback)`.
- Store the image, projection-view matrix, inverse projection-view matrix, callback timestamp approximation, width, height, and a portable relative image path.
- Do not request every render frame. The plugin docs warn that CPU readback is asynchronous, not current-frame data, and the depth sensor updates below render rate.
- Do not interpret the returned `Image` bytes as meters. The Meta swapchain is `GL_DEPTH_COMPONENT16` / `D16_UNORM`: Godot reports a single-channel `FORMAT_RH` image, but CPU readback bytes are normalized unsigned 16-bit samples (`u16_unorm_le`), not IEEE half floats. The Godot spool records it as `openxr_window_depth_normalized`.
- Apply `GodotQuestCapture/patches/godot_openxr_vendors_v4_meta_depth_capture.patch` and `GodotQuestCapture/patches/godot_openxr_vendors_v4_vulkan_depth_readback.patch` to the tested `4.2.0-stable` source build. The first exposes per-frame FOV, `nearZ`/`farZ`, runtime display time, and depth-eye pose; the second copies Vulkan `D16_UNORM` depth into a Godot-owned texture for CPU readback. `godot_openxr_vendors_meta_depth_capture.patch` targets the newer v5 source layout.
- Reject or omit non-finite runtime projection metadata when serializing JSON. An unpatched Quest run returned non-finite projection matrices; writing them as `nan` makes the spool invalid JSON.
- The offline packer converts each normalized sample to metric camera Z using the inverse projection-view matrix. With OpenGL clip coordinates, `inverse(P * V) * [ndc_x, ndc_y, 2 * d - 1, 1]` has homogeneous `w = 1 / z_meters`.

Production:

- Implement a GDExtension wrapper over `XR_META_environment_depth`.
- Export the depth image with runtime timestamp, `nearZ`/`farZ`, per-eye projection/view data, and confidence/validity if exposed.
- Prefer a native wrapper that attaches the acquisition/runtime image timestamp rather than the asynchronous callback timestamp.
- Convert depth to uint16 millimeters before muxing:

```text
uint16_mm = clamp(round(depth_meters * 1000.0), 0, 65535)
```

SpatialMP4 MVP should store one depth stream. Use left depth as canonical unless we extend reader semantics for stereo depth.

## Head, Controller, and Hand Pose

Head pose:

- MVP reads `XRCamera3D.global_transform`.
- Packer writes the nearest head pose for each pose sample into the primary `mett` pose track.
- Packet payload is 56 bytes: 7 doubles.

Controller pose:

- Use `XRController3D` nodes for left/right controller transforms.
- Store in session spool as CSV/JSONL:

```text
timestamp_ns, source, x, y, z, qx, qy, qz, qw, tracking_valid
```

Hand pose:

- Use `XRHandTracker` for `/user/hand_tracker/left` and `/user/hand_tracker/right`.
- Store 26 joint transforms when available:

```text
timestamp_ns, hand, joint_index, flags, radius_m, x, y, z, qx, qy, qz, qw
```

SpatialMP4 v1 compatibility:

- Keep controller and hand data in sidecar files inside the spool and optionally as extra non-primary MP4 data tracks.
- Existing reader ignores or misclassifies multiple pose streams, so the primary compatible MP4 must retain a single `pose_position=head` track.

SpatialMP4 v2 extension:

- Allow multiple `mett` pose streams.
- Add metadata:

```text
mime_type=application/pose
pose_position=head|left_controller|right_controller|left_hand_root|right_hand_root
pose_schema=spatialmp4.pose7.v1
```

- Add hand-joints stream:

```text
mime_type=application/vnd.spatialmp4.handjoints
pose_position=left_hand|right_hand
pose_schema=spatialmp4.hand26.v1
```

Suggested hand packet:

```text
uint32 magic = 'H26A'
uint16 version = 1
uint16 joint_count = 26
uint32 valid_mask
repeated 26:
  float32 radius_m
  float64 x, y, z, qx, qy, qz, qw
```

## Time Model

Use one session time model across all sources:

```text
session_start_unix_us
session_start_elapsed_realtime_ns
```

For each sample:

```text
sample_mono_ns = source timestamp in CLOCK_BOOTTIME or elapsedRealtimeNanos domain
sample_unix_us = session_start_unix_us
               + (sample_mono_ns - session_start_elapsed_realtime_ns) / 1000
mp4_pts = sample_unix_us - session_start_unix_us
mp4_time_base = 1/1000000
```

Rules:

- Use monotonic timestamps for matching.
- Use Unix time only as session metadata and filenames.
- Write `track_base_time=session_start_unix_us`.
- Poses are matched to RGB/depth by nearest monotonic timestamp. The current reader accepts only about 1 ms pose difference, so pose sampling must be at least display rate and preferably timestamp-aligned natively.

## Coordinate Model

Godot world coordinates and OpenXR local space are not automatically the same as SpatialMP4's expected world/head/sensor model.

Required transforms:

- `T_world_head` from OpenXR/Godot.
- `T_head_imu` or `T_imu_head` convention must be fixed.
- `T_imu_rgb_left`, `T_imu_rgb_right`, `T_imu_depth` from Camera2/OpenXR metadata.
- `T_world_sensor = T_world_head * T_head_imu * T_imu_sensor`.

Decision:

- Store raw Godot/OpenXR transforms in the spool.
- Store calibration transforms separately.
- Packer converts into SpatialMP4's expected `ecam` convention and validates with reprojection.

## SpatialMP4 Packer

The packer should live in `SpatialMP4` because it must share FFmpeg patches and reader tests.

Inputs:

```text
session/manifest.json
session/rgb/*.yuv or left/right raw folders
session/depth/*.raw or *.bin
session/poses/head.jsonl
session/poses/controllers.jsonl
session/poses/hands.jsonl
```

Outputs:

```text
recording.mp4
recording.sidecar.json  optional for v1 controller/hand data
```

Steps:

1. Load manifest and validate sensor metadata.
2. Pair RGB left/right frames by timestamp.
3. Convert YUV_420_888 to encoder input.
4. Encode HEVC side-by-side stereo.
5. Convert depth to uint16 millimeters and write raw1 packets. The packer accepts metric depth directly, converts QRC float normalized samples through `near_z`/`far_z`, and converts Godot `D16_UNORM` normalized samples through `depth_inverse_projection_view`.
6. Write head pose packets as 7 doubles.
7. Write metadata boxes through patched FFmpeg:
   - `icam`
   - `ecam`
   - `dstr`
   - `tbtm`
   - `mett`
8. Run reader smoke test on output.

Current implementation:

```bash
python3 scripts/godot_spool_pack.py <godot-session-dir> <packet-output-dir>
```

By default it fails if RGB pairs, depth frames, head pose, or controller/hand sidecars are missing. Use `--allow-partial` only for debugging incomplete captures.

Existing packet packages can be checked without re-packing:

```bash
python3 scripts/godot_spool_pack.py --validate-packet <packet-output-dir>
```

The validator enforces the muxer-facing contract: RGB payload files match their `YUV_420_888` plane offset/size metadata, depth payloads are exactly `width * height * 2` bytes, head pose packets are exactly 56 bytes each with valid offsets, camera intrinsics are present, and at least one controller or hand sidecar is present in strict mode.

This script creates a pre-mux packet package:

```text
packet-output-dir/
  package_manifest.json
  rgb/
    stereo_pairs.jsonl
    left_yuv/*.yuv
    right_yuv/*.yuv
  depth/
    depth_frames.jsonl
    raw1_u16_mm/*.raw
  pose/
    head_pose_packets.jsonl
    head_pose_mett.bin
  sidecars/
    controllers.jsonl
    hands.jsonl
  muxer/
    metadata.json
    rgb_icam.bin
    rgb_ecam.bin
    rgb_dstr.bin
    depth_icam.bin
    depth_ecam.bin
    depth_dstr.bin
```

The package is the validated intermediate input to the offline MP4 muxer. It enforces the SpatialMP4 reader payload contract for:

- depth: uint16 little-endian millimeters after linearization, suitable for the `raw1` depth track;
- head pose: 7 little-endian float64 values in `x,y,z,qx,qy,qz,qw` order, suitable for the primary `mett` pose track;
- RGB: paired left/right raw YUV frames with timestamp deltas, consumed by the muxer for side-by-side HEVC encoding.
- RGB pair records preserve per-eye `YUV_420_888` plane metadata when the Godot Android plugin emits `*_camera_frames.jsonl`; older QuestRealityCapture-style folders are still paired by timestamped file names as a fallback.
- muxer side-data: `muxer/*.bin` files are little-endian double arrays matching the patched FFmpeg writer's `AV_PKT_DATA_ICAM`, `AV_PKT_DATA_ECAM`, and `AV_PKT_DATA_DISTORTION_COEFFICIENTS` inputs.

The offline muxer reads this packet package, converts each `YUV_420_888` eye pair into one side-by-side YUV420 frame, encodes HEVC, writes `raw1` depth and `mett` head-pose packets, and emits the patched FFmpeg SpatialMP4 boxes (`icam`, `ecam`, `dstr`, `tbtm`, `mett`).

Writer-side FFmpeg support requires both local patches:

```bash
SKIP_DEPS=1 SKIP_OPENCV=1 FFMPEG_MINIMAL=1 bash scripts/build_ffmpeg.sh
```

`build_ffmpeg.sh` defaults to FFmpeg `8.1.1` (`n8.1.1`) and applies `ffmpeg_8_1_1_read.patch` for reading custom boxes plus `ffmpeg_8_1_1_enc.patch` for writing `raw1`, `mett`, `icam`, `ecam`, `dstr`, and `tbtm`. Omit `SKIP_DEPS=1` on a machine where the needed build packages are not installed; `FFMPEG_MINIMAL=1` avoids unrelated optional codecs during format validation.

Current muxer implementation:

```bash
cmake -S . -B build -DBUILD_PYTHON=OFF -DBUILD_TESTING=ON -DBUILD_READER_SMOKE=ON
cmake --build build --target spatialmp4_packet_muxer spatialmp4_reader_smoke
./build/spatialmp4_packet_muxer <packet-output-dir> <output.mp4>
```

This C++ muxer writes the RGB HEVC side-by-side track, the depth `raw1` track, and the primary head-pose `mett` track with `icam/ecam/dstr/tbtm` metadata. Depth and pose packets retain the packer's microsecond PTS values; the muxer also emits valid depth frame-rate and packet-duration information required by the existing reader.

Local verification on 2026-05-26:

- `python3 -m pytest python/tests/test_godot_spool_pack.py -q` passes.
- `cmake --build ... --target spatialmp4_packet_muxer spatialmp4_reader_smoke` passes against the local patched FFmpeg `8.1.1` (`n8.1.1`) build.
- Full offline integration with `SPATIALMP4_PACKET_MUXER`, patched `ffprobe`, and `SPATIALMP4_READER_SMOKE` passes: `10 passed`.
- A synthetic Godot spool containing `FORMAT_RH` normalized OpenXR-style depth converts to millimeters, validates, and muxes into an MP4 containing HEVC RGB, `raw1` depth, and `mett` pose streams.
- Patched `ffprobe` reads back RGB/depth `icam/ecam/dstr/tbtm` metadata and confirms the pose data track. The existing reader patch does not expose the embedded `mett` pose metadata as stream tags, but `SpatialML::Reader` classifies the pose stream by its `none` codec without `icam_0` and reads pose packet payloads directly.
- `spatialmp4_reader_smoke` loads RGB, depth, and pose from the generated synthetic MP4 through the existing `SpatialML::Reader`.
- Godot `4.5.1` plus OpenXR Vendors `4.2.0-stable` exported and ran on Quest 3 serial `2G0YC1ZF7S0C2D`.
- A device session before the RGB/depth-metadata fixes produced `2500` head-pose rows, `5007` controller-pose rows, `44` hand rows, and `266` per-eye depth records. It established that OpenXR pose and depth access work, but is not packable because it has no RGB frames and contained non-finite depth matrices.
- Device logging established that environment depth only started after the Meta passthrough feature was enabled. Static comparison with `QuestRealityCapture` identified the RGB blocker: Meta Camera2 vendor-tag values must be read as `byte[]`.
- A post-fix Quest 3 session `20260526_130042` successfully called `QuestCapturePlugin.startCameras()`, found passthrough Camera2 IDs `50` and `51`, opened both at `1280x1280`, and wrote `2579` left YUV frames, `2600` right YUV frames, `94` depth JSONL records, `96` depth payloads, `912` head-pose rows, and `1810` controller-pose rows.
- The sampled real-device package `device_captures/20260526_130042_sample` contains `12` paired RGB frames, `12` valid left-depth frames, `400` head-pose rows, and `800` controller-pose rows. It packs and muxes with FFmpeg `8.1.1` into `/tmp/quest_capture_20260526_130042_sample_v2.mp4`; `spatialmp4_reader_smoke` loads RGB `1280x1280` per eye, depth `320x320`, and pose samples from the output. The MP4 contains `12` HEVC RGB frames, `12` `raw1` depth frames, and `400` `mett` pose packets.
- The packer now tolerates a single trailing malformed JSONL record per file. This is needed because force-stopping the Android activity can leave the last RGB/depth/pose line partially written; malformed records in the middle of a file still fail validation.
- The full integration check is run with:

```bash
SPATIALMP4_PACKET_MUXER=<patched-build>/spatialmp4_packet_muxer \
SPATIALMP4_FFPROBE=scripts/build_ffmpeg/ffmpeg_install/bin/ffprobe \
SPATIALMP4_READER_SMOKE=<patched-build>/spatialmp4_reader_smoke \
python3 -m pytest python/tests/test_godot_spool_pack.py -q
```

## MVP Milestones

M1: Godot capture spool

- Godot project starts on Quest 3 with OpenXR.
- Android plugin writes left/right YUV frames and Camera2 metadata.
- GDScript writes head/controller pose JSONL.
- Depth sampler writes async raw normalized-depth images, projection matrices, and patched callback calibration fields at a controlled rate.

M2: Offline packer

- Convert one Godot spool session with normalized OpenXR depth into SpatialMP4.
- Output passes `Reader` smoke test:
  - has RGB,
  - has depth,
  - has pose,
  - RGB frames split into left/right,
  - depth is non-empty and meters after reader conversion,
  - pose timestamps match within 1 ms.

M3: Device-ready packaging

- Use Android MediaCodec for HEVC.
- Use patched FFmpeg muxer or native MP4 writer on Android.
- Add drop-frame accounting and thermal/performance telemetry.

M4: Multi-pose/hand extension

- Add reader support for named pose tracks.
- Add hand-joint packet parser.
- Preserve v1 reader behavior for primary head pose.

## Risks

- RGB camera availability depends on Horizon OS, Quest model, user permission, and passthrough feature state.
- Depth CPU readback is asynchronous and may be stale. Native OpenXR timestamping is required for high-accuracy RGBD.
- Depth readbacks are timestamped at asynchronous callback delivery in the current Godot path; a native runtime timestamp is still required for high-accuracy RGBD synchronization.
- The FOV-derived depth intrinsics are supported; depth-to-head extrinsics remain identity-marked until device calibration is captured and must be replaced for accurate RGBD reprojection.
- Camera2 vendor-tag/extrinsic conventions must be empirically validated against reprojection.
- `scripts/godot_depth_rgb_align.py` performs diagnostic direct reprojection from Environment Depth into a selected Camera2 RGB view. It pairs `runtime_display_time_ns` with Camera2 `camera_sensor_timestamp_ns` when present, uses the captured inverse projection-view matrix, composes RGB lens pose with interpolated head pose, rejects unmodelled nonzero RGB distortion by default, and emits an aligned-depth array plus visual overlay/report.
- The `rgb_extrinsics` field written into the mp4 by this chain is the
  **raw Camera2 `lens_pose_translation/rotation` packed into a 4×4
  matrix via the naïve quaternion → matrix conversion** — it is *not*
  a ready-to-use RGB-camera-to-IMU SE(3). Consumers must run the Quest
  3 Camera2-to-Unity conversion (see
  [`docs/rgb_extrinsics_conventions.md`](rgb_extrinsics_conventions.md)
  and `_openquest_head_from_camera` in `scripts/godot_depth_rgb_align.py`)
  before composing it with the head pose. The field name and layout are
  preserved for cross-device compatibility (Pico writes a different
  semantic in the same slot); device-specific dispatch is the consumer's
  responsibility.
- On-device HEVC plus depth readback plus MP4 muxing may hit thermal and IO limits.
- Current SpatialMP4 writer is a demo; production muxing needs the encoder patch path or a new writer.

## Acceptance Criteria

- A Quest 3 recording can be captured without Unity.
- The recording includes RGB, depth, head pose, and at least one controller or hand pose source.
- The generated MP4 opens with `SpatialML::Reader`.
- Reader can load at least one synchronized RGBD frame with valid head pose.
- Sidecar or v2 MP4 tracks preserve controller/hand data with timestamps tied to the same session base.
- A validation command reports frame counts, timestamp ranges, missing samples, and max RGB/depth/pose delta.
