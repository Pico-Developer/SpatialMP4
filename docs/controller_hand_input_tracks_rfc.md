# RFC: Controller, Hand, and Controller Input Timed Tracks

Status: Draft implemented in GodotQuestCapture live writer.

## Context

Quest capture currently writes RGB, depth, and head pose into the SpatialMP4 file,
while controller pose and hand joints remain as JSONL sidecars. That split makes
timestamp alignment and file lifecycle management harder, and sidecars can be
lost or mismatched with the MP4. Controller input state is also not recorded.

## Goals

- Store controller pose, hand joints, and controller input in MP4 timed tracks.
- Keep RGB/depth pose association based on head pose only.
- Avoid a single unified packet format; readers should align streams by PTS.
- Keep hand joints compact with float32 precision.
- Store controller input as event-driven data because most frames have no input
  activity.

## Non-goals

- Pose-only MP4 capture without RGB is not handled in this phase.
- Compression for hand joints or depth is not handled in this phase.
- Raw OpenXR action paths are not stored per packet. The v1 input bitmask is the
  stable semantic interface.

## Track layout

All new tracks use `codec_tag=mett` and `AVMEDIA_TYPE_DATA`. Tracks are
distinguished by stream metadata:

| Track id | Kind | Schema | Notes |
| --- | --- | --- | --- |
| `head` | `rigid_pose` | `spatialmp4.rigid_pose.v1` | Existing head pose; only source used for RGB/depth pose lookup. |
| `left_controller` | `rigid_pose` | `spatialmp4.rigid_pose.v1` | Left controller grip pose. |
| `right_controller` | `rigid_pose` | `spatialmp4.rigid_pose.v1` | Right controller grip pose. |
| `left_hand` | `hand_joints` | `spatialmp4.hand_joints.v1` | Left OpenXR hand joints. |
| `right_hand` | `hand_joints` | `spatialmp4.hand_joints.v1` | Right OpenXR hand joints. |
| `left_controller_input` | `controller_input` | `spatialmp4.controller_input.v1` | Left controller input events/snapshots. |
| `right_controller_input` | `controller_input` | `spatialmp4.controller_input.v1` | Right controller input events/snapshots. |

Track identity is carried in the stream `handler_name` (the MP4 `hdlr` box),
formatted as:

```text
handler_name = spatialmp4:<metadata_kind>:<track_id>
# e.g. spatialmp4:rigid_pose:head
#      spatialmp4:rigid_pose:left_controller
#      spatialmp4:controller_input:left_controller_input
#      spatialmp4:hand_joints:left_hand
```

This is the authoritative classification channel. It is used **instead of** plain
per-stream `av_dict` metadata because the FFmpeg MP4 muxer does not persist
arbitrary stream metadata across a write/read round-trip (verified: even with
`movflags=use_metadata_tags`, custom keys such as `track_id`/`metadata_kind` are
dropped), whereas the `hdlr` box `handler_name` reliably round-trips. The reader
parses `handler_name` first (see `ParseSpatialHandler` in `reader.cc`).

The following descriptive keys are still written for self-documentation and for
any tooling that does preserve them, but readers must not depend on them:

```text
mime_type=application/x-spatialmp4-rigid-pose
metadata_kind=rigid_pose
payload_schema=spatialmp4.rigid_pose.v1
track_id=head
pose_position=head
```

## Rigid pose payload

`spatialmp4.rigid_pose.v1` is unchanged from current head pose:

```c
double px;
double py;
double pz;
double qx;
double qy;
double qz;
double qw;
```

## Hand joints payload

`spatialmp4.hand_joints.v1` uses little-endian float32 joint data:

```c
uint32 magic;      // 'HJNT'
uint16 version;    // 1
uint16 joint_count;

repeated joint_count:
  uint16 joint_id;
  uint16 flags;
  float  radius_m;
  float  px, py, pz;
  float  qx, qy, qz, qw;
```

For 26 joints, this is about 944 bytes per hand frame. At 30 Hz for two hands,
the added data rate is about 0.45 Mbps, or about 0.2 GB/hour.

## Controller input payload

`spatialmp4.controller_input.v1` is event-stream-first. A full state snapshot is
written at recording start, and whenever an input event occurs the writer emits
an `EVENT` packet followed by a `SNAPSHOT` packet at the same timestamp.

```c
uint32 magic;          // 'CINP'
uint16 version;        // 1
uint16 packet_type;    // 1=SNAPSHOT, 2=EVENT

uint16 controller;     // 0=left, 1=right
uint16 reserved;

uint64 available_mask;
uint64 pressed_mask;
uint64 touched_mask;
uint64 changed_mask;   // button/touch bits changed since previous state

float trigger_value;   // 0..1
float grip_value;      // 0..1
float thumbstick_x;    // -1..1
float thumbstick_y;    // -1..1
float trackpad_x;      // -1..1, 0 if unavailable
float trackpad_y;      // -1..1, 0 if unavailable
```

Input bit assignment:

```c
TRIGGER_CLICK       = 0
TRIGGER_TOUCH       = 1
GRIP_CLICK          = 2
THUMBSTICK_CLICK    = 3
THUMBSTICK_TOUCH    = 4
A_CLICK             = 5
A_TOUCH             = 6
B_CLICK             = 7
B_TOUCH             = 8
X_CLICK             = 9
X_TOUCH             = 10
Y_CLICK             = 11
Y_TOUCH             = 12
MENU_CLICK          = 13
SYSTEM_CLICK        = 14
THUMBREST_TOUCH     = 15
TRACKPAD_CLICK      = 16
TRACKPAD_TOUCH      = 17
```

The fixed bitmask maps to OpenXR action semantics. On Quest Touch controllers,
`ax_button/ax_touch` map to X on the left hand and A on the right hand;
`by_button/by_touch` map to Y on the left hand and B on the right hand.
`available_mask` distinguishes unsupported inputs from inputs that are supported
but currently false.

## Reader behavior

Existing reader APIs remain head-pose compatible:

- `HasPose()` means a head pose track exists.
- `GetPoseFrames()` returns only head pose frames.
- RGB/depth pose lookup uses only head pose frames.

New APIs should expose independent timed tracks:

- `ListTimedMetadataTracks()`
- `GetRigidPoseFrames(track_id)`
- `GetHandJointFrames(track_id)`
- `GetControllerInputFrames(track_id)`

For controller input state at an arbitrary media timestamp, readers should use
`last sample <= timestamp`, not nearest-neighbor matching. Button/input states
are stateful and should not be pulled backward from the future.

## Compatibility

Old files with a single head `mett` track remain readable. New files contain
multiple `mett` tracks, so readers must classify tracks by the `handler_name`
identity (see above) rather than assuming a single pose stream. When no
`spatialmp4:` handler is present and there is exactly one rigid-pose-like `mett`
stream, it is treated as legacy head pose; when several unlabeled rigid-pose
`mett` streams are present, the highest stream index is treated as head pose so
that RGB/depth alignment keeps working (the writer always creates the head track
last).
