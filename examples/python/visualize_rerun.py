import typer
import os
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import spatialmp4 as sm
import rerun as rr
import rerun.blueprint as rrb


def pico_pose_to_open3d(extrinsic):
    convert = np.array([
        [0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    output = convert @ extrinsic
    return output


def main(
    video_file: str,
    depth_only: bool = False,
    rgb_only: bool = False,
):
    """Visualize spatialmp4 using rerun."""
    reader = sm.Reader(video_file)

    if depth_only:
        reader.set_read_mode(sm.ReadMode.DEPTH_ONLY)
    elif rgb_only:
        reader.set_read_mode(sm.ReadMode.RGB_ONLY)
    else:
        reader.set_read_mode(sm.ReadMode.DEPTH_FIRST)

    if not reader.has_depth():
        typer.echo(typer.style(f"No depth found in input file", fg=typer.colors.RED))
        return
    if not reader.has_pose():
        typer.echo(typer.style(f"No depth found in input file", fg=typer.colors.RED))
        return

    blueprint=rrb.Horizontal(
        rrb.Vertical(
            rrb.Spatial3DView(name="3D", origin="world"),
            rrb.TextDocumentView(name="Description", origin="/description"),
            row_shares=[7, 3],
        ),
        rrb.Vertical(
            rrb.Spatial2DView(
                name="RGB & Depth",
                origin="world/camera/image",
                overrides={"world/camera/image/rgb": rr.Image.from_fields(opacity=0.5)},
            ),
            rrb.Tabs(
                rrb.Spatial2DView(name="RGB", origin="world/camera/image", contents="world/camera/image/rgb"),
                rrb.Spatial2DView(name="Depth", origin="world/camera/image", contents="world/camera/image/depth"),
            ),
            name="2D",
            row_shares=[3, 3, 2],
        ),
        column_shares=[2, 1],
    )
    rr.init(f"spatialmp4_{os.path.basename(video_file)}", spawn=True)
    rr.send_blueprint(blueprint)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)    # same as open3d
    view_coord = rr.ViewCoordinates.UBR


    if depth_only:
        width = reader.get_depth_width()
        height = reader.get_depth_height()
        fx = float(reader.get_depth_intrinsics().fx)
        fy = float(reader.get_depth_intrinsics().fy)
        cx = float(reader.get_depth_intrinsics().cx)
        cy = float(reader.get_depth_intrinsics().cy)
    else:
        width = reader.get_rgb_width()
        height = reader.get_rgb_height()
        fx = float(reader.get_rgb_intrinsics_left().fx)
        fy = float(reader.get_rgb_intrinsics_left().fy)
        cx = float(reader.get_rgb_intrinsics_left().cx)
        cy = float(reader.get_rgb_intrinsics_left().cy)

    while reader.has_next():
        if depth_only:
            depth_frame = reader.load_depth()
            timestamp = depth_frame.timestamp
            depth_np = depth_frame.depth
            TWH = depth_frame.pose    # T_W_H
            if TWH.timestamp == 0:
                continue    # invalid pose data
            extrinsic = np.eye(4)
            extrinsic[:3, 3] = [TWH.x, TWH.y, TWH.z]
            extrinsic[:3, :3] = Rotation.from_quat((TWH.qx, TWH.qy, TWH.qz, TWH.qw)).as_matrix()
        elif rgb_only:
            frame_rgb = reader.load_rgb()
            timestamp = frame_rgb.timestamp

            __import__('ipdb').set_trace()
            T_W_Hrgb = frame_rgb.pose.as_se3()
            T_W_Irgb = sm.head_to_imu(T_W_Hrgb, sm.HEAD_MODEL_OFFSET)
            T_W_Srgb = T_W_Irgb * T_I_Srgb
            pass

        else:
            rgbd = reader.load_rgbd(True)
            timestamp = rgbd.timestamp
            depth_np = rgbd.depth
            extrinsic = rgbd.T_W_S
        print(f"Loading frame {reader.get_index() + 1} / {reader.get_frame_count()}, timestamp: {timestamp}")

        # preprocess on depthmap
        depth_uint16 = (depth_np * 1000).astype(np.uint16)
        sobelx = cv2.Sobel(depth_uint16, cv2.CV_32F, 1, 0, ksize=3)
        sobely = cv2.Sobel(depth_uint16, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(sobelx**2 + sobely**2)
        depth_np[grad_mag > 500] = 0
        depth_np[(depth_np < 0.2) | (depth_np > 5)] = 0

        extrinsic = pico_pose_to_open3d(extrinsic)

        rr.set_time_seconds("time", timestamp)
        rr.log("world/xyz", rr.Arrows3D(vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]))
        rr.log("world/camera/image", rr.Pinhole(
            resolution=[width, height],
            focal_length=[fx, fy],
            principal_point=[cx, cy],
            camera_xyz=view_coord,
        ))
        position = extrinsic[:3, 3]
        rotation = Rotation.from_matrix(extrinsic[:3, :3]).as_quat()
        rr.log("world/camera", rr.Transform3D(translation=position, rotation=rr.Quaternion(xyzw=rotation)))
        rr.log("world/camera/image/depth", rr.DepthImage(depth_np, meter=1.0))
        if not depth_only:
            rr.log("world/camera/image/rgb", rr.Image(rgbd.rgb, color_model="BGR").compress(jpeg_quality=95))


if __name__ == "__main__":#
    typer.run(main)
