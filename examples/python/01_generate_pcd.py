from ast import dump
from re import I
from socket import IP_DEFAULT_MULTICAST_LOOP, IP_DROP_MEMBERSHIP
from typing import Optional
import typer
import spatialmp4 as sm
import open3d as o3d
import numpy as np
import cv2
import rerun as rr
import rerun.blueprint as rrb
from scipy.spatial.transform import Rotation
import os
import pickle

'''
Rerun for different data source:

    colmap: 
        rr.ViewCoordinates.RIGHT_HAND_Y_DOWN
        rr.log("camera", rr.ViewCoordinates.RDF, static=True)

'''

def pico_pose_to_open3d(extrinsic):
    convert = np.eye(4)
    return convert @ extrinsic

def generate_pcd(
    video_file: str,
    skip_last_frame: bool = typer.Option(True, help="skip last frame"),
    output: Optional[str] = typer.Option(None, help="output name"),
):
    """Generate pcd file from spatialmp4, using TSDF in open3d.
    """
    if not output:
        output = video_file.replace(".mp4", "_pcd.ply")

    reader = sm.Reader(video_file)
    reader.set_read_mode(sm.ReadMode.DEPTH_FIRST)

    if not reader.has_depth():
        typer.echo(typer.style(f"No depth found in input file", fg=typer.colors.RED))
    if not reader.has_pose():
        typer.echo(typer.style(f"No depth found in input file", fg=typer.colors.RED))

    voxel_length = 0.01
    sdf_trunc = 0.04
    down_voxel_size = 0.02
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor
    )

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
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_X_UP, static=True)    # same as open3d

    positions = []
    frames = []

    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    width = reader.get_rgb_width()
    height = reader.get_rgb_height()
    intrinsic.set_intrinsics(
        width=width,
        height=height,
        fx=float(reader.get_rgb_intrinsics_left().fx),
        fy=float(reader.get_rgb_intrinsics_left().fy),
        cx=float(reader.get_rgb_intrinsics_left().cx),
        cy=float(reader.get_rgb_intrinsics_left().cy)
    )
    intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
        width=height,
        height=width,
        fx=float(reader.get_rgb_intrinsics_left().fy),
        fy=float(reader.get_rgb_intrinsics_left().fx),
        cx=float(reader.get_rgb_intrinsics_left().cy),
        cy=width - 1 - float(reader.get_rgb_intrinsics_left().cx)
    )

    while reader.has_next():
        rgbd = reader.load_rgbd(True)
        if skip_last_frame and reader.get_index() == reader.get_frame_count() - 1:
            continue
        print(f"Loading frame {reader.get_index() + 1} / {reader.get_frame_count()}, timestamp: {rgbd.timestamp}")

        if reader.get_index() > 20:
            break

        # preprocess on depthmap
        depth_np = rgbd.depth
        depth_uint16 = (depth_np * 1000).astype(np.uint16)
        sobelx = cv2.Sobel(depth_uint16, cv2.CV_32F, 1, 0, ksize=3)
        sobely = cv2.Sobel(depth_uint16, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(sobelx**2 + sobely**2)
        depth_np[grad_mag > 500] = 0
        depth_np[(depth_np < 0.2) | (depth_np > 5)] = 0

        extrinsic = rgbd.T_W_S
        extrinsic_o3d = pico_pose_to_open3d(extrinsic)

        # rorate 
        rgb_rotated = cv2.rotate(rgbd.rgb, cv2.ROTATE_90_CLOCKWISE)
        depth_rotated = cv2.rotate(depth_np, cv2.ROTATE_90_CLOCKWISE)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rgb_rotated),
            o3d.geometry.Image(depth_rotated),
            depth_scale=1.0,
            depth_trunc=5.0,
            convert_rgb_to_intensity=True
        )
        volume.integrate(rgbd_image, intrinsic_o3d, extrinsic_o3d)

        pcd_i = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic_o3d)
        pcd_i.transform(extrinsic_o3d)
        pcd_i.paint_uniform_color([0, 0, 1])
        frames.append(pcd_i)

        rr.set_time_seconds("time", rgbd.timestamp)
        rr.log("world/xyz", rr.Arrows3D(vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]))
        rr.log("world/camera/image", rr.Pinhole(
            resolution=[width, height],
            focal_length=[intrinsic.intrinsic_matrix[0, 0], intrinsic.intrinsic_matrix[1, 1]],
            principal_point=[intrinsic.intrinsic_matrix[0, 2], intrinsic.intrinsic_matrix[1, 2]],
            camera_xyz=rr.ViewCoordinates.URF,
        ))
        position = extrinsic[:3, 3]
        rotation = Rotation.from_matrix(extrinsic[:3, :3]).as_quat()
        rr.log("world/camera", rr.Transform3D(translation=position, rotation=rr.Quaternion(xyzw=rotation)))
        rr.log("world/camera/image/rgb", rr.Image(rgbd.rgb, color_model="BGR").compress(jpeg_quality=95))
        rr.log("world/camera/image/depth", rr.DepthImage(depth_np, meter=1.0))

    pcd = volume.extract_point_cloud()
    pcd.paint_uniform_color([1, 1, 0])

    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

    vis=o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    idx=0
    vis.add_geometry(origin)
    vis.add_geometry(frames[idx])
    #vis.update_renderer()

    def save_view_control(vis):
        vc = vis.get_view_control()
        params = vc.convert_to_pinhole_camera_parameters()
        data = {
            "extrinsic": params.extrinsic,
            "intrinsic_matrix": params.intrinsic.intrinsic_matrix,
            "width": params.intrinsic.width,
            "height": params.intrinsic.height,
        }
        with open("view_control.pkl", "wb") as f:
            pickle.dump(data, f)
        print("Saved view control")

    def click_next(vis):
        nonlocal idx
        print('a_click')
        vis.clear_geometries()
        vis.add_geometry(origin)
        vis.add_geometry(pcd)
        vis.add_geometry(frames[idx])
        idx = (idx+1) % len(frames)

        if os.path.exists("view_control.pkl"):
            vc = vis.get_view_control()
            with open("view_control.pkl", "rb") as f:
                data = pickle.load(f)
            params = o3d.camera.PinholeCameraParameters()
            params.extrinsic = data["extrinsic"]
            params.intrinsic = o3d.camera.PinholeCameraIntrinsic(
                data["width"], data["height"], data["intrinsic_matrix"]
            )
            vc.convert_from_pinhole_camera_parameters(params)


    def click_prev(vis):
        nonlocal idx
        print('b_click')
        vis.clear_geometries()
        vis.add_geometry(origin)
        vis.add_geometry(pcd)
        vis.add_geometry(frames[idx])
        idx = (idx-1) % len(frames)
        idx = max(idx, 0)
        if os.path.exists("view_control.pkl"):
            vc = vis.get_view_control()
            with open("view_control.pkl", "rb") as f:
                data = pickle.load(f)
            params = o3d.camera.PinholeCameraParameters()
            params.extrinsic = data["extrinsic"]
            params.intrinsic = o3d.camera.PinholeCameraIntrinsic(
                data["width"], data["height"], data["intrinsic_matrix"]
            )
            vc.convert_from_pinhole_camera_parameters(params)

    def exit_key(vis):
        vis.destroy_window()
    import glfw
    vis.register_key_callback(glfw.KEY_A,click_next)
    vis.register_key_callback(glfw.KEY_B,click_prev)
    vis.register_key_callback(glfw.KEY_S,save_view_control)
    vis.register_key_callback(glfw.KEY_ESCAPE,exit_key)
    vis.poll_events()
    vis.run()
    #vis.destroy_window() 


    # print("[Info] Removing non-finite points...")
    # pcd = pcd.remove_non_finite_points()
    # print("[Info] Downsampling point cloud using voxel grid...")
    # pcd = pcd.voxel_down_sample(voxel_size=args.down_voxel_size)
    # print("[Info] Removing statistical outliers...")
    # _, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    # print(f"[Info] Valid points: {len(ind)} / {len(pcd.points)} ({len(ind) / len(pcd.points) * 100:.2f} %)")
    # pcd = pcd.select_by_index(ind)

    o3d.io.write_point_cloud(output, pcd)
    print(f"[Info] Saved {len(pcd.points)} total points to {output}")


if __name__ == "__main__":#
    typer.run(generate_pcd)