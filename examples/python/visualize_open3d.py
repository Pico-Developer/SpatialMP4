import typer
import os
import pickle
import cv2
import numpy as np
import open3d as o3d
import spatialmp4 as sm

'''
Rerun for different data source:

    colmap: 
        rr.ViewCoordinates.RIGHT_HAND_Y_DOWN
        rr.log("camera", rr.ViewCoordinates.RDF, static=True)

'''

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
    topk: int = None,
):
    """Generate pcd file from spatialmp4, using TSDF in open3d.
    """
    reader = sm.Reader(video_file)
    reader.set_read_mode(sm.ReadMode.DEPTH_FIRST)

    if not reader.has_depth():
        typer.echo(typer.style(f"No depth found in input file", fg=typer.colors.RED))
        return
    if not reader.has_pose():
        typer.echo(typer.style(f"No depth found in input file", fg=typer.colors.RED))
        return

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
        if topk is not None and reader.get_index() >= topk:
            break
        print(f"Loading frame {reader.get_index() + 1} / {reader.get_frame_count()}, timestamp: {rgbd.timestamp}")

        # preprocess on depthmap
        depth_np = rgbd.depth
        depth_uint16 = (depth_np * 1000).astype(np.uint16)
        sobelx = cv2.Sobel(depth_uint16, cv2.CV_32F, 1, 0, ksize=3)
        sobely = cv2.Sobel(depth_uint16, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(sobelx**2 + sobely**2)
        depth_np[grad_mag > 500] = 0
        depth_np[(depth_np < 0.2) | (depth_np > 5)] = 0

        extrinsic = rgbd.T_W_S
        extrinsic = pico_pose_to_open3d(extrinsic)

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rgbd.rgb),
            o3d.geometry.Image(depth_np),
            depth_scale=1.0,
            depth_trunc=5.0,
            convert_rgb_to_intensity=True
        )

        pcd_i = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
        pcd_i.transform(np.linalg.inv(extrinsic))
        pcd_i.paint_uniform_color([0, 0, 1])
        frames.append(pcd_i)

    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    vis=o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    idx=0
    vis.add_geometry(origin)
    vis.add_geometry(frames[idx])

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
    
    vis.register_key_callback(65, click_next)           # glfw.KEY_A
    vis.register_key_callback(66, click_prev)           # glfw.KEY_B
    vis.register_key_callback(83, save_view_control)    # glfw.KEY_S
    vis.register_key_callback(256, exit_key)            # glfw.KEY_ESCAPE
    vis.poll_events()
    vis.run()


if __name__ == "__main__":#
    typer.run(main)
