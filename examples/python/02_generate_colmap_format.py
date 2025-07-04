import spatialmp4 as sm
import os
import cv2
import numpy as np

def write_cameras_txt(path, width, height, fx, fy, cx, cy):
    with open(path, 'w') as f:
        # SIMPLE_PINHOLE model: <model_id> <model_name> <width> <height> <params>
        # params: fx, cx, cy
        f.write(f"1 PINHOLE {width} {height} {fx} {cx} {cy}\n")

def write_images_txt(path, poses, img_names, camera_id=1):
    with open(path, 'w') as f:
        for i, (pose, name) in enumerate(zip(poses, img_names)):
            # <image_id> <qw> <qx> <qy> <qz> <tx> <ty> <tz> <camera_id> <image_name>
            f.write(f"{i+1} {pose['qw']} {pose['qx']} {pose['qy']} {pose['qz']} {pose['tx']} {pose['ty']} {pose['tz']} {camera_id} {name}\n\n")

def main(input_file, output_dir):
    reader = sm.Reader(input_file)
    imgdir = os.path.join(output_dir, "images")
    colmapdir = os.path.join(output_dir, "sparse/0/")
    os.makedirs(imgdir, exist_ok=True)
    os.makedirs(colmapdir, exist_ok=True)

    # 获取相机内参
    width = reader.get_rgb_width()
    height = reader.get_rgb_height()
    intr = reader.get_rgb_intrinsics_left()
    fx = intr.fx
    fy = intr.fy
    cx = intr.cx
    cy = intr.cy

    reader.set_read_mode(sm.ReadMode.RGB_ONLY)

    img_names = []
    poses = []

    while reader.has_next():
        # Load RGB frame
        rgb_frame = reader.load_rgb()
        img = rgb_frame.left_rgb
        pose = rgb_frame.pose
        i = len(img_names)
        img_name = f"{i+1:08d}.png"
        cv2.imwrite(os.path.join(imgdir, img_name), img)
        img_names.append(img_name)
        poses.append({
            "qw": pose.qw,
            "qx": pose.qx,
            "qy": pose.qy,
            "qz": pose.qz,
            "tx": pose.x,
            "ty": pose.y,
            "tz": pose.z,
        })

    # 写 cameras.txt
    write_cameras_txt(os.path.join(colmapdir, "cameras.txt"), width, height, fx, fy, cx, cy)
    # 写 images.txt
    write_images_txt(os.path.join(colmapdir, "images.txt"), poses, img_names)
    # 空的 points3D.txt
    open(os.path.join(colmapdir, "points3D.txt"), 'w').close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-file", type=str, required=True)
    parser.add_argument("--output-dir", type=str, required=True)
    args = parser.parse_args()
    main(args.input_file, args.output_dir)
