import os
import subprocess
from typing import Optional

import numpy as np
import typer
import shutil
from tqdm import tqdm

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import spatialmp4 as sm


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = qx * qx + qy * qy + qz * qz + qw * qw
    if norm < 1e-12:
        return np.eye(3)
    s = 2.0 / norm
    x = qx * s
    y = qy * s
    z = qz * s
    wx = qw * x
    wy = qw * y
    wz = qw * z
    xx = qx * x
    xy = qx * y
    xz = qx * z
    yy = qy * y
    yz = qy * z
    zz = qz * z
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def pose_to_matrix(pose: sm.PoseFrame) -> np.ndarray:
    rot = quat_to_rot(pose.qx, pose.qy, pose.qz, pose.qw)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[:3, 3] = np.array([pose.x, pose.y, pose.z], dtype=np.float64)
    return mat


def make_cube(size: float) -> tuple[np.ndarray, list[tuple[int, int]]]:
    half = size * 0.5
    vertices = np.array(
        [
            [-half, -half, -half],
            [half, -half, -half],
            [half, half, -half],
            [-half, half, -half],
            [-half, -half, half],
            [half, -half, half],
            [half, half, half],
            [-half, half, half],
        ],
        dtype=np.float64,
    )
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]
    return vertices, edges


def update_cube_lines(lines, vertices, edges, camera_origin, min_lw=0.8, max_lw=3.0):
    midpoints = []
    for i, j in edges:
        midpoints.append((vertices[i] + vertices[j]) * 0.5)
    midpoints = np.array(midpoints, dtype=np.float64)
    distances = np.linalg.norm(midpoints - camera_origin, axis=1)
    d_min = distances.min()
    d_max = distances.max()
    if d_max <= d_min + 1e-9:
        widths = np.full_like(distances, max_lw)
    else:
        t = (distances - d_min) / (d_max - d_min)
        widths = max_lw - t * (max_lw - min_lw)
    for line, (i, j), lw in zip(lines, edges, widths):
        xs = [vertices[i, 0], vertices[j, 0]]
        ys = [vertices[i, 1], vertices[j, 1]]
        zs = [vertices[i, 2], vertices[j, 2]]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)
        line.set_linewidth(float(lw))


def apply_world_axes_transform(T_W_H: np.ndarray) -> np.ndarray:
    transform = np.diag([-1.0, 1.0, -1.0, 1.0])
    return transform @ T_W_H


def compute_bounds(
    reader: sm.Reader,
    topk: Optional[int],
    sample: int,
    total_samples: int,
) -> tuple[np.ndarray, np.ndarray]:
    min_xyz = None
    max_xyz = None
    valid_count = 0
    total_count = 0

    with tqdm(total=total_samples, desc="Bounds", unit="frame") as pbar:
        while reader.has_next():
            if topk is not None and reader.get_index() >= topk:
                break
            if sample > 1 and reader.get_index() % sample != 0:
                reader.load_rgb()
                continue
            rgb_frame = reader.load_rgb()
            pose = rgb_frame.pose
            total_count += 1
            if pose.timestamp == 0:
                pbar.update(1)
                continue
            T_W_H = apply_world_axes_transform(pose_to_matrix(pose))
            t = T_W_H[:3, 3]
            if min_xyz is None:
                min_xyz = t.copy()
                max_xyz = t.copy()
            else:
                min_xyz = np.minimum(min_xyz, t)
                max_xyz = np.maximum(max_xyz, t)
            valid_count += 1
            pbar.update(1)

    if min_xyz is None:
        raise ValueError("No valid pose frames found")

    typer.echo(f"Bounds pass complete: {valid_count} valid pose frames, {total_count} total frames")
    return min_xyz, max_xyz


def main(
    video_file: str,
    topk: Optional[int] = typer.Option(None, help="Limit to the first N frames"),
    cube_size: float = typer.Option(0.2, help="Cube size in meters"),
    axis_len: float = typer.Option(0.4, help="World axis length in meters"),
    output_dir: Optional[str] = typer.Option(
        None,
        help="Output directory (default: <video>_pose_frames)",
    ),
    output_video: Optional[str] = typer.Option(
        None,
        help="Output video path (default: <output_dir>.mp4 when set)",
    ),
    sample: int = typer.Option(5, help="Sample every N frames"),
):
    if sample < 1:
        typer.echo(typer.style("sample must be >= 1", fg=typer.colors.RED))
        raise typer.Exit(code=1)

    typer.echo(f"Opening video: {video_file}")
    reader = sm.Reader(video_file)
    reader.set_read_mode(sm.ReadMode.RGB_ONLY)

    if not reader.has_rgb():
        typer.echo(typer.style("No RGB stream found in input file", fg=typer.colors.RED))
        raise typer.Exit(code=1)
    if not reader.has_pose():
        typer.echo(typer.style("No pose stream found in input file", fg=typer.colors.RED))
        raise typer.Exit(code=1)

    typer.echo("First pass: computing pose bounds...")
    total_frames = reader.get_frame_count()
    if topk is not None:
        total_frames = min(total_frames, topk)
    total_samples = (total_frames + sample - 1) // sample
    min_xyz, max_xyz = compute_bounds(reader, topk, sample, total_samples)
    reader.reset()

    if output_dir is None:
        base_name = os.path.splitext(os.path.basename(video_file))[0]
        output_dir = f"tmp_vis_{base_name}"
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    typer.echo(f"Rendering frames to {output_dir}")

    fig = plt.figure(figsize=(12, 5))
    ax_img = fig.add_subplot(1, 2, 1)
    ax_pose = fig.add_subplot(1, 2, 2, projection="3d")
    ax_img.set_axis_off()
    ax_pose.set_xlabel("X")
    ax_pose.set_ylabel("Y")
    ax_pose.set_zlabel("Z")
    ax_pose.set_box_aspect((1, 1, 1))

    ax_pose.plot([0, -axis_len], [0, 0], [0, 0], color="r", linewidth=2)
    ax_pose.plot([0, 0], [0, axis_len], [0, 0], color="g", linewidth=2)
    ax_pose.plot([0, 0], [0, 0], [0, -axis_len], color="b", linewidth=2)
    ax_pose.text(-axis_len, 0, 0, "X", color="r")
    ax_pose.text(0, axis_len, 0, "Y", color="g")
    ax_pose.text(0, 0, -axis_len, "Z", color="b")

    margin = max(cube_size * 2.0, 0.5)
    min_xyz = np.minimum(min_xyz, np.zeros(3))
    max_xyz = np.maximum(max_xyz, np.zeros(3))
    ax_pose.set_xlim(min_xyz[0] - margin, max_xyz[0] + margin)
    ax_pose.set_ylim(min_xyz[1] - margin, max_xyz[1] + margin)
    ax_pose.set_zlim(min_xyz[2] - margin, max_xyz[2] + margin)
    ax_pose.dist = 2
    ax_pose.view_init(elev=30, azim=-60, roll=0)

    cube_vertices, cube_edges = make_cube(cube_size * 2.0)
    cube_lines = [
        ax_pose.plot([], [], [], color="black", linewidth=1)[0] for _ in cube_edges
    ]
    camera_origin = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    img_artist = None
    rendered = 0

    typer.echo("Second pass: rendering frames...")
    with tqdm(total=total_samples, desc="Rendering", unit="frame") as pbar:
        while reader.has_next():
            if topk is not None and reader.get_index() >= topk:
                break
            if sample > 1 and reader.get_index() % sample != 0:
                reader.load_rgb()
                continue
            rgb_frame = reader.load_rgb()
            pose = rgb_frame.pose
            if pose.timestamp == 0:
                pbar.update(1)
                continue

            left_rgb = rgb_frame.left_rgb
            if left_rgb is not None and left_rgb.ndim == 3 and left_rgb.shape[2] == 3:
                left_rgb = left_rgb[:, :, ::-1]

            if img_artist is None:
                img_artist = ax_img.imshow(left_rgb)
            else:
                img_artist.set_data(left_rgb)

            T_W_H = apply_world_axes_transform(pose_to_matrix(pose))
            transformed = (T_W_H[:3, :3] @ cube_vertices.T).T + T_W_H[:3, 3]
            update_cube_lines(cube_lines, transformed, cube_edges, camera_origin)
            ax_pose.set_title(
                f"Pose frame {reader.get_index() + 1}/{reader.get_frame_count()} "
                f"t={pose.timestamp:.3f}s"
            )

            frame_path = os.path.join(output_dir, f"frame_{rendered:06d}.png")
            fig.savefig(frame_path, dpi=100, bbox_inches="tight")
            rendered += 1
            pbar.update(1)

    plt.close(fig)
    typer.echo(f"Done. Rendered {rendered} frames to {output_dir}")

    if output_video is None:
        output_video = f"{output_dir}.mp4"
    typer.echo(f"Encoding video with ffmpeg to {output_video}")
    ffmpeg_fps = float(reader.get_rgb_fps()) / float(sample)
    ffmpeg_cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        f"{ffmpeg_fps:.6f}",
        "-i",
        os.path.join(output_dir, "frame_%06d.png"),
        "-vf",
        "scale=trunc(iw/2)*2:trunc(ih/2)*2",
        "-pix_fmt",
        "yuv420p",
        output_video,
    ]
    try:
        subprocess.run(ffmpeg_cmd, check=True)
        typer.echo("Video encoding completed.")
    except FileNotFoundError:
        typer.echo(typer.style("ffmpeg not found in PATH.", fg=typer.colors.RED))
    except subprocess.CalledProcessError as exc:
        typer.echo(typer.style(f"ffmpeg failed with exit code {exc.returncode}", fg=typer.colors.RED))


if __name__ == "__main__":
    typer.run(main)
