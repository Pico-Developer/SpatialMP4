import numpy as np
import spatialmp4


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    trace = np.trace(R)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def quaternion_rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    q_vec = np.array([x, y, z])
    uv = np.cross(q_vec, v)
    uuv = np.cross(q_vec, uv)
    return v + 2.0 * (w * uv + uuv)


def compute_expected_head_to_imu(head_pose: np.ndarray, offset: np.ndarray) -> np.ndarray:
    R = head_pose[:3, :3]
    t = head_pose[:3, 3]
    slam_q = rotation_matrix_to_quaternion(R)
    slam_t = t - quaternion_rotate_vector(slam_q, offset)
    q = np.array([slam_q[0], slam_q[2], -slam_q[1], slam_q[3]])
    R_out = quaternion_to_matrix(q)
    t_out = np.array([slam_t[1], -slam_t[0], slam_t[2]])
    result = np.eye(4)
    result[:3, :3] = R_out
    result[:3, 3] = t_out
    return result


def test_head_to_imu_identity_offset():
    head_pose = np.eye(4)
    offset = np.array([0.1, -0.2, 0.3])
    imu_pose = spatialmp4.head_to_imu(head_pose, offset)
    expected = compute_expected_head_to_imu(head_pose, offset)
    assert np.allclose(imu_pose, expected, atol=1e-12)


def test_head_to_imu_general_pose():
    angle = np.deg2rad(45.0)
    Rz = np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle), np.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    head_pose = np.eye(4)
    head_pose[:3, :3] = Rz
    head_pose[:3, 3] = np.array([0.5, -1.2, 0.8])
    offset = np.array([-0.05, 0.08, -0.12])
    imu_pose = spatialmp4.head_to_imu(head_pose, offset)
    expected = compute_expected_head_to_imu(head_pose, offset)
    assert np.allclose(imu_pose, expected, atol=1e-10)


def test_head_to_imu_fortran_order_input():
    angle = np.deg2rad(30.0)
    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(angle), -np.sin(angle)],
        [0.0, np.sin(angle), np.cos(angle)],
    ])
    head_pose = np.eye(4, order="F")
    head_pose[:3, :3] = Rx
    head_pose[:3, 3] = np.array([0.3, -0.4, 0.5])
    offset = np.array([0.12, -0.08, 0.04])

    expected = compute_expected_head_to_imu(np.array(head_pose), offset)
    imu_pose = spatialmp4.head_to_imu(head_pose, offset)
    assert np.allclose(imu_pose, expected, atol=1e-12)
