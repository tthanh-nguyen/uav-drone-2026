import math
import numpy as np
def rotation_to_euler(rmat):
    sy = math.sqrt(rmat[0, 0] ** 2 + rmat[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(rmat[2, 1], rmat[2, 2])
        pitch = math.atan2(-rmat[2, 0], sy)
        yaw = math.atan2(rmat[1, 0], rmat[0, 0])
    else:
        roll = math.atan2(-rmat[1, 2], rmat[1, 1])
        pitch = math.atan2(-rmat[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def rotmat_to_quat(R):

    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
    r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
    r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]
    
    trace = r11 + r22 + r33
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (r32 - r23) * s
        qy = (r13 - r31) * s
        qz = (r21 - r12) * s
    else:
        if r11 > r22 and r11 > r33:
            s = 2.0 * np.sqrt(1.0 + r11 - r22 - r33)
            qw = (r32 - r23) / s
            qx = 0.25 * s
            qy = (r12 + r21) / s
            qz = (r13 + r31) / s
        elif r22 > r33:
            s = 2.0 * np.sqrt(1.0 + r22 - r11 - r33)
            qw = (r13 - r31) / s
            qx = (r12 + r21) / s
            qy = 0.25 * s
            qz = (r23 + r32) / s
        else:
            s = 2.0 * np.sqrt(1.0 + r33 - r11 - r22)
            qw = (r21 - r12) / s
            qx = (r13 + r31) / s
            qy = (r23 + r32) / s
            qz = 0.25 * s

    return qx,qy,qz,qw




def euler_to_rotmat(roll, pitch, yaw):
    """Z-Y-X noi tai, don vi DO."""
    r, p, y_ = math.radians(roll), math.radians(pitch), math.radians(yaw)
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y_), math.sin(y_))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])

def quat_to_rotmat(x, y, z, w):
    """(x, y, z, w) -> 3x3. Thu tu geometry_msgs, KHONG phai px4_msgs."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def camera_in_marker(rmat,tvec):
    return (-rmat.T @ np.asarray(tvec).reshape(3,1)).ravel()