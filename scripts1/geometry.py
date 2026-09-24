import numpy as np
from transform import euler_to_rotmat, quat_to_rotmat
import math
R_NED_FROM_CAM_DOWN = np.array([[0.0, 1.0,  0.0],
                                [-1.0, 0.0,  0.0],
                                [0.0,  0.0, 1.0]])
class CameraExtrinsic:
    def __init__(self, mount_roll = 0.0, mount_pitch = 0.0, mount_yaw = 0.0, offset=(0.0,0.0,0.0)):
        self.R_ned_from_cam = (euler_to_rotmat(mount_roll,mount_pitch,mount_yaw)@R_NED_FROM_CAM_DOWN)
        self.offset = np.asarray(offset, dtype=float).reshape(3)

def marker_yaw_enu(R_cam_from_marker, q_xyzw, extrinsic):
    R = quat_to_rotmat(*q_xyzw) @ extrinsic.R_ned_from_cam @ R_cam_from_marker
    return math.atan2(R[1, 0], R[0, 0])