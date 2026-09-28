
import json
import numpy as np
def load_calibration(path):
    with open(path) as f:
        data = json.load(f)
    camera_matrix = np.array(data["matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["distortion"], dtype=np.float64).reshape(-1, 1)


    w = data.get("image_width")
    h = data.get("image_height")
    if not (w and h):
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        raise RuntimeError(f"Write it into the JSON or set the calib_size parameter.")
    return camera_matrix, dist_coeffs, (int(w), int(h))
    

def fit_camera_matrix(camera_matrix, calib_size, frame_size):
    """Scale intrinsics from the calibration resolution to the running one."""
    cw, ch = calib_size
    fw, fh = frame_size
    if (cw, ch) == (fw, fh):
        return camera_matrix

    sx, sy = fw / cw, fh / ch
    if abs(sx - sy) > 0.01:
        raise RuntimeError(
            f"Aspect ratio mismatch (sx={sx:.3f} sy={sy:.3f}): the camera is "
            f"cropping rather than scaling, so the intrinsics cannot be scaled "
            f"correctly. Recalibrate at {fw}x{fh}.")

    scaled = camera_matrix.copy()
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled