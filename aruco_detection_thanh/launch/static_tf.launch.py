from launch import LaunchDescription
from launch_ros.actions import Node

# ĐIỀN ĐÚNG GIÁ TRỊ TỪ self.extrinsic TRONG CODE CỦA BẠN
R_NED_FROM_CAM = [ 0.0, 1.0, 0.0,
                  -1.0, 0.0, 0.0,
                   0.0, 0.0, 1.0]

OFFSET = [0.07, 0.0, 0.07]


def generate_launch_description():
    # base_link_frd -> camera_optical_frame (chỉ để hiển thị, trùng với R và offset)
    from scipy.spatial.transform import Rotation
    import numpy as np
    qx, qy, qz, qw = Rotation.from_matrix(
        np.array(R_NED_FROM_CAM).reshape(3, 3)).as_quat()

    frd_to_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='frd_to_camera_optical',
        arguments=[
            '--x', str(OFFSET[0]), '--y', str(OFFSET[1]), '--z', str(OFFSET[2]),
            '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
            '--frame-id', 'base_link_frd', '--child-frame-id', 'camera_optical_frame'],
    )

    # base_link_frd -> base_link (FLU), quay 180 độ quanh x
    frd_to_flu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='frd_to_flu',
        arguments=['--roll', '3.1416',
                   '--frame-id', 'base_link_frd', '--child-frame-id', 'base_link'],
    )

    # aruco_marker -> base_link_frd (động)
    drone_tf = Node(
        package='aruco_detection_thanh', executable='marker_tf.py',
        name='drone_from_marker_tf', output='screen',
        parameters=[{
            'topic': '/posestamp_camOfThanh',
            'R': R_NED_FROM_CAM,
            'offset': OFFSET,
        }],
    )

    return LaunchDescription([frd_to_camera, frd_to_flu, drone_tf])
