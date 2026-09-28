#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

class DroneFromMarkerTF(Node):
    def __init__(self):
        super().__init__('drone_from_marker_tf')
        self.declare_parameter('topic', '/posestamp_camOfThanh')
        # R_ned_from_cam (9 số, theo hàng) và offset (3 số): lấy đúng từ code của bạn
        self.declare_parameter('R', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('offset', [0.0, 0.0, 0.0])

        self.R_bc = np.array(self.get_parameter('R').value).reshape(3, 3)
        self.t_bc = np.array(self.get_parameter('offset').value)

        self.br = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, self.get_parameter('topic').value,
        self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        p = msg.pose.position
        t_cm = np.array([p.x, p.y, p.z])

        # vị trí marker so với drone (giống p_body_ned trong code của bạn)
        t_bm = self.R_bc @ t_cm + self.t_bc

        # hướng marker so với drone: cố định theo thực tế
        R_bm = np.array([[0.0, 1.0,  0.0],
                                [1.0, 0.0,  0.0],
                                [0.0, 0.0, -1.0]])

        # nghịch đảo: drone so với marker
        #R_mb = R_bm.T
        t_mb = -R_mb @ t_bm
        qx, qy, qz, qw = Rotation.from_matrix(R_mb).as_quat()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'aruco_marker'
        t.child_frame_id = 'base_link_frd'
        t.transform.translation.x = float(t_mb[0])
        t.transform.translation.y = float(t_mb[1])
        t.transform.translation.z = float(t_mb[2])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = DroneFromMarkerTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()