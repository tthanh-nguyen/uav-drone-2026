#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from skydroid_msgs.msg import GimbalCommand


class GimbalCenterPublisher(Node):

  def __init__(self):
    super().__init__('gimbal_center_publisher')
    self.publisher_ = self.create_publisher(GimbalCommand, '/gimbal_command', 10)
   
    self.timer = self.create_timer(0.5, self.send_center_command)

  def send_center_command(self):
    msg = GimbalCommand()
    msg.control_mode = 0  # MANUAL
   
    msg.mode            = 0

    # uint8 PTZ_STOP=0
    # uint8 PTZ_UP=1
    # uint8 PTZ_DOWN=2
    # uint8 PTZ_CENTER=3
    # uint8 PTZ_FOLLOW=4
    # uint8 PTZ_LOCK=5
    msg.ptz_cmd         = 1



    msg.enable_pitch    = False
    msg.enable_yaw      = False
    msg.pitch_deg       = 0.0
    msg.yaw_deg         = 0.0
    msg.pitch_speed_dps = 0.0
    msg.yaw_speed_dps   = 0.0

    msg.pitch_vel_dps   = 0.0
    msg.yaw_vel_dps     = 0.0

    self.publisher_.publish(msg)
    self.get_logger().info('Sent Gimbal Center Command (ptz_cmd: 3)!')

   
    self.timer.cancel()


def main(args=None):
  rclpy.init(args=args)
  node = GimbalCenterPublisher()
  rclpy.spin_once(node, timeout_sec=1.0)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()