#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
import rclpy
from collections import deque

def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec*1e-9

class MarkerLocalizer(Node):
    def __init__(self):
        super().__init__('marker_localization')
        qos = QoSProfile(depth=1,reliability =(ReliabilityPolicy.BEST_EFFORT))
        # self.sub_pose_aruco = self.create_subscription(PoseStamped, "/posestamp_camOfThanh",self.chay,qos)
        self.sub_pose_local = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.on_mavros_pose,qos)
        
        self.declare_parameter("buffer-s",2.0)
        self.buffer_s = self.get_parameter("buffer-s").value
        self._attitude = deque()

    def on_mavros_pose(self,msg):
        q = msg.pose.orientation
        pose = msg.pose.position
        self._push_attitude(stamp_to_sec(msg.header.stamp),(q.x,q.y,q.z,q.w),(pose.x,pose.y,pose.z))
    def _push_attitude(self,t, q_xyzw, drone_pos):

        self._attitude.append((t, q_xyzw, drone_pos))
        # print("attitude",self._attitude)
        print(len(self._attitude))
        print("--------------------")
        cutoff = t - self.buffer_s
        while self._attitude and self._attitude[0][0]<cutoff:
            self._attitude.popleft()

    def on_imu(self,msg):
        q = msg.orientation
        self._push_attitude(stamp_to_sec(msg.header.stamp),(q.x,q.y,q.z,q.w),(0.0,0.0,0.0))

    def _lookup(self,t):
        if not self._attitude:
            return None
        best = min(self._attitude, key = lambda s:abs(s[0]-t))
        return best if abs(best[0] - t) <= self.max_attitude_age else None
    def on_marker_pose(self,msg):
        t_capture = stamp_to_sec(msg.header.stamp)
        sample = self._lookup(t_capture)

        if sample is None:
            return

        _, q_xyzw, drone_pos = sample

        t = msg.pose.position

        p_cam = np.array([t.x,t.y,t.z,t.w],dtype = float)

def main(args=None):
    rclpy.init(args=args)
    brake = MarkerLocalizer()
    rclpy.spin(brake)
    brake.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()

                