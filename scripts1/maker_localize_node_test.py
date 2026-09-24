#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped,PointStamped

import rclpy
from collections import deque
import numpy as np
from geometry import CameraExtrinsic, marker_yaw_enu
from transform import quat_to_rotmat
import time,math
from sensor_msgs.msg import Image, Imu
from px4_msgs.msg import VehicleOdometry
from aruco_msgs.msg import MarkerLocalizerHealth

def stamp_to_sec_odom(stamp):
    # return stamp.sec + stamp.nanosec*1e-9
    return stamp*1e-6
def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec*1e-9
    

class MarkerLocalizer(Node):
    def __init__(self):
        super().__init__('marker_localization')
        qos = QoSProfile(depth=1,reliability =(ReliabilityPolicy.BEST_EFFORT))
        #self.create_subscription(PoseStamped, "/posestamp_camOfThanh",self.chay,qos)
        
        #self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.on_mavros_pose,qos)

        
        # self.create_subscription(Imu, "/mavros/imu/data", self.on_imu, qos)
        # replace by below
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_imu, qos)
        self.create_subscription(PoseStamped, "/posestamp_camOfThanh", self.on_marker_pose,qos)


        self.pub_rel = self.create_publisher(PointStamped, "/marker_rel", qos)
        self.pub_pose = self.create_publisher(PoseStamped, "/marker_pose", qos)

        self.declare_parameter("buffer-s",2.0)


        
        self._attitude = deque()

        self.declare_parameter("mount_roll_deg", 0.0)
        self.declare_parameter("mount_pitch_deg", 0.0)
        self.declare_parameter("mount_yaw_deg", 0.0)
        self.declare_parameter("cam_offset", (0.07,0.0,-0.07))
        
        # self.declare_parameter("cam_offset", (0.0,0.0,0.0))


        self.declare_parameter("max_attitude_age_s", 0.15)
        self.declare_parameter("level_frame_id", "base_link_level")
        self.declare_parameter("map_frame_id", "map")

        p=self.get_parameter

        self.max_attitude_age = float(p("max_attitude_age_s").value)
        self.buffer_s = p("buffer-s").value
        self.level_frame_id = p("level_frame_id").value
        self.map_frame_id = p("map_frame_id").value
        offset = list(p("cam_offset").value)
        self.extrinsic  = CameraExtrinsic(mount_roll = float(p("mount_roll_deg").value),
                                          mount_pitch = float(p("mount_pitch_deg").value),
                                          mount_yaw = float(p("mount_yaw_deg").value),offset=offset)


        self.last_odom_time = None
        self.last_marker_pose_time = None
        self.last_rel_publish_time = None
        self.last_pose_publish_time = None

        self.last_time_sync_error = None
        self.last_transform_time = None

        self.health_timer = self.create_timer(1.0,self.health_check)

        self.odom_timeout = 1.0
        self.marker_pose_timeout = 1.0
        self.output_timeout = 1.0
        self.max_attitude_age = float(
            self.get_parameter("max_attitude_age_s").value
        )
        self.health_timer = self.create_timer(
                    1.0,
                    self.health_check
                )
        self.pub_health = self.create_publisher(MarkerLocalizerHealth,"/drone/marker_localizer_health",10)
   
    def on_mavros_pose(self,msg):
        # cais nayf dunfg gps
        q = msg.pose.orientation
        pose = msg.pose.position
        self._push_attitude(stamp_to_sec(msg.header.stamp),(q.x,q.y,q.z,q.w),(pose.x,pose.y,pose.z))
    def _push_attitude(self,t, q_xyzw, drone_pos):

        self._attitude.append((t, q_xyzw, drone_pos))
        # print("attitude",self._attitude)
        
        cutoff = t - self.buffer_s
        while self._attitude and self._attitude[0][0]<cutoff:
            self._attitude.popleft()

    def on_imu(self,msg):
        # cai nay chay trong nha
        self.last_odom_time = time.monotonic()
        q = msg.q
        drone_pos = msg.position
        self._push_attitude(stamp_to_sec_odom(msg.timestamp_sample),(q[1],q[2],q[3],q[0]),(drone_pos[0],drone_pos[1],drone_pos[2]))

    def _lookup(self,t):
        if not self._attitude:
            return None
        best = min(self._attitude, key = lambda s:abs(s[0]-t))
        return best if abs(best[0] - t) <= self.max_attitude_age else None

    
    
    def on_marker_pose(self,msg):
        self.last_marker_pose_time = time.monotonic()
        t_capture = stamp_to_sec(msg.header.stamp)
        sample = self._lookup(t_capture)

        if sample is None:
            return
        time_sync_error = abs(sample[0] - t_capture)

        self.last_time_sync_error = time_sync_error

        _, q_xyzw, drone_pos = sample
        
        t = msg.pose.position
        print("t o day:    ", t)
        p_cam = np.array([t.x,t.y,t.z],dtype = float)

        print("p_cam", p_cam)

        p_body_ned = (self.extrinsic.R_ned_from_cam @ p_cam + self.extrinsic.offset)
        R_enu_from_ned = quat_to_rotmat(*q_xyzw)
        print("p_body_flu", p_body_ned)
        p_rel = R_enu_from_ned @ p_body_ned
        # p_rel = p_body_ned
        self.last_transform_time = time.monotonic()
        header_level = msg.header
        header_level.frame_id = self.level_frame_id

        rel = PointStamped()
        rel.header = header_level
        rel.point.x, rel.point.y, rel.point.z = map(float, p_rel)
        self.pub_rel.publish(rel)
        self.last_rel_publish_time = time.monotonic()

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame_id
        out.pose.position.x = float(drone_pos[0] + p_rel[0])
        out.pose.position.y = float(drone_pos[1] + p_rel[1])
        out.pose.position.z = float(drone_pos[2] + p_rel[2])
        # Huong cua MARKER, khong phai cua drone: phai di qua ca chuoi
        # camera -> than -> ENU. Chi lay yaw, bo phan nghieng vi ambiguity.
        o= msg.pose.orientation
        yaw = marker_yaw_enu(quat_to_rotmat(o.x, o.y, o.z, o.w), q_xyzw, self.extrinsic)
        out.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_pose.publish(out)
        self.last_pose_publish_time = time.monotonic()

        # p_rel[2] am khi marker o duoi drone -> doi dau thanh do cao duong
    def health_check(self):

        now = time.monotonic()

        # ==========================================
        # 1. CHECK ODOMETRY
        # ==========================================

        if self.last_odom_time is None:
            odometry_ok = False
            odometry_age = float("inf")
        else:
            odometry_age = now - self.last_odom_time
            odometry_ok = odometry_age < self.odom_timeout


        # ==========================================
        # 2. CHECK MARKER POSE INPUT
        # ==========================================

        if self.last_marker_pose_time is None:
            marker_pose_ok = False
            marker_pose_age = float("inf")
        else:
            marker_pose_age = now - self.last_marker_pose_time
            marker_pose_ok = marker_pose_age < self.marker_pose_timeout


        # ==========================================
        # 3. CHECK TIME SYNCHRONIZATION
        # ==========================================

        if self.last_time_sync_error is None:

            time_sync_ok = False

            time_sync_error = float("inf")

        else:

            time_sync_error = self.last_time_sync_error

            time_sync_ok = (
                time_sync_error <= self.max_attitude_age
            )


        # ==========================================
        # 4. CHECK TRANSFORM
        # ==========================================

        if self.last_transform_time is None:

            transform_ok = False

        else:

            transform_age = now - self.last_transform_time

            transform_ok = (
                transform_age < self.output_timeout
            )


        # ==========================================
        # 5. CHECK OUTPUT
        # ==========================================

        if self.last_rel_publish_time is None:

            output_ok = False

        else:

            rel_age = now - self.last_rel_publish_time

            output_ok = rel_age < self.output_timeout


        # ==========================================
        # 6. MARKER AVAILABLE
        # ==========================================

        marker_available = marker_pose_ok


        # ==========================================
        # 7. DETERMINE LEVEL
        # ==========================================

        # ERROR:
        # Không có odometry hoặc localizer không hoạt động

        if not odometry_ok:

            level = 2
            status = "ODOMETRY ERROR"


        elif not time_sync_ok and marker_pose_ok:

            level = 2
            status = "TIME SYNC ERROR"


        elif not transform_ok and marker_pose_ok:

            level = 2
            status = "TRANSFORM ERROR"


        elif not output_ok and marker_pose_ok:

            level = 2
            status = "OUTPUT ERROR"


        # WARN:
        # Hệ thống vẫn chạy nhưng chưa có marker

        elif not marker_available:

            level = 1
            status = "WAITING FOR MARKER"


        # OK

        else:

            level = 0
            status = "MARKER LOCALIZER OK"


        # ==========================================
        # 8. PUBLISH HEALTH
        # ==========================================

        health = MarkerLocalizerHealth()

        health.odometry_ok = odometry_ok
        health.marker_pose_ok = marker_pose_ok
        health.time_sync_ok = time_sync_ok
        health.transform_ok = transform_ok
        health.output_ok = output_ok

        health.marker_available = marker_available

        if np.isfinite(odometry_age):
            health.odometry_age = float(odometry_age)
        else:
            health.odometry_age = -1.0

        if np.isfinite(marker_pose_age):
            health.marker_pose_age = float(marker_pose_age)
        else:
            health.marker_pose_age = -1.0

        if np.isfinite(time_sync_error):
            health.time_sync_error = float(time_sync_error)
        else:
            health.time_sync_error = -1.0

        health.level = level
        health.status = status

        self.pub_health.publish(health)
def main(args=None):
    rclpy.init(args=args)
    brake = MarkerLocalizer()
    rclpy.spin(brake)
    brake.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()

                