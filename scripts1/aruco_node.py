#!/usr/bin/env python3
import json
import math
import os

import cv2

import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from load_camera import open_camera
from skydroid_msgs.msg import GimbalCommand, GimbalState

from rclpy.executors import ExternalShutdownException

from std_msgs.msg import Bool, Float32
import cv2.aruco as aruco

from rclpy.qos import QoSProfile, ReliabilityPolicy

import detector, threading
from calibration import load_calibration
from transform import rotmat_to_quat
from geometry import CameraExtrinsic
from aruco_msgs.msg import ArucoHealth
import visualization as viz

try:
    from cv_bridge import CvBridge

except ImportError:
    CvBridge = None

class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__("aruco_pose_node")
        

        
        
  
        self.declare_parameter("cam", 2)
        self.declare_parameter("marker-size",detector.DEFAULT_MARKER_SIZE)
        self.declare_parameter("id", -1)
        self.declare_parameter("calib",detector.DEFAULT_CALIB_FILE)
        self.declare_parameter("side","left")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("ambiguity_warn", 0.7)
        self.declare_parameter("detector_type", "fractal")
        self.declare_parameter("threaded_capture", False)
        self.declare_parameter("undistored_in_node", True)
        
        self.marker_size = self.get_parameter("marker-size").value
        self.cam = self.get_parameter("cam").value
        self.target_id = self.get_parameter("id").value
        self.calib = self.get_parameter("calib").value
        self.publish_debug_image = self.get_parameter("publish_debug_image").value
        self.ambiguity_warn = self.get_parameter("ambiguity_warn").value
        self.detector_type = self.get_parameter("detector_type").value
        self.undistort = self.get_parameter("undistored_in_node").value
        qos = QoSProfile(depth=1,reliability =(ReliabilityPolicy.BEST_EFFORT))

      

        

        self.declare_parameter("mount_roll_deg", 0.0)
        self.declare_parameter("mount_pitch_deg", 0.0)
        self.declare_parameter("mount_yaw_deg", 0.0)
        self.declare_parameter("cam_offset", (0.07,0.0,-0.07))
        offset = list(self.get_parameter("cam_offset").value)
       ######
        self.extrinsic  = CameraExtrinsic(mount_roll = float(self.get_parameter("mount_roll_deg").value),
                                          mount_pitch = float(self.get_parameter("mount_pitch_deg").value),
                                          mount_yaw = float(self.get_parameter("mount_yaw_deg").value),offset=offset)
       ######
        
        # self.sub_gimbal_state = self.create_subscription(GimbalState, '/gimbal_state', self.check_cam_down,10)

        
        if self.publish_debug_image:
            if CvBridge is None:
                self.get_logger().warn("cv_bridge missing -> publish_debug_image disabled")
                self.publish_debug_image = False
            else:
                from sensor_msgs.msg import Image
               
                self.bridge = CvBridge()
                self.sub_image = self.create_subscription(Image, "/skydroid/rgb/image_raw", self.get_image,qos)
                self.pub_image = self.create_publisher(Image, "/image_camOfThanh", qos)
        self.url = "rtsp://192.168.144.108:554/stream=1"
        self.camera_matrix, self.dist_coeffs, self.calib_size = load_calibration(self.calib)
        # frame_size = self._open_camera(self.url, self.calib_size, 
        #                                self.get_parameter("threaded_capture").value)
        self.detector= detector.MarkerDetector(dict_id=getattr(aruco, detector.DEFAULT_DICT) ,marker_size=self.marker_size, 
                                      camera_matrix=self.camera_matrix, dist_coeffs =  self.dist_coeffs, 
                                      detector_type = self.detector_type,calib_size =self.calib_size)
        self.get_logger().info(f"OpenCV {cv2.__version__}  |  dictionary {detector.DEFAULT_DICT}  ")
        self.get_logger().info(f"Marker size: {self.marker_size} cm")


        # ------------status--------------


        self.frame_id = "Camera Image"
        self.n_frame = 0
        self.n_detected = 0
        self._last_cb = None
        self._fps = 0.0
        self.lastest_frame = None
        self.frame_lock = threading.Lock()
        self.new_frame_event = threading.Event()

        self.stop_detector = False

        self._undist_maps = None
        self._undist_size = None



        

        

        
        # Thread chạy ArUco
        self.detector_thread = threading.Thread(
            target=self.detector_loop,
            daemon=True
        )
        self.detector_thread.start()



        
        
                    
        # INITIALIZE THE CAMERA
        # self.cap = cv2.VideoCapture(self.cam)

        # if self.marker_size == detector.DEFAULT_MARKER_SIZE:
        #     self.get_logger().info("  (mac dinh, suy ra tu PDF in A3 100% - do lai neu khoang cach sai)")
        # else:
        #     self.get_logger().info("")
        # if not self.cap.isOpened():
        #     raise SystemExit(f"Khong mo duoc camera index {self.cam}")
        
        
        
        self.pub_poststamp = self.create_publisher(PoseStamped,"/posestamp_camOfThanh",qos)
        self.check_cam = None
        self.pub_health = self.create_publisher(ArucoHealth,"/drone/aruco_health",10)
        # self.create_timer(1.0/30.0,self.on_time)




   

    def calculate_zero_xy_pixel(self, z_cam):
        """
        Tính pixel trên ảnh mà tại đó:
            p_body_flu.x = 0
            p_body_flu.y = 0

        z_cam: khoảng cách marker-camera theo trục Z camera.
        """

        R = self.extrinsic.R_ned_from_cam
        offset = self.extrinsic.offset

        # Muốn marker nằm trên trục Z của drone:
        # p_body_flu = [0, 0, Z_body]

        # p_cam = R.T @ (p_body - offset)

        R_T = R.T

        # p_cam[2] = z_cam
        #
        # R_T[2,2] * Z_body
        # - (R_T @ offset)[2]
        # = z_cam

        offset_cam = R_T @ offset

        Z_body = (
            z_cam + offset_cam[2]
        ) / R_T[2, 2]

        p_body = np.array([
            0.0,
            0.0,
            Z_body
        ])

        # Đổi từ FLU -> Camera
        p_cam = R_T @ (p_body - offset)

        X = p_cam[0]
        Y = p_cam[1]
        Z = p_cam[2]

        # Camera matrix
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Project camera coordinate -> pixel
        u = fx * X / Z + cx
        v = fy * Y / Z + cy

        return int(round(u)), int(round(v))



    def check_cam_down(self,msg):
            self.check_cam = msg.pitch_deg
    def chaykhi_down(self):
        
        if self.check_cam is None:
            return

        
        if int(self.check_cam) == -90:
            self.on_time()
        else:
            self.set_gimbal_pitch(-90.0) # Quay xuống vuông góc

    def set_gimbal_pitch(self, pitch_angle):
        """Hàm phụ trợ gửi lệnh điều khiển Gimbal"""
        state = GimbalCommand()
        state.control_mode = 0  # MANUAL
        state.mode = 1
        state.enable_pitch = True
        state.enable_yaw = False
        state.pitch_deg = float(pitch_angle)
        state.yaw_deg = 0.0
        state.pitch_speed_dps = 20.0
        state.yaw_speed_dps = 0.0
        state.pitch_vel_dps = 0.0
        state.yaw_vel_dps = 0.0
        self.pub_gimbale_state.publish(state)

    def reset_gimbal(self):
        """Hàm gọi khi tắt chương trình để đưa camera về lại vị trí ban đầu (0 độ)"""
        self.get_logger().info("Đang đưa Gimbal về vị trí ban đầu (0 deg)...")
        # Gửi lệnh trả về 0 độ nhiều lần để đảm bảo Gimbal nhận được tin nhắn trước khi node đóng
        for _ in range(5):
            self.set_gimbal_pitch(0.0)
            time.sleep(0.05)


    def _pulblish_pose(self,stamp, det):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        t = det.tvec.ravel()
        
        msg.pose.position.x = float(t[0])
        msg.pose.position.y = float(t[1])
        msg.pose.position.z = float(t[2])

        qx,qy,qz,qw= rotmat_to_quat(cv2.Rodrigues(det.rvec)[0])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
                                
        self.pub_poststamp.publish(msg) 
        self.last_pose_time = time.monotonic()
    def get_image(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        except Exception as e:
        
            self.get_logger().error(f"CvBridge error: {e}")

        now = time.monotonic()
        
        h, w = frame.shape[:2]
        self.frame_size = (w, h)

    
        
        if self.undistort:
            if self._undist_maps is None or self._undist_size != (w,h):
                self._undist_maps = cv2.initUndistortRectifyMap(      
                    self.camera_matrix, self.dist_coeffs, None,
                    self.camera_matrix, (w, h), cv2.CV_16SC2)
                self._undist_size = (w, h)
            frame = cv2.remap(frame, self._undist_maps[0], self._undist_maps[1], cv2.INTER_LINEAR)
                
        
        with self.frame_lock:

            self.lastest_frame = frame
            self.n_frame += 1
            
        
        self.new_frame_event.set()

        

        
    def detector_loop(self):

        

        last_processed_frame = 0

        while not self.stop_detector:
            
      

            if not self.new_frame_event.wait(timeout=0.05):
                continue
            self.new_frame_event.clear()

            with self.frame_lock:
                frame, idx = self.lastest_frame, self.n_frame
            if frame is None or idx == last_processed_frame:
                continue
            last_processed_frame = idx

            try: 
                self._process_frame(frame)
            except Exception as e:
                self.get_logger().error(f"Detector error: {e}", throttle_duration_sec=2.0)

    def _process_frame(self,frame):
        t0= time.perf_counter()

        stamp = self.get_clock().now().to_msg()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        t_det0= time.perf_counter()
        #NGAY TAI DAY===========
        
        # LAY DUOC MAY CAI GIA TRI IS BASED ON THIS FUNCTION PROCESS_ARUCO
        detections=self.detector.process(gray)
    
        detect_ms = (time.perf_counter() - t_det0)*1000
        
        self.last_detect_ms = detect_ms
    
        if self._last_cb is not None:
            dt = t0 - self._last_cb
            if dt > 0:
                self._fps = 0.9 *self._fps + 0.1 *(1.0/dt) if self._fps else 1.0/dt
        self._last_cb = t0


        # target = self._select_target(detections)

        # if target is not None:
        #     self.n_detected += 1
        #     self.marker_rate.tick(now)
        #     level, reason, hard_fail =  self._evaluate_pose(target, now)
        #     self.pose_state = (level,reason)
        #     if not (self.reject_bad_pose and hard_fail):
        #         self._pulblish_pose(stamp,target)
        if detections:
            self.n_detected +=0
            det = detections[0]
          
            self._pulblish_pose(stamp,det) 
                
        if self.publish_debug_image:
            print("do dayyy")
            self._publish_debug(stamp, frame, detections, t0, detect_ms)

    def _publish_debug(self,stamp, frame, detections, t0, detect_ms):
            viz.draw_markers(frame, detections)
            y = 30
            if detections:
                for det in detections:
                    y = viz.draw_detection(frame, det, self.camera_matrix, self.dist_coeffs, 
                                           self.marker_size, y = y, ambiguity_warn = self.ambiguity_warn)
    
            else:
                viz.draw_no_marker(frame,y)
    
            frame_ms = (time.perf_counter() - t0) * 1000
            viz.draw_stats(frame, fps = self._fps, frame_ms = frame_ms, detect_ms = detect_ms, 
                           detected=self.n_detected, total=self.n_frame)
            #draw cai tam
            for det in detections:
                
                # Lấy giá trị Z (trục Z chính là tvec[2])
                z_distance = det.tvec[2][0]
                u_target, v_target = self.calculate_zero_xy_pixel(z_distance)
    
                cv2.circle(frame,(u_target, v_target), 6, (0, 0, 255), -1)
    
            msg = self.bridge.cv2_to_imgmsg(frame,encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            
            self.pub_image.publish(msg)  
    
    
    
    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Nhan Ctrl+C, dang dung node...")
    finally:
        # 1. Dừng thread phát hiện ảnh trước để không publish đè dữ liệu mới
        node.stop_detector = True
        if hasattr(node, 'detector_thread') and node.detector_thread.is_alive():
            node.detector_thread.join(timeout=1.0)

        # 2. Kiểm tra ROS 2 context còn sống không trước khi gửi
        if rclpy.ok():
            try:
                # Gọi hàm reset
                # node.reset_gimbal()
                
                # BẮT BUỘC: Spin nhẹ vài chu kỳ để ROS 2 đẩy packet out-of-buffer ra network
                rclpy.spin_once(node, timeout_sec=0.2)
            except Exception as e:
                node.get_logger().error(f"Loi khi reset gimbal: {e}")

        # 3. Dọn dẹp node và shutdown context
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
