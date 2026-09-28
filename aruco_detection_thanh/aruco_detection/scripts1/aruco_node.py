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



from std_msgs.msg import Bool, Float32
import cv2.aruco as aruco

from rclpy.qos import QoSProfile, ReliabilityPolicy

import detector 
from calibration import load_calibration
from transform import rotmat_to_quat

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
        self.declare_parameter("threaded_capture", True)
        
        self.marker_size = self.get_parameter("marker-size").value
        self.cam = self.get_parameter("cam").value
        self.id = self.get_parameter("id").value
        self.calib = self.get_parameter("calib").value
        self.publish_debug_image = self.get_parameter("publish_debug_image").value
        self.ambiguity_warn = self.get_parameter("ambiguity_warn").value
        self.detector_type = self.get_parameter("detector_type").value
        self.url = "rtsp://192.168.144.108:554/stream=1"
        if self.publish_debug_image:
            if CvBridge is None:
                self.get_logger().warn("cv_bridge missing -> publish_debug_image disabled")
                self.publish_debug_image = False
            else:
                from sensor_msgs.msg import Image
                self.bridge = CvBridge()
                self.pub_image = self.create_publisher(Image, "/image_camOfThanh", 10)
        self.camera_matrix, self.dist_coeffs, self.calib_size = load_calibration(self.calib)
        frame_size = self._open_camera(self.url, self.calib_size, 
                                       self.get_parameter("threaded_capture").value)
        self.detector= detector.MarkerDetector(dict_id=getattr(aruco, detector.DEFAULT_DICT) ,marker_size=self.marker_size, 
                                      camera_matrix=self.camera_matrix, dist_coeffs =  self.dist_coeffs, 
                                      detector_type = self.detector_type,calib_size =self.calib_size)
        self.get_logger().info(f"OpenCV {cv2.__version__}  |  dictionary {detector.DEFAULT_DICT}  ")
        self.get_logger().info(f"Marker size: {self.marker_size} cm")

        
        self.frame_id = "Camera Image"
        self.n_frame = 0
        self.n_detected = 0
        self._last_cb = None
        self._fps = 0.0
        
                    
        # INITIALIZE THE CAMERA
        # self.cap = cv2.VideoCapture(self.cam)

        # if self.marker_size == detector.DEFAULT_MARKER_SIZE:
        #     self.get_logger().info("  (mac dinh, suy ra tu PDF in A3 100% - do lai neu khoang cach sai)")
        # else:
        #     self.get_logger().info("")
        # if not self.cap.isOpened():
        #     raise SystemExit(f"Khong mo duoc camera index {self.cam}")
        
        qos = QoSProfile(depth=1,reliability =(ReliabilityPolicy.BEST_EFFORT))
        
        self.pub_poststamp = self.create_publisher(PoseStamped,"/posestamp_camOfThanh",qos)
        self.create_timer(1.0/30.0,self.on_time)
        

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
        msg.pose.orientation.x = qy
        msg.pose.orientation.x = qz
        msg.pose.orientation.x = qw
                                
        self.pub_poststamp.publish(msg) 

    def on_time(self):
        t0= time.perf_counter()

        ok, not_yet_distored = self.cam.read()
        if not ok:
            return

        frame = cv2.undistort(not_yet_distored, self.camera_matrix,self.dist_coeffs)
        self.n_frame +=1

       
    
        

        if self._last_cb is not None:
            dt = t0 - self._last_cb
            if dt > 0:
                self._fps = 0.9 *self._fps + 0.1 *(1.0/dt) if self._fps else 1.0/dt
        self._last_cb = t0

        stamp = self.get_clock().now().to_msg()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        t_det0= time.perf_counter()
        #NGAY TAI DAY===========

       
        detections=self.detector.process_simultaneously(gray)

        detect_ms = (time.perf_counter() - t_det0)*1000


        # a,b = self.detector.process_simultaneously(gray)
        # if a:

        if detections:
            self.n_detected +=0
            det = detections[0]
            print("dettttttt",det)
            print("===============================================")
            self._pulblish_pose(stamp,det) 

      
        if self.publish_debug_image:
            self._publish_debug(stamp, frame, detections, t0, detect_ms)

    def _open_camera(self, source, calib_size, threaded):
        self.cam = open_camera(source, width=calib_size[0], height=calib_size[1],
                               threaded=threaded)
        self.get_logger().info(f"Camera: {self.cam}")

        # The requested resolution is only a request: cap.set() fails silently
        # on many USB cameras. Compare against what actually came back, because
        # a mismatch makes every distance wrong by the resolution ratio.
        frame_size = self.cam.resolution
        if frame_size != calib_size:
            self.get_logger().warn(f"Camera returns {frame_size[0]}x{frame_size[1]} instead of the "
                f"calibrated {calib_size[0]}x{calib_size[1]}")
        return frame_size

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

        msg = self.bridge.cv2_to_imgmsg(frame,encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        self.pub_image.publish(msg)  


if __name__ == "__main__":

    rclpy.init()
    node = ArucoPoseNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
