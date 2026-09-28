#!/usr/bin/env python3
"""Công cụ dùng chung cho health check.
 
- RateMeter   : đo tần số / tuổi dữ liệu bằng cửa sổ trượt (thread-safe)
- RatioMeter  : tỉ lệ thành công trong cửa sổ trượt
- grade()     : chấm OK/WARN/ERROR/STALE theo ngưỡng
- Hysteresis  : chống nhảy trạng thái (lên mức xấu nhanh, về OK chậm)
- make_status : đóng gói diagnostic_msgs/DiagnosticStatus
"""
import glob
import math
import os
import threading
import time
from collections import deque
 
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

OK, WARN, ERROR, STALE = 0,1,2,3
LEVEL_NAME = {OK: "OK", WARN: "WARN", ERROR: "ERROR", STALE: "STALE"}
_SEVERITY = {OK:0, WARN: 1, ERROR: 2, STALE:3}
