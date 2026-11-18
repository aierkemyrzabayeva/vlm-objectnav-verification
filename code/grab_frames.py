#!/usr/bin/env python3
"""
grab_frames.py - run inside the ROS container while the simulation is up.

Rotates the robot r1 in place at (0, 0) to 0/90/180/270 degrees and, at every heading, saves
  <out>/color_<yaw>.png   color frame from /realsense_camera/color/image_raw
  <out>/depth_<yaw>.npy   depth frame from /realsense_camera/depth/image_raw (float32, metres)
and prints image size / encoding / depth statistics (needed to check the TD3 depth normalisation
and to feed real frames to the VLM).

Usage: python3 grab_frames.py OUT_DIR
"""
import math
import os
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Image

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
rospy.init_node("grab_frames")
bridge = CvBridge()
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=5)
time.sleep(2.0)


def place(yaw_deg):
    yaw = math.radians(yaw_deg)
    ms = ModelState()
    ms.model_name = "r1"
    ms.pose.position.x = 0.0
    ms.pose.position.y = 0.0
    ms.pose.position.z = 0.0
    ms.pose.orientation.z = math.sin(yaw / 2.0)
    ms.pose.orientation.w = math.cos(yaw / 2.0)
    for _ in range(5):
        pub.publish(ms)
        time.sleep(0.2)


for yaw_deg in (0, 90, 180, 270):
    place(yaw_deg)
    rospy.sleep(3.0)
    color = rospy.wait_for_message("/realsense_camera/color/image_raw", Image, timeout=30)
    depth = rospy.wait_for_message("/realsense_camera/depth/image_raw", Image, timeout=30)
    bgr = bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
    cv2.imwrite(os.path.join(out, f"color_{yaw_deg}.png"), bgr)
    d = np.array(bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough"), dtype=np.float32)
    np.save(os.path.join(out, f"depth_{yaw_deg}.npy"), d)
    fin = np.isfinite(d)
    print(f"yaw={yaw_deg:3d} color={color.width}x{color.height} enc={color.encoding} | "
          f"depth={depth.width}x{depth.height} enc={depth.encoding} finite={fin.mean():.2f} "
          f"min={np.nanmin(d[fin]) if fin.any() else float('nan'):.2f} "
          f"median={np.median(d[fin]) if fin.any() else float('nan'):.2f} "
          f"max={np.nanmax(d[fin]) if fin.any() else float('nan'):.2f}", flush=True)
print("GRAB_DONE")
