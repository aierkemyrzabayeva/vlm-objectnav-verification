#!/usr/bin/env python3
"""
diag_turn.py - is the heading used by SimEnv correct?

Starts the authors' environment in our world, teleports the robot to a known pose, rotates it to several target
headings with SimEnv._face_heading and prints, after every step, the pose according to the odometry (what the code
uses) next to the true pose from /gazebo/model_states. Columns:
  d_yaw   = odom yaw - true yaw        (should be ~0)
  d_pos   = distance between odom and true position   (should be ~0)
  err_tgt = true yaw - commanded target (should be within ~0.12 rad)
No VLM and no policy are needed. Run through diag_turn.sh.
"""
import math
import os
import sys
import time

BASE = os.environ.get("VLMNAV_BASE", os.getcwd())
sys.path.insert(0, os.path.join(BASE, "vlm_nav", "ours"))

import real_env_ours as renv                                   # noqa: E402
renv.LAUNCHFILE = os.environ.get("OURS_LAUNCH", f"{BASE}/vlm_nav/ours/multi_robot_scenario.launch")
import rospy                                                    # noqa: E402
from gazebo_msgs.msg import ModelStates                         # noqa: E402
from sim_env import SimEnv, wrap                                # noqa: E402

genv = renv.GazeboEnv()
time.sleep(5)
sim = SimEnv(renv, genv, None, "http://127.0.0.1:1", turn_sign=float(os.environ.get("TURN_SIGN", "-1.0")))
print(f"turn_sign = {sim.turn_sign}", flush=True)
ERRS = []


def true_pose():
    genv.unpause()
    try:
        st = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=10)
    finally:
        genv.pause()
    p = st.pose[st.name.index("r1")]
    q = p.orientation
    return p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def report(tag, target=None):
    sim._run_sim(0.3)
    tx, ty, tyaw = true_pose()
    ox, oy, oyaw = genv.odom_x, genv.odom_y, genv.odom_yaw
    extra = ""
    if target is not None:
        ERRS.append(abs(wrap(tyaw - target)))
        extra = f"  err_tgt={wrap(tyaw - target):6.2f}"
    print(f"{tag:<30s} odom=({ox:6.2f},{oy:6.2f},{oyaw:6.2f}) true=({tx:6.2f},{ty:6.2f},{tyaw:6.2f}) "
          f"d_yaw={wrap(oyaw - tyaw):6.2f} d_pos={math.hypot(ox - tx, oy - ty):5.2f}{extra}", flush=True)


genv.forced_start = (2.0, -1.0, 0.7)
genv.goal_x, genv.goal_y = 3.0, -1.0
genv.reset()
report("after reset to (2,-1,0.70)")
report("... 0.3 s later")
for deg in (0, 90, 180, -90, 45, -135, 90):
    theta = math.radians(deg)
    t0 = time.time()
    sim._face_heading(theta)
    report(f"face {deg:4d} deg (tgt {theta:5.2f})", theta)
tw = sim.Twist()
tw.linear.x = 0.3
for _ in range(10):
    genv.vel_pub.publish(tw)
    sim._run_sim(0.1)
genv.vel_pub.publish(sim.Twist())
report("after 1 s forward at 0.3 m/s")
print(f"TURN_CHECK max |err_tgt| = {max(ERRS):.2f} rad ->", "PASS" if max(ERRS) < 0.2 else "FAIL", flush=True)
print("DIAG_DONE", flush=True)
