#!/usr/bin/env python3
"""
sim_env.py - implements the `env` interface of verify_repair.py on top of
  * the authors' GazeboEnv (patched copy real_env_ours.py) + their TD3 policy   -> go_to()
  * the VLM server (vlm_verify_server.py)                                        -> verify()
Runs in the ROS container. `localize()` is pure Python (unit-tested offline).
"""
import math
import time

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def localize(box, depth_mm, fx, cam_xy_yaw):
    """World (x, y) of the object in `box` = (x_min, y_min, x_max, y_max), normalised image coordinates.
    depth_mm: HxW uint16/float depth in millimetres, registered with the colour image (same size and FOV).
    fx: focal length in pixels of that image. cam_xy_yaw: camera pose in the world."""
    h, w = depth_mm.shape[:2]
    x0, y0, x1, y1 = [min(max(float(v), 0.0), 1.0) for v in box]
    if x1 <= x0 or y1 <= y0:
        return None
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)           # central 30% of the box
    hw, hh = 0.15 * (x1 - x0), 0.15 * (y1 - y0)
    u0, u1 = int((cx - hw) * w), max(int((cx + hw) * w), int((cx - hw) * w) + 1)
    v0, v1 = int((cy - hh) * h), max(int((cy + hh) * h), int((cy - hh) * h) + 1)
    patch = np.asarray(depth_mm[v0:v1, u0:u1], dtype=np.float64)
    ok = np.isfinite(patch) & (patch > 0)
    if ok.sum() < 5:
        return None
    z = float(np.median(patch[ok])) / 1000.0
    right = ((cx * w) - w / 2.0) / fx * z                # image x grows to the right
    left = -right
    x, y, yaw = cam_xy_yaw
    return (x + z * math.cos(yaw) - left * math.sin(yaw), y + z * math.sin(yaw) + left * math.cos(yaw))


def inside_room(pos, half, margin=0.1):
    """A localised object cannot lie outside the walls (depth through a thin object often hits the wall behind it)."""
    return abs(pos[0]) <= half + margin and abs(pos[1]) <= half + margin


class SimEnv:
    def __init__(self, renv, genv, model, vlm_url, prompts="A", rule="and", cam_forward=0.55, step_budget=2500,
                 recover=1, peripheral=True, max_waypoints=6, turn_sign=-1.0, sweep=0.0):
        import rospy
        from cv_bridge import CvBridge
        from gazebo_msgs.srv import SetModelState
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import CameraInfo, Image
        self.rospy, self.Twist = rospy, Twist
        self.renv, self.e, self.model = renv, genv, model
        self.url, self.prompts, self.rule = vlm_url.rstrip("/"), prompts.split(","), rule
        self.cam_forward, self.step_budget, self.recover_tries = cam_forward, step_budget, recover
        self.peripheral = peripheral
        self.max_waypoints = max_waypoints
        # Measured with diag_turn.py: for this robot model a POSITIVE angular.z turns the robot CLOCKWISE (odom yaw
        # decreases), opposite to the ROS convention. turn_sign = -1 compensates in our own turning loops; the TD3
        # policy is unaffected because it was trained in closed loop on the same robot.
        self.turn_sign = turn_sign
        # heading sweep at the approach pose: when the first look says no, look again at +-sweep rad before rejecting
        # (the remembered point of a search hit is only accurate to ~0.6 m, which puts the object near the image edge)
        self.sweep_offsets = (0.0, sweep, -sweep) if sweep > 0 else (0.0,)
        self.recoveries = 0
        self.bridge = CvBridge()
        self._color, self._depth = None, None
        rospy.Subscriber("/realsense_camera/color/image_raw", Image, lambda m: setattr(self, "_color", m), queue_size=1)
        rospy.Subscriber("/realsense_camera/depth/image_raw", Image, lambda m: setattr(self, "_depth", m), queue_size=1)
        info = rospy.wait_for_message("/realsense_camera/color/camera_info", CameraInfo, timeout=60)
        self.fx = info.K[0]
        rospy.wait_for_service("/gazebo/set_model_state", timeout=60)
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.vlm_calls, self.vlm_time, self.path, self.steps_used = 0, 0.0, 0.0, 0
        self.legs = []
        self.obstacles, self.room_half = [], 4.5
        self._home_yaw = 0.0
        self.debug_dir, self.debug_prefix, self._dbg_k = None, "", 0
        self.vlm_log, self.scene_dev = [], 0.0
        self.vocab = ""

    # ------------------------------------------------------------- env interface
    def robot_xy(self):
        return (self.e.odom_x, self.e.odom_y)

    def time(self):
        t = getattr(self.e, "_sim_t", None)
        return t if t is not None else time.time()

    def go_to(self, x, y, stop_within, max_steps=400):
        """Drive with the TD3 policy until within stop_within of (x, y). If the controller gets stuck or aborts early
        (collision / "extremely close"), back up,
        turn and retry (identical for every experimental condition). False on collision/stuck/timeout."""
        ok, reason = self._leg(x, y, stop_within, max_steps)
        tries = 0
        while not ok and reason in ("stuck", "collision") and tries < self.recover_tries and self.steps_used < self.step_budget:
            self._recover(tries)
            tries += 1
            self.recoveries += 1
            ok, reason = self._leg(x, y, stop_within, max_steps)
        return ok

    def _leg(self, x, y, stop_within, max_steps):
        if math.hypot(self.e.odom_x - x, self.e.odom_y - y) <= stop_within:
            return True, "goal"
        e = self.e
        old = self.renv.GOAL_REACHED_DIST
        self.renv.GOAL_REACHED_DIST = stop_within
        try:
            e.goal_x, e.goal_y = x, y
            obs = self._new_leg()
            last = (e.odom_x, e.odom_y)
            steps, done, reached = 0, False, False
            while not done and steps < max_steps and self.steps_used < self.step_budget:
                action, _ = self.model.predict(obs, deterministic=True)
                (img, sc), _r, done, target = e.step(action)
                obs = {"image": img, "scalars": sc}
                cur = (e.odom_x, e.odom_y)
                self.path += math.hypot(cur[0] - last[0], cur[1] - last[1])
                last = cur
                steps += 1
                self.steps_used += 1
                reached = bool(target)
            reason = "goal" if reached else ("stuck" if e.stuck_counter >= 100 else
                                             ("max_steps" if steps >= max_steps else "collision"))   # early abort
            self.legs.append({"target": (round(x, 2), round(y, 2)), "steps": steps, "reason": reason})
            return reached, reason
        finally:
            self.renv.GOAL_REACHED_DIST = old

    def _recover(self, k):
        """Back up about 0.4 m, then turn 90 degrees (alternating direction) to get out of a stuck situation."""
        tw = self.Twist()
        tw.linear.x = -0.25
        for _ in range(16):
            self.e.vel_pub.publish(tw)
            self._run_sim(0.1)
        self.e.vel_pub.publish(self.Twist())
        self._face_heading(wrap(self.e.odom_yaw + (math.pi / 2.0) * (1 if k % 2 == 0 else -1)))

    def verify(self, cls, face_xy=None, scan=False):
        """face_xy: look at the remembered point; a 'yes' counts only together with WHERE the object was seen, so a
        moved object that is visible elsewhere does not confirm the stale position (it becomes a new observation).
        scan: rotate in place through evenly spaced, overlapping headings."""
        if face_xy is not None:
            self._face(face_xy[0], face_xy[1])
            theta0 = math.atan2(face_xy[1] - self.e.odom_y, face_xy[0] - self.e.odom_x)
            for k, off in enumerate(self.sweep_offsets):
                if k > 0:                                              # look a little to the side before rejecting
                    self._face_heading(wrap(theta0 + off))
                bgr, depth = self._fresh_frames()
                if self._yes(bgr, cls):
                    pos = self._locate(bgr, depth, cls)
                    self._log(cls, bgr, True, pos, "face" if k == 0 else "sweep")
                    return [pos if pos is not None else face_xy]       # no box / no depth: accept at the looked-at point
                self._log(cls, bgr, False, None, "face" if k == 0 else "sweep")
            # peripheral look: the crop misses what stands beside the looked-at point; ask once on the full frame.
            # A localised answer far from the remembered point is evidence about where the object went.
            if self.peripheral and any(p.endswith("c") for p in self.prompts) and self._yes(bgr, cls, full=True):
                pos = self._locate(bgr, depth, cls)
                self._log(cls, bgr, True, pos, "peripheral")
                if pos is not None and math.hypot(pos[0] - face_xy[0], pos[1] - face_xy[1]) > 1.0:
                    return [pos]
            return []
        if scan:
            # evenly spaced headings with overlap: crop prompts see only ~49 deg, full-frame prompts ~85 deg
            fov = 49.4 if any(p.endswith("c") for p in self.prompts) else 85.2
            n = int(math.ceil(2.0 * math.pi / math.radians(0.82 * fov)))
            yaw0 = self.e.odom_yaw
            for k in range(n):
                self._face_heading(wrap(yaw0 + k * 2.0 * math.pi / n))
                bgr, depth = self._fresh_frames()
                yes = self._yes(bgr, cls)                        # the configured (possibly cropped) prompts
                pos = self._locate(bgr, depth, cls) if yes else None
                self._log(cls, bgr, yes, pos, "scan")
                if pos is not None:
                    return [pos]
            return []
        return []

    def _locate(self, bgr, depth, cls):
        """World position (estimated object centre) of the object the VLM reports, from detect box + depth."""
        box = self._detect(bgr, cls)
        if box is None:
            return None
        cam = (self.e.odom_x + self.cam_forward * math.cos(self.e.odom_yaw),
               self.e.odom_y + self.cam_forward * math.sin(self.e.odom_yaw), self.e.odom_yaw)
        pos = localize(box, depth, self.fx * depth.shape[1] / float(bgr.shape[1]), cam)
        if pos is None:
            return None
        d = math.hypot(pos[0] - cam[0], pos[1] - cam[1])
        if d >= 1e-6:
            k = 0.4 / d                                       # surface point -> typical object centre
            pos = (pos[0] + (pos[0] - cam[0]) * k, pos[1] + (pos[1] - cam[1]) * k)
        return pos if inside_room(pos, self.room_half) else None   # outside the walls: invalid localisation

    def _log(self, cls, bgr, yes, pos, tag=""):
        self._dbg_k += 1
        fn = ""
        if self.debug_dir:
            import cv2
            import os
            os.makedirs(self.debug_dir, exist_ok=True)
            fn = f"{self.debug_prefix}_{self._dbg_k}.png"
            cv2.imwrite(os.path.join(self.debug_dir, fn), bgr)
        self.vlm_log.append({"k": self._dbg_k, "tag": tag, "cls": cls, "yes": int(yes), "frame": fn,
                             "robot": [round(self.e.odom_x, 2), round(self.e.odom_y, 2), round(self.e.odom_yaw, 2)],
                             "pos": None if pos is None else [round(pos[0], 2), round(pos[1], 2)]})

    def search_waypoints(self, cls):
        """3x3 grid over the room (spacing ~0.31 of the room side), obstacle-free points, nearest-neighbour order."""
        h = 0.31 * 2 * self.room_half
        pts = [(ix * h, iy * h) for ix in (-1, 0, 1) for iy in (-1, 0, 1)]
        pts = [p for p in pts if all(math.hypot(p[0] - o["x"], p[1] - o["y"]) > o["radius"] + 0.55 for o in self.obstacles)]
        order, cur = [], (self.e.odom_x, self.e.odom_y)
        while pts and len(order) < self.max_waypoints:
            nxt = min(pts, key=lambda p: math.hypot(p[0] - cur[0], p[1] - cur[1]))
            order.append(nxt)
            pts.remove(nxt)
            cur = nxt
        return order

    # ------------------------------------------------------------- episode control
    def start_episode(self, sx, sy, yaw, moves, truth, room):
        """Reset the world, place the robot, then put EVERY object on its expected position (this does not rely on
        reset_world restoring models that were moved by set_model_state) and check the result."""
        e = self.e
        e.forced_start = (sx, sy, yaw)
        e.goal_x, e.goal_y = sx + 1.0, sy
        e.reset()
        for o in truth:
            self._put(o["id"], o["x"], o["y"], o.get("yaw", 0.0))
        self.obstacles, self.room_half = [dict(o) for o in truth], room / 2.0
        self.vocab = ",".join(sorted({o["class"] for o in truth}) + ["nothing"])
        self.vlm_calls, self.vlm_time, self.path, self.steps_used, self.legs = 0, 0.0, 0.0, 0, []
        self.recoveries, self.vlm_log, self._dbg_k = 0, [], 0
        self._run_sim(0.6)
        self.scene_dev = self._scene_deviation(truth)
        if self.scene_dev > 0.3:
            print(f"WARNING: scene deviates from the expected state by {self.scene_dev:.2f} m", flush=True)

    # ------------------------------------------------------------- helpers
    def _scene_deviation(self, truth):
        """Largest distance between the real model positions (Gazebo) and the expected ones; -1 if unavailable.
        /gazebo/model_states is only published while the world runs, so the pause is lifted for the read."""
        from gazebo_msgs.msg import ModelStates
        try:
            self.e.unpause()
            try:
                st = self.rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=10)
            finally:
                self.e.pause()
            dev = 0.0
            for o in truth:
                if o["id"] in st.name:
                    i = st.name.index(o["id"])
                    dev = max(dev, math.hypot(st.pose[i].position.x - o["x"], st.pose[i].position.y - o["y"]))
            return dev
        except Exception as ex:                       # never abort an experiment because of a diagnostic
            print(f"WARNING: scene check failed ({type(ex).__name__})", flush=True)
            return -1.0

    def _put(self, name, x, y, yaw):
        from gazebo_msgs.msg import ModelState
        ms = ModelState()
        ms.model_name = name
        ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = x, y, 0.0
        ms.pose.orientation.z, ms.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        ms.reference_frame = "world"
        if not self._set_state(ms).success:
            raise RuntimeError(f"could not move model {name}")

    def _run_sim(self, dt):
        self.e.unpause()
        self.e._sim_wait(dt)
        self.e.pause()

    def _new_leg(self):
        """Start a new navigation leg WITHOUT reset_world (which would put relocated objects back)."""
        e, r = self.e, self.renv
        e.current_step, e.stuck_counter, e.near_wall_counter, e.time_step = 0, 0, 0, 0
        e.last_robot_x = e.last_robot_y = None
        e.prev_action = np.zeros(2, dtype=np.float32)
        e.last_action = np.zeros(2, dtype=np.float32)
        e.prev_wall_real = None
        e.cum_rotation, e.last_yaw = 0.0, None
        d = float(np.linalg.norm([e.odom_x - e.goal_x, e.odom_y - e.goal_y]))
        e.prev_distance = d
        scalars = np.array([0.0, 0.0, 0.0, 0.0, d, e.compute_angle_to_goal(), e.min_wall_real], dtype=np.float32)
        if e.normed_depth is not None:
            import cv2
            img = cv2.resize(e.normed_depth, (r.IMG_WIDTH, r.IMG_HEIGHT))[None, ...]
        else:
            img = np.zeros((1, r.IMG_HEIGHT, r.IMG_WIDTH), dtype=np.float32)
        return {"image": img, "scalars": scalars}

    def _face(self, x, y):
        self._face_heading(math.atan2(y - self.e.odom_y, x - self.e.odom_x))

    def _face_heading(self, theta):
        for _ in range(80):
            err = wrap(theta - self.e.odom_yaw)
            if abs(err) < 0.12:
                break
            tw = self.Twist()
            w = max(-1.0, min(1.0, 1.5 * err))
            if abs(w) < 0.25:
                w = 0.25 if err > 0 else -0.25
            tw.angular.z = self.turn_sign * w
            self.e.vel_pub.publish(tw)
            self._run_sim(0.1)
        self.e.vel_pub.publish(self.Twist())
        self._run_sim(0.05)

    def _fresh_frames(self):
        """Colour (BGR) and depth (mm) frames captured after the robot stopped."""
        t0 = self.time()
        for _ in range(6):
            self._run_sim(0.4)
            c, d = self._color, self._depth
            if c is not None and d is not None and c.header.stamp.to_sec() >= t0 + 0.3 and d.header.stamp.to_sec() >= t0 + 0.3:
                break
        bgr = self.bridge.imgmsg_to_cv2(self._color, desired_encoding="bgr8")
        depth = np.array(self.bridge.imgmsg_to_cv2(self._depth, desired_encoding="passthrough"), dtype=np.float32)
        return bgr, depth

    def _post(self, path, bgr, data):
        import cv2
        import requests
        ok, buf = cv2.imencode(".png", bgr)
        t = time.time()
        r = requests.post(self.url + path, files={"image": ("f.png", buf.tobytes(), "image/png")}, data=data, timeout=120)
        r.raise_for_status()
        self.vlm_calls += 1
        self.vlm_time += time.time() - t
        return r.json()

    def _yes(self, bgr, cls, full=False):
        """Ask the VLM with the configured prompts; full=True drops the crop suffix (full-frame question)."""
        prompts = [p.rstrip("c") for p in self.prompts] if full else self.prompts
        ys = []
        for pk in prompts:
            y = int(self._post("/verify", bgr, {"cls": cls, "prompt": pk, "options": self.vocab})["yes"])
            ys.append(y)
            if self.rule == "and" and not y:
                return False
            if self.rule == "or" and y:
                return True
        return all(ys) if self.rule == "and" else any(ys)

    def _detect(self, bgr, cls):
        objs = self._post("/detect", bgr, {"cls": cls}).get("objects", [])
        if not objs:
            return None
        o = objs[0]
        return (o["x_min"], o["y_min"], o["x_max"], o["y_max"])
