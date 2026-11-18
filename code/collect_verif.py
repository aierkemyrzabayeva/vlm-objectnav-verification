#!/usr/bin/env python3
"""
collect_verif.py - build the VERIFICATION dataset for the VLM (runs inside the ROS container while
Gazebo with the generated world is up).

For every object instance and a few approach azimuths the robot is teleported to a pose
`approach` metres from the object's surface, facing it, and a colour frame is saved
  * condition 'present' : world exactly as in the manifest
  * condition 'moved'   : the target object is moved out of the room (this is what the robot sees when it
                          arrives at a remembered position whose object has been relocated)
The target is restored afterwards. index.json stores camera intrinsics and the exact object poses at
capture time, so ground truth can be computed later (eval_moondream.py).

Usage: python3 collect_verif.py --world office_s1 --out DIR [--per_obj 3] [--approach 1.5]
       python3 collect_verif.py --selftest
"""
import argparse
import json
import math
import os
import sys
import time

CAM_FORWARD = 0.55          # camera offset in front of the robot base (pixel_to_cords.py: 0.55, 0.02, 0.32)
OUTSIDE = (40.0, 40.0)      # where the 'moved' object is parked (far outside the room)


def plan_poses(man, per_obj=3, approach=1.5):
    """Deterministic list of robot poses (x, y, yaw) per target instance."""
    half = man["room_size"] / 2.0
    inst = man["instances"]
    plan = []
    for t in inst:
        base = math.atan2(-t["y"], -t["x"])                   # from the object towards the room centre
        offsets = [0.0, 55.0, -55.0, 110.0, -110.0, 165.0]
        got = 0
        for off in offsets:
            if got >= per_obj:
                break
            az = base + math.radians(off)
            d = t["radius"] + approach
            x, y = t["x"] + d * math.cos(az), t["y"] + d * math.sin(az)
            if abs(x) > half - 0.5 or abs(y) > half - 0.5:
                continue
            if any(o["id"] != t["id"] and math.hypot(x - o["x"], y - o["y"]) < o["radius"] + 0.5 for o in inst):
                continue
            yaw = math.atan2(t["y"] - y, t["x"] - x)
            plan.append({"target": t["id"], "k": got, "x": x, "y": y, "yaw": yaw})
            got += 1
    return plan


def selftest():
    import subprocess
    import tempfile
    d = tempfile.mkdtemp()
    gen = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gen_world.py")
    subprocess.run([sys.executable, gen, "--seed", "1", "--out", d, "--name", "w"], check=True,
                   stdout=subprocess.DEVNULL)
    man = json.load(open(os.path.join(d, "w.json")))
    plan = plan_poses(man)
    byt = {}
    for p in plan:
        byt[p["target"]] = byt.get(p["target"], 0) + 1
    assert all(byt.get(i["id"], 0) >= 1 for i in man["instances"]), byt
    tg = {i["id"]: i for i in man["instances"]}
    for p in plan:
        t = tg[p["target"]]
        dd = math.hypot(p["x"] - t["x"], p["y"] - t["y"])
        assert abs(dd - (t["radius"] + 1.5)) < 1e-6
        # robot looks at the object centre
        assert abs(math.atan2(math.sin(p["yaw"] - math.atan2(t["y"] - p["y"], t["x"] - p["x"])),
                              math.cos(p["yaw"] - math.atan2(t["y"] - p["y"], t["x"] - p["x"])))) < 1e-9
    assert plan == plan_poses(man)
    print(f"SELFTEST_OK poses={len(plan)} frames_expected={2 * len(plan)} objects={len(man['instances'])}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="office_s1")
    ap.add_argument("--worlds_dir", default=os.path.join(os.environ.get("VLMNAV_BASE", "."), "vlm_nav", "worlds"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--per_obj", type=int, default=3)
    ap.add_argument("--approach", type=float, default=1.5)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.out:
        ap.error("--out is required")

    import cv2
    import rospy
    from cv_bridge import CvBridge
    from gazebo_msgs.msg import ModelState, ModelStates
    from gazebo_msgs.srv import SetModelState
    from sensor_msgs.msg import CameraInfo, Image

    man = json.load(open(os.path.join(a.worlds_dir, a.world + ".json")))
    os.makedirs(a.out, exist_ok=True)
    rospy.init_node("collect_verif")
    bridge = CvBridge()
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def put(name, x, y, yaw=0.0):
        ms = ModelState()
        ms.model_name = name
        ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = x, y, 0.0
        ms.pose.orientation.z, ms.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        ms.reference_frame = "world"
        return set_state(ms).success

    def pose_of(name):
        st = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=20)
        i = st.name.index(name)
        return st.pose[i].position.x, st.pose[i].position.y

    info = rospy.wait_for_message("/realsense_camera/color/camera_info", CameraInfo, timeout=60)
    fx, width, height = info.K[0], info.width, info.height
    hfov = 2 * math.degrees(math.atan(width / (2.0 * fx)))
    print(f"camera: {width}x{height} fx={fx:.1f} hfov={hfov:.1f} deg", flush=True)

    inst = {i["id"]: i for i in man["instances"]}
    frames = []
    for p in plan_poses(man, a.per_obj, a.approach):
        tgt = inst[p["target"]]
        for cond in ("present", "moved"):
            if cond == "moved":
                ok = put(tgt["id"], OUTSIDE[0], OUTSIDE[1], 0.0)
                time.sleep(1.0)
                px, py = pose_of(tgt["id"])
                if not ok or math.hypot(px - OUTSIDE[0], py - OUTSIDE[1]) > 1.0:
                    print(f"STATIC_MOVE_FAILED for {tgt['id']}: pose now ({px:.1f},{py:.1f})", flush=True)
                    sys.exit(2)
            put("r1", p["x"], p["y"], p["yaw"])
            time.sleep(1.5)
            img = rospy.wait_for_message("/realsense_camera/color/image_raw", Image, timeout=30)
            bgr = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
            fn = f"{p['target']}_{p['k']}_{cond}.png"
            cv2.imwrite(os.path.join(a.out, fn), bgr)
            objs = []
            for o in man["instances"]:
                x, y = (OUTSIDE if (cond == "moved" and o["id"] == tgt["id"]) else (o["x"], o["y"]))
                objs.append({"id": o["id"], "class": o["class"], "x": x, "y": y, "radius": o["radius"]})
            frames.append({"file": fn, "condition": cond, "target": tgt["id"], "target_class": tgt["class"],
                           "robot": [p["x"], p["y"], p["yaw"]], "objects": objs})
            print(f"  saved {fn}", flush=True)
        put(tgt["id"], tgt["x"], tgt["y"], tgt["yaw"])          # restore
        time.sleep(0.5)

    index = {"world": a.world, "camera": {"hfov_deg": hfov, "width": width, "height": height,
                                          "cam_forward": CAM_FORWARD},
             "approach": a.approach, "frames": frames}
    json.dump(index, open(os.path.join(a.out, "index.json"), "w"), indent=1)
    print(f"COLLECT_DONE frames={len(frames)}", flush=True)


if __name__ == "__main__":
    main()
