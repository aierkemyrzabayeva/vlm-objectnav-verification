#!/usr/bin/env python3
"""
test_controller.py - "oracle" test of the authors' TD3 policy in OUR generated world.

The goal given to the controller is the ground-truth position of an object (taken from the world
manifest), so memory / VLM errors are excluded. This isolates how well the controller alone works
in our world. Everything else we measure later is relative to this upper bound.

Goal point = object centre shifted towards the robot start by the object's footprint radius
(this imitates the baseline, whose camera pipeline returns the closest surface point, and keeps the
goal reachable for large objects such as tables or trees).
  success      = env reports target reached (within GOAL_REACHED_DIST = 0.8 m of the goal point)
  near_object  = final distance to the object centre <= radius + 1.0 m   (secondary, like DualMap's 1 m)
  SPL          = success * shortest / max(path, shortest), shortest = straight-line start->goal
                 (upper bound of the true SPL because obstacles are ignored)

Run through ctrl_test.sh (it starts the ROS container and a virtual display).
Usage: python3 test_controller.py --world office_s1 --out results/ctrl_office_s1.csv [--starts 2]
       python3 test_controller.py --selftest        (pure-Python check of the sampling logic)
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

BASE = os.environ.get("VLMNAV_BASE", os.getcwd())
ROBOT_CLEAR = 0.9      # free distance (to object footprint edge) required at the start pose
MIN_TRIP = 2.0         # start is at least this far from the target's footprint edge
WALL_MARGIN = 0.8


def wilson(k, n, z=1.96):
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def sample_episodes(man, starts, seed):
    """For every object instance: `starts` random free start poses with a goal point on its footprint."""
    rng = np.random.RandomState(seed)
    half = man["room_size"] / 2.0
    inst = sorted(man["instances"], key=lambda i: i["id"])
    eps = []
    for tgt in inst:
        for k in range(starts):
            for _ in range(5000):
                x = rng.uniform(-half + WALL_MARGIN, half - WALL_MARGIN)
                y = rng.uniform(-half + WALL_MARGIN, half - WALL_MARGIN)
                if any(math.hypot(x - o["x"], y - o["y"]) < o["radius"] + ROBOT_CLEAR for o in inst):
                    continue
                if math.hypot(x - tgt["x"], y - tgt["y"]) < tgt["radius"] + MIN_TRIP:
                    continue
                break
            else:
                raise RuntimeError("could not sample a start pose")
            yaw = rng.uniform(-math.pi, math.pi)
            d = math.hypot(x - tgt["x"], y - tgt["y"])
            gx = tgt["x"] + (x - tgt["x"]) / d * tgt["radius"]
            gy = tgt["y"] + (y - tgt["y"]) / d * tgt["radius"]
            eps.append({"obj": tgt["id"], "cls": tgt["class"], "radius": tgt["radius"],
                        "cx": tgt["x"], "cy": tgt["y"], "sx": x, "sy": y, "yaw": yaw,
                        "gx": gx, "gy": gy, "k": k})
    return eps


def selftest():
    import subprocess
    import tempfile
    d = tempfile.mkdtemp()
    gen = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gen_world.py")
    subprocess.run([sys.executable, gen, "--seed", "3", "--out", d, "--name", "w"], check=True,
                   stdout=subprocess.DEVNULL)
    man = json.load(open(os.path.join(d, "w.json")))
    eps = sample_episodes(man, 3, 0)
    assert len(eps) == 3 * len(man["instances"])
    for e in eps:
        for o in man["instances"]:
            assert math.hypot(e["sx"] - o["x"], e["sy"] - o["y"]) >= o["radius"] + ROBOT_CLEAR - 1e-9
        assert math.hypot(e["gx"] - e["cx"], e["gy"] - e["cy"]) - e["radius"] < 1e-9
    assert eps == sample_episodes(man, 3, 0), "sampling must be deterministic"
    lo, hi = wilson(5, 10)
    assert 0.23 < lo < 0.25 and 0.75 < hi < 0.77, (lo, hi)
    print(f"SELFTEST_OK episodes={len(eps)} wilson(5/10)=({lo:.3f},{hi:.3f})")


def run_episode(env, model, ep, max_wall_steps=450):
    env.forced_start = (ep["sx"], ep["sy"], ep["yaw"])
    env.goal_x, env.goal_y = ep["gx"], ep["gy"]
    o = env.reset()
    obs = {"image": o[0], "scalars": o[1]}
    last = (env.odom_x, env.odom_y)
    path, steps, done, reached = 0.0, 0, False, False
    t0 = time.time()
    while not done and steps < max_wall_steps:
        action, _ = model.predict(obs, deterministic=True)
        (img, sc), _r, done, target = env.step(action)
        obs = {"image": img, "scalars": sc}
        cur = (env.odom_x, env.odom_y)
        path += math.hypot(cur[0] - last[0], cur[1] - last[1])
        last = cur
        steps += 1
        reached = bool(target)
    if reached:
        reason = "goal"
    elif steps >= getattr(env, "max_steps", 400):
        reason = "max_steps"
    elif getattr(env, "stuck_counter", 0) >= 100:
        reason = "stuck"
    else:
        reason = "collision_or_other"
    shortest = math.hypot(ep["sx"] - ep["gx"], ep["sy"] - ep["gy"])
    final_goal = math.hypot(last[0] - ep["gx"], last[1] - ep["gy"])
    final_center = math.hypot(last[0] - ep["cx"], last[1] - ep["cy"])
    return {"success": int(reached), "steps": steps, "path": round(path, 3), "shortest": round(shortest, 3),
            "spl": round((1.0 if reached else 0.0) * shortest / max(path, shortest, 1e-6), 4),
            "final_goal": round(final_goal, 3), "final_center": round(final_center, 3),
            "near_object": int(final_center <= ep["radius"] + 1.0), "reason": reason,
            "wall_s": round(time.time() - t0, 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="office_s1")
    ap.add_argument("--worlds_dir", default=f"{BASE}/vlm_nav/worlds")
    ap.add_argument("--policies", nargs="+", default=[f"{BASE}/vlm_nav/TD3/td3_gazebo_custom_policy.zip"])
    ap.add_argument("--starts", type=int, default=2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    man = json.load(open(os.path.join(a.worlds_dir, a.world + ".json")))
    eps = sample_episodes(man, a.starts, a.seed)
    print(f"world={a.world} instances={len(man['instances'])} episodes per policy={len(eps)}", flush=True)

    sys.path.insert(0, os.path.join(BASE, "vlm_nav", "ours"))
    import real_env_ours as renv
    renv.LAUNCHFILE = os.environ.get("OURS_LAUNCH", f"{BASE}/vlm_nav/ours/multi_robot_scenario.launch")
    from policy_io import load_policy

    env = renv.GazeboEnv()
    time.sleep(5)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    fields = ["world", "policy", "obj", "cls", "k", "sx", "sy", "gx", "gy", "success", "steps", "path",
              "shortest", "spl", "final_goal", "final_center", "near_object", "reason", "wall_s"]
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for pol in a.policies:
            name = os.path.basename(pol)
            model = load_policy(pol)
            print(f"== policy {name}", flush=True)
            rows = []
            for ep in eps:
                res = run_episode(env, model, ep)
                row = {"world": a.world, "policy": name, "obj": ep["obj"], "cls": ep["cls"], "k": ep["k"],
                       "sx": round(ep["sx"], 3), "sy": round(ep["sy"], 3),
                       "gx": round(ep["gx"], 3), "gy": round(ep["gy"], 3), **res}
                rows.append(row)
                w.writerow(row)
                f.flush()
                print(f"  {ep['obj']:<22} start=({ep['sx']:.1f},{ep['sy']:.1f}) -> {res['reason']:<18} "
                      f"steps={res['steps']:3d} path={res['path']:.1f}m spl={res['spl']:.2f}", flush=True)
            n = len(rows)
            k = sum(r["success"] for r in rows)
            lo, hi = wilson(k, n)
            reasons = {}
            for r in rows:
                reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
            print(f"SUMMARY {name}: SR={k}/{n}={k / n:.2f} (95% CI {lo:.2f}-{hi:.2f}) "
                  f"SPL={np.mean([r['spl'] for r in rows]):.3f} "
                  f"near_object={sum(r['near_object'] for r in rows)}/{n} "
                  f"mean steps={np.mean([r['steps'] for r in rows]):.0f} reasons={reasons}", flush=True)
    print("CTRL_DONE", flush=True)
    os.system("pkill -f gzserver; pkill -f roslaunch; pkill -f roscore")


if __name__ == "__main__":
    main()
