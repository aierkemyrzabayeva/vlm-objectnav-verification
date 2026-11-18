#!/usr/bin/env python3
"""
run_experiment.py - episodes for the paper's main comparison (one Gazebo world per job).

Scenarios (query = an object CLASS, as in ObjectNav)
  S1  duplicates : a class with two instances in the room, nothing moves
  S2  relocation : one object has been moved after the memory was built (memory still holds the old position)
Conditions
  B0   baseline-like  : one memory entry per class (emulates the baseline's merge), closest entry, NO verification
  B1   instance memory: one entry per instance, closest one, NO verification
  B1r  like B1 but with the same retry budget for reaching the goal as the full method (separates the effect of
       verification from the effect of extra attempts against an unreliable controller)
  B2   verify-and-repair on the single-entry memory
  FULL instance memory + verify-and-repair (+ ranking, fallback search)
The memory is built from the world manifest with Gaussian localisation noise (sigma = --sigma, to be calibrated
from the real mapping pipeline); everything else (controller, VLM, depth, collisions) is the real simulation.

Scoring (ground truth from the manifest, positions after the move)
  decision_ok : the mission's final goal lies within radius + 0.6 m of a true instance of the class
  near        : the robot ends within radius + 1.0 m of a true instance
  success     : controller reached its final goal AND near
  SR (primary, computed in analyze_exp.py) : near AND the mission declared a goal (not NOT_FOUND)
Usage (through exp.sh):  python3 run_experiment.py --world office_s1 --scenarios S1 S2 --episodes 5 --out results/exp_s1.csv
Offline test:            python3 run_experiment.py --mock --world office_s1 --episodes 3 --out /tmp/mock.csv
"""
import argparse
import csv
import json
import math
import os
import random
import sys
import time

BASE = os.environ.get("VLMNAV_BASE", os.getcwd())
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dataclasses import replace
from instance_memory import InstanceMemory, dist                      # noqa: E402
from verify_repair import Config, run_baseline, run_mission           # noqa: E402

CONDITIONS = {"B0": ("single", "baseline"), "B1": ("instance", "baseline"), "B1r": ("instance", "baseline"),
              "B2": ("single", "mission"), "FULL": ("instance", "mission")}


def build_memory(man, kind, rng, sigma=0.2, obs=3):
    mem = InstanceMemory()
    by_cls = {}
    for i in sorted(man["instances"], key=lambda i: i["id"]):
        by_cls.setdefault(i["class"], []).append(i)
    for cls, lst in by_cls.items():
        for o in ([lst[0]] if kind == "single" else lst):
            for k in range(obs):
                dx = max(-0.5, min(0.5, rng.gauss(0, sigma)))
                dy = max(-0.5, min(0.5, rng.gauss(0, sigma)))
                mem.observe(cls, o["x"] + dx, o["y"] + dy, conf=0.7, t=float(k))
    return mem


def make_episodes(man, scenario, n, seed):
    from test_controller import sample_episodes
    rng = random.Random(seed)
    inst = {i["id"]: i for i in man["instances"]}
    counts = {}
    for i in man["instances"]:
        counts[i["class"]] = counts.get(i["class"], 0) + 1
    cand = []
    if scenario == "S1":
        for c, k in sorted(counts.items()):
            if k >= 2:
                tgt = sorted(i["id"] for i in man["instances"] if i["class"] == c)[0]
                cand.append((tgt, c, []))
    elif scenario == "S2":
        for rl in man.get("relocations", []):
            if counts[rl["class"]] == 1:                           # keep the query unambiguous
                cand.append((rl["id"], rl["class"], [(rl["id"], rl["to"][0], rl["to"][1], 0.0)]))
    if not cand:
        return []
    per = max(1, int(math.ceil(n / float(len(cand)))))
    eps = []
    starts = sample_episodes(man, per, seed)
    for tgt, cls, moves in cand:
        for st in [s for s in starts if s["obj"] == tgt][:per]:
            truth = []
            for o in man["instances"]:
                o = dict(o)
                for (name, x, y, _yaw) in moves:
                    if name == o["id"]:
                        o["x"], o["y"] = x, y
                truth.append(o)
            eps.append({"scenario": scenario, "target": tgt, "cls": cls, "start": (st["sx"], st["sy"], st["yaw"]),
                        "moves": moves, "truth": truth})
    rng.shuffle(eps)
    return eps[:n]


def score(res, sim, truth, cls):
    rx, ry = sim.robot_xy()
    ti = [o for o in truth if o["class"] == cls]
    near = any(dist(rx, ry, o["x"], o["y"]) <= o["radius"] + 1.0 for o in ti)
    dec = res.goal is not None and any(dist(res.goal[0], res.goal[1], o["x"], o["y"]) <= o["radius"] + 0.6 for o in ti)
    return int(res.reached and near), int(dec), int(near)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="office_s1")
    ap.add_argument("--worlds_dir", default=f"{BASE}/vlm_nav/worlds")
    ap.add_argument("--scenarios", nargs="+", default=["S1", "S2"])
    ap.add_argument("--conditions", nargs="+", default=["B0", "B1", "B1r", "B2", "FULL"])
    ap.add_argument("--baseline_retries", type=int, default=3, help="extra attempts of the B1r baseline")
    ap.add_argument("--episodes", type=int, default=5, help="episodes per scenario (each is run under all conditions)")
    ap.add_argument("--policy", default=f"{BASE}/vlm_nav/TD3/td3_gazebo_custom_policy.zip")
    ap.add_argument("--vlm_url", default="http://127.0.0.1:8123")
    ap.add_argument("--prompts", default="A")
    ap.add_argument("--rule", default="and", choices=["and", "or"])
    ap.add_argument("--n_views", type=int, default=1, help="approach azimuths per verification")
    ap.add_argument("--view_rule", default="any", choices=["any", "majority", "all"])
    ap.add_argument("--extra_views", type=int, default=0, help="deprecated: n_views = extra_views + 1")
    ap.add_argument("--recover", type=int, default=2, help="recovery manoeuvres per leg after stuck/collision (same for all conditions)")
    ap.add_argument("--peripheral", type=int, default=1, help="full-frame look when the crop prompt says no")
    ap.add_argument("--sweep", type=float, default=0.0, help="radians: extra looks at +-sweep before a rejection (0 = off)")
    ap.add_argument("--search_verify_dist", type=float, default=2.5, help="standoff for candidates found by the search (m)")
    ap.add_argument("--sigma", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames_dir", default=None, help="save the frames of every verification (debugging, figures)")
    ap.add_argument("--mock", action="store_true")
    a = ap.parse_args()

    man = json.load(open(os.path.join(a.worlds_dir, a.world + ".json")))
    if a.mock:
        from test_logic import MockSim
        sim = MockSim(room=man["room_size"])
    else:
        import real_env_ours as renv
        renv.LAUNCHFILE = os.environ.get("OURS_LAUNCH", f"{BASE}/vlm_nav/ours/multi_robot_scenario.launch")
        from policy_io import load_policy
        from sim_env import SimEnv
        genv = renv.GazeboEnv()
        time.sleep(5)
        sim = SimEnv(renv, genv, load_policy(a.policy), a.vlm_url, a.prompts, a.rule, recover=a.recover,
                     peripheral=bool(a.peripheral), sweep=a.sweep)

    cfg = Config(n_views=max(a.n_views, a.extra_views + 1), view_rule=a.view_rule, search_verify_dist=a.search_verify_dist)
    fields = ["world", "scenario", "episode", "cond", "cls", "target", "sx", "sy", "success", "decision_ok", "near",
              "reached", "reason", "n_attempts", "outcomes", "vlm_calls", "vlm_s", "path", "sim_s", "wall_s",
              "legs", "recoveries", "scene_dev", "vlm_log", "prompts", "rule", "n_views", "view_rule", "sweep", "search_dist", "sigma"]
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    rng = random.Random(a.seed)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for sc in a.scenarios:
            eps = make_episodes(man, sc, a.episodes, a.seed)
            print(f"== scenario {sc}: {len(eps)} episodes x {len(a.conditions)} conditions", flush=True)
            for ei, ep in enumerate(eps):
                conds = list(a.conditions)
                rng.shuffle(conds)
                for cond in conds:
                    kind, runner = CONDITIONS[cond]
                    mem = build_memory(man, kind, random.Random(a.seed * 1000 + ei), a.sigma)
                    sim.start_episode(ep["start"][0], ep["start"][1], ep["start"][2], ep["moves"], ep["truth"],
                                      man["room_size"])
                    if a.frames_dir and hasattr(sim, "debug_dir"):
                        sim.debug_dir, sim.debug_prefix = a.frames_dir, f"{a.world}_{sc}_ep{ei}_{cond}"
                    t0, ts0 = time.time(), sim.time()
                    cfg_c = replace(cfg, baseline_retries=a.baseline_retries) if cond == "B1r" else cfg
                    res = (run_baseline if runner == "baseline" else run_mission)(mem, ep["cls"], sim, cfg_c)
                    succ, dec, near = score(res, sim, ep["truth"], ep["cls"])
                    row = {"world": a.world, "scenario": sc, "episode": ei, "cond": cond, "cls": ep["cls"],
                           "target": ep["target"], "sx": round(ep["start"][0], 2), "sy": round(ep["start"][1], 2),
                           "success": succ, "decision_ok": dec, "near": near, "reached": int(res.reached),
                           "reason": res.reason, "n_attempts": len(res.attempts),
                           "outcomes": ">".join(x.outcome for x in res.attempts),
                           "vlm_calls": getattr(sim, "vlm_calls", 0), "vlm_s": round(getattr(sim, "vlm_time", 0.0), 2),
                           "path": round(getattr(sim, "path", 0.0), 2), "sim_s": round(sim.time() - ts0, 1),
                           "wall_s": round(time.time() - t0, 1),
                           "legs": json.dumps(getattr(sim, "legs", [])), "recoveries": getattr(sim, "recoveries", 0), "scene_dev": round(getattr(sim, "scene_dev", 0.0), 3),
                           "vlm_log": json.dumps(getattr(sim, "vlm_log", [])), "prompts": a.prompts, "rule": a.rule,
                           "n_views": cfg.n_views, "view_rule": cfg.view_rule, "sweep": a.sweep, "search_dist": a.search_verify_dist, "sigma": a.sigma}
                    w.writerow(row)
                    f.flush()
                    print(f"  {sc} ep{ei} {cond:<4s} {ep['cls']:<16s} -> success={succ} decision_ok={dec} "
                          f"reason={res.reason:<15s} vlm={row['vlm_calls']} path={row['path']}m", flush=True)
    print("EXP_DONE", flush=True)


if __name__ == "__main__":
    main()
