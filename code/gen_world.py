#!/usr/bin/env python3
"""
gen_world.py - reproducible Gazebo world generator for the object-navigation benchmark.

Creates:
  <out>/<name>.world   Gazebo 11 world (static room + objects, robot is spawned by the launch file)
  <out>/<name>.json    ground-truth manifest: instances, duplicate classes, planned relocations

Scenarios supported by the manifest:
  - duplicates   : some classes appear twice (--n_dup), separated by >= --dup_min_sep metres
  - relocation   : --n_moves objects get a new collision-free position ("to") that the mission
                   dispatcher applies after exploration via /gazebo/set_model_state

Only Python standard library is used.
Example:
  python3 gen_world.py --seed 1 --n_classes 8 --n_dup 2 --n_moves 2 --out ../worlds --name office_s1
"""
import argparse
import json
import math
import os
import random

# model dir name (in GAZEBO_MODEL_PATH) -> (human-readable class for the VLM, footprint radius in m)
CATALOG = {
    "bookshelf":            ("bookshelf",        0.55),
    "fire_hydrant":         ("fire hydrant",     0.30),
    "first_2015_trash_can": ("trash can",        0.35),
    "person_standing":      ("person",           0.35),
    "postbox":              ("postbox",          0.40),
    "speed_limit_sign":     ("speed limit sign", 0.30),
    "stop_sign":            ("stop sign",        0.30),
    "table":                ("table",            0.70),
    "cabinet":              ("cabinet",          0.45),
    "construction_cone":    ("traffic cone",     0.25),
    "oak_tree":             ("tree",             0.60),
    "construction_barrel":  ("barrel",           0.35),
    "lamp_post":            ("lamp post",        0.30),
    "cardboard_box":        ("cardboard box",    0.35),
}

# Classes the VLM verifier separated reliably in the development worlds 1-5 (see the verification study):
# high recall and no systematic confusion with another class. Barrels and traffic cones are visually
# indistinguishable for the VLM, thin objects (signs, lamp post) are missed, so they are only in the "full" catalogue.
RELIABLE = ["cabinet", "cardboard_box", "fire_hydrant", "person_standing", "postbox", "table",
            "first_2015_trash_can", "oak_tree"]

START_XY = (0.0, 0.0)      # robot spawn point (set in the launch file)
START_CLEAR = 1.6          # keep this radius around the robot free
WALL_CLEAR = 0.6           # margin from the walls (added to object radius)
OBJ_GAP = 0.7              # free gap between footprints of two objects (robot must pass)
WALL_T = 0.2
WALL_H = 2.5


def sample_position(rng, half, radius, placed, avoid_xy=None, min_dist_from=None, tries=2000):
    """Rejection sampling of (x, y) that respects walls, start area and other objects."""
    lim = half - WALL_CLEAR - radius
    for _ in range(tries):
        x = rng.uniform(-lim, lim)
        y = rng.uniform(-lim, lim)
        if math.hypot(x - START_XY[0], y - START_XY[1]) < START_CLEAR + radius:
            continue
        ok = True
        for (px, py, pr) in placed:
            if math.hypot(x - px, y - py) < radius + pr + OBJ_GAP:
                ok = False
                break
        if not ok:
            continue
        if avoid_xy is not None and min_dist_from is not None:
            if any(math.hypot(x - ax, y - ay) < min_dist_from for (ax, ay) in avoid_xy):
                continue
        return x, y
    raise RuntimeError("could not place object; reduce --n_classes/--n_dup or increase --room")


def build_layout(args):
    rng = random.Random(args.seed)
    last = None
    for attempt in range(50):
        try:
            return _build_layout_once(args, rng)
        except RuntimeError as e:
            last = e
    raise SystemExit(f"layout failed after 50 attempts: {last}")


def _build_layout_once(args, rng):
    models = sorted(RELIABLE if args.catalog == "reliable" else CATALOG.keys())
    if args.n_classes > len(models):
        raise SystemExit(f"--n_classes max is {len(models)}")
    chosen = rng.sample(models, args.n_classes)
    if args.n_dup > args.n_classes:
        raise SystemExit("--n_dup cannot exceed --n_classes")
    dup_models = set(rng.sample(chosen, args.n_dup))

    half = args.room / 2.0
    placed = []          # (x, y, r)
    instances = []
    counters = {}
    for model in chosen:
        cls, r = CATALOG[model]
        copies = 2 if model in dup_models else 1
        first_xy = []
        for _ in range(copies):
            x, y = sample_position(rng, half, r, placed,
                                   avoid_xy=first_xy, min_dist_from=args.dup_min_sep)
            first_xy.append((x, y))
            placed.append((x, y, r))
            k = counters.get(model, 0)
            counters[model] = k + 1
            instances.append({
                "id": f"{model}_{k}",
                "model": model,
                "class": cls,
                "x": round(x, 3), "y": round(y, 3),
                "yaw": round(rng.uniform(0, 2 * math.pi), 3),
                "radius": r,
            })

    # relocations: prefer instances whose class is unique (so a query stays unambiguous)
    unique_ids = [i for i in instances if counters[i["model"]] == 1]
    pool = unique_ids if len(unique_ids) >= args.n_moves else instances
    movers = rng.sample(pool, min(args.n_moves, len(pool)))
    relocations = []
    cur = [(i["x"], i["y"], i["radius"]) for i in instances]
    for m in movers:
        others = [(i["x"], i["y"], i["radius"]) for i in instances if i["id"] != m["id"]]
        # also avoid earlier relocation targets
        others += [(rl["to"][0], rl["to"][1], CATALOG[rl["model"]][1]) for rl in relocations]
        nx, ny = sample_position(rng, half, m["radius"], others,
                                 avoid_xy=[(m["x"], m["y"])], min_dist_from=args.min_move)
        relocations.append({"id": m["id"], "model": m["model"], "class": m["class"],
                            "from": [m["x"], m["y"]], "to": [round(nx, 3), round(ny, 3)]})
    return instances, sorted(dup_models), relocations


def wall_xml(name, x, y, sx, sy):
    return f"""    <model name='{name}'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {WALL_H/2:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'><geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size></box></geometry></collision>
        <visual name='visual'>
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size></box></geometry>
          <material><ambient>0.85 0.85 0.8 1</ambient><diffuse>0.85 0.85 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def world_xml(instances, room):
    half = room / 2.0
    L = room + WALL_T
    out = []
    out.append("<?xml version='1.0'?>\n<sdf version='1.6'>\n  <world name='default'>\n")
    out.append("""    <light name='sun' type='directional'>
      <cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface></collision>
        <visual name='visual'><cast_shadows>0</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.55 0.5 0.45 1</diffuse></material></visual>
      </link>
    </model>
""")
    out.append(wall_xml("wall_north", 0, half + WALL_T / 2, L, WALL_T))
    out.append(wall_xml("wall_south", 0, -half - WALL_T / 2, L, WALL_T))
    out.append(wall_xml("wall_east", half + WALL_T / 2, 0, WALL_T, L))
    out.append(wall_xml("wall_west", -half - WALL_T / 2, 0, WALL_T, L))
    for i in instances:
        out.append(f"""    <include>
      <uri>model://{i['model']}</uri>
      <name>{i['id']}</name>
      <static>true</static>
      <pose>{i['x']} {i['y']} 0 0 0 {i['yaw']}</pose>
    </include>
""")
    out.append("  </world>\n</sdf>\n")
    return "".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--catalog", choices=["full", "reliable"], default="full",
                    help="full: 14 classes; reliable: the 8 classes the VLM verifier separates (development worlds)")
    ap.add_argument("--room", type=float, default=9.0, help="room side length in metres (gmapping area is +-5 m)")
    ap.add_argument("--n_classes", type=int, default=8)
    ap.add_argument("--n_dup", type=int, default=2, help="how many classes get two instances")
    ap.add_argument("--n_moves", type=int, default=2, help="how many objects get a planned relocation")
    ap.add_argument("--dup_min_sep", type=float, default=3.0, help="min distance between two same-class instances")
    ap.add_argument("--min_move", type=float, default=3.0, help="min relocation distance")
    ap.add_argument("--out", default="worlds")
    ap.add_argument("--name", default=None)
    args = ap.parse_args()

    name = args.name or f"office_s{args.seed}"
    instances, dup_models, relocations = build_layout(args)
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, name + ".world"), "w") as f:
        f.write(world_xml(instances, args.room))
    manifest = {
        "name": name, "seed": args.seed, "room_size": args.room, "start": list(START_XY),
        "params": {k: v for k, v in vars(args).items() if k not in ("out", "name")}, "instances": instances,
        "duplicate_models": dup_models, "relocations": relocations,
    }
    with open(os.path.join(args.out, name + ".json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"[ok] {name}: {len(instances)} instances, {len(set(i['model'] for i in instances))} classes, "
          f"{len(dup_models)} duplicated classes, {len(relocations)} relocations")


if __name__ == "__main__":
    main()
