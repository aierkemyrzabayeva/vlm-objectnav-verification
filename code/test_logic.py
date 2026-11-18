#!/usr/bin/env python3
"""
test_logic.py - offline checks for instance_memory.py and verify_repair.py.

IMPORTANT: the toy simulator below is a LOGIC test (teleport-style motion, perfect VLM, no
collisions). Its numbers are NOT experimental results and must never appear in the paper.

Usage:  python3 test_logic.py [path/to/gen_world.py]
"""
import json
import math
import os
import random
import subprocess
import sys
import tempfile

from instance_memory import InstanceMemory, dist
from verify_repair import Config, run_baseline, run_mission

HERE = os.path.dirname(os.path.abspath(__file__))


# ------------------------------------------------------------------ unit tests
def test_association():
    m = InstanceMemory(gate=0.75)
    a = m.observe("chair", 1.0, 1.0, t=1)
    b = m.observe("chair", 1.3, 1.1, t=2)          # same object
    c = m.observe("chair", 4.0, 1.0, t=3)          # second chair
    d = m.observe("table", 1.0, 1.0, t=4)          # other class, same place
    assert a.id == b.id and a.n_obs == 2
    assert c.id != a.id and d.id != a.id
    assert len(m.items) == 3


def test_tentative_ranks_below_confirmed():
    m = InstanceMemory()
    for k in range(5):
        m.observe("cone", 3.0, 0.0, conf=0.8, t=k)    # confirmed, a bit farther
    m.observe("cone", 1.5, 0.0, conf=0.8, t=5)         # single sighting (maybe hallucination), closer
    ranked = m.rank("cone", (0.0, 0.0), 6)
    assert ranked[0][1].x > 2.5, "confirmed instance should win over a single closer sighting"


def test_missing_and_resurrection():
    m = InstanceMemory()
    i = m.observe("box", 2.0, 2.0, t=0)
    m.observe("box", 2.0, 2.0, t=1)
    m.mark_missing(i.id, 5)
    assert m.rank("box", (0, 0), 6) == []
    j = m.observe("box", 2.1, 2.0, t=10)               # seen again at the old place
    assert j.id == i.id and j.status == "active" and j.n_failed == 0


def test_jsonl_roundtrip():
    m = InstanceMemory()
    m.observe("tree", 1.0, 2.0, conf=0.7, t=3)
    m.mark_verified(next(iter(m.items)), 4)
    p = os.path.join(tempfile.mkdtemp(), "mem.jsonl")
    m.save_jsonl(p)
    m2 = InstanceMemory.load_jsonl(p)
    a, b = list(m.items.values())[0], list(m2.items.values())[0]
    assert (a.cls, round(a.x, 6), a.n_verified) == (b.cls, round(b.x, 6), b.n_verified)
    rec = json.loads(open(p).readline())
    for key in ("object_class", "world_x", "world_y", "observation_count", "confidence", "timestamp"):
        assert key in rec, f"baseline field {key} missing"


# ------------------------------------------------------------------ toy simulator
class ToyEnv:
    def __init__(self, truth, start=(0.0, 0.0), view_r=3.0, room=9.0):
        self.truth = [dict(o) for o in truth]
        self.pos = list(start)
        self.path = 0.0
        self.vlm_calls = 0
        self._t = 0.0
        self.view_r = view_r
        self.room = room

    def robot_xy(self):
        return tuple(self.pos)

    def time(self):
        return self._t

    def go_to(self, x, y, stop_within):
        d = dist(self.pos[0], self.pos[1], x, y)
        if d > stop_within:
            k = (d - stop_within) / d
            nx, ny = self.pos[0] + (x - self.pos[0]) * k, self.pos[1] + (y - self.pos[1]) * k
            self.path += dist(self.pos[0], self.pos[1], nx, ny)
            self._t += dist(self.pos[0], self.pos[1], nx, ny) / 0.5
            self.pos = [nx, ny]
        return True

    def verify(self, cls, face_xy=None, scan=False):
        self.vlm_calls += 1
        self._t += 0.5
        return [(o["x"], o["y"]) for o in self.truth
                if o["class"] == cls and dist(self.pos[0], self.pos[1], o["x"], o["y"]) <= self.view_r]

    def search_waypoints(self, cls):
        h = self.room / 2 - 1.5
        pts = [(x, y) for x in (-h, 0, h) for y in (-h, 0, h)]
        pts.sort(key=lambda p: dist(self.pos[0], self.pos[1], p[0], p[1]))
        return pts

    def at_true_object(self, cls, tol=1.1):
        return any(o["class"] == cls and dist(self.pos[0], self.pos[1], o["x"], o["y"]) <= tol
                   for o in self.truth)

    def nearest_true_dist(self, cls):
        return min(dist(0, 0, o["x"], o["y"]) for o in self.truth if o["class"] == cls)


def test_extra_view_rescues_false_rejection():
    """A flaky verifier says 'no' on the first view although the object is there; a second view fixes it."""
    from instance_memory import InstanceMemory as IM

    class Flaky(ToyEnv):
        def verify(self, cls, face_xy=None, scan=False):
            self.vlm_calls += 1
            self._n = getattr(self, "_n", 0) + 1
            if self._n == 1:
                return []                                    # first look: false rejection
            return super().verify(cls, face_xy, scan)

    truth = [{"id": "t0", "class": "tree", "x": 3.0, "y": 3.0}]
    m = IM(); m.observe("tree", 3.0, 3.0, t=0); m.observe("tree", 3.0, 3.0, t=1)
    r1 = run_mission(m, "tree", Flaky(truth), Config(n_views=1, allow_search=False))
    m = IM(); m.observe("tree", 3.0, 3.0, t=0); m.observe("tree", 3.0, 3.0, t=1)
    r2 = run_mission(m, "tree", Flaky(truth), Config(n_views=2, view_rule="any", allow_search=False))
    assert r1.reason == "NOT_FOUND" and r2.reason == "verified", (r1.reason, r2.reason)


class MockSim(ToyEnv):
    """Offline stand-in for sim_env.SimEnv (used by run_experiment.py --mock): a noisy yes/no verifier."""

    def __init__(self, room=9.0, p_tp=0.85, p_fp=0.05, seed=0):
        super().__init__([], room=room)
        import random
        self.rng = random.Random(seed)
        self.p_tp, self.p_fp = p_tp, p_fp
        self.vlm_time, self.legs = 0.0, []

    def start_episode(self, sx, sy, yaw, moves, truth, room):
        self.truth = [dict(o) for o in truth]
        self.pos = [sx, sy]
        self.path, self.vlm_calls, self._t, self.legs = 0.0, 0, 0.0, []

    def verify(self, cls, face_xy=None, scan=False):
        self.vlm_calls += 1
        self._t += 0.5
        if face_xy is not None:
            present = any(o["class"] == cls and dist(o["x"], o["y"], face_xy[0], face_xy[1]) <= 1.0 for o in self.truth)
            yes = self.rng.random() < (self.p_tp if present else self.p_fp)
            return [face_xy] if yes else []
        seen = [(o["x"], o["y"]) for o in self.truth if o["class"] == cls
                and dist(self.pos[0], self.pos[1], o["x"], o["y"]) <= self.view_r]
        return [s for s in seen if self.rng.random() < self.p_tp]


def test_view_rules():
    """majority / all / any decided from a scripted sequence of answers (one answer per view)."""
    from verify_repair import _decided
    assert _decided([1], 3, "majority") is None and _decided([1, 1], 3, "majority") is True
    assert _decided([0, 0], 3, "majority") is False and _decided([1, 0], 3, "majority") is None
    assert _decided([1, 0, 1], 3, "majority") is True
    assert _decided([1], 2, "all") is None and _decided([1, 1], 2, "all") is True and _decided([1, 0], 2, "all") is False
    assert _decided([0], 3, "any") is None and _decided([0, 1], 3, "any") is True and _decided([0, 0, 0], 3, "any") is False

    from instance_memory import InstanceMemory as IM

    class Scripted(ToyEnv):
        def __init__(self, truth, answers):
            super().__init__(truth)
            self.answers = list(answers)

        def verify(self, cls, face_xy=None, scan=False):
            self.vlm_calls += 1
            if face_xy is None:
                return []
            return [face_xy] if self.answers.pop(0) else []

    truth = [{"id": "t0", "class": "tree", "x": 3.0, "y": 3.0}]

    def run(answers, rule, n):
        m = IM(); m.observe("tree", 3.0, 3.0, t=0); m.observe("tree", 3.0, 3.0, t=1)
        env = Scripted(truth, answers)
        r = run_mission(m, "tree", env, Config(n_views=n, view_rule=rule, allow_search=False))
        return r.reason, env.vlm_calls
    assert run([0, 1, 1], "majority", 3) == ("verified", 3)          # needs the 3rd look
    assert run([1, 1, 0], "majority", 3) == ("verified", 2)          # settled after two looks
    assert run([0, 0, 1], "majority", 3) == ("NOT_FOUND", 2)         # two noes settle it
    assert run([1, 1], "all", 2) == ("verified", 2)
    assert run([1, 0], "all", 2) == ("NOT_FOUND", 2)
    assert run([0, 0, 1], "any", 3) == ("verified", 3)


def test_unreachable_view_is_not_enough():
    """With 'majority' a single 'yes' is not accepted when the other views cannot be reached."""
    from instance_memory import InstanceMemory as IM

    class OneStop(ToyEnv):
        def __init__(self, truth):
            super().__init__(truth)
            self.moves = 0

        def go_to(self, x, y, stop_within):
            self.moves += 1
            if self.moves == 1:
                return super().go_to(x, y, stop_within)  # only the first approach works
            return False

    truth = [{"id": "t0", "class": "tree", "x": 3.0, "y": 3.0}]
    m = IM(); m.observe("tree", 3.0, 3.0, t=0); m.observe("tree", 3.0, 3.0, t=1)
    r = run_mission(m, "tree", OneStop(truth), Config(n_views=3, view_rule="majority", allow_search=False))
    assert r.reason == "NOT_FOUND", r.reason
    m = IM(); m.observe("tree", 3.0, 3.0, t=0); m.observe("tree", 3.0, 3.0, t=1)
    r = run_mission(m, "tree", OneStop(truth), Config(n_views=3, view_rule="any", allow_search=False))
    assert r.reason == "verified", r.reason


def test_unreachable_instance_gets_a_second_chance():
    """The first approach fails (controller), the search re-finds the object at the same place: it must be tried again."""
    from instance_memory import InstanceMemory as IM

    class FirstFails(ToyEnv):
        def __init__(self, truth):
            super().__init__(truth)
            self.calls = 0

        def go_to(self, x, y, stop_within):
            self.calls += 1
            return False if self.calls == 1 else super().go_to(x, y, stop_within)

        def search_waypoints(self, cls):
            return [(1.0, 1.0)]

        def verify(self, cls, face_xy=None, scan=False):
            self.vlm_calls += 1
            if scan:
                return [(3.0, 3.0)]                        # the scan sees the object where the memory has it
            return [face_xy] if face_xy is not None else []

    truth = [{"id": "t0", "class": "tree", "x": 3.0, "y": 3.0}]
    m = IM(); m.observe("tree", 3.0, 3.0, t=0); m.observe("tree", 3.0, 3.0, t=1)
    r = run_mission(m, "tree", FirstFails(truth), Config(n_views=1, allow_search=True))
    assert r.reason == "found_by_search" and r.goal == (3.0, 3.0), (r.reason, r.goal, [a.outcome for a in r.attempts])
    assert [a.outcome for a in r.attempts] == ["unreachable", "verified"], [a.outcome for a in r.attempts]

    class AlwaysRejects(FirstFails):
        def go_to(self, x, y, stop_within):
            return ToyEnv.go_to(self, x, y, stop_within)

        def verify(self, cls, face_xy=None, scan=False):
            self.vlm_calls += 1
            return [(3.0, 3.0)] if scan else []              # the look at the point says no, the scan keeps saying yes

    m = IM(); m.observe("tree", 3.0, 3.0, t=0); m.observe("tree", 3.0, 3.0, t=1)
    r = run_mission(m, "tree", AlwaysRejects(truth), Config(n_views=1, allow_search=True))
    assert r.reason == "NOT_FOUND", r.reason                  # a REJECTED instance is not verified again


def test_baseline_retries():
    from instance_memory import InstanceMemory as IM

    class FlakyGo(ToyEnv):
        def __init__(self, truth):
            super().__init__(truth)
            self.calls = 0

        def go_to(self, x, y, stop_within):
            self.calls += 1
            return False if self.calls <= 2 else super().go_to(x, y, stop_within)

    truth = [{"id": "t0", "class": "tree", "x": 3.0, "y": 3.0}]

    def run(retries):
        m = IM(); m.observe("tree", 3.0, 3.0, t=0)
        env = FlakyGo(truth)
        return run_baseline(m, "tree", env, Config(baseline_retries=retries)).reached, env.calls
    assert run(0) == (False, 1) and run(2) == (True, 3) and run(5) == (True, 3)


def test_search_hits_use_the_larger_standoff():
    """Memory candidates are verified from verify_dist, candidates found by the search from search_verify_dist."""
    from instance_memory import InstanceMemory as IM

    class Rec(ToyEnv):
        def __init__(self, truth):
            super().__init__(truth)
            self.stops = []

        def go_to(self, x, y, stop_within):
            self.stops.append(stop_within)
            return super().go_to(x, y, stop_within)

        def search_waypoints(self, cls):
            return [(0.0, 0.0)]

        def verify(self, cls, face_xy=None, scan=False):
            self.vlm_calls += 1
            if scan:
                return [(-3.0, 2.0)]                     # the moved object is found elsewhere
            if face_xy is not None and abs(face_xy[0] - (-3.0)) < 0.5:
                return [face_xy]                          # confirmed there
            return []                                     # the remembered place is empty

    truth = [{"id": "t0", "class": "tree", "x": -3.0, "y": 2.0}]
    m = IM(); m.observe("tree", 3.0, 3.0, t=0); m.observe("tree", 3.0, 3.0, t=1)      # stale memory
    env = Rec(truth)
    r = run_mission(m, "tree", env, Config(n_views=1, verify_dist=1.5, search_verify_dist=2.5))
    assert r.reason == "found_by_search", r.reason
    assert 1.5 in env.stops and 2.5 in env.stops and env.stops.index(1.5) < env.stops.index(2.5), env.stops


def build_memory(instances, rng, noise=0.1, obs=4, baseline_single_entry=False):
    """Perfectly explored world with small localisation noise. The baseline stores ONE entry per
    class (position of the more often seen instance), ours stores every instance."""
    m = InstanceMemory()
    by_cls = {}
    for i in instances:
        by_cls.setdefault(i["class"], []).append(i)
    for cls, lst in by_cls.items():
        use = [lst[0]] if baseline_single_entry else lst
        for o in use:
            for k in range(obs):
                m.observe(cls, o["x"] + rng.gauss(0, noise), o["y"] + rng.gauss(0, noise), conf=0.8, t=k)
    return m


def toy_experiment(gen_script, seeds=range(1, 21)):
    cfg = Config()
    d = tempfile.mkdtemp()
    res = {"reloc": {"base": [], "ours": [], "ours_calls": [], "ours_path": [], "base_path": []},
           "dup": {"base_path": [], "ours_path": [], "base_ok": [], "ours_ok": []}}
    for s in seeds:
        subprocess.run([sys.executable, gen_script, "--seed", str(s), "--out", d, "--name", f"w{s}"],
                       check=True, stdout=subprocess.DEVNULL)
        man = json.load(open(os.path.join(d, f"w{s}.json")))
        truth0 = [{"id": i["id"], "class": i["class"], "x": i["x"], "y": i["y"]} for i in man["instances"]]
        rng = random.Random(s)

        # --- scenario S2: relocation after exploration (memory still holds the old positions)
        for rl in man["relocations"]:
            truth = [dict(o) for o in truth0]
            for o in truth:
                if o["id"] == rl["id"]:
                    o["x"], o["y"] = rl["to"]
            cls = rl["class"]
            if sum(1 for o in truth if o["class"] == cls) > 1:
                continue
            for name, runner in (("base", run_baseline), ("ours", run_mission)):
                env = ToyEnv(truth)
                mem = build_memory(truth0, random.Random(s), baseline_single_entry=(name == "base"))
                r = runner(mem, cls, env, cfg)
                res["reloc"][name].append(1 if (r.reached and env.at_true_object(cls)) else 0)
                res["reloc"][name + "_path"].append(env.path)
                if name == "ours":
                    res["reloc"]["ours_calls"].append(r.vlm_calls)

        # --- scenario S1: duplicated classes, nothing moved
        for cls in {o["class"] for o in truth0}:
            if sum(1 for o in truth0 if o["class"] == cls) < 2:
                continue
            for name, runner in (("base", run_baseline), ("ours", run_mission)):
                env = ToyEnv(truth0)
                if name == "base":   # baseline keeps the instance seen most often; emulate a random one
                    lst = [o for o in truth0 if o["class"] == cls]
                    keep = rng.choice(lst)
                    mem = build_memory([o for o in truth0 if o["class"] != cls] + [keep], random.Random(s),
                                       baseline_single_entry=True)
                else:
                    mem = build_memory(truth0, random.Random(s))
                r = runner(mem, cls, env, cfg)
                res["dup"][name + "_ok"].append(1 if (r.reached and env.at_true_object(cls)) else 0)
                res["dup"][name + "_path"].append(env.path / max(env.nearest_true_dist(cls), 1e-6))
    return res


def mean(v):
    return sum(v) / len(v) if v else float("nan")


if __name__ == "__main__":
    for fn in (test_association, test_tentative_ranks_below_confirmed,
               test_missing_and_resurrection, test_jsonl_roundtrip, test_extra_view_rescues_false_rejection,
               test_view_rules, test_unreachable_view_is_not_enough, test_unreachable_instance_gets_a_second_chance, test_baseline_retries,
               test_search_hits_use_the_larger_standoff):
        fn()
        print("PASS", fn.__name__)
    gen = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "gen_world.py")
    if os.path.exists(gen):
        r = toy_experiment(gen)
        rb, ro = r["reloc"], r["dup"]
        print(f"[toy] relocation  n={len(rb['base'])}: baseline success={mean(rb['base']):.2f}  "
              f"ours success={mean(rb['ours']):.2f}  ours VLM calls/episode={mean(rb['ours_calls']):.1f}  "
              f"path base={mean(rb['base_path']):.1f}m ours={mean(rb['ours_path']):.1f}m")
        print(f"[toy] duplicates  n={len(ro['base_ok'])}: baseline success={mean(ro['base_ok']):.2f}  "
              f"ours success={mean(ro['ours_ok']):.2f}  path/nearest base={mean(ro['base_path']):.2f} "
              f"ours={mean(ro['ours_path']):.2f}")
        print("[toy] reminder: logic check only, not experimental results")
