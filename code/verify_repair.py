#!/usr/bin/env python3
"""
verify_repair.py - mission logic: retrieve -> approach -> VERIFY -> (repair | finish).

Pure Python. The robot / simulator is hidden behind a small `env` interface so the same logic
can be unit-tested offline and later wired to ROS + TD3 + the Moondream server.

env interface (duck-typed):
    env.robot_xy()                       -> (x, y)
    env.time()                           -> float seconds
    env.go_to(x, y, stop_within)         -> bool   drive until within `stop_within` m of (x, y);
                                                   False if the controller fails (collision/timeout)
    env.verify(cls, face_xy=None, scan=False)
                                         -> [(x, y), ...]  positions of objects of `cls` the VLM sees.
                                            face_xy=(x, y): turn towards that point first and answer only
                                            'is it there?' (returns [face_xy] on yes, [] on no).
                                            scan=True: rotate in place and localise what is seen.
    env.search_waypoints(cls)            -> iterable of (x, y) fallback search poses
    env.vlm_calls                        -> int, running counter (used for cost metrics)
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from instance_memory import InstanceMemory, dist


@dataclass
class Config:
    verify_dist: float = 1.5      # stop and verify at this distance from the remembered position
    search_verify_dist: float = 2.5  # candidates found by the search have an uncertain position (~0.6 m median):
                                     # verify from farther away so that the error stays inside the field of view
    match_radius: float = 1.0     # a detection this close to the remembered position confirms it
    goal_tol: float = 0.8         # final stop distance (same as GOAL_REACHED_DIST of the baseline env)
    max_attempts: int = 3         # candidates tried before the fallback search
    n_views: int = 1              # approach azimuths used for one verification (1 = a single look)
    view_rule: str = "any"        # any | majority | all of the n_views answers confirm the object
    view_offset_deg: float = 55.0 # azimuth step around the remembered position
    allow_search: bool = True
    max_search_checks: int = 3    # search hits that are verified at the approach pose before giving up
    baseline_retries: int = 0     # extra attempts to reach the goal for the no-verification baselines (B1r)


@dataclass
class Attempt:
    instance_id: str
    x: float
    y: float
    outcome: str                  # verified | not_found | unreachable


@dataclass
class MissionResult:
    reached: bool                 # controller reached the final goal (ground truth is checked elsewhere)
    reason: str                   # verified | found_by_search | NOT_FOUND | no_candidate
    attempts: List[Attempt] = field(default_factory=list)
    vlm_calls: int = 0
    goal: Optional[Tuple[float, float]] = None


def _alt_pose(inst, robot_xy, cfg, sign, radius=None):
    """Another approach pose on the circle of radius `radius` (default verify_dist) around the remembered position."""
    r = cfg.verify_dist if radius is None else radius
    az = math.atan2(robot_xy[1] - inst.y, robot_xy[0] - inst.x) + sign * math.radians(cfg.view_offset_deg)
    return inst.x + r * math.cos(az), inst.y + r * math.sin(az)


def _decided(answers, n_views, rule):
    """True/False as soon as the outcome of the n_views looks is settled, otherwise None."""
    yes = sum(answers)
    no = len(answers) - yes
    remaining = n_views - len(answers)
    if rule == "any":
        if yes >= 1:
            return True
        return False if remaining == 0 else None
    if rule == "all":
        if no >= 1:
            return False
        return True if remaining == 0 else None
    need = n_views // 2 + 1                                    # majority
    if yes >= need:
        return True
    if no > n_views - need:
        return False
    return None


def run_baseline(memory: InstanceMemory, cls: str, env, cfg: Config) -> MissionResult:
    """Baseline behaviour: closest remembered entry of the class, no verification, no fallback."""
    rx, ry = env.robot_xy()
    cands = [i for i in memory.items.values() if i.cls == cls]     # status is ignored by the baseline
    if not cands:
        return MissionResult(False, "no_candidate")
    inst = min(cands, key=lambda i: dist(i.x, i.y, rx, ry))
    ok = False
    for _ in range(1 + cfg.baseline_retries):                 # B1r: the same retry budget as the full method
        ok = env.go_to(inst.x, inst.y, cfg.goal_tol)
        if ok:
            break
    return MissionResult(ok, "no_verification", [Attempt(inst.id, inst.x, inst.y, "unverified")],
                         0, (inst.x, inst.y))


def run_mission(memory: InstanceMemory, cls: str, env, cfg: Config) -> MissionResult:
    """Ours: rank candidates, verify on arrival (from several sides if needed), mark stale entries, try the next
    one, then search the room. A candidate found by the search must pass the same approach verification."""
    calls0 = env.vlm_calls
    attempts: List[Attempt] = []
    tried = set()
    rejected = set()                  # verification said 'not there'  -> never verified again in this mission
    unreachable = {}                  # id -> number of failed approaches (not evidence of absence)

    def finish(reached, reason, goal):
        return MissionResult(reached, reason, attempts, env.vlm_calls - calls0, goal)

    def approach_and_verify(inst, vdist):
        """('verified', point) | ('not_found', None) | ('unreachable', None); vdist = standoff distance of this candidate"""
        if not env.go_to(inst.x, inst.y, vdist):
            attempts.append(Attempt(inst.id, inst.x, inst.y, "unreachable"))
            unreachable[inst.id] = unreachable.get(inst.id, 0) + 1
            return "unreachable", None
        first_view_xy = env.robot_xy()
        answers, hit_pt, verdict = [], None, None
        for v in range(max(1, cfg.n_views)):
            if v > 0:                                          # look at the same spot from another side
                ax, ay = _alt_pose(inst, first_view_xy, cfg, 1 if v % 2 == 1 else -1, vdist)
                if not env.go_to(ax, ay, 0.4):
                    break
            dets = env.verify(cls, face_xy=(inst.x, inst.y))
            hit = [d for d in dets if dist(d[0], d[1], inst.x, inst.y) <= cfg.match_radius]
            for d in dets:                                      # whatever else the VLM sees is evidence
                if d not in hit:
                    memory.observe(cls, d[0], d[1], conf=0.6, t=env.time())
            answers.append(1 if hit else 0)
            if hit and hit_pt is None:
                hit_pt = min(hit, key=lambda p: dist(p[0], p[1], inst.x, inst.y))
            verdict = _decided(answers, cfg.n_views, cfg.view_rule)
            if verdict is not None:
                break
        if verdict is None:                                     # a view could not be reached: decide on what we have
            yes = sum(answers)
            if cfg.view_rule == "any":
                verdict = yes >= 1
            elif len(answers) < 2:                              # 'all' / 'majority' need at least two looks
                verdict = False
            elif cfg.view_rule == "all":
                verdict = yes == len(answers)
            else:
                verdict = yes * 2 > len(answers)
        t = env.time()
        if verdict and hit_pt is not None:
            memory.mark_verified(inst.id, t, hit_pt[0], hit_pt[1])
            attempts.append(Attempt(inst.id, inst.x, inst.y, "verified"))
            return "verified", hit_pt
        memory.mark_missing(inst.id, t)
        rejected.add(inst.id)
        attempts.append(Attempt(inst.id, inst.x, inst.y, "not_found"))
        return "not_found", None

    for _ in range(cfg.max_attempts):
        ranked = [i for _, i in memory.rank(cls, env.robot_xy(), env.time()) if i.id not in tried]
        if not ranked:
            break
        inst = ranked[0]
        tried.add(inst.id)
        status, d = approach_and_verify(inst, cfg.verify_dist)
        if status == "verified":
            return finish(env.go_to(d[0], d[1], cfg.goal_tol), "verified", d)

    if cfg.allow_search:
        checked = 0
        for wx, wy in env.search_waypoints(cls):
            if checked >= cfg.max_search_checks:
                break
            if not env.go_to(wx, wy, 0.5):
                continue
            for pos in env.verify(cls, scan=True):         # scan: rotate in place, localise what the VLM reports
                inst = memory.observe(cls, pos[0], pos[1], conf=0.8, t=env.time())
                if inst.id in rejected or unreachable.get(inst.id, 0) >= 2:
                    continue                                  # an instance we could not reach gets a second chance
                tried.add(inst.id)
                checked += 1
                status, d = approach_and_verify(inst, cfg.search_verify_dist)   # a search hit must pass the approach check too
                if status == "verified":
                    return finish(env.go_to(d[0], d[1], cfg.goal_tol), "found_by_search", d)
    return finish(False, "NOT_FOUND", None)
