#!/usr/bin/env python3
"""
instance_memory.py - instance-level semantic object memory (pure Python, no ROS).

Differences to the baseline memory (pixel_to_cords.save_object_detection):
  * one entry per INSTANCE, not per class (nearest-neighbour data association with a distance gate)
  * every instance carries evidence: number of observations, mean VLM confidence, recency,
    number of successful / failed arrival verifications
  * instances can be marked 'missing' (kept as tombstones, excluded from ranking, resurrected
    if the object is seen again near the old position)
  * candidates are RANKED by reliability and distance instead of "closest wins"
  * JSONL export uses the baseline field names so the baseline retriever can still read it
"""
import json
import math
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple


def dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


@dataclass
class Instance:
    id: str
    cls: str
    x: float
    y: float
    z: float = 0.0
    n_obs: int = 1
    conf_mean: float = 0.5
    first_seen: float = 0.0
    last_seen: float = 0.0
    status: str = "active"            # active | missing
    n_verified: int = 0
    n_failed: int = 0
    missing_since: Optional[float] = None


class InstanceMemory:
    def __init__(self, gate: float = 0.75, min_obs: int = 2, n_cap: int = 25,
                 w_dist: float = 0.15, tau: float = 600.0, verified_bonus: float = 0.5):
        self.gate = gate                  # max distance to associate a detection with an instance (m)
        self.min_obs = min_obs            # below this the instance is "tentative"
        self.n_cap = n_cap                # caps the running-mean weight so positions can still adapt
        self.w_dist = w_dist              # score penalty per metre of distance
        self.tau = tau                    # recency time constant (s)
        self.verified_bonus = verified_bonus
        self.items: Dict[str, Instance] = {}
        self._next = 0

    # ---------------------------------------------------------------- updates
    def observe(self, cls: str, x: float, y: float, conf: float = 0.5,
                t: float = 0.0, z: float = 0.0) -> Instance:
        """Associate a detection with an existing instance or create a new one."""
        best, best_d = None, self.gate
        for inst in self.items.values():
            if inst.cls != cls:
                continue
            d = dist(inst.x, inst.y, x, y)
            if d <= best_d:
                best, best_d = inst, d
        if best is not None:
            a = 1.0 / min(best.n_obs + 1, self.n_cap)
            best.x += a * (x - best.x)
            best.y += a * (y - best.y)
            best.z += a * (z - best.z)
            best.conf_mean += a * (conf - best.conf_mean)
            best.n_obs += 1
            best.last_seen = t
            if best.status == "missing":           # it is back
                best.status = "active"
                best.missing_since = None
                best.n_failed = 0
            return best
        inst = Instance(id=f"{cls.replace(' ', '_')}_{self._next:04d}", cls=cls, x=x, y=y, z=z,
                        n_obs=1, conf_mean=conf, first_seen=t, last_seen=t)
        self._next += 1
        self.items[inst.id] = inst
        return inst

    def mark_verified(self, inst_id: str, t: float, x: Optional[float] = None,
                      y: Optional[float] = None):
        inst = self.items[inst_id]
        inst.n_verified += 1
        inst.n_failed = 0
        inst.status = "active"
        inst.missing_since = None
        inst.last_seen = t
        if x is not None and y is not None:        # close-range detection is more accurate
            inst.x = 0.5 * inst.x + 0.5 * x
            inst.y = 0.5 * inst.y + 0.5 * y

    def mark_missing(self, inst_id: str, t: float):
        inst = self.items[inst_id]
        inst.n_failed += 1
        inst.status = "missing"
        inst.missing_since = t

    # ---------------------------------------------------------------- queries
    def active(self, cls: str) -> List[Instance]:
        return [i for i in self.items.values() if i.cls == cls and i.status == "active"]

    def score(self, inst: Instance, robot_xy: Tuple[float, float], t_now: float) -> float:
        reliability = (1.0 - math.exp(-inst.n_obs / float(self.min_obs))) * inst.conf_mean
        recency = math.exp(-max(0.0, t_now - inst.last_seen) / self.tau)
        bonus = self.verified_bonus if inst.n_verified > 0 else 0.0
        d = dist(inst.x, inst.y, robot_xy[0], robot_xy[1])
        return reliability * (0.5 + 0.5 * recency) + bonus - self.w_dist * d

    def rank(self, cls: str, robot_xy: Tuple[float, float], t_now: float = 0.0):
        """Active candidates of a class, best first, as list of (score, Instance)."""
        scored = [(self.score(i, robot_xy, t_now), i) for i in self.active(cls)]
        scored.sort(key=lambda p: -p[0])
        return scored

    # ---------------------------------------------------------------- persistence
    def to_records(self) -> List[dict]:
        out = []
        for i in self.items.values():
            out.append({
                "id": i.id, "object_class": i.cls,
                "world_x": i.x, "world_y": i.y, "world_z": i.z,
                "observation_count": i.n_obs, "confidence": i.conf_mean,
                "timestamp": i.last_seen, "detection_method": "instance_memory",
                "status": i.status, "n_verified": i.n_verified, "n_failed": i.n_failed,
                "first_seen": i.first_seen, "missing_since": i.missing_since,
            })
        return out

    def save_jsonl(self, path: str):
        with open(path, "w") as f:
            for r in self.to_records():
                f.write(json.dumps(r) + "\n")

    @classmethod
    def load_jsonl(cls_, path: str, **kw) -> "InstanceMemory":
        """Reads our files and also the baseline detected_objects.jsonl (one record = one instance)."""
        mem = cls_(**kw)
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                r = json.loads(line)
                inst = Instance(
                    id=r.get("id", f"obj_{mem._next:04d}"), cls=r["object_class"],
                    x=r["world_x"], y=r["world_y"], z=r.get("world_z", 0.0),
                    n_obs=r.get("observation_count", 1), conf_mean=r.get("confidence", 0.5),
                    first_seen=r.get("first_seen", r.get("timestamp", 0.0)),
                    last_seen=r.get("timestamp", 0.0), status=r.get("status", "active"),
                    n_verified=r.get("n_verified", 0), n_failed=r.get("n_failed", 0),
                    missing_since=r.get("missing_since"))
                mem.items[inst.id] = inst
                mem._next += 1
        return mem
