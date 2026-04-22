"""Compute a safe standoff pose around a target object.

Extracted from ``frontier_planner_node._pick_standoff_goal``. Reachability
checks are left to the caller so this module stays pure Python.
"""

from __future__ import annotations

import math
from typing import Iterator, List, Tuple


def standoff_candidates(
    tx: float,
    ty: float,
    base_yaw: float,
    distance_m: float,
    arc_deg: float = 10.0,
    max_arc_deg: float = 90.0,
) -> List[Tuple[float, float, float]]:
    """Generate candidate ``(gx, gy, yaw)`` standoff poses around ``(tx, ty)``.

    The robot is placed ``distance_m`` away from the target, initially
    directly behind ``base_yaw`` (so the camera faces the object). Extra
    candidates are produced by rotating ±``arc_deg`` up to ``max_arc_deg``.
    Each candidate's yaw points at the target.
    """
    d_rad = math.radians(max(1e-3, arc_deg))
    max_steps = int(round(max(0.0, max_arc_deg) / max(1e-3, arc_deg)))
    offsets: List[float] = [0.0]
    for k in range(1, max_steps + 1):
        offsets.append(+k * d_rad)
        offsets.append(-k * d_rad)

    candidates: List[Tuple[float, float, float]] = []
    for off in offsets:
        cang = base_yaw + off
        gx = tx - distance_m * math.cos(cang)
        gy = ty - distance_m * math.sin(cang)
        yaw_face = math.atan2(ty - gy, tx - gx)
        candidates.append((gx, gy, yaw_face))
    return candidates


def iter_standoff_candidates(
    tx: float,
    ty: float,
    base_yaw: float,
    distance_m: float,
    arc_deg: float = 10.0,
    max_arc_deg: float = 90.0,
) -> Iterator[Tuple[float, float, float]]:
    """Iterator flavour for callers that can bail out early."""
    yield from standoff_candidates(tx, ty, base_yaw, distance_m, arc_deg, max_arc_deg)
