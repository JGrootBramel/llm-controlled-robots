"""ROS stubs and PYTHONPATH for unit tests (no real rospy)."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
_SRC = _ROOT / "src"
# Repo root must be on path so `import tests.ros_stubs` resolves (pytest pythonpath is only `src`).
for _p in (_ROOT, _SRC):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from tests import ros_stubs  # noqa: E402

ros_stubs.install()
