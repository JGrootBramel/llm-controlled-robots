"""Stubs so limo_llm_control.tools imports; jpl-rosa still loads tool modules."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
_SRC = _ROOT / "src"
for _p in (_ROOT, _SRC):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from tests import ros_stubs  # noqa: E402

ros_stubs.install()
