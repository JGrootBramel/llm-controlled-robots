"""
Configure PYTHONPATH and fake ROS/langchain modules before importing limo_llm_control.

Used by `tests/unit/conftest.py` (and `tests/rosa/conftest.py`). For new unit tests under
`tests/unit/`, rely on that conftest instead of importing this module directly.
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
_SRC = _ROOT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from tests import ros_stubs  # noqa: E402

ros_stubs.install()
