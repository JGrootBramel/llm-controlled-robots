"""Validate that the refactored launch files parse.

Runs ``roslaunch-check`` on each launch file. This catches typos in
``<node>``/``<param>`` / remaps that would only be discovered at runtime.

Marked ``@pytest.mark.ros_graph`` because it depends on ROS1 being
available on the host — running under ROS2-galactic pytest plugins this
test will be skipped automatically.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

pytestmark = pytest.mark.ros_graph


_LAUNCH_DIR = (
    Path(__file__).resolve().parents[2]
    / "catkin_ws"
    / "src"
    / "limo_rosa_bridge"
    / "launch"
)


_LAUNCH_FILES = [
    "autonomy_core.launch",
    "autonomy_perception.launch",
    "autonomy_manipulation.launch",
    "rosa_bridge.launch",
]


@pytest.mark.parametrize("name", _LAUNCH_FILES)
def test_launch_parses(name: str) -> None:
    path = _LAUNCH_DIR / name
    assert path.exists(), f"missing launch file: {path}"

    checker = shutil.which("roslaunch-check")
    if checker is None:
        pytest.skip("roslaunch-check not installed (needs ROS1)")

    # roslaunch-check writes errors to stdout; non-zero exit = invalid file.
    result = subprocess.run(
        [checker, str(path)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, (
        f"roslaunch-check failed for {name}:\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )
