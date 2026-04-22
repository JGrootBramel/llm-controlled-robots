"""Smoke tests against a live ROS master (opt-in via RUN_ROS_GRAPH_TESTS)."""

from __future__ import annotations

import os

import pytest

pytest.importorskip("rospy")

pytestmark = pytest.mark.ros_graph


def _ros_graph_enabled() -> bool:
    return (
        os.environ.get("RUN_ROS_GRAPH_TESTS") == "1"
        and bool(os.environ.get("ROS_MASTER_URI", "").strip())
    )


@pytest.mark.skipif(
    not _ros_graph_enabled(),
    reason="Set RUN_ROS_GRAPH_TESTS=1 and ROS_MASTER_URI (e.g. http://localhost:11311)",
)
def test_rosmaster_reports_topics() -> None:
    import rospy

    rospy.init_node("pytest_ros_graph", anonymous=True, disable_signals=True)
    topics = rospy.get_published_topics()
    assert isinstance(topics, list)
    names = [t[0] for t in topics]
    assert any(n == "/rosout" or n.endswith("/rosout") for n in names), (
        f"Expected /rosout in graph; got sample: {names[:10]}"
    )


@pytest.mark.skipif(
    not _ros_graph_enabled(),
    reason="Set RUN_ROS_GRAPH_TESTS=1 and ROS_MASTER_URI",
)
def test_rostopic_list_subprocess() -> None:
    import shutil
    import subprocess

    if not shutil.which("rostopic"):
        pytest.skip("rostopic not on PATH")

    uri = os.environ["ROS_MASTER_URI"]
    out = subprocess.run(
        ["rostopic", "list"],
        capture_output=True,
        text=True,
        timeout=10,
        env={**os.environ, "ROS_MASTER_URI": uri},
        check=False,
    )
    assert out.returncode == 0, out.stderr
    assert "/rosout" in out.stdout
