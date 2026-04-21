"""Tests for perception ROSA tools."""

from __future__ import annotations

from unittest.mock import patch

import pytest

from limo_llm_control.tools import perception

pytestmark = pytest.mark.unit


@patch("limo_llm_control.tools.perception.runner.spawn_node", return_value="started")
def test_start_object_finder_empty_prompt(_spawn: object) -> None:
    out = perception.start_object_finder_node(prompt="  ")
    assert "Invalid" in out
    assert "started" not in out


def test_update_object_query_empty() -> None:
    out = perception.update_object_query("  ")
    assert "Invalid" in out


@patch("limo_llm_control.tools.perception.rospy.wait_for_service", side_effect=Exception("no"))
def test_grasp_detected_object_service_unavailable(_wait: object) -> None:
    out = perception.grasp_detected_object()
    assert "unavailable" in out.lower()
