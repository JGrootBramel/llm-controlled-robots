"""Tests for navigation ROSA tools (validation paths, mocked spawn)."""

from __future__ import annotations

from unittest.mock import patch

import pytest

from limo_llm_control.tools import navigation

pytestmark = pytest.mark.unit


@patch("limo_llm_control.tools.navigation.runner.spawn_node", return_value="spawned")
def test_start_cam_coverage_range_too_low(_spawn: object) -> None:
    out = navigation.start_cam_coverage_node(range_m=0.05)
    assert "range_m" in out
    assert "spawned" not in out


@patch("limo_llm_control.tools.navigation.runner.spawn_node", return_value="ok")
def test_start_cam_coverage_empty_map_topic(_spawn: object) -> None:
    out = navigation.start_cam_coverage_node(map_topic="  ")
    assert "Invalid" in out


@patch("limo_llm_control.tools.navigation.runner.spawn_node", return_value="ok")
def test_start_cam_coverage_valid_calls_spawn(_spawn: object) -> None:
    out = navigation.start_cam_coverage_node(range_m=2.0)
    assert out == "ok"


@patch("limo_llm_control.tools.navigation.runner.spawn_node", return_value="ok")
def test_start_straight_planner_heading_samples_out_of_range(_spawn: object) -> None:
    out = navigation.start_straight_planner_node(heading_samples=2)
    assert "heading_samples" in out


@patch("limo_llm_control.tools.navigation.runner.spawn_node", return_value="ok")
def test_start_straight_planner_valid(_spawn: object) -> None:
    out = navigation.start_straight_planner_node()
    assert out == "ok"


@patch("limo_llm_control.tools.navigation.rospy.sleep")
def test_go_to_map_pose_empty_goal_topic(_sleep: object) -> None:
    out = navigation.go_to_map_pose(0.0, 0.0, goal_topic="")
    assert "Invalid" in out
