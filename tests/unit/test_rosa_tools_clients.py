"""Mock-based tests for the slimmed ROSA tool clients.

These tests exercise navigation/perception/manipulation/mission as thin
rospy clients: patch ``wait_for_service`` and ``ServiceProxy`` / ``Publisher``
and assert each tool hits the right service or topic.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from limo_llm_control.tools import manipulation, mission, navigation, perception

pytestmark = pytest.mark.unit


# ---------------------------------------------------------------- navigation


def _ok_proxy(**kwargs):
    resp = SimpleNamespace(success=True, message="ok")
    return MagicMock(return_value=resp)


@patch("limo_llm_control.tools.navigation.rospy.wait_for_service")
@patch("limo_llm_control.tools.navigation.rospy.ServiceProxy")
def test_start_exploration_calls_set_bool_true(mock_proxy_cls, _wait):
    mock_proxy = _ok_proxy()
    mock_proxy_cls.return_value = mock_proxy
    out = navigation.start_exploration.func() if hasattr(navigation.start_exploration, "func") else navigation.start_exploration()
    mock_proxy_cls.assert_called_once()
    args, _ = mock_proxy_cls.call_args
    assert args[0] == "/exploration_enabled"
    mock_proxy.assert_called_once_with(True)
    assert "OK" in out


@patch("limo_llm_control.tools.navigation.rospy.wait_for_service")
@patch("limo_llm_control.tools.navigation.rospy.ServiceProxy")
def test_stop_exploration_calls_set_bool_false(mock_proxy_cls, _wait):
    mock_proxy = _ok_proxy()
    mock_proxy_cls.return_value = mock_proxy
    fn = navigation.stop_exploration.func if hasattr(navigation.stop_exploration, "func") else navigation.stop_exploration
    fn()
    args, _ = mock_proxy_cls.call_args
    assert args[0] == "/exploration_enabled"
    mock_proxy.assert_called_once_with(False)


@patch("limo_llm_control.tools.navigation.rospy.wait_for_service")
@patch("limo_llm_control.tools.navigation.rospy.ServiceProxy")
def test_reset_exploration_uses_trigger_service(mock_proxy_cls, _wait):
    mock_proxy = _ok_proxy()
    mock_proxy_cls.return_value = mock_proxy
    fn = navigation.reset_exploration.func if hasattr(navigation.reset_exploration, "func") else navigation.reset_exploration
    fn()
    args, _ = mock_proxy_cls.call_args
    assert args[0] == "/exploration_reset"
    mock_proxy.assert_called_once_with()


@patch("limo_llm_control.tools.navigation.rospy.wait_for_service", side_effect=Exception("nope"))
def test_service_unavailable_surfaces_nicely(_wait):
    fn = navigation.start_exploration.func if hasattr(navigation.start_exploration, "func") else navigation.start_exploration
    out = fn()
    assert "unavailable" in out.lower()


@patch("limo_llm_control.tools.navigation.rospy.Publisher")
@patch("limo_llm_control.tools.navigation.rospy.sleep")
def test_go_to_map_pose_publishes_to_default_topic(_sleep, mock_pub_cls):
    mock_pub = MagicMock()
    mock_pub_cls.return_value = mock_pub
    fn = navigation.go_to_map_pose.func if hasattr(navigation.go_to_map_pose, "func") else navigation.go_to_map_pose
    out = fn(1.0, 2.0, yaw_deg=90.0)
    assert mock_pub_cls.call_args.args[0] == "/move_base_simple/goal"
    mock_pub.publish.assert_called_once()
    msg = mock_pub.publish.call_args.args[0]
    assert abs(msg.pose.position.x - 1.0) < 1e-9
    assert abs(msg.pose.position.y - 2.0) < 1e-9
    assert msg.header.frame_id == "map"
    assert "1.00" in out and "2.00" in out


# ---------------------------------------------------------------- perception


@patch("limo_llm_control.tools.perception.rospy.wait_for_service")
@patch("limo_llm_control.tools.perception.rospy.ServiceProxy")
def test_enable_red_cube_detector_hits_right_service(mock_proxy_cls, _wait):
    mock_proxy = _ok_proxy()
    mock_proxy_cls.return_value = mock_proxy
    fn = perception.enable_red_cube_detector.func if hasattr(perception.enable_red_cube_detector, "func") else perception.enable_red_cube_detector
    fn(True)
    assert mock_proxy_cls.call_args.args[0] == "/red_cube_detector/enable"
    mock_proxy.assert_called_once_with(True)


@patch("limo_llm_control.tools.perception.rospy.wait_for_service")
@patch("limo_llm_control.tools.perception.rospy.ServiceProxy")
def test_snapshot_red_cube_hits_trigger(mock_proxy_cls, _wait):
    mock_proxy = _ok_proxy()
    mock_proxy_cls.return_value = mock_proxy
    fn = perception.snapshot_red_cube.func if hasattr(perception.snapshot_red_cube, "func") else perception.snapshot_red_cube
    fn()
    assert mock_proxy_cls.call_args.args[0] == "/red_cube_detector/snapshot"


@patch("limo_llm_control.tools.perception.rospy.wait_for_message")
def test_get_latest_red_cube_formats_pose(mock_wait):
    msg = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        pose=SimpleNamespace(position=SimpleNamespace(x=1.25, y=-0.5, z=0.0)),
    )
    mock_wait.return_value = msg
    fn = perception.get_latest_red_cube.func if hasattr(perception.get_latest_red_cube, "func") else perception.get_latest_red_cube
    out = fn(timeout_s=0.1)
    assert "1.25" in out and "-0.50" in out and "map" in out


@patch("limo_llm_control.tools.perception.rospy.wait_for_message", side_effect=Exception("timeout"))
def test_get_latest_red_cube_timeout_message(_wait):
    fn = perception.get_latest_red_cube.func if hasattr(perception.get_latest_red_cube, "func") else perception.get_latest_red_cube
    out = fn(timeout_s=0.05)
    assert "no pose" in out.lower()


# --------------------------------------------------------------- manipulation


@pytest.mark.parametrize(
    "fn_name, expected_service",
    [
        ("approach_object", "/approach_object/approach"),
        ("cancel_approach", "/approach_object/cancel"),
        ("pick_object", "/arm_control/pick"),
        ("pick_object_vendor_sync", "/arm_control/pick_vendor_sync"),
        ("place_object", "/arm_control/place"),
        ("arm_go_home", "/arm_control/go_home"),
    ],
)
@patch("limo_llm_control.tools.manipulation.rospy.wait_for_service")
@patch("limo_llm_control.tools.manipulation.rospy.ServiceProxy")
def test_manipulation_tool_hits_expected_service(
    mock_proxy_cls, _wait, fn_name, expected_service
):
    mock_proxy = _ok_proxy()
    mock_proxy_cls.return_value = mock_proxy
    fn_obj = getattr(manipulation, fn_name)
    fn = fn_obj.func if hasattr(fn_obj, "func") else fn_obj
    out = fn()
    assert mock_proxy_cls.call_args.args[0] == expected_service
    assert "OK" in out


# -------------------------------------------------------------------- mission


@patch("limo_llm_control.tools.mission.rospy")
def test_fetch_red_cubes_happy_path_sequence(mock_rospy):
    """When /red_cubes/found reports True immediately, mission runs all steps."""
    mock_rospy.wait_for_service = MagicMock()
    mock_rospy.sleep = MagicMock()
    mock_rospy.Time = SimpleNamespace(now=MagicMock(return_value=0.0))
    mock_rospy.Publisher = MagicMock(return_value=MagicMock())

    # ServiceProxy stub: always returns success.
    def _svc_ctor(name, _type):
        return MagicMock(return_value=SimpleNamespace(success=True, message=f"{name} ok"))

    mock_rospy.ServiceProxy = MagicMock(side_effect=_svc_ctor)

    # /red_cubes/found -> True on the first poll.
    mock_rospy.wait_for_message = MagicMock(return_value=SimpleNamespace(data=True))

    fn = mission.fetch_red_cubes.func if hasattr(mission.fetch_red_cubes, "func") else mission.fetch_red_cubes
    out = fn(delivery_x=1.0, delivery_y=2.0, delivery_yaw_deg=0.0, max_cubes=1,
            detection_timeout_s=0.1)

    # Service calls hit each expected service at least once.
    called_services = [c.args[0] for c in mock_rospy.ServiceProxy.call_args_list]
    for expected in (
        "/red_cube_detector/enable",
        "/exploration_enabled",
        "/red_cube_detector/snapshot",
        "/approach_object/approach",
        "/arm_control/pick",
        "/arm_control/place",
    ):
        assert expected in called_services, f"missing {expected} in {called_services}"

    assert "delivered 1/1" in out


@patch("limo_llm_control.tools.mission.rospy")
def test_fetch_red_cubes_timeout_bails(mock_rospy):
    mock_rospy.wait_for_service = MagicMock()
    mock_rospy.sleep = MagicMock()
    mock_rospy.Time = SimpleNamespace(now=MagicMock(return_value=0.0))
    mock_rospy.Publisher = MagicMock(return_value=MagicMock())

    def _svc_ctor(name, _type):
        return MagicMock(return_value=SimpleNamespace(success=True, message="ok"))

    mock_rospy.ServiceProxy = MagicMock(side_effect=_svc_ctor)
    mock_rospy.wait_for_message = MagicMock(side_effect=Exception("no msg"))

    fn = mission.fetch_red_cubes.func if hasattr(mission.fetch_red_cubes, "func") else mission.fetch_red_cubes
    out = fn(max_cubes=1, detection_timeout_s=0.1)

    called_services = [c.args[0] for c in mock_rospy.ServiceProxy.call_args_list]
    # Approach/pick/place never happen on timeout.
    assert "/approach_object/approach" not in called_services
    assert "/arm_control/pick" not in called_services
    assert "delivered 0/1" in out
