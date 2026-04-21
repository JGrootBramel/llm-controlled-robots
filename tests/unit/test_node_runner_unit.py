"""Unit tests for limo_llm_control.tools._node_runner helpers (no ROS required)."""

from __future__ import annotations

import pytest

from limo_llm_control.tools import _node_runner as nr

pytestmark = pytest.mark.unit


def test_validate_float_in_range() -> None:
    assert nr.validate_float("x", 1.5, 0.0, 2.0) is None


def test_validate_float_below_range() -> None:
    err = nr.validate_float("x", -0.1, 0.0, 2.0)
    assert err is not None
    assert "x" in err


def test_validate_float_above_range() -> None:
    assert nr.validate_float("x", 3.0, 0.0, 2.0) is not None


def test_validate_float_non_numeric() -> None:
    assert nr.validate_float("x", "nope", 0.0, 2.0) is not None  # type: ignore[arg-type]


def test_validate_int_in_range() -> None:
    assert nr.validate_int("n", 5, 1, 10) is None


def test_validate_int_out_of_range() -> None:
    assert nr.validate_int("n", 0, 1, 10) is not None


def test_validate_int_non_int_string() -> None:
    assert nr.validate_int("n", "x", 1, 10) is not None  # type: ignore[arg-type]


def test_build_ros_private_param_args() -> None:
    args = nr._build_ros_private_param_args({"foo": 1, "bar": "baz"})
    assert set(args) == {"_foo:=1", "_bar:=baz"}
