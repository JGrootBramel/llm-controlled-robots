"""Tests for scan / formatting helpers in cubes tools."""

from __future__ import annotations

import pytest

from limo_llm_control.tools import cubes

pytestmark = pytest.mark.unit


def test_format_scan_result_empty() -> None:
    s = cubes._format_scan_result("cubes", [])
    assert "No cubes detected" in s


def test_format_scan_result_nonempty() -> None:
    s = cubes._format_scan_result("objects", [(0.1, 0.2)])
    assert "Object 1" in s
    assert "x=0.1" in s
