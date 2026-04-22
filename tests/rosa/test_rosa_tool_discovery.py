"""ROSA loads limo_llm_control.tools with a mocked LLM (no API calls)."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

pytest.importorskip("rosa")

pytestmark = pytest.mark.rosa


def _collect_tool_names(agent: object) -> set[str]:
    names: set[str] = set()
    for attr in ("tools", "_tools"):
        tools = getattr(agent, attr, None)
        if tools:
            for t in tools:
                n = getattr(t, "name", None)
                if isinstance(n, str):
                    names.add(n)
            if names:
                return names
    inner = getattr(agent, "_ROSA__tools", None)
    if inner is not None:
        if hasattr(inner, "__iter__") and not isinstance(inner, (str, bytes)):
            for t in inner:
                n = getattr(t, "name", None)
                if isinstance(n, str):
                    names.add(n)
        nested = getattr(inner, "tools", None)
        if nested:
            for t in nested:
                n = getattr(t, "name", None)
                if isinstance(n, str):
                    names.add(n)
    return names


def test_rosa_loads_expected_limo_tools() -> None:
    from rosa import ROSA

    llm = MagicMock()
    agent = ROSA(
        ros_version=1,
        llm=llm,
        tool_packages=["limo_llm_control.tools"],
        streaming=False,
        verbose=False,
    )
    names = _collect_tool_names(agent)
    assert names, "Expected ROSA to expose at least one tool name"
    for required in ("turn_in_place", "get_autonomy_status"):
        assert required in names, f"Missing tool {required!r}; have: {sorted(names)}"
