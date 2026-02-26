"""
Legacy LIMO autonomy tool wrappers.

This module is kept only for backward compatibility. All real implementations
now live in the canonical package `limo_llm_control.tools`.

New code should import from:

    import limo_llm_control.tools as robot_tools

or, for the old top-level alias:

    import tools as robot_tools

All public tool functions from `limo_llm_control.tools` are re-exported here,
so any existing imports like:

    from tools.limo_autonomy_tools import start_cam_coverage_node

continue to work without duplicating code.
"""

from limo_llm_control.tools import *  # noqa: F401,F403
from limo_llm_control.tools import __all__  # re-export the same public API
