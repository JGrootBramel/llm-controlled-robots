#!/usr/bin/env python3
"""
ROSA Robot Controller - Natural Language Control for LIMO Cobot

This script integrates ROSA (Robot Operating System Agent) with the LIMO Cobot
simulation to enable natural language control of the robot.

Usage:
    python3 rosa_robot_controller.py [--provider gemini|openai] [--api-key YOUR_API_KEY]

Requirements:
    - ROS 1 Noetic running
    - Simulation must be started (Gazebo + RViz)
    - LLM API key (OpenAI or Google Gemini)
    - ROSA venv activated: source /opt/rosa-venv/bin/activate
"""

import os
import sys
import json

# Debug logging setup
# Debug mode expects logs INSIDE the repo at `.cursor/debug.log`.
# Use a path relative to this file so it works both on host and inside Docker volume mounts.
DEBUG_LOG_PATH = os.path.abspath(
    os.getenv(
        "ROSA_DEBUG_LOG_PATH",
        os.path.join(os.path.dirname(__file__), "..", ".cursor", "debug.log"),
    )
)

def debug_log(location, message, data=None, hypothesis_id=None):
    """Write debug log entry"""
    try:
        import time
        run_id = os.getenv("ROSA_DEBUG_RUN_ID", "run1")
        log_entry = {
            "timestamp": int(time.time() * 1000),
            "location": location,
            "message": message,
            "data": data or {},
            "sessionId": "debug-session",
            "runId": run_id,
            "hypothesisId": hypothesis_id
        }
        with open(DEBUG_LOG_PATH, "a") as f:
            f.write(json.dumps(log_entry) + "\n")
    except Exception:
        pass  # Silently fail if logging doesn't work

# region agent log
# Module load stamp to confirm which code version is running inside Docker.
debug_log("rosa_robot_controller.py:module", "Module loaded (debug stamp)", {
    "stamp": "2025-12-23T00:00Z-agent-v2",
    "debug_log_path": DEBUG_LOG_PATH,
}, "H0")
# endregion

# region agent log
# Some dependencies (notably newer Google/LangChain stacks) expect
# importlib.metadata.packages_distributions (present in Python 3.10+).
# On Python 3.9, we can safely patch it from the importlib-metadata backport.
try:
    import importlib.metadata as _ilm
    if not hasattr(_ilm, "packages_distributions"):
        try:
            import importlib_metadata as _ilm_backport  # pip package: importlib-metadata
            _ilm.packages_distributions = _ilm_backport.packages_distributions  # type: ignore[attr-defined]
            debug_log("rosa_robot_controller.py:preflight", "Patched importlib.metadata.packages_distributions from backport", {
                "python_version": sys.version.split()[0],
            }, "H9")
        except Exception as e:
            debug_log("rosa_robot_controller.py:preflight", "Could not patch importlib.metadata.packages_distributions", {
                "error_type": type(e).__name__,
                "error_message": str(e),
            }, "H9")
except Exception as e:
    debug_log("rosa_robot_controller.py:preflight", "importlib.metadata preflight failed", {
        "error_type": type(e).__name__,
        "error_message": str(e),
    }, "H9")
# endregion

# region agent log
debug_log("rosa_robot_controller.py:preflight", "Python preflight before rospy import", {
    "executable": sys.executable,
    "python_version": sys.version.split()[0],
    "virtual_env": os.getenv("VIRTUAL_ENV", ""),
    "cwd": os.getcwd(),
    "sys_path_head": sys.path[:5],
}, "H7")
# endregion

# ROS imports (can fail if ROS python deps aren't installed in current venv)
try:
    import rospy
    from geometry_msgs.msg import Twist
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from sensor_msgs.msg import JointState
    import actionlib
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
except Exception as e:
    # region agent log
    debug_log("rosa_robot_controller.py:preflight", "Failed importing rospy / ROS deps", {
        "error_type": type(e).__name__,
        "error_message": str(e),
        "hint": "If ModuleNotFoundError: rospkg, install into venv: python3 -m pip install rospkg catkin_pkg",
    }, "H7")
    # endregion
    raise


def is_gemini_compatible_schema(schema):
    """
    Check if a JSON schema is compatible with Gemini's strict requirements.
    
    Gemini rejects:
    - $defs references
    - anyOf/oneOf/allOf without explicit type
    - Nested arrays (items.items) without explicit type
    - Variadic arguments (v__args)
    - Properties with nested items.items without type
    """
    if not isinstance(schema, dict):
        return True  # Non-dict schemas are usually fine
    
    # Convert to string for quick checks
    schema_str = str(schema)
    
    # Check for $defs (not supported)
    if '$defs' in schema or '$ref' in schema_str:
        return False
    
    # Check for variadic arguments (v__args anywhere in schema)
    if 'v__args' in schema_str:
        return False
    
    # Check for nested items without type - more aggressive check
    def check_nested_items(obj, depth=0, path=""):
        if depth > 15:  # Prevent infinite recursion
            return True
        if not isinstance(obj, dict):
            return True
        
        # Check if this is an items definition
        if 'items' in obj:
            items_def = obj['items']
            # If items is a dict and has nested items, check for type
            if isinstance(items_def, dict):
                if 'items' in items_def:
                    # Nested items found - MUST have type
                    if 'type' not in items_def:
                        print(f"[DEBUG] Rejecting schema: nested items.items without type at {path}")
                        return False
                    # Recursively check the nested items
                    if not check_nested_items(items_def, depth + 1, f"{path}.items"):
                        return False
                # Also check if items itself needs a type (for array items)
                if 'type' not in items_def and 'anyOf' not in items_def and 'oneOf' not in items_def:
                    # This might be okay, but let's be strict
                    pass
        
        # Check properties recursively (this is where the error is happening)
        if 'properties' in obj:
            for prop_name, prop_schema in obj['properties'].items():
                if isinstance(prop_schema, dict):
                    # Check if this property has problematic patterns
                    if 'v__args' in str(prop_schema):
                        print(f"[DEBUG] Rejecting schema: v__args in property '{prop_name}'")
                        return False
                    # Recursively check properties
                    if not check_nested_items(prop_schema, depth + 1, f"{path}.properties.{prop_name}"):
                        return False
        
        # Recursively check all values
        for key, value in obj.items():
            if key not in ['type', 'description', 'title', 'enum', 'default']:  # Skip safe keys
                if not check_nested_items(value, depth + 1, f"{path}.{key}"):
                    return False
        
        return True
    
    return check_nested_items(schema)


def adapt_tool_for_gemini(tool):
    """
    Adapt a LangChain tool to be Gemini-compatible by fixing schema issues.
    
    Returns None if tool cannot be adapted, or a new tool with fixed schema.
    """
    try:
        # Get the tool's schema
        if not hasattr(tool, 'args_schema'):
            return tool  # No schema, assume compatible
        
        original_schema = tool.args_schema
        if not hasattr(original_schema, 'schema'):
            return tool  # Can't inspect, return as-is
        
        schema_dict = original_schema.schema()
        
        # Check if already compatible
        if is_gemini_compatible_schema(schema_dict):
            return tool
        
        # Try to fix the schema
        fixed_schema = fix_schema_for_gemini(schema_dict)
        
        if fixed_schema is None:
            # Can't fix, return None to filter out
            return None
        
        # Create a new tool with fixed schema
        from pydantic import BaseModel, create_model
        from typing import get_type_hints
        
        # For now, if we can't easily fix it, return None to filter
        # This is a conservative approach - we'll use only compatible tools
        return None
        
    except Exception as e:
        debug_log("rosa_robot_controller.py:adapt_tool", "Error adapting tool", {
            "tool_name": getattr(tool, 'name', 'unknown'),
            "error": str(e)
        }, "J")
        return None


def fix_schema_for_gemini(schema):
    """
    Attempt to fix a schema to be Gemini-compatible.
    Returns fixed schema dict or None if cannot be fixed.
    """
    if not isinstance(schema, dict):
        return schema
    
    # Remove $defs and $ref
    if '$defs' in schema:
        # For now, we can't easily resolve $defs, so return None
        return None
    
    # Fix nested items without type
    def fix_nested_items(obj):
        if not isinstance(obj, dict):
            return obj
        
        new_obj = {}
        for key, value in obj.items():
            if key == 'items' and isinstance(value, dict):
                # If nested items, ensure type is present
                if 'items' in value and 'type' not in value:
                    # Try to infer type or default to 'array'
                    value = {**value, 'type': 'array'}
                new_obj[key] = fix_nested_items(value)
            else:
                new_obj[key] = fix_nested_items(value)
        
        return new_obj
    
    try:
        fixed = fix_nested_items(schema)
        # Remove any remaining problematic fields
        fixed.pop('$defs', None)
        fixed.pop('$ref', None)
        return fixed
    except Exception:
        return None


class GeminiToolFilterWrapper:
    """
    Wrapper around ChatGoogleGenerativeAI that filters incompatible tools
    before they're bound to the LLM.
    """
    def __init__(self, base_llm):
        self.base_llm = base_llm
        self._original_bind_tools = getattr(base_llm, 'bind_tools', None)
        self._original_bind = getattr(base_llm, 'bind', None)
    
    def __getattr__(self, name):
        # Delegate to base_llm for any missing attributes
        return getattr(self.base_llm, name)
    
    def bind_tools(self, tools, **kwargs):
        """
        Filter tools before binding to Gemini.
        This is called by LangChain when tools are bound to the LLM.
        """
        print(f"\n[DEBUG] GeminiToolFilterWrapper.bind_tools CALLED with {len(tools)} tools")
        print(f"[DEBUG] Tool names: {[getattr(t, 'name', 'unknown') for t in tools[:10]]}")
        
        # #region agent log
        debug_log("rosa_robot_controller.py:GeminiToolFilterWrapper.bind_tools", "bind_tools called", {
            "tool_count": len(tools),
            "tool_names": [getattr(t, 'name', 'unknown') for t in tools[:10]]
        }, "N")
        # #endregion
        
        # Filter tools for Gemini compatibility
        compatible_tools = filter_tools_for_gemini(tools)
        
        print(f"[DEBUG] After filtering: {len(compatible_tools)} tools will be bound")
        
        # #region agent log
        debug_log("rosa_robot_controller.py:GeminiToolFilterWrapper.bind_tools", "After filtering", {
            "original_count": len(tools),
            "compatible_count": len(compatible_tools),
            "filtered_count": len(tools) - len(compatible_tools)
        }, "N2")
        # #endregion
        
        # Bind only compatible tools
        if self._original_bind_tools:
            result = self._original_bind_tools(compatible_tools, **kwargs)
            print(f"[DEBUG] bind_tools returned, result type: {type(result)}")
            return result
        else:
            # Fallback: use bind method if bind_tools doesn't exist
            print(f"[DEBUG] Using fallback bind method")
            return self.base_llm.bind(tools=compatible_tools, **kwargs)
    
    def bind(self, **kwargs):
        """
        Intercept bind calls that might include tools.
        """
        # #region agent log
        if 'tools' in kwargs:
            debug_log("rosa_robot_controller.py:GeminiToolFilterWrapper.bind", "bind called with tools", {
                "tool_count": len(kwargs['tools'])
            }, "R")
        # #endregion
        
        if 'tools' in kwargs:
            kwargs['tools'] = filter_tools_for_gemini(kwargs['tools'])
        if self._original_bind:
            return self._original_bind(**kwargs)
        return self.base_llm.bind(**kwargs)
    
    def __call__(self, *args, **kwargs):
        # Delegate all other calls to base_llm
        return self.base_llm(*args, **kwargs)
    
    def invoke(self, *args, **kwargs):
        # CRITICAL: Filter tools if they're in kwargs or bound to the LLM
        print(f"[DEBUG] GeminiToolFilterWrapper.invoke called")
        
        # Check if tools are bound to the LLM
        if hasattr(self.base_llm, 'bound_tools'):
            bound_tools = getattr(self.base_llm, 'bound_tools', [])
            if bound_tools:
                print(f"[DEBUG] Found {len(bound_tools)} bound tools, filtering...")
                filtered = filter_tools_for_gemini(bound_tools)
                # Try to update bound tools
                try:
                    self.base_llm.bound_tools = filtered
                    print(f"[DEBUG] Updated bound_tools to {len(filtered)} tools")
                except:
                    pass
        
        # Also check kwargs for tools
        if 'tools' in kwargs:
            kwargs['tools'] = filter_tools_for_gemini(kwargs['tools'])
        
        return self.base_llm.invoke(*args, **kwargs)
    
    def stream(self, *args, **kwargs):
        # Delegate stream calls
        return self.base_llm.stream(*args, **kwargs)


def filter_tools_for_gemini(tools):
    """
    Filter and adapt tools to be Gemini-compatible.
    Returns list of compatible tools.
    """
    compatible_tools = []
    incompatible_tools = []
    incompatible_count = 0
    
    print(f"\n[DEBUG] Filtering {len(tools)} tools for Gemini compatibility...")
    
    for i, tool in enumerate(tools):
        try:
            tool_name = getattr(tool, 'name', f'tool_{i}')
            # Check if tool has a schema
            if hasattr(tool, 'args_schema'):
                schema = tool.args_schema
                if hasattr(schema, 'schema'):
                    try:
                        schema_dict = schema.schema()
                        # Check for problematic patterns
                        schema_str = json.dumps(schema_dict) if isinstance(schema_dict, dict) else str(schema_dict)
                        
                        # More aggressive checks for problematic patterns
                        has_defs = '$defs' in schema_str or '$ref' in schema_str
                        has_vargs = 'v__args' in schema_str or '"v__args"' in schema_str
                        # Check for nested items.items without type (multiple patterns)
                        has_nested_items = (
                            '"items":{"items"' in schema_str or 
                            '"items": {"items"' in schema_str or
                            '"items":{"items":' in schema_str or
                            'items":{"items":{' in schema_str
                        )
                        # Check for properties with problematic nested structures
                        has_problematic_properties = (
                            '"properties":{"items":{"items"' in schema_str or
                            '"properties": {"items": {"items"' in schema_str
                        )
                        
                        if has_defs or has_vargs or has_nested_items or has_problematic_properties:
                            incompatible_tools.append({
                                'name': tool_name,
                                'index': i,
                                'has_defs': has_defs,
                                'has_vargs': has_vargs,
                                'has_nested_items': has_nested_items,
                                'has_problematic_properties': has_problematic_properties
                            })
                            incompatible_count += 1
                            print(f"[DEBUG] FILTERED tool {i}: {tool_name} - defs={has_defs}, vargs={has_vargs}, nested={has_nested_items}, props={has_problematic_properties}")
                            continue
                        
                        # Also check with our compatibility function
                        if not is_gemini_compatible_schema(schema_dict):
                            incompatible_tools.append({
                                'name': tool_name,
                                'index': i,
                                'reason': 'Failed compatibility check'
                            })
                            incompatible_count += 1
                            print(f"[DEBUG] FILTERED tool {i}: {tool_name} - failed compatibility check")
                            continue
                    except Exception as schema_error:
                        print(f"[DEBUG] Error checking schema for tool {i} ({tool_name}): {schema_error}")
                        incompatible_count += 1
                        continue
            
            compatible_tools.append(tool)
            if i < 10:  # Print first 10 compatible tools
                print(f"[DEBUG] KEPT tool {i}: {tool_name}")
        except Exception as e:
            incompatible_count += 1
            print(f"[DEBUG] ERROR processing tool {i}: {e}")
            continue
    
    print(f"[DEBUG] Filtering result: {len(compatible_tools)}/{len(tools)} tools kept, {incompatible_count} filtered")
    if incompatible_tools:
        print(f"[DEBUG] Filtered tools: {[t['name'] for t in incompatible_tools[:10]]}")
    
    debug_log("rosa_robot_controller.py:filter_tools", "Tool filtering complete", {
        "original_count": len(tools),
        "compatible_count": len(compatible_tools),
        "filtered_count": incompatible_count,
        "incompatible_tools": incompatible_tools[:20]  # First 20 for debugging
    }, "M")
    
    return compatible_tools

# region agent log
def sanitize_schema_for_gemini(schema):
    """
    Gemini requires explicit types for array items and rejects LangChain artifacts like v__args.
    This sanitizer is a safe fallback to patch schemas rather than failing the entire request.
    """
    if not isinstance(schema, dict):
        return schema

    # Remove v__args if present
    props = schema.get("properties")
    if isinstance(props, dict) and "v__args" in props:
        props.pop("v__args", None)
        req = schema.get("required")
        if isinstance(req, list) and "v__args" in req:
            schema["required"] = [r for r in req if r != "v__args"]

    # Ensure arrays have typed items
    if schema.get("type") == "array":
        items = schema.get("items")
        if not isinstance(items, dict) or not items:
            schema["items"] = {"type": "string"}
        else:
            schema["items"] = sanitize_schema_for_gemini(items)

    # Recurse
    if isinstance(props, dict):
        for k, v in list(props.items()):
            props[k] = sanitize_schema_for_gemini(v)

    if "items" in schema and isinstance(schema["items"], dict):
        schema["items"] = sanitize_schema_for_gemini(schema["items"])

    return schema
# endregion

# ROSA imports (must be in ROSA venv)
try:
    from rosa import ROSA
    from langchain.tools import tool
except ImportError as e:
    # region agent log
    debug_log("rosa_robot_controller.py:imports", "ROSA/LangChain import failed", {
        "error_type": type(e).__name__,
        "error_message": str(e),
        "hint": "If missing provider package: pip install langchain-google-genai (gemini) or langchain-openai (openai)",
    }, "H8")
    # endregion
    print("Error: ROSA or LangChain not found!")
    print("Make sure you're in the ROSA virtual environment:")
    print("  source /opt/rosa-venv/bin/activate")
    print(f"\nImport error: {e}")
    sys.exit(1)

# region agent log
# Global store for "known-good" robot control tools, used to force Gemini tool list if needed.
_GEMINI_CUSTOM_TOOLS = None
_GEMINI_PATCHED = False

def _maybe_patch_gemini_class():
    """Patch ChatGoogleGenerativeAI methods at the class level so clones/bound instances keep instrumentation."""
    global _GEMINI_PATCHED
    if _GEMINI_PATCHED:
        return
    _GEMINI_PATCHED = True
    try:
        # Import lazily so OpenAI runs do not require Gemini deps installed.
        from langchain_google_genai import ChatGoogleGenerativeAI  # type: ignore
        cls = ChatGoogleGenerativeAI

        # Patch bind_tools to observe/override tool lists at bind time (often where schema conversion happens).
        if hasattr(cls, "bind_tools"):
            _orig_bind_tools = cls.bind_tools
            def _patched_bind_tools(self, tools, **kwargs):
                # #region agent log
                debug_log("rosa_robot_controller.py:gemini_class_bind_tools", "ChatGoogleGenerativeAI.bind_tools called", {
                    "tool_count": len(tools) if tools is not None else None,
                    "tool_names_head": [getattr(t, "name", "unknown") for t in (tools or [])[:15]],
                    "force_custom_only": bool(os.getenv("ROSA_GEMINI_FORCE_CUSTOM_TOOLS_ONLY")),
                    "custom_tool_count": len(_GEMINI_CUSTOM_TOOLS) if _GEMINI_CUSTOM_TOOLS else 0,
                }, "H16")
                # #endregion

                if os.getenv("ROSA_GEMINI_FORCE_CUSTOM_TOOLS_ONLY") and _GEMINI_CUSTOM_TOOLS:
                    tools = list(_GEMINI_CUSTOM_TOOLS)
                    # #region agent log
                    debug_log("rosa_robot_controller.py:gemini_class_bind_tools", "Forced tools in bind_tools to custom-only", {
                        "tool_names": [getattr(t, "name", "unknown") for t in tools],
                    }, "H16")
                    # #endregion

                return _orig_bind_tools(self, tools, **kwargs)
            cls.bind_tools = _patched_bind_tools

        # Patch bind as a fallback if tools are passed via bind(tools=...)
        if hasattr(cls, "bind"):
            _orig_bind = cls.bind
            def _patched_bind(self, **kwargs):
                # #region agent log
                debug_log("rosa_robot_controller.py:gemini_class_bind", "ChatGoogleGenerativeAI.bind called", {
                    "kw_keys": list(kwargs.keys())[:25],
                    "has_tools_kw": ("tools" in kwargs),
                    "force_custom_only": bool(os.getenv("ROSA_GEMINI_FORCE_CUSTOM_TOOLS_ONLY")),
                }, "H17")
                # #endregion

                if os.getenv("ROSA_GEMINI_FORCE_CUSTOM_TOOLS_ONLY") and _GEMINI_CUSTOM_TOOLS and "tools" in kwargs:
                    kwargs["tools"] = list(_GEMINI_CUSTOM_TOOLS)
                    # #region agent log
                    debug_log("rosa_robot_controller.py:gemini_class_bind", "Forced tools in bind(tools=...) to custom-only", {
                        "tool_names": [getattr(t, "name", "unknown") for t in kwargs["tools"]],
                    }, "H17")
                    # #endregion

                return _orig_bind(self, **kwargs)
            cls.bind = _patched_bind

        if hasattr(cls, "_generate"):
            _orig_generate = cls._generate
            def _patched_generate(self, messages, stop=None, run_manager=None, **kwargs):
                # #region agent log
                debug_log("rosa_robot_controller.py:gemini_class_generate", "ChatGoogleGenerativeAI._generate called", {
                    "kw_keys": list(kwargs.keys())[:25],
                    "has_tools_kw": ("tools" in kwargs),
                    "has_functions_kw": ("functions" in kwargs),
                    "force_custom_only": bool(os.getenv("ROSA_GEMINI_FORCE_CUSTOM_TOOLS_ONLY")),
                }, "H15")
                # #endregion

                # If tools are passed, optionally force them to the known-good robot tools
                if os.getenv("ROSA_GEMINI_FORCE_CUSTOM_TOOLS_ONLY") and _GEMINI_CUSTOM_TOOLS:
                    if "tools" in kwargs:
                        kwargs["tools"] = list(_GEMINI_CUSTOM_TOOLS)
                    if "functions" in kwargs:
                        # Some paths may use OpenAI-style "functions"
                        kwargs["functions"] = list(_GEMINI_CUSTOM_TOOLS)
                    # #region agent log
                    debug_log("rosa_robot_controller.py:gemini_class_generate", "Forced Gemini tools to custom-only", {
                        "custom_tool_names": [getattr(t, "name", "unknown") for t in _GEMINI_CUSTOM_TOOLS],
                        "custom_tool_count": len(_GEMINI_CUSTOM_TOOLS),
                    }, "H15")
                    # #endregion

                return _orig_generate(self, messages, stop=stop, run_manager=run_manager, **kwargs)
            cls._generate = _patched_generate
    except Exception as e:
        debug_log("rosa_robot_controller.py:gemini_class_generate", "Failed to patch ChatGoogleGenerativeAI class", {
            "error_type": type(e).__name__,
            "error_message": str(e),
        }, "H15")
# endregion


class ROSARobotController:
    """ROSA-based natural language robot controller"""
    
    def __init__(self, llm_provider="gemini", api_key=None):
        """
        Initialize ROSA robot controller
        
        Args:
            llm_provider: "gemini" or "openai" (default: "gemini")
            api_key: API key for the LLM provider (or set as environment variable)
        """
        # Initialize ROS node
        rospy.init_node('rosa_robot_controller', anonymous=True)
        
        # Robot control publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.arm_trajectory_pub = rospy.Publisher('/arm_controller/command', 
                                                   JointTrajectory, queue_size=10)
        
        # Arm action client for MoveIt
        self.arm_action_client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        # Wait for action server
        rospy.loginfo("Waiting for arm controller action server...")
        self.arm_action_client.wait_for_server(timeout=rospy.Duration(5.0))
        
        # Joint state subscriber
        self.joint_states = {}
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, 
                                                 self.joint_state_callback)
        
        # Wait for joint states
        rospy.sleep(1.0)
        
        # Initialize LLM
        self.llm = self._setup_llm(llm_provider, api_key)
        
        # Create custom tools for robot control
        custom_tools = self._create_tools()
        self._custom_tools = custom_tools
        # Make available globally for Gemini force-tool mode
        global _GEMINI_CUSTOM_TOOLS
        _GEMINI_CUSTOM_TOOLS = list(custom_tools)
        if str(llm_provider).lower() == "gemini":
            _maybe_patch_gemini_class()
        else:
            # #region agent log
            debug_log("rosa_robot_controller.py:init", "Skipping Gemini class patch (non-gemini provider)", {
                "provider": llm_provider,
            }, "H_SKIP_GEMINI_PATCH")
            # #endregion
        
        # #region agent log
        debug_log("rosa_robot_controller.py:83", "Custom tools created", {
            "tool_count": len(custom_tools),
            "tool_names": [getattr(t, 'name', 'unknown') for t in custom_tools],
            "llm_provider": llm_provider
        }, "A")
        # #endregion
        
        # Initialize ROSA first to get its built-in tools
        rospy.loginfo("Initializing ROSA agent...")
        
        # #region agent log
        debug_log("rosa_robot_controller.py:90", "Before ROSA initialization", {
            "llm_type": type(self.llm).__name__,
            "custom_tools_count": len(custom_tools)
        }, "C")
        # #endregion
        
        try:
            # Initialize ROSA - it will add its built-in tools automatically
            self.agent = ROSA(ros_version=1, llm=self.llm, tools=custom_tools)

            # #region agent log
            # Introspect ROSA internals so we can patch tools in the right place (ROSA versions differ).
            try:
                ex = getattr(self.agent, "agent_executor", None)
                debug_log("rosa_robot_controller.py:rosa_introspect", "ROSA executor introspection", {
                    "has_agent_executor": ex is not None,
                    "executor_type": type(ex).__name__ if ex is not None else None,
                    "executor_has_tools_attr": hasattr(ex, "tools") if ex is not None else False,
                    "executor_dir_has_tools": ("tools" in dir(ex)) if ex is not None else False,
                    "agent_has_tools_attr": hasattr(self.agent, "tools"),
                    "agent_tools_len": (len(getattr(self.agent, "tools", [])) if hasattr(self.agent, "tools") and getattr(self.agent, "tools", None) is not None else None),
                    "agent_dir_has_tools": ("tools" in dir(self.agent)),
                    "has_agent_attr": hasattr(ex, "agent") if ex is not None else False,
                    "has_llm_chain": (hasattr(getattr(ex, "agent", None), "llm_chain") if ex is not None and hasattr(ex, "agent") else False),
                }, "H13")
            except Exception as e:
                debug_log("rosa_robot_controller.py:rosa_introspect", "ROSA executor introspection failed", {
                    "error_type": type(e).__name__,
                    "error_message": str(e),
                }, "H13")
            # #endregion
            
            # For Gemini, we MUST filter tools immediately after initialization
            # because ROSA adds built-in tools that aren't Gemini-compatible
            if llm_provider.lower() == "gemini":
                print("\n[DEBUG] ===== GEMINI MODE: Filtering tools =====")
                rospy.loginfo("Filtering tools for Gemini compatibility...")
                
                # IMPORTANT: Gemini tool schemas are extremely strict.
                # ROSA's built-in tools frequently generate schemas Gemini rejects (v__args, nested items, $defs).
                # For reliable robot control, only bind our small, known-good robot tools.
                if hasattr(self.agent, 'agent_executor'):
                    executor = self.agent.agent_executor
                    if hasattr(executor, 'tools'):
                        current_tools = list(executor.tools)
                        print(f"[DEBUG] ROSA executor currently has {len(current_tools)} total tools")
                        print(f"[DEBUG] First 10 tool names: {[getattr(t, 'name', 'unknown') for t in current_tools[:10]]}")

                        # Bind only our robot control tools (and still run them through the filter defensively)
                        compatible_tools = filter_tools_for_gemini(list(custom_tools))
                        print(f"[DEBUG] Binding ONLY custom robot tools for Gemini: {len(compatible_tools)} tools")
                        print(f"[DEBUG] Custom tool names: {[getattr(t, 'name', 'unknown') for t in compatible_tools]}")
                        
                        # #region agent log
                        debug_log("rosa_robot_controller.py:gemini_tools", "Gemini tool binding decision", {
                            "executor_tool_count_before": len(current_tools),
                            "custom_tools_count": len(custom_tools),
                            "bound_tools_count": len(compatible_tools),
                            "bound_tool_names": [getattr(t, 'name', 'unknown') for t in compatible_tools],
                        }, "H10")
                        # #endregion
                        
                        # CRITICAL: Replace tools in executor
                        executor.tools = compatible_tools
                        print(f"[DEBUG] Updated executor.tools to {len(executor.tools)} tools")
                        
                        # Also update agent's tools if it has them
                        if hasattr(self.agent, 'tools'):
                            self.agent.tools = compatible_tools
                        
                        # Monkey-patch the LLM chain to ensure tools are filtered on bind
                        if hasattr(executor, 'agent') and hasattr(executor.agent, 'llm_chain'):
                            llm_chain = executor.agent.llm_chain
                            if hasattr(llm_chain, 'llm'):
                                # Store original methods
                                original_llm = llm_chain.llm
                                
                                # Create a wrapper that filters tools
                                if not isinstance(original_llm, GeminiToolFilterWrapper):
                                    llm_chain.llm = GeminiToolFilterWrapper(original_llm)
                                    rospy.loginfo("Wrapped LLM with tool filter")
                        
                        # CRITICAL: Also monkey-patch the executor's invoke to filter tools
                        # This ensures tools are filtered even if binding happens during invoke
                        original_invoke = executor.invoke
                        def filtered_invoke(inputs, config=None, **kwargs):
                            print(f"\n[DEBUG] executor.invoke CALLED")
                            # Ensure tools are filtered before invoke
                            if hasattr(executor, 'tools'):
                                print(f"[DEBUG] Executor has {len(executor.tools)} tools before filtering")
                                executor.tools = filter_tools_for_gemini(executor.tools)
                                print(f"[DEBUG] Executor now has {len(executor.tools)} tools after filtering")
                            
                            # Also check agent's LLM chain
                            if hasattr(executor, 'agent') and hasattr(executor.agent, 'llm_chain'):
                                llm_chain = executor.agent.llm_chain
                                if hasattr(llm_chain, 'llm'):
                                    llm = llm_chain.llm
                                    print(f"[DEBUG] LLM type: {type(llm)}")
                                    # Check if LLM has bound tools
                                    if hasattr(llm, 'bound_tools'):
                                        print(f"[DEBUG] LLM has bound_tools attribute")
                            
                            return original_invoke(inputs, config=config, **kwargs)
                        executor.invoke = filtered_invoke
                        rospy.loginfo("Patched executor.invoke to filter tools")
            
            # #region agent log
            debug_log("rosa_robot_controller.py:95", "ROSA initialized successfully", {
                "has_agent": hasattr(self, 'agent'),
                "llm_provider": llm_provider
            }, "D")
            # #endregion
            
        except Exception as e:
            # #region agent log
            debug_log("rosa_robot_controller.py:115", "ROSA initialization failed", {
                "error_type": type(e).__name__,
                "error_message": str(e),
                "error_traceback": str(sys.exc_info())
            }, "F")
            # #endregion
            raise
        
        rospy.loginfo("ROSA Robot Controller initialized!")
        rospy.loginfo("You can now control the robot using natural language.")
    
    def _setup_llm(self, provider, api_key=None):
        """Setup LLM based on provider"""
        # #region agent log
        debug_log("rosa_robot_controller.py:_setup_llm", "LLM setup entry", {
            "provider": provider,
            "api_key_provided": bool(api_key),
            "env_has_openai": bool(os.getenv("OPENAI_API_KEY")),
            "env_has_google": bool(os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_GENAI_API_KEY")),
            "env_has_azure_endpoint": bool(os.getenv("AZURE_OPENAI_ENDPOINT")),
            "env_has_azure_deployment": bool(os.getenv("AZURE_OPENAI_DEPLOYMENT_NAME")),
        }, "H1")
        # #endregion

        if provider.lower() == "openai":
            # Import lazily so Gemini deps are not required for OpenAI runs.
            try:
                from langchain_openai import ChatOpenAI  # type: ignore
            except ImportError as e:
                # #region agent log
                debug_log("rosa_robot_controller.py:_setup_llm", "Missing OpenAI provider dependency", {
                    "provider": provider,
                    "error_message": str(e),
                }, "H_OPENAI_IMPORT")
                # #endregion
                raise ImportError("Missing dependency for OpenAI provider. Install: pip install langchain-openai") from e

            api_key = api_key or os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OpenAI API key required. Set OPENAI_API_KEY environment variable.")

            model_name = os.getenv("OPENAI_MODEL", "gpt-4o-mini")
            
            # Check if Azure OpenAI (has endpoint and deployment)
            azure_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT")
            deployment_name = os.getenv("AZURE_OPENAI_DEPLOYMENT_NAME")
            api_version = os.getenv("AZURE_OPENAI_API_VERSION", "2024-02-15-preview")
            
            print(f"[DEBUG] Azure OpenAI check: endpoint={azure_endpoint}, deployment={deployment_name}")
            
            if azure_endpoint and deployment_name:
                # Clean up endpoint (remove trailing slash if present)
                azure_endpoint = azure_endpoint.rstrip('/')
                
                print(f"[DEBUG] Using Azure OpenAI: endpoint={azure_endpoint}, deployment={deployment_name}, api_version={api_version}")
                
                # Set environment variables for Azure OpenAI
                # langchain-openai reads these environment variables for Azure configuration
                os.environ['OPENAI_API_TYPE'] = 'azure'
                os.environ['OPENAI_API_BASE'] = azure_endpoint
                os.environ['OPENAI_API_VERSION'] = api_version
                os.environ['OPENAI_API_KEY'] = api_key
                os.environ['OPENAI_DEPLOYMENT_NAME'] = deployment_name
                
                # Use ChatOpenAI - it will read Azure config from environment variables
                # For Azure, the model parameter is the deployment name
                return ChatOpenAI(
                    model=deployment_name,
                    temperature=0,
                    openai_api_key=api_key
                )
            else:
                # Use standard OpenAI
                print(f"[DEBUG] Using standard OpenAI (no Azure endpoint/deployment found)")
                llm = ChatOpenAI(model=model_name, temperature=0, openai_api_key=api_key)
                # #region agent log
                debug_log("rosa_robot_controller.py:_setup_llm", "Created standard OpenAI LLM", {
                    "llm_class": type(llm).__name__,
                    "model": model_name
                }, "H3")
                # #endregion
                return llm
        
        elif provider.lower() == "gemini":
            # Import lazily so OpenAI deps are not required for Gemini runs.
            try:
                from langchain_google_genai import ChatGoogleGenerativeAI  # type: ignore
            except ImportError as e:
                # #region agent log
                debug_log("rosa_robot_controller.py:_setup_llm", "Missing Gemini provider dependency", {
                    "provider": provider,
                    "error_message": str(e),
                }, "H_GEMINI_IMPORT")
                # #endregion
                raise ImportError("Missing dependency for Gemini provider. Install: pip install langchain-google-genai") from e

            api_key = api_key or os.getenv("GOOGLE_API_KEY")
            print(f"[DEBUG] Checking for GOOGLE_API_KEY: found={bool(api_key)}, length={len(api_key) if api_key else 0}")
            if not api_key:
                # Try alternative env var names
                api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_GENAI_API_KEY")
                print(f"[DEBUG] Tried alternative names: found={bool(api_key)}")
            if not api_key:
                raise ValueError("Google API key required. Set GOOGLE_API_KEY environment variable.")

            # #region agent log
            debug_log("rosa_robot_controller.py:_setup_llm", "Gemini API key resolved", {
                "used_env_var": "GOOGLE_API_KEY" if os.getenv("GOOGLE_API_KEY") else ("GEMINI_API_KEY" if os.getenv("GEMINI_API_KEY") else ("GOOGLE_GENAI_API_KEY" if os.getenv("GOOGLE_GENAI_API_KEY") else "arg")),
                "model": "gemini-2.5-flash"
            }, "H2")
            # #endregion
            
            # Create Gemini LLM with tool filtering wrapper
            base_llm = ChatGoogleGenerativeAI(model="gemini-2.5-flash", temperature=0, google_api_key=api_key)
            # #region agent log
            debug_log("rosa_robot_controller.py:_setup_llm", "Created Gemini LLM", {
                "llm_class": type(base_llm).__name__,
                "has_generate": hasattr(base_llm, "_generate"),
                "has_invoke": hasattr(base_llm, "invoke"),
                "has_bind_tools": hasattr(base_llm, "bind_tools"),
                "has_bind": hasattr(base_llm, "bind"),
                "has_convert_tools": hasattr(base_llm, "_convert_tools_to_gemini_format"),
            }, "H4")
            # #endregion
            
            # CRITICAL: Monkey-patch multiple methods that might handle tools
            try:
                # Patch _generate method which is where the actual API call happens
                if hasattr(base_llm, '_generate'):
                    original_generate = base_llm._generate
                    def filtered_generate(messages, stop=None, run_manager=None, **kwargs):
                        # #region agent log
                        debug_log("rosa_robot_controller.py:gemini_generate", "_generate entry", {
                            "kw_keys": list(kwargs.keys())[:25],
                            "has_tools_kw": ("tools" in kwargs),
                            "has_functions_kw": ("functions" in kwargs),
                        }, "H11")
                        # #endregion
                        print(f"[DEBUG] _generate called, checking for tools in kwargs...")
                        # Tools might be in kwargs or in the LLM's bound_tools
                        if 'tools' in kwargs:
                            print(f"[DEBUG] Found tools in kwargs: {len(kwargs['tools'])}")
                            # #region agent log
                            debug_log("rosa_robot_controller.py:gemini_generate", "Gemini _generate received tools", {
                                "tool_count": len(kwargs["tools"]),
                                "tool_names_head": [getattr(t, "name", "unknown") for t in kwargs["tools"][:15]],
                            }, "H11")
                            # #endregion

                            filtered_tools = filter_tools_for_gemini(kwargs['tools'])
                            kwargs['tools'] = filtered_tools

                            # Try to sanitize any schemas on remaining tools (best-effort)
                            for t in kwargs["tools"]:
                                try:
                                    if hasattr(t, "args_schema") and hasattr(t.args_schema, "schema"):
                                        d = t.args_schema.schema()
                                        sanitize_schema_for_gemini(d)
                                except Exception:
                                    pass

                            # #region agent log
                            debug_log("rosa_robot_controller.py:gemini_generate", "Gemini _generate tools after filtering", {
                                "tool_count": len(kwargs["tools"]),
                                "tool_names": [getattr(t, "name", "unknown") for t in kwargs["tools"]],
                            }, "H11")
                            # #endregion
                        return original_generate(messages, stop=stop, run_manager=run_manager, **kwargs)
                    base_llm._generate = filtered_generate
                    print("[DEBUG] Patched _generate method")
                
                # Also try _convert_tools_to_gemini_format if it exists
                if hasattr(base_llm, '_convert_tools_to_gemini_format'):
                    original_convert = base_llm._convert_tools_to_gemini_format
                    def filtered_convert_tools(tools):
                        print(f"[DEBUG] _convert_tools_to_gemini_format called with {len(tools)} tools")
                        # #region agent log
                        debug_log("rosa_robot_controller.py:gemini_convert_tools", "Gemini convert_tools received tools", {
                            "tool_count": len(tools),
                            "tool_names_head": [getattr(t, "name", "unknown") for t in tools[:15]],
                        }, "H12")
                        # #endregion

                        filtered = filter_tools_for_gemini(tools)
                        print(f"[DEBUG] Converting {len(filtered)} filtered tools to Gemini format")
                        # #region agent log
                        debug_log("rosa_robot_controller.py:gemini_convert_tools", "Gemini convert_tools after filtering", {
                            "tool_count": len(filtered),
                            "tool_names": [getattr(t, "name", "unknown") for t in filtered],
                        }, "H12")
                        # #endregion
                        return original_convert(filtered)
                    base_llm._convert_tools_to_gemini_format = filtered_convert_tools
                    print("[DEBUG] Patched _convert_tools_to_gemini_format")
            except Exception as e:
                print(f"[DEBUG] Could not patch methods: {e}")
                import traceback
                traceback.print_exc()
            
            return GeminiToolFilterWrapper(base_llm)
        
        else:
            raise ValueError(f"Unknown LLM provider: {provider}. Use 'openai' or 'gemini'")
    
    def joint_state_callback(self, msg):
        """Update joint states"""
        for i, name in enumerate(msg.name):
            self.joint_states[name] = msg.position[i] if i < len(msg.position) else 0.0

    def _publish_twist_for(self, twist: Twist, duration_s: float, rate_hz: float = 10.0) -> None:
        """Publish a Twist repeatedly for a duration, then publish a stop Twist once."""
        duration_s = max(0.0, float(duration_s))
        rate_hz = max(1.0, float(rate_hz))
        end_time = rospy.Time.now() + rospy.Duration.from_sec(duration_s)

        # #region agent log
        debug_log("rosa_robot_controller.py:_publish_twist_for", "Publishing twist for duration", {
            "duration_s": duration_s,
            "rate_hz": rate_hz,
            "linear_x": float(getattr(twist.linear, "x", 0.0)),
            "angular_z": float(getattr(twist.angular, "z", 0.0)),
        }, "H20")
        # #endregion

        r = rospy.Rate(rate_hz)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(twist)
            r.sleep()

        # Stop once at the end
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        # #region agent log
        debug_log("rosa_robot_controller.py:_publish_twist_for", "Published stop twist", {}, "H20")
        # #endregion
    
    def move_base_forward(self, speed: float = 0.3, duration_s: float = 1.0) -> str:
        """Move the mobile base forward at the specified speed (0.0 to 1.0).
        
        Args:
            speed: Forward speed (default: 0.3 m/s)
            duration_s: How long to move before auto-stopping (default: 1.0s)
        """
        cmd = Twist()
        cmd.linear.x = max(0.0, min(1.0, float(speed)))
        self._publish_twist_for(cmd, duration_s=duration_s)
        return f"Moving forward at {speed} m/s for {duration_s}s"
    
    def move_base_backward(self, speed: float = 0.3, duration_s: float = 1.0) -> str:
        """Move the mobile base backward at the specified speed (0.0 to 1.0).
        
        Args:
            speed: Backward speed (default: 0.3 m/s)
            duration_s: How long to move before auto-stopping (default: 1.0s)
        """
        cmd = Twist()
        cmd.linear.x = -max(0.0, min(1.0, float(speed)))
        self._publish_twist_for(cmd, duration_s=duration_s)
        return f"Moving backward at {speed} m/s for {duration_s}s"
    
    def turn_base_left(self, angular_speed: float = 0.5, duration_s: float = 1.0) -> str:
        """Turn the mobile base left (counter-clockwise) at the specified angular speed.
        
        Args:
            angular_speed: Angular speed in rad/s (default: 0.5)
            duration_s: How long to turn before auto-stopping (default: 1.0s)
        """
        cmd = Twist()
        cmd.angular.z = max(0.0, min(1.0, float(angular_speed)))
        self._publish_twist_for(cmd, duration_s=duration_s)
        return f"Turning left at {angular_speed} rad/s for {duration_s}s"
    
    def turn_base_right(self, angular_speed: float = 0.5, duration_s: float = 1.0) -> str:
        """Turn the mobile base right (clockwise) at the specified angular speed.
        
        Args:
            angular_speed: Angular speed in rad/s (default: 0.5)
            duration_s: How long to turn before auto-stopping (default: 1.0s)
        """
        cmd = Twist()
        cmd.angular.z = -max(0.0, min(1.0, float(angular_speed)))
        self._publish_twist_for(cmd, duration_s=duration_s)
        return f"Turning right at {angular_speed} rad/s for {duration_s}s"
    
    def stop_base(self) -> str:
        """Stop the mobile base immediately."""
        cmd = Twist()
        # Publish stop a few times to ensure the controller receives it.
        self._publish_twist_for(cmd, duration_s=0.2, rate_hz=10.0)
        return "Stopped the mobile base"
    
    def get_robot_state(self) -> str:
        """Get the current state of the robot including joint positions."""
        if not self.joint_states:
            return "No joint state information available yet"
        
        state_info = "Robot State:\n"
        for joint_name, position in self.joint_states.items():
            state_info += f"  {joint_name}: {position:.3f} rad\n"
        
        return state_info
    
    def move_arm_to_position(self, joint_positions: dict) -> str:
        """Move the arm to specified joint positions.
        
        Args:
            joint_positions: Dictionary of joint names to target positions (in radians)
                Example: {"joint2_to_joint1": 0.5, "joint3_to_joint2": -0.3}
        """
        # Get arm joint names
        arm_joints = [name for name in self.joint_states.keys() 
                     if 'joint' in name.lower() and 'arm' not in name.lower()]
        
        if not arm_joints:
            return "No arm joints found"
        
        # Create trajectory
        traj = JointTrajectory()
        traj.joint_names = arm_joints
        traj.header.stamp = rospy.Time.now()
        
        point = JointTrajectoryPoint()
        point.positions = [joint_positions.get(name, self.joint_states.get(name, 0.0)) 
                          for name in arm_joints]
        point.velocities = [0.0] * len(arm_joints)
        point.time_from_start = rospy.Duration(2.0)  # 2 second duration
        
        traj.points = [point]
        self.arm_trajectory_pub.publish(traj)
        
        return f"Moving arm to positions: {joint_positions}"
    
    def _create_tools(self):
        """Create LangChain tools from controller methods"""
        from langchain.tools import tool
        
        # Create wrapper functions that capture self
        controller = self
        
        @tool
        def move_base_forward(speed: float = 0.3, duration_s: float = 1.0) -> str:
            """Move the mobile base forward at speed for duration_s seconds, then stop.
            
            Notes:
            - If the user doesn't specify speed/duration_s, use the defaults (speed=0.3, duration_s=1.0) without asking follow-up questions.
            - speed is clamped to [0.0, 1.0].
            """
            return controller.move_base_forward(speed, duration_s)
        
        @tool
        def move_base_backward(speed: float = 0.3, duration_s: float = 1.0) -> str:
            """Move the mobile base backward at speed for duration_s seconds, then stop.
            
            Notes:
            - If the user doesn't specify speed/duration_s, use the defaults (speed=0.3, duration_s=1.0) without asking follow-up questions.
            - speed is clamped to [0.0, 1.0].
            """
            return controller.move_base_backward(speed, duration_s)
        
        @tool
        def turn_base_left(angular_speed: float = 0.5, duration_s: float = 1.0) -> str:
            """Turn the mobile base left (counter-clockwise) at angular_speed for duration_s seconds, then stop.
            
            Notes:
            - If the user doesn't specify angular_speed/duration_s, use the defaults (angular_speed=0.5, duration_s=1.0) without asking follow-up questions.
            - angular_speed is clamped to [0.0, 1.0].
            """
            return controller.turn_base_left(angular_speed, duration_s)
        
        @tool
        def turn_base_right(angular_speed: float = 0.5, duration_s: float = 1.0) -> str:
            """Turn the mobile base right (clockwise) at angular_speed for duration_s seconds, then stop.
            
            Notes:
            - If the user doesn't specify angular_speed/duration_s, use the defaults (angular_speed=0.5, duration_s=1.0) without asking follow-up questions.
            - angular_speed is clamped to [0.0, 1.0].
            """
            return controller.turn_base_right(angular_speed, duration_s)
        
        @tool
        def stop_base() -> str:
            """Stop the mobile base immediately."""
            return controller.stop_base()
        
        @tool
        def get_robot_state() -> str:
            """Get the current state of the robot including joint positions."""
            return controller.get_robot_state()
        
        @tool
        def move_arm_to_position(joint_name: str, position: float) -> str:
            """Move a single arm joint to a specified position.
            
            Args:
                joint_name: Name of the joint (e.g., 'joint2_to_joint1')
                position: Target position in radians
            """
            return controller.move_arm_to_position({joint_name: position})
        
        return [
            move_base_forward,
            move_base_backward,
            turn_base_left,
            turn_base_right,
            stop_base,
            get_robot_state,
            move_arm_to_position,
        ]
    
    def run_interactive(self):
        """Run interactive ROSA command loop"""
        print("\n" + "="*70)
        print("ROSA Robot Controller - Natural Language Control")
        print("="*70)
        print("\nThe robot is ready! You can control it using natural language.")
        print("\nExample commands:")
        print("  - 'Move the robot forward' (defaults: speed=0.3 for 1s)")
        print("  - 'Move forward at 0.2 for 3 seconds'")
        print("  - 'Turn left' (defaults: angular_speed=0.5 for 1s)")
        print("  - 'Turn right at 0.7 for 2 seconds'")
        print("  - 'Stop the robot' (immediate)")
        print("  - 'What is the current state of the robot?'")
        print("  - 'Move the arm to a neutral position'")
        print("\nType 'quit' or 'exit' to stop.")
        print("="*70 + "\n")
        
        while not rospy.is_shutdown():
            try:
                user_input = input("\nYou: ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("\nStopping ROSA controller...")
                    break
                
                if not user_input:
                    continue
                
                # Use ROSA to process the command
                print("\nProcessing command...")
                
                # #region agent log
                debug_log("rosa_robot_controller.py:296", "Before agent.invoke", {
                    "user_input": user_input,
                    "llm_type": type(self.llm).__name__
                }, "G")
                
                # For Gemini, ensure tools are filtered one more time before invoke
                if hasattr(self, 'agent') and hasattr(self.agent, 'agent_executor'):
                    executor = self.agent.agent_executor
                    if hasattr(executor, 'tools'):
                        current_tools = list(executor.tools)
                        filtered = filter_tools_for_gemini(current_tools)
                        if len(filtered) < len(current_tools):
                            executor.tools = filtered
                            debug_log("rosa_robot_controller.py:305", "Re-filtered tools before invoke", {
                                "before": len(current_tools),
                                "after": len(filtered)
                            }, "S")
                # #endregion
                
                try:
                    # Encourage the LLM to use tool defaults instead of asking follow-up questions
                    llm_instruction = (
                        "Instruction: When calling tools, if the user doesn't specify optional parameters "
                        "(like speed, angular_speed, duration_s), use the tool defaults without asking follow-up questions.\n"
                        "User: "
                    )
                    response = self.agent.invoke(llm_instruction + user_input)
                    
                    # #region agent log
                    debug_log("rosa_robot_controller.py:304", "After agent.invoke success", {
                        "response_length": len(str(response)) if response else 0,
                        "response_head": (str(response)[:180] if response else ""),
                        "looks_like_gemini_tool_schema_error": ("Invalid argument provided to Gemini" in str(response)) if response else False
                    }, "H")
                    # #endregion
                    
                    print(f"\nROSA: {response}")
                except Exception as invoke_error:
                    # #region agent log
                    debug_log("rosa_robot_controller.py:310", "agent.invoke failed", {
                        "error_type": type(invoke_error).__name__,
                        "error_message": str(invoke_error),
                        "error_traceback": str(sys.exc_info())
                    }, "I")
                    # #endregion
                    raise
                
            except KeyboardInterrupt:
                print("\n\nStopping ROSA controller...")
                break
            except Exception as e:
                print(f"\nError: {e}")
                rospy.logerr(f"Error processing command: {e}")
        
        # Stop robot before exiting
        self.stop_base()
        print("\nRobot controller stopped.")


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='ROSA Robot Controller')
    parser.add_argument('--provider', choices=['openai', 'gemini'], 
                       default='gemini', help='LLM provider (default: gemini)')
    parser.add_argument('--api-key', type=str, help='API key (or set environment variable)')
    parser.add_argument('--azure-endpoint', type=str, help='Azure OpenAI endpoint (or set AZURE_OPENAI_ENDPOINT)')
    parser.add_argument('--azure-deployment', type=str, help='Azure OpenAI deployment name (or set AZURE_OPENAI_DEPLOYMENT_NAME)')
    parser.add_argument('--azure-api-version', type=str, default='2024-02-15-preview', 
                       help='Azure OpenAI API version (default: 2024-02-15-preview)')
    
    args = parser.parse_args()
    
    try:
        # Set Azure OpenAI environment variables if provided via command line
        if args.azure_endpoint:
            os.environ['AZURE_OPENAI_ENDPOINT'] = args.azure_endpoint
        if args.azure_deployment:
            os.environ['AZURE_OPENAI_DEPLOYMENT_NAME'] = args.azure_deployment
        if args.azure_api_version:
            os.environ['AZURE_OPENAI_API_VERSION'] = args.azure_api_version
        
        # Create controller instance
        controller = ROSARobotController(llm_provider=args.provider, api_key=args.api_key)
        
        # Run interactive loop
        controller.run_interactive()
        
    except ValueError as e:
        print(f"\nConfiguration Error: {e}")
        print("\nTo use OpenAI (standard):")
        print("  export OPENAI_API_KEY='your-api-key'")
        print("  python3 rosa_robot_controller.py --provider openai")
        print("\nTo use Azure OpenAI:")
        print("  export OPENAI_API_KEY='your-azure-key'")
        print("  export AZURE_OPENAI_ENDPOINT='https://your-resource.openai.azure.com/'")
        print("  export AZURE_OPENAI_DEPLOYMENT_NAME='your-deployment-name'")
        print("  python3 rosa_robot_controller.py --provider openai")
        print("\nTo use Google Gemini:")
        print("  export GOOGLE_API_KEY='your-api-key'")
        print("  python3 rosa_robot_controller.py --provider gemini")
        sys.exit(1)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

