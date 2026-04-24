import os
import json
import re
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from rosa import ROSA

load_dotenv()
os.environ["OPENAI_API_KEY"] = os.getenv("OPENAI_API_KEY")


def _sanitize_ros_environment() -> None:
    """Remove common ROS2 env vars that break ROS1 service clients."""
    # In mixed shells, ROS1 topic tools may appear to work while service calls fail.
    # Drop ROS2-specific variables before importing rospy-based tools.
    for key in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "ROS_DOMAIN_ID"):
        if key in os.environ:
            os.environ.pop(key, None)


def _validate_ros_environment() -> None:
    """Fail fast when ROS1 networking is not configured."""
    master = os.getenv("ROS_MASTER_URI", "").strip()
    ros_distro = os.getenv("ROS_DISTRO", "").strip()

    if not master:
        raise RuntimeError(
            "ROS_MASTER_URI is not set. Start with scripts/run_rosa_with_ros.sh "
            "or export ROS_MASTER_URI=http://<robot_ip>:11311."
        )

    ros_ip = os.getenv("ROS_IP", "").strip()
    ros_host = os.getenv("ROS_HOSTNAME", "").strip()

    print(
        f"[ROSA ENV] ROS_MASTER_URI={master} ROS_IP={ros_ip or '-'} "
        f"ROS_HOSTNAME={ros_host or '-'} ROS_DISTRO={ros_distro or '-'}",
        flush=True,
    )


_sanitize_ros_environment()

# 1. Importiere dein gesamtes Tool-Modul
# (Passe den Importpfad an, falls deine __init__.py woanders liegt)
import limo_llm_control.tools as robot_tools

_PICKUP_TOOL_NAMES = [
    # base motion (needed to reposition before/after pickup)
    "turn_in_place",
    "drive_distance",
    "is_red_cube_found",
    "snapshot_red_cube",
    "get_latest_red_cube",
    "approach_object",
    "cancel_approach",
    "pick_object",
    "pick_at_pose",
    "drop_at_pose",
    "arm_go_home",
    "place_object",
    "explore_and_fetch_all_cubes",
    "halt_robot",
]


# --- TOOLS DYNAMISCH LADEN + LOGGING FÜR JEDEN AUFRUF ---
def _wrap_with_tool_logging(tool, name):
    """Wrap a LangChain StructuredTool so each call is logged (Pydantic tools forbid assigning .invoke)."""
    orig_func = getattr(tool, "func", None)
    orig_coro = getattr(tool, "coroutine", None)

    def _log(payload_repr: str) -> None:
        msg = f"[ROSA TOOL] {name} invoked with input={payload_repr}"
        print(msg, flush=True)
        try:
            import rospy

            rospy.loginfo(msg)
        except Exception:
            pass

    if orig_func is not None:

        def logged_func(*args, **kwargs):
            payload = kwargs if kwargs else args
            _log(repr(payload))
            return orig_func(*args, **kwargs)

        return tool.model_copy(update={"func": logged_func})

    if orig_coro is not None:

        async def logged_coro(*args, **kwargs):
            payload = kwargs if kwargs else args
            _log(repr(payload))
            return await orig_coro(*args, **kwargs)

        return tool.model_copy(update={"coroutine": logged_coro})

    return tool

tool_profile = os.getenv("ROSA_TOOL_PROFILE", "pickup").strip().lower()
if tool_profile == "all":
    selected_tool_names = list(robot_tools.__all__)
else:
    # Default to minimal pickup flow to avoid unrelated tool interference.
    selected_tool_names = [
        name for name in _PICKUP_TOOL_NAMES if name in set(robot_tools.__all__)
    ]

all_my_tools = []
for tool_name in selected_tool_names:
    t = getattr(robot_tools, tool_name)
    all_my_tools.append(_wrap_with_tool_logging(t, tool_name))
print(f"Lade {len(all_my_tools)} Tools (profile={tool_profile}): {selected_tool_names}")


# --- INITIALIZE LLM & PROMPT ---
llm = ChatOpenAI(model="gpt-4o") 

# --- INITIALIZE ROSA ---
_validate_ros_environment()

agent = ROSA(
    ros_version=1, 
    llm=llm, 
    tools=all_my_tools,  # <--- Hier übergeben wir die dynamisch erzeugte Liste!
)


def _run_startup_healthcheck() -> None:
    """Run autonomy stack health check once at startup."""
    tool = getattr(robot_tools, "healthcheck_autonomy_stack", None)
    if tool is None:
        print("[ROSA HEALTH] healthcheck_autonomy_stack tool not found.", flush=True)
        return
    try:
        raw = tool.invoke({})
    except Exception as exc:
        print(f"[ROSA HEALTH] healthcheck failed to run: {exc}", flush=True)
        return

    try:
        report = json.loads(raw)
    except Exception:
        print(f"[ROSA HEALTH] unexpected output: {raw}", flush=True)
        return

    overall_ready = bool(report.get("overall_ready", False))
    missing_topics = report.get("missing_required_topics", []) or []
    missing_services = report.get("missing_required_services", []) or []

    if overall_ready:
        print("[ROSA HEALTH] READY: required mapping/exploration/perception dependencies are up.", flush=True)
        return

    print(
        "[ROSA HEALTH] NOT READY: missing required dependencies "
        f"(topics={len(missing_topics)}, services={len(missing_services)}).",
        flush=True,
    )
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)


def _call_tool(tool_name: str, **kwargs) -> str:
    tool = getattr(robot_tools, tool_name, None)
    if tool is None:
        return f"FAIL: tool '{tool_name}' not available"
    try:
        if hasattr(tool, "invoke"):
            return str(tool.invoke(kwargs or {}))
        return str(tool(**kwargs))
    except Exception as exc:
        return f"FAIL: {tool_name} raised: {exc}"


def _extract_xyz(text: str):
    x = re.search(r"x\s*=\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
    y = re.search(r"y\s*=\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
    z = re.search(r"z\s*=\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
    if not (x and y and z):
        return None
    return float(x.group(1)), float(y.group(1)), float(z.group(1))


def _extract_detected_xyz(text: str):
    m = re.search(
        r"x\s*=\s*(-?\d+(?:\.\d+)?)\s*,\s*y\s*=\s*(-?\d+(?:\.\d+)?)\s*,\s*z\s*=\s*(-?\d+(?:\.\d+)?)",
        text,
        re.IGNORECASE,
    )
    if not m:
        return None
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


def _extract_home_pose(text: str):
    x = re.search(r"x\s*=\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
    y = re.search(r"y\s*=\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
    yaw = re.search(r"yaw_deg\s*=\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
    if not (x and y and yaw):
        return None
    return float(x.group(1)), float(y.group(1)), float(yaw.group(1))


def _deterministic_red_cube_flow(user_input: str):
    txt = user_input.lower()
    is_scan = "scan" in txt and "red cube" in txt
    is_approach = "approach" in txt and "red cube" in txt
    is_pick = (
        (("pick" in txt) or ("grab" in txt))
        and "red cube" in txt
    )
    is_continuous_scan_grab = (
        ("scan" in txt and ("continuous" in txt or "continuously" in txt))
        and ("grab" in txt or "pick" in txt)
        and "cube" in txt
    )
    is_drop = (
        ("drop" in txt or "place" in txt)
        and ("x=" in txt and "y=" in txt and "z=" in txt)
    )
    is_scan_fetch_all = (
        ("scan" in txt or "explore" in txt)
        and ("all" in txt or "every" in txt)
        and ("red cube" in txt or "red cubes" in txt)
        and ("fetch" in txt or "collect" in txt or "pick" in txt or "grab" in txt)
    )
    if not (is_scan or is_approach or is_pick or is_drop or is_scan_fetch_all):
        return None

    steps = []
    if is_scan_fetch_all:
        home = _call_tool("get_current_map_pose")
        steps.append(f"home_pose: {home}")
        parsed = _extract_home_pose(home)
        if parsed is None:
            steps.append("scan_fetch_all: FAIL: could not parse current home pose.")
            return "\n".join(steps)
        hx, hy, hyaw = parsed
        steps.append(
            "scan_fetch_all: "
            + _call_tool(
                "explore_and_fetch_all_cubes",
                home_x=hx,
                home_y=hy,
                home_yaw_deg=hyaw,
                exploration_duration_s=45.0,
                deduplication_radius_m=0.10,
                max_cubes=20,
            )
        )
        return "\n".join(steps)

    if is_continuous_scan_grab:
        steps.append(f"detector: {_call_tool('enable_red_cube_detector', enabled=True)}")
        max_attempts = 5
        for i in range(max_attempts):
            found = _call_tool("is_red_cube_found", timeout_s=1.0)
            steps.append(f"scan[{i+1}]: {found}")
            if "found=True" not in found:
                continue
            snap = _call_tool("snapshot_red_cube")
            steps.append(f"snapshot[{i+1}]: {snap}")
            latest = _call_tool("get_latest_red_cube", timeout_s=1.0)
            steps.append(f"pose[{i+1}]: {latest}")
            xyz = _extract_detected_xyz(latest)
            if xyz is None:
                continue
            x_m, y_m, z_m = xyz
            pick_res = _call_tool("pick_at_pose", x_m=x_m, y_m=y_m, z_m=z_m, frame_id="map")
            steps.append(f"pick[{i+1}]: {pick_res}")
            if "Pick result: OK:" in pick_res:
                break
        return "\n".join(steps)

    steps.append(f"snapshot: {_call_tool('snapshot_red_cube')}")

    if is_scan and not (is_approach or is_pick):
        return "\n".join(steps)

    if is_approach:
        approach_res = _call_tool("approach_object")
        steps.append(f"approach: {approach_res}")

    if is_pick:
        approach_res = _call_tool("approach_object")
        steps.append(f"approach: {approach_res}")
        xyz = _extract_xyz(user_input)
        if xyz is not None:
            x_m, y_m, z_m = xyz
            steps.append(
                "pick_at_pose: "
                + _call_tool("pick_at_pose", x_m=x_m, y_m=y_m, z_m=z_m, frame_id="map")
            )
        else:
            # Re-read latest detector pose after approach, then pick that explicit pose.
            latest = _call_tool("get_latest_red_cube", timeout_s=1.0)
            steps.append(f"pose: {latest}")
            detected = _extract_detected_xyz(latest)
            if detected is None:
                steps.append(
                    "pick_at_pose: FAIL: no valid detected x/y/z available yet."
                )
            else:
                x_m, y_m, z_m = detected
                steps.append(
                    "pick_at_pose: "
                    + _call_tool(
                        "pick_at_pose",
                        x_m=x_m,
                        y_m=y_m,
                        z_m=z_m,
                        frame_id="map",
                    )
                )
    if is_drop:
        xyz = _extract_xyz(user_input)
        if xyz is None:
            steps.append("drop_at_pose: FAIL: missing x/y/z coordinates")
        else:
            x_m, y_m, z_m = xyz
            steps.append(
                "drop_at_pose: "
                + _call_tool("drop_at_pose", x_m=x_m, y_m=y_m, z_m=z_m, frame_id="map")
            )
    return "\n".join(steps)

# --- MAIN LOOP ---
if __name__ == "__main__":
    print("\n✅ ROSA Limo Agent Ready! Type 'exit' to quit.")
    _run_startup_healthcheck()
    
    while True:
        user_input = input("\nYou: ")
        if user_input.lower() in ["exit", "quit"]:
            break
            
        try:
            forced = _deterministic_red_cube_flow(user_input)
            if forced is not None:
                response = forced
            else:
                response = agent.invoke(user_input)
            print(f"ROSA: {response}")
            
        except Exception as e:
            print(f"Error: {e}")