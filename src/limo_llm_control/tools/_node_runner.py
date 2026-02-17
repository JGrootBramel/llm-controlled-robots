"""
Shared logic for spawning/stopping autonomy node subprocesses and ROS service calls.

Used by navigation, perception, and diagnostics tools. Node scripts live in repo src/
(cam_coverage.py, frontier_planner.py, etc.); this runs them as subprocesses and
tracks state. All ROS communication uses rospy (native ROS1).
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Empty

# src/ directory (parent of limo_llm_control)
_TOOLS_DIR = Path(__file__).resolve().parent
_SRC_DIR = _TOOLS_DIR.parents[1]
LOG_DIR = _SRC_DIR.parent / "logs"

NODE_SCRIPTS: Dict[str, str] = {
    "cam_coverage": "cam_coverage.py",
    "frontier_planner": "frontier_planner.py",
    "straight_planner": "straight_planner.py",
    "object_finder": "object_finder.py",
    "blue_cube_grasper": "pick_cube_blue.py",
}

_PROCESSES: Dict[str, subprocess.Popen] = {}
_PROCESS_META: Dict[str, dict] = {}
_PROCESS_LOG_HANDLES: Dict[str, Any] = {}
_LOCK = threading.Lock()


def _remote_cfg() -> Dict[str, Any]:
    """
    Build remote execution config from environment variables.

    Env vars:
    - LIMO_TOOL_EXEC_MODE: "local" (default) or "ssh"
    - LIMO_ROBOT_HOST: robot hostname/IP for ssh mode
    - LIMO_ROBOT_USER: ssh user (default: current USER)
    - LIMO_ROBOT_PORT: ssh port (default: 22)
    - LIMO_ROBOT_WORKDIR: repo root on robot (default: ~/llm-controlled-robots)
    - LIMO_ROBOT_PYTHON: python interpreter on robot (default: python3)
    - LIMO_ROBOT_DISPLAY: display for GUI debug windows (default: :0)
    - LIMO_ROBOT_XAUTHORITY: optional XAUTHORITY path for GUI forwarding/auth
    """
    mode = os.environ.get("LIMO_TOOL_EXEC_MODE", "local").strip().lower()
    host = os.environ.get("LIMO_ROBOT_HOST", "").strip()
    user = os.environ.get("LIMO_ROBOT_USER", os.environ.get("USER", "")).strip()
    port = int(os.environ.get("LIMO_ROBOT_PORT", "22"))
    workdir = os.environ.get("LIMO_ROBOT_WORKDIR", "~/llm-controlled-robots").strip()
    py = os.environ.get("LIMO_ROBOT_PYTHON", "python3").strip()
    display = os.environ.get("LIMO_ROBOT_DISPLAY", ":0").strip()
    xauth = os.environ.get("LIMO_ROBOT_XAUTHORITY", "").strip()
    return {
        "mode": mode,
        "host": host,
        "user": user,
        "port": port,
        "workdir": workdir,
        "python": py,
        "display": display,
        "xauth": xauth,
    }


def _local_spawn_node(node_key: str, params: Dict[str, object]) -> str:
    """Spawn node locally on the machine where tools run."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    script = _SRC_DIR / NODE_SCRIPTS[node_key]
    if not script.exists():
        return f"Cannot start '{node_key}': script not found at {script}."

    with _LOCK:
        old_proc = _PROCESSES.get(node_key)
        if old_proc is not None and old_proc.poll() is None:
            try:
                old_proc.terminate()
                old_proc.wait(timeout=2.0)
            except Exception:
                old_proc.kill()
            finally:
                _PROCESSES.pop(node_key, None)
                _PROCESS_META.pop(node_key, None)
                old_log = _PROCESS_LOG_HANDLES.pop(node_key, None)
                if old_log is not None:
                    try:
                        old_log.close()
                    except Exception:
                        pass

        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = LOG_DIR / f"{node_key}_{stamp}.log"
        log_handle = open(log_path, "a", encoding="utf-8")

        cmd = [sys.executable, str(script)] + _build_ros_private_param_args(params)
        proc = subprocess.Popen(
            cmd,
            cwd=str(_SRC_DIR),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
        )

        _PROCESSES[node_key] = proc
        _PROCESS_LOG_HANDLES[node_key] = log_handle
        _PROCESS_META[node_key] = {
            "pid": proc.pid,
            "started_unix_s": time.time(),
            "script": str(script),
            "params": params,
            "log_file": str(log_path),
            "cmd": cmd,
            "exec_mode": "local",
        }

    time.sleep(0.2)
    if proc.poll() is not None:
        return (
            f"Failed to start '{node_key}' (exit code {proc.returncode}). "
            f"Check log: {log_path}."
        )

    return f"Started '{node_key}' locally (pid={proc.pid}). Log: {log_path}."


def _ssh_target(cfg: Dict[str, Any]) -> str:
    if cfg["user"]:
        return f"{cfg['user']}@{cfg['host']}"
    return cfg["host"]


def _remote_spawn_node(node_key: str, params: Dict[str, object]) -> str:
    """
    Spawn node via SSH on the robot host.

    The node is detached with nohup on the robot. We track remote PID and a remote log path.
    """
    cfg = _remote_cfg()
    if not cfg["host"]:
        return (
            "Remote exec mode is enabled but LIMO_ROBOT_HOST is not set. "
            "Set LIMO_ROBOT_HOST (and optional LIMO_ROBOT_USER/PORT/WORKDIR)."
        )

    # Stop previous instance (remote) first.
    _ = _remote_stop_node(node_key)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    remote_log_path = f"{cfg['workdir']}/logs/{node_key}_{stamp}.log"
    remote_script = f"{cfg['workdir']}/src/{NODE_SCRIPTS[node_key]}"

    params_args = " ".join(shlex.quote(arg) for arg in _build_ros_private_param_args(params))
    exports = [f"export DISPLAY={shlex.quote(cfg['display'])}"]
    if cfg["xauth"]:
        exports.append(f"export XAUTHORITY={shlex.quote(cfg['xauth'])}")
    export_cmd = " && ".join(exports)

    # Keep shell POSIX-safe; detach and echo pid back.
    remote_cmd = (
        "set -e; "
        f"mkdir -p {shlex.quote(cfg['workdir'])}/logs; "
        f"cd {shlex.quote(cfg['workdir'])}; "
        f"{export_cmd}; "
        f"nohup {shlex.quote(cfg['python'])} {shlex.quote(remote_script)} {params_args} "
        f">> {shlex.quote(remote_log_path)} 2>&1 & echo $!"
    )
    ssh_cmd = [
        "ssh",
        "-p",
        str(cfg["port"]),
        _ssh_target(cfg),
        remote_cmd,
    ]

    try:
        out = subprocess.check_output(ssh_cmd, text=True, stderr=subprocess.STDOUT, timeout=10.0).strip()
        remote_pid = int(out.splitlines()[-1].strip())
    except Exception as exc:
        return f"Failed to start '{node_key}' on robot via ssh: {exc}"

    with _LOCK:
        _PROCESS_META[node_key] = {
            "pid": remote_pid,
            "started_unix_s": time.time(),
            "script": remote_script,
            "params": params,
            "log_file": remote_log_path,
            "cmd": ssh_cmd,
            "exec_mode": "ssh",
            "remote_host": cfg["host"],
            "remote_user": cfg["user"],
            "remote_port": cfg["port"],
        }

    # Quick remote aliveness check.
    alive = _remote_pid_alive(node_key)
    if not alive:
        return (
            f"Failed to verify '{node_key}' on robot after start. "
            f"Check remote log: {remote_log_path}."
        )
    return (
        f"Started '{node_key}' on robot {_ssh_target(cfg)} (pid={remote_pid}). "
        f"Remote log: {remote_log_path}."
    )


def _remote_pid_alive(node_key: str) -> bool:
    with _LOCK:
        meta = _PROCESS_META.get(node_key, {})
    if not meta or meta.get("exec_mode") != "ssh":
        return False

    pid = int(meta["pid"])
    cfg = _remote_cfg()
    target = _ssh_target(cfg)
    ssh_cmd = ["ssh", "-p", str(cfg["port"]), target, f"kill -0 {pid} >/dev/null 2>&1; echo $?"]
    try:
        out = subprocess.check_output(ssh_cmd, text=True, stderr=subprocess.STDOUT, timeout=4.0).strip()
        return out.endswith("0")
    except Exception:
        return False


def _remote_stop_node(node_key: str) -> str:
    with _LOCK:
        meta = _PROCESS_META.get(node_key)
    if not meta or meta.get("exec_mode") != "ssh":
        return f"Node '{node_key}' is not running."

    pid = int(meta["pid"])
    cfg = _remote_cfg()
    if not cfg["host"]:
        return f"Cannot stop '{node_key}' remotely: LIMO_ROBOT_HOST is not set."
    target = _ssh_target(cfg)
    ssh_cmd = [
        "ssh",
        "-p",
        str(cfg["port"]),
        target,
        f"kill {pid} >/dev/null 2>&1 || true",
    ]
    try:
        subprocess.check_output(ssh_cmd, text=True, stderr=subprocess.STDOUT, timeout=6.0)
    except Exception:
        # Continue cleanup even if kill command reports an issue.
        pass

    with _LOCK:
        _PROCESS_META.pop(node_key, None)
    return f"Stopped '{node_key}' on robot (former pid={pid})."


def validate_float(name: str, value: float, low: float, high: float) -> Optional[str]:
    try:
        val = float(value)
    except (TypeError, ValueError):
        return f"Invalid '{name}': expected a numeric value."
    if not (low <= val <= high):
        return f"Invalid '{name}': expected {low} <= {name} <= {high}, got {val}."
    return None


def validate_int(name: str, value: int, low: int, high: int) -> Optional[str]:
    try:
        val = int(value)
    except (TypeError, ValueError):
        return f"Invalid '{name}': expected an integer value."
    if not (low <= val <= high):
        return f"Invalid '{name}': expected {low} <= {name} <= {high}, got {val}."
    return None


def _build_ros_private_param_args(params: Dict[str, object]) -> list:
    args = []
    for key, value in params.items():
        args.append(f"_{key}:={value}")
    return args


def spawn_node(node_key: str, params: Dict[str, object]) -> str:
    """Spawn one autonomy node process; replace existing if already running."""
    if node_key not in NODE_SCRIPTS:
        return f"Unknown node key '{node_key}'. Valid keys: {sorted(NODE_SCRIPTS.keys())}."
    cfg = _remote_cfg()
    if cfg["mode"] == "ssh":
        return _remote_spawn_node(node_key, params)
    return _local_spawn_node(node_key, params)


def stop_node(node_key: str) -> str:
    """Stop one node process if active."""
    with _LOCK:
        meta = _PROCESS_META.get(node_key, {})
    if meta.get("exec_mode") == "ssh":
        return _remote_stop_node(node_key)

    with _LOCK:
        proc = _PROCESSES.get(node_key)
        if proc is None:
            return f"Node '{node_key}' is not running."

        try:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=3.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

        _PROCESSES.pop(node_key, None)
        meta = _PROCESS_META.pop(node_key, None)
        log_handle = _PROCESS_LOG_HANDLES.pop(node_key, None)
        if log_handle is not None:
            try:
                log_handle.close()
            except Exception:
                pass

    pid = meta["pid"] if meta else "?"
    return f"Stopped '{node_key}' (former pid={pid})."


def stop_all_nodes() -> str:
    """Stop all managed autonomy node subprocesses."""
    with _LOCK:
        keys = list(_PROCESSES.keys())
    if not keys:
        return "No managed autonomy nodes are currently running."
    results = [stop_node(k) for k in keys]
    return " | ".join(results)


def call_reset_cam_coverage() -> str:
    """Call /cam_coverage/reset service (rospy)."""
    try:
        rospy.wait_for_service("/cam_coverage/reset", timeout=2.0)
        reset_srv = rospy.ServiceProxy("/cam_coverage/reset", Empty)
        reset_srv()
        return "Coverage reset service call succeeded."
    except Exception as exc:
        return f"Coverage reset failed: {exc}"


def get_managed_processes_snapshot() -> Dict[str, dict]:
    """Return snapshot of managed process state (caller holds or doesn't need lock)."""
    with _LOCK:
        snapshot = {}
        # Local processes
        for node_key, proc in _PROCESSES.items():
            meta = _PROCESS_META.get(node_key, {})
            snapshot[node_key] = {
                "pid": meta.get("pid"),
                "alive": proc.poll() is None,
                "started_unix_s": meta.get("started_unix_s"),
                "log_file": meta.get("log_file"),
                "exec_mode": meta.get("exec_mode", "local"),
            }
        # Remote-only tracked processes
        for node_key, meta in _PROCESS_META.items():
            if meta.get("exec_mode") != "ssh":
                continue
            if node_key not in snapshot:
                snapshot[node_key] = {
                    "pid": meta.get("pid"),
                    "alive": _remote_pid_alive(node_key),
                    "started_unix_s": meta.get("started_unix_s"),
                    "log_file": meta.get("log_file"),
                    "exec_mode": "ssh",
                    "remote_host": meta.get("remote_host"),
                }
        return snapshot


def get_ros_object_state() -> Dict[str, Any]:
    """Query object-related ROS topics via rospy (short timeouts)."""
    state = {
        "object_detection_ready": None,
        "object_found": None,
        "object_approached": None,
        "object_pose": None,
    }
    try:
        msg = rospy.wait_for_message("/object_detection_ready", Bool, timeout=0.4)
        state["object_detection_ready"] = bool(msg.data)
    except Exception:
        pass
    try:
        msg = rospy.wait_for_message("/object_found", Bool, timeout=0.4)
        state["object_found"] = bool(msg.data)
    except Exception:
        pass
    try:
        msg = rospy.wait_for_message("/object_approached", Bool, timeout=0.4)
        state["object_approached"] = bool(msg.data)
    except Exception:
        pass
    try:
        pose_msg = rospy.wait_for_message("/object_pose", PoseStamped, timeout=0.4)
        state["object_pose"] = {
            "frame_id": pose_msg.header.frame_id,
            "x": float(pose_msg.pose.position.x),
            "y": float(pose_msg.pose.position.y),
            "z": float(pose_msg.pose.position.z),
        }
    except Exception:
        pass
    return state


def get_autonomy_status_json() -> str:
    """Return JSON status: managed nodes + ROS object state."""
    from ..ros_clients import ensure_rospy

    ensure_rospy()
    payload = {
        "managed_nodes": get_managed_processes_snapshot(),
        "ros_state": get_ros_object_state(),
    }
    return json.dumps(payload, ensure_ascii=True)
