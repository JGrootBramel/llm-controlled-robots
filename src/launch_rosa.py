#!/usr/bin/env python3
import os
import sys
import traceback
from pathlib import Path
from typing import Optional

# Paths: this file lives at .../src/launch_rosa.py
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
vendor_dir = os.path.join(repo_root, "vendor")

# Ensure repo src/ and vendor/ are on PYTHONPATH so limo_llm_control and tools resolve
if vendor_dir not in sys.path:
    sys.path.insert(0, vendor_dir)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from dotenv import load_dotenv
load_dotenv()

# Central logging (same as main.py): one run-dated log file under logs/ plus console
import logging
from limo_llm_control.logging_config import configure_logging

log_path = configure_logging()
logger = logging.getLogger("limo_llm_control.launch_rosa")
logger.info("Log file for this run: %s", log_path)

# Default ROS master (your physical LIMO cobot)
os.environ.setdefault("ROS_MASTER_URI", "http://192.168.0.151:11311")
ros_master = os.environ.get("ROS_MASTER_URI", "")
ros_ip = os.environ.get("ROS_IP", "") or os.environ.get("ROS_HOSTNAME", "")
logger.info("ROS_MASTER_URI=%s ROS_IP/ROS_HOSTNAME=%s", ros_master or "(not set)", ros_ip or "(not set)")

# Connect to robot ROS master early so /rosout is captured in the same log
from limo_llm_control.ros_clients import ensure_rospy
from limo_llm_control.rosout_bridge import start_rosout_bridge
ensure_rospy()
start_rosout_bridge()

try:
    from langchain_openai import ChatOpenAI
except Exception as e:
    logger.exception("Missing dependency: langchain_openai")
    raise

try:
    from rosa import ROSA
except Exception as e:
    logger.exception("Failed to import rosa package")
    raise

# Tools: re-exported from limo_llm_control.tools
try:
    from tools import (
        turn_in_place,
        get_autonomy_status,
        reset_cam_coverage,
        start_blue_cube_grasper_node,
        start_cam_coverage_node,
        start_frontier_planner_node,
        start_object_finder_node,
        start_straight_planner_node,
        stop_autonomy_nodes,
        update_object_query,
    )
except Exception as e:
    logger.warning("Could not import tools (limo_llm_control.tools): %s", e)
    raise

OPENAI_API_KEY: Optional[str] = os.environ.get("OPENAI_API_KEY")
if not OPENAI_API_KEY:
    logger.error("OPENAI_API_KEY not set")
    raise SystemExit("Please set the OPENAI_API_KEY environment variable before running this script.")

llm = ChatOpenAI(model=os.environ.get("ROSA_MODEL", "gpt-4o"), temperature=0, api_key=OPENAI_API_KEY)

# Instantiate ROSA and tell it to load tools from the local `tools` package
agent = ROSA(
    ros_version=1,
    llm=llm,
    tools=[
        turn_in_place,
        get_autonomy_status,
        reset_cam_coverage,
        start_blue_cube_grasper_node,
        start_cam_coverage_node,
        start_frontier_planner_node,
        start_object_finder_node,
        start_straight_planner_node,
        stop_autonomy_nodes,
        update_object_query,
    ],          # <-- force include your tool
    tool_packages=["tools"],        # optional; keep if you want package discovery too
    blacklist=["rosservice_list"],  # <-- disable the buggy tool
    streaming=False,
    verbose=True,
)

# What tools did ROSA load internally?
print("ROSA __tools type:", type(agent._ROSA__tools))
print("ROSA __tools:", agent._ROSA__tools)
tools_obj = agent._get_tools(
    ros_version=1, 
    packages=["tools"], 
    tools=[
        turn_in_place,
        get_autonomy_status,
        reset_cam_coverage,
        start_blue_cube_grasper_node,
        start_cam_coverage_node,
        start_frontier_planner_node,
        start_object_finder_node,
        start_straight_planner_node,
        stop_autonomy_nodes,
        update_object_query,
    ],
    blacklist=[]
)
print("ROSA _get_tools():", tools_obj)
for attr in ("tools", "_tools", "langchain_tools"):
    if hasattr(tools_obj, attr):
        vals = getattr(tools_obj, attr)
        try:
            print(f"{attr} count:", len(vals))
            for t in vals:
                print(" -", getattr(t, "name", repr(t)))
        except Exception:
            print(f"{attr}:", vals)


def repl():
    logger.info("ROSA agent ready. Type a question (or 'exit' to quit).")
    while True:
        try:
            query = input("You: ").strip()
            if not query:
                continue
            if query.lower() in ("exit", "quit", "q"):
                logger.info("User quit.")
                break
            logger.info("User query: %s", query)
            resp = agent.invoke(query)
            logger.info("ROSA response: %s", resp)
            print("ROSA:", resp)
        except KeyboardInterrupt:
            logger.info("Interrupted by user.")
            print("\nInterrupted. Exiting.")
            break
        except Exception as e:
            logger.exception("Error during invocation: %s", e)
            traceback.print_exc()
            print("Error during invocation:", e)


if __name__ == "__main__":
    repl()
