"""
Main entrypoint for LIMO LLM control (remote PC).

Uses ROSA with tools from limo_llm_control.tools (motion, navigation, perception, diagnostics).
Configure ROS_MASTER_URI and ROS_IP so this process connects to the robot's roscore (rospy).
"""

from __future__ import annotations

import logging
import os
import sys
import traceback
from pathlib import Path

# Ensure repo src/ is on path so "limo_llm_control" resolves
_src = Path(__file__).resolve().parent.parent
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

logger = logging.getLogger(__name__)


def main() -> None:
    from dotenv import load_dotenv

    load_dotenv()

    # Central logging: one run-dated file under logs/ plus console
    from .logging_config import configure_logging

    log_path = configure_logging()
    logger.info("Log file for this run: %s", log_path)

    openai_key = os.environ.get("OPENAI_API_KEY")
    if not openai_key:
        logger.error("OPENAI_API_KEY not set.")
        raise SystemExit("Please set OPENAI_API_KEY.")

    # Connect to robot ROS master early so we can capture /rosout into the same log
    from .ros_clients import ensure_rospy
    from .rosout_bridge import start_rosout_bridge

    ros_master = os.environ.get("ROS_MASTER_URI", "")
    ros_ip = os.environ.get("ROS_IP", "") or os.environ.get("ROS_HOSTNAME", "")
    logger.info("ROS_MASTER_URI=%s ROS_IP/ROS_HOSTNAME=%s", ros_master or "(not set)", ros_ip or "(not set)")
    ensure_rospy()
    start_rosout_bridge()

    from langchain_openai import ChatOpenAI
    from rosa import ROSA

    llm = ChatOpenAI(
        model=os.environ.get("ROSA_MODEL", "gpt-4o"),
        temperature=0,
        api_key=openai_key,
    )

    # Tools from limo_llm_control.tools (rospy-based capability wrappers)
    agent = ROSA(
        ros_version=1,
        llm=llm,
        tool_packages=["limo_llm_control.tools"],
        streaming=False,
        verbose=True,
    )

    logger.info("ROSA agent ready (limo_llm_control tools). Type a question or 'exit' to quit.")
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
            logger.exception("Error during invoke: %s", e)
            traceback.print_exc()
            print("Error:", e)


if __name__ == "__main__":
    main()
