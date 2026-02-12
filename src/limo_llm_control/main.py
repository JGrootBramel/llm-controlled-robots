"""
Main entrypoint for LIMO LLM control (remote PC).

Uses ROSA with tools from limo_llm_control.tools (motion, navigation, perception, diagnostics).
Configure ROS_MASTER_URI and ROS_IP so this process connects to the robot's roscore (rospy).
"""

from __future__ import annotations

import os
import sys
import traceback
from pathlib import Path

# Ensure repo src/ is on path so "limo_llm_control" resolves
_src = Path(__file__).resolve().parent.parent
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))


def main() -> None:
    from dotenv import load_dotenv

    load_dotenv()

    openai_key = os.environ.get("OPENAI_API_KEY")
    if not openai_key:
        raise SystemExit("Please set OPENAI_API_KEY.")

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

    print("ROSA agent ready (limo_llm_control tools). Type a question or 'exit' to quit.")
    while True:
        try:
            query = input("You: ").strip()
            if not query:
                continue
            if query.lower() in ("exit", "quit", "q"):
                print("Goodbye.")
                break
            resp = agent.invoke(query)
            print("ROSA:", resp)
        except KeyboardInterrupt:
            print("\nInterrupted. Exiting.")
            break
        except Exception as e:
            traceback.print_exc()
            print("Error:", e)


if __name__ == "__main__":
    main()
