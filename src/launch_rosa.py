import os, sys
pkg_dir = os.path.dirname(os.path.abspath(__file__))        # .../<your_pkg>/scripts
pkg_dir = os.path.dirname(pkg_dir)                          # .../<your_pkg>
vendor_dir = os.path.join(pkg_dir, "vendor")                # .../<your_pkg>/vendor
if vendor_dir not in sys.path:
    sys.path.insert(0, vendor_dir)

#!/usr/bin/env python3
import os
import sys
import traceback
from typing import Optional
from dotenv import load_dotenv

load_dotenv()  # Load environment variables from .env file if present

# Paths: this file lives at .../src/launch_rosa.py
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
vendor_dir = os.path.join(repo_root, "vendor")

# Ensure local `vendor/` (vendored rosa) and the `src` directory are on PYTHONPATH
if vendor_dir not in sys.path:
    sys.path.insert(0, vendor_dir)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Default ROS master (your physical LIMO cobot)
os.environ.setdefault("ROS_MASTER_URI", "http://192.168.0.151:11311")

print(f"Using ROS_MASTER_URI={os.environ.get('ROS_MASTER_URI')}")

try:
    from langchain_openai import ChatOpenAI
except Exception as e:  # pragma: no cover - user environment dependent
    print("Missing dependency: langchain_openai. Install required Python packages.")
    raise

try:
    from rosa import ROSA
except Exception as e:  # pragma: no cover - vendor package may vary
    print("Failed to import 'rosa' package. Ensure vendor/rosa or venv has rosa installed.")
    raise

# Ensure the local `tools` package is importable so ROSA can discover tool modules.
try:
    import tools  # noqa: F401
except Exception as e:
    print("Warning: could not import local 'tools' package:", e)

OPENAI_API_KEY: Optional[str] = os.environ.get("OPENAI_API_KEY")
if not OPENAI_API_KEY:
    raise SystemExit("Please set the OPENAI_API_KEY environment variable before running this script.")

# Create an LLM client for ROSA; allow overriding model via ROSA_MODEL env var
llm = ChatOpenAI(model=os.environ.get("ROSA_MODEL", "gpt-4o"), temperature=0, api_key=OPENAI_API_KEY)

from tools.turn_in_place import turn_in_place

# Instantiate ROSA and tell it to load tools from the local `tools` package
agent = ROSA(
    ros_version=1,
    llm=llm,
    tools=[turn_in_place],          # <-- force include your tool
    tool_packages=["tools"],        # optional; keep if you want package discovery too
    blacklist=["rosservice_list"],  # <-- disable the buggy tool
    streaming=False,
    verbose=True,
)

# ROSATools usually exposes a list-like or a .tools attribute; handle both safely
rt = agent._ROSA__tools
tool_list = getattr(rt, "tools", None) or getattr(rt, "_tools", None) or rt
try:
    names = [getattr(t, "name", str(t)) for t in tool_list]
except TypeError:
    # if rt isn't iterable, fall back to dir()
    names = [a for a in dir(rt) if "tool" in a.lower()]
print("Resolved tool names:", names)

def repl():
    print("ROSA agent ready. Type a question (or 'exit' to quit).")
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
            print("Error during invocation:", e)


if __name__ == "__main__":
    repl()