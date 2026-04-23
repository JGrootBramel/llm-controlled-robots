import os
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

all_my_tools = []
for tool_name in robot_tools.__all__:
    t = getattr(robot_tools, tool_name)
    all_my_tools.append(_wrap_with_tool_logging(t, tool_name))
print(f"Lade {len(all_my_tools)} Tools: {robot_tools.__all__}")


# --- INITIALIZE LLM & PROMPT ---
llm = ChatOpenAI(model="gpt-4o") 

# --- INITIALIZE ROSA ---
_validate_ros_environment()

agent = ROSA(
    ros_version=1, 
    llm=llm, 
    tools=all_my_tools,  # <--- Hier übergeben wir die dynamisch erzeugte Liste!
)

# --- MAIN LOOP ---
if __name__ == "__main__":
    print("\n✅ ROSA Limo Agent Ready! Type 'exit' to quit.")
    
    while True:
        user_input = input("\nYou: ")
        if user_input.lower() in ["exit", "quit"]:
            break
            
        try:
            response = agent.invoke(user_input)
            print(f"ROSA: {response}")
            
        except Exception as e:
            print(f"Error: {e}")