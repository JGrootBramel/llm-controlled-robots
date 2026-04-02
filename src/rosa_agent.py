import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from rosa import ROSA

# 1. Importiere dein gesamtes Tool-Modul
# (Passe den Importpfad an, falls deine __init__.py woanders liegt)
import limo_llm_control.tools as robot_tools

load_dotenv()
os.environ["OPENAI_API_KEY"] = os.getenv("OPENAI_API_KEY")

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