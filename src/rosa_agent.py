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
    """Wrap a LangChain tool so every invoke is logged to stdout and ROS."""
    orig_invoke = tool.invoke
    def logged_invoke(input, config=None):
        msg = f"[ROSA TOOL] {name} invoked with input={input!r}"
        print(msg, flush=True)
        try:
            import rospy
            rospy.loginfo(msg)
        except Exception:
            pass
        return orig_invoke(input, config)
    tool.invoke = logged_invoke
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