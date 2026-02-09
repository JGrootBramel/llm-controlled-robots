import os
import time
import roslibpy
from langchain.agents import tool
from langchain_openai import ChatOpenAI
from rosa import ROSA
from dotenv import load_dotenv
load_dotenv()

# --- CONFIGURATION ---
LIMO_IP = "192.168.0.105" 
LIMO_PORT = 11311  # <--- CHECK YOUR ROBOT IP
os.environ["OPENAI_API_KEY"] = os.getenv("OPENAI_API_KEY") # <--- ENTER YOUR KEY HERE

os.environ["ROS_MASTER_URI"] = f"http://{LIMO_IP}:{LIMO_PORT}" 

# 2. Who am I? (The PC)
# Replace with your PC's IP (run 'hostname -I' to find it)
os.environ["ROS_IP"] = "192.168.0.151"

# --- CONNECT TO ROBOT (GLOBAL CLIENT) ---
# We keep one connection open for the tools to use
client = roslibpy.Ros(host=LIMO_IP, port=LIMO_PORT)
client.run()
print(f"Connected to Limo: {client.is_connected}")

# --- DEFINE TOOLS ---

@tool
def drive_distance(meters: float) -> str:
    """
    Drives the robot forward or backward by a specific distance in meters.
    Positive values move forward, negative values move backward.
    Use this when the user wants to move, drive, or approach something.
    """
    if not client.is_connected:
        return "Error: Robot not connected."
    
    print(f"TOOL USED: Driving {meters} meters...")
    
    # Setup Publisher
    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
    
    speed = 0.2
    if meters < 0:
        speed = -0.2
        meters = abs(meters)
        
    duration = meters / 0.2
    
    # Move
    move_msg = roslibpy.Message({
        'linear': {'x': speed, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    })
    
    stop_msg = roslibpy.Message({
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    })
    
    start = time.time()
    while time.time() - start < duration:
        talker.publish(move_msg)
        time.sleep(0.1)
        
    # Stop
    talker.publish(stop_msg)
    time.sleep(0.5)
    
    return f"Successfully drove {meters} meters."

@tool
def scan_for_object(object_name: str) -> str:
    """
    Scans the visual field for a specific object (e.g., 'blue cube', 'red ball').
    Returns the distance to the object in meters if found.
    """
    print(f"TOOL USED: Scanning camera for {object_name}...")
    
    # --- MOCKED VISION LOGIC ---
    # Real logic would use OpenCV here to subscribe to /camera/rgb/image_raw
    # For now, we simulate finding a 'blue cube' at 1.5 meters away.
    
    time.sleep(1) # Simulate processing time
    
    if "blue cube" in object_name.lower():
        return "Found blue cube at distance: 1.5 meters"
    elif "red ball" in object_name.lower():
        return "Found red ball at distance: 0.8 meters"
    else:
        return f"Could not find {object_name}."

# --- INITIALIZE ROSA (CORRECTED) ---

# 1. Create the LLM Object
llm = ChatOpenAI(model="gpt-4o") 

# 1. The "Lobotomy" List (Block all default ROSA tools)
# ROSA normally tries to call these to "verify" the system. We block them.
nuclear_blacklist = [
    "ros_node_list", "ros_topic_list", "ros_service_list", "ros_param_list",
    "ros_node_info", "ros_topic_info", "ros_service_info", "ros_param_info",
    "ros_msg_info", "ros_srv_info", "ros_action_info"
]

# 2. The "Bridge Mode" Prompt (Brainwash the AI)
# We override the system prompt to force it to trust our tools.
bridge_prompt = (
    "You are a robot controller using a WebSocket Bridge. "
    "CRITICAL PROTOCOL: "
    "1. Do NOT try to list nodes, topics, or services. You do not have permission. "
    "2. Do NOT check system health. Assume the system is ready. "
    "3. Use ONLY the provided tools 'drive_distance' and 'scan_for_object'. "
    "4. If the user gives a command, execute it immediately using the tools."
)

# 3. Initialize
llm = ChatOpenAI(model="gpt-4o")
agent = ROSA(
    ros_version=1, 
    llm=llm, 
    tools=[drive_distance, scan_for_object], 
    blacklist=nuclear_blacklist  # <--- Apply the blacklist
)


# --- MAIN LOOP ---
if __name__ == "__main__":
    print("\n🤖 ROSA Limo Agent Ready! Type 'exit' to quit.")
    
    while True:
        user_input = input("\nYou: ")
        if user_input.lower() in ["exit", "quit"]:
            break
            
        try:
            response = agent.invoke(user_input)
            print(f"ROSA: {response}")
            
        except Exception as e:
            print(f"Error: {e}")

    client.terminate()

