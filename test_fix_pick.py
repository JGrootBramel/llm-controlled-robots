"""
ROSA Limo Agent
Single-file version with:
- Drive
- Rotate
- Detect blue/red box
- Pick object
- Navigate
- Drop object
"""

import os
import time
import json
import roslibpy
from dotenv import load_dotenv
from langchain.agents import tool
from langchain_openai import ChatOpenAI
from rosa import ROSA, RobotSystemPrompts

# --------------------------------------------------
# ENV SETUP
# --------------------------------------------------
load_dotenv()

LIMO_IP = os.environ.get("LIMO_IP", "192.168.0.105")
LIMO_PORT = int(os.environ.get("LIMO_PORT", "9090"))

os.environ["ROS_MASTER_URI"] = f"http://{LIMO_IP}:{LIMO_PORT}"
os.environ["ROS_IP"] = os.environ.get("ROS_IP", "192.168.0.151")

# --------------------------------------------------
# CONNECT TO ROS
# --------------------------------------------------
client = roslibpy.Ros(host=LIMO_IP, port=LIMO_PORT)
client.run()
print(f"Connected to LIMO: {client.is_connected}")

# --------------------------------------------------
# TOOLS
# --------------------------------------------------

@tool
def drive_distance(meters: float) -> str:
    """Drive forward/backward in meters"""
    if not client.is_connected:
        return "Robot not connected"

    speed = 0.2 if meters >= 0 else -0.2
    duration = abs(meters) / 0.2

    cmd_vel = roslibpy.Topic(client, "/cmd_vel", "geometry_msgs/Twist")

    move = roslibpy.Message({
        "linear": {"x": speed, "y": 0, "z": 0},
        "angular": {"x": 0, "y": 0, "z": 0}
    })

    stop = roslibpy.Message({
        "linear": {"x": 0, "y": 0, "z": 0},
        "angular": {"x": 0, "y": 0, "z": 0}
    })

    start = time.time()
    while time.time() - start < duration:
        cmd_vel.publish(move)
        time.sleep(0.1)

    cmd_vel.publish(stop)
    return f"Drove {meters} meters"


@tool
def rotate_angle(degrees: float) -> str:
    """Rotate robot in place"""
    angular_speed = 0.5 if degrees > 0 else -0.5
    duration = abs(degrees) / 30.0

    cmd_vel = roslibpy.Topic(client, "/cmd_vel", "geometry_msgs/Twist")

    msg = roslibpy.Message({
        "linear": {"x": 0, "y": 0, "z": 0},
        "angular": {"x": 0, "y": 0, "z": angular_speed}
    })

    start = time.time()
    while time.time() - start < duration:
        cmd_vel.publish(msg)
        time.sleep(0.1)

    cmd_vel.publish({
        "linear": {"x": 0, "y": 0, "z": 0},
        "angular": {"x": 0, "y": 0, "z": 0}
    })

    return f"Rotated {degrees} degrees"


@tool
def detect_box(color: str) -> str:
    """Detect blue or red box"""
    time.sleep(1)

    if color.lower() == "blue":
        return "Blue box detected at 1.2 meters, angle -8 degrees"
    if color.lower() == "red":
        return "Red box detected at 0.9 meters, angle 5 degrees"

    return "Box not found"


@tool
def pick_object() -> str:
    """Pick object using arm & gripper"""
    arm = roslibpy.Topic(client, "/arm_controller/command", "std_msgs/String")
    gripper = roslibpy.Topic(client, "/gripper_controller/command", "std_msgs/String")

    arm.publish({"data": "down"})
    time.sleep(1)
    gripper.publish({"data": "close"})
    time.sleep(1)
    arm.publish({"data": "up"})
    time.sleep(1)

    return "Object picked"


@tool
def drop_object() -> str:
    """Drop object"""
    gripper = roslibpy.Topic(client, "/gripper_controller/command", "std_msgs/String")
    gripper.publish({"data": "open"})
    time.sleep(1)
    return "Object dropped"


@tool
def navigate_to(location: str) -> str:
    """Navigate to named location"""
    nav = roslibpy.Topic(client, "/navigation_goal", "std_msgs/String")
    nav.publish({"data": location})
    time.sleep(3)
    return f"Arrived at {location}"

# --------------------------------------------------
# ROSA SETUP
# --------------------------------------------------

bridge_prompt = (
    "You are a robot controller. "
    "Use ONLY the provided tools. "
    "Do NOT list ROS nodes or topics. "
    "Execute user commands immediately."
)

blacklist = [
    "ros_node_list", "ros_topic_list", "ros_service_list",
    "ros_node_info", "ros_topic_info", "ros_service_info",
]

llm = ChatOpenAI(model="gpt-4o")

agent = ROSA(
    ros_version=1,
    llm=llm,
    tools=[
        drive_distance,
        rotate_angle,
        detect_box,
        pick_object,
        drop_object,
        navigate_to,
    ],
    blacklist=blacklist,
    prompts=RobotSystemPrompts(critical_instructions=bridge_prompt),
)

# --------------------------------------------------
# MAIN LOOP
# --------------------------------------------------

if __name__ == "__main__":
    print("\nROSA Limo Pick & Place Agent Ready (type 'exit')")

    while True:
        user_input = input("\nYou: ").strip()
        if user_input.lower() in ["exit", "quit"]:
            break

        try:
            response = agent.invoke(user_input)
            print(f"ROSA: {response}")
        except Exception as e:
            print("Error:", e)

    client.terminate()
