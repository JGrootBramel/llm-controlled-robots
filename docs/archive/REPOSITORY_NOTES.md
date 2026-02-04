# Repository Notes - Key Information Extracted

This document contains important information extracted from the original `limo_cobot_sim` and `rosa` repositories before they were removed from the workspace.

## LIMO Cobot Simulation Repository Notes

### Launch Command
```bash
roslaunch limo_cobot_moveit demo_gazebo.launch
```

### Controller Configuration
- **Arm Controller**: `effort_controllers/JointTrajectoryController`
- **Topic**: `/arm_controller/command` (trajectory_msgs/JointTrajectory)
- **Action**: `/arm_controller/follow_joint_trajectory` (FollowJointTrajectoryAction)
- **Joints**: 
  - joint2_to_joint1
  - joint3_to_joint2
  - joint4_to_joint3
  - joint5_to_joint4
  - joint6_to_joint5
  - joint6output_to_joint6

### Controller Manager
- Default in `demo.launch`: `fake` (for visualization only)
- For Gazebo: `ros_control` (real controllers)
- Controllers spawned: `arm_controller`, `gripper_controller`, `joint_state_controller`

### PID Gains (from ros_controllers.yaml)
- P: 100 (very high, for stiffness)
- D: 1
- I: 1
- i_clamp: 1

### Mobile Base Control
- Topic: `/cmd_vel` (geometry_msgs/Twist)
- Standard ROS control method

### Standard Control Methods
1. **rqt_robot_steering**: GUI sliders for base control
2. **teleop_twist_keyboard**: Keyboard control for base
3. **MoveIt**: High-level motion planning for arm

## ROSA Library Notes

### Purpose
ROSA (Robot Operating System Agent) is an AI-powered assistant for ROS1 and ROS2 systems using natural language queries.

### Key Features
- Built on LangChain framework
- Supports ROS 1 (Noetic) and ROS 2 (Humble, Iron, Jazzy)
- Tool-based agent architecture
- Streaming support
- Chat history accumulation

### ROS 1 Tools Available
- `rosgraph_get()`: Get ROS graph (nodes ↔ topics)
- `rostopic_list()`: List topics
- `rostopic_pub()`: Publish to topics
- `rostopic_echo()`: Subscribe and read messages
- `rostopic_info()`: Get topic info
- `rosnode_list()`: List nodes
- `rosservice_list()`: List services
- `rosservice_call()`: Call services
- `rosparam_get/set()`: Get/set parameters

### Usage Example
```python
from rosa import ROSA

llm = get_your_llm_here()
agent = ROSA(ros_version=1, llm=llm)
agent.invoke("Show me a list of topics that have publishers but no subscribers")
```

### Integration Points
- Can publish to `/cmd_vel` for base control
- Can publish to `/arm_controller/command` for arm control
- Can use MoveIt services for high-level planning
- Can monitor `/joint_states` for robot state

### Installation
```bash
pip install jpl-rosa
```

## Important Configuration Files

### Controller Config (ros_controllers.yaml)
- Located in: `limo_cobot_moveit/config/ros_controllers.yaml`
- Defines arm_controller and gripper_controller
- Uses JointTrajectoryController type

### MoveIt Config (simple_moveit_controllers.yaml)
- Located in: `limo_cobot_moveit/config/simple_moveit_controllers.yaml`
- Defines MoveIt interface to controllers
- Action namespace: `follow_joint_trajectory`

## Notes for Future Development

1. **Standard Control Methods**: Use MoveIt for high-level planning, direct topics for low-level control
2. **Smooth Motion**: Use action clients instead of topic publishers, set appropriate trajectory durations (0.5-1.0s)
3. **ROSA Integration**: Can be used for natural language control, diagnostics, and inspection
4. **Controller Tuning**: Current PID gains (P=100) are very high, may need adjustment for smoother motion

