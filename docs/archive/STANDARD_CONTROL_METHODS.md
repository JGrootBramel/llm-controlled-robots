# Standard ROS Control Methods

This guide shows you how to control the LIMO Cobot using standard ROS tools with GUI sliders and interfaces.

## Prerequisites

The simulation must be running. Start it with:
```bash
./run_simulation.sh
```

## Method 1: Mobile Base Control with Sliders

### Using rqt_robot_steering

**What it does**: Provides GUI sliders to control the mobile base (forward/backward, left/right).

**How to use**:

1. **Access the container** (in a new terminal):
   ```bash
   docker exec -it <container_id> /bin/bash
   ```

2. **Run the steering tool**:
   ```bash
   rosrun rqt_robot_steering rqt_robot_steering
   ```

3. **Control the robot**:
   - Use the sliders in the GUI window
   - Left slider: Forward/Backward
   - Right slider: Left/Right rotation
   - The robot moves smoothly as you adjust the sliders

**What it publishes**: Commands to `/cmd_vel` topic (geometry_msgs/Twist)

---

## Method 2: Arm Control with Joint Sliders

### Using joint_state_publisher_gui

**What it does**: Provides GUI sliders for each joint of the robotic arm.

**How to use**:

1. **Access the container**:
   ```bash
   docker exec -it <container_id> /bin/bash
   ```

2. **Run the joint state publisher GUI**:
   ```bash
   rosrun joint_state_publisher_gui joint_state_publisher_gui
   ```

3. **Control the arm**:
   - You'll see sliders for each joint
   - Adjust sliders to move individual joints
   - The arm moves in real-time in Gazebo

**Note**: This works best when using the `fake` controller manager. For real controllers, you may need to use MoveIt or direct topic publishing.

---

## Method 3: Keyboard Control for Base

### Using teleop_twist_keyboard

**What it does**: Keyboard control for the mobile base (standard ROS tool).

**How to use**:

1. **Access the container**:
   ```bash
   docker exec -it <container_id> /bin/bash
   ```

2. **Run the teleop tool**:
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```

3. **Control keys**:
   ```
   Moving around:
      u    i    o
      j    k    l
      m    ,    .

   q/z : increase/decrease max speeds by 10%
   w/x : increase/decrease only linear speed by 10%
   e/c : increase/decrease only angular speed by 10%
   space key, k : force stop
   anything else : stop smoothly
   ```

**What it publishes**: Commands to `/cmd_vel` topic

---

## Method 4: MoveIt Motion Planning (High-Level)

### Using MoveIt for Arm Control

**What it does**: High-level motion planning with collision avoidance for the arm.

**How to use**:

1. **MoveIt is already running** when you start the simulation (RViz shows MoveIt interface)

2. **In RViz**:
   - Use the "Motion Planning" panel
   - Drag the interactive markers to set target positions
   - Click "Plan" to see the planned trajectory
   - Click "Execute" to move the arm

3. **Or use MoveIt services** (programmatically):
   ```bash
   # List available services
   rosservice list | grep moveit
   
   # Example: Plan and execute via service
   rosservice call /move_group/plan_exec_path "{}"
   ```

**What it uses**: `/arm_controller/follow_joint_trajectory` action server

---

## Method 5: Direct Topic Publishing

### Using rostopic pub

**What it does**: Directly publish commands to control topics.

**Mobile Base Example**:
```bash
# Move forward at 0.5 m/s
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# Turn left at 0.5 rad/s
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -r 10
```

**Arm Example** (more complex, requires trajectory message):
```bash
# This is complex, better to use MoveIt or the controller script
```

---

## Recommended Workflow

### For Testing and Development:
1. **Base**: Use `rqt_robot_steering` (sliders) or `teleop_twist_keyboard` (keyboard)
2. **Arm**: Use MoveIt in RViz (interactive markers) or the custom `robot_controller.py` script

### For Production/LLM Control:
1. **Base**: ROSA or direct topic publishing to `/cmd_vel`
2. **Arm**: MoveIt services/actions or trajectory publishing to `/arm_controller/command`

---

## Installing Missing Tools

If any tool is not available in the container, you can install it:

```bash
# Inside the container
apt-get update
apt-get install -y ros-noetic-rqt-robot-steering
apt-get install -y ros-noetic-teleop-twist-keyboard
apt-get install -y ros-noetic-joint-state-publisher-gui
```

---

## Comparison Table

| Method | Base Control | Arm Control | GUI | Best For |
|--------|-------------|-------------|-----|----------|
| `rqt_robot_steering` | ✅ Sliders | ❌ | ✅ | Base testing |
| `joint_state_publisher_gui` | ❌ | ✅ Sliders | ✅ | Arm testing (fake mode) |
| `teleop_twist_keyboard` | ✅ Keyboard | ❌ | ❌ | Base testing |
| MoveIt (RViz) | ❌ | ✅ Interactive | ✅ | Arm planning |
| `robot_controller.py` | ✅ Keyboard | ✅ Keyboard | ❌ | Both (custom) |
| ROSA | ✅ Natural Lang | ✅ Natural Lang | ❌ | LLM control |

---

## Quick Start Example

**Control base with sliders**:
```bash
# Terminal 1: Simulation running
./run_simulation.sh

# Terminal 2: Access container and run slider
docker exec -it <container_id> /bin/bash
rosrun rqt_robot_steering rqt_robot_steering
```

**Control arm with MoveIt**:
- MoveIt is already running in RViz
- Use the interactive markers in the "Motion Planning" panel
- Plan and execute trajectories

---

## Notes

- All these methods use standard ROS topics/services/actions
- They work with the same controllers your custom script uses
- MoveIt is the recommended method for arm control in production
- For LLM integration, ROSA can use any of these methods programmatically

