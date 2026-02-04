# Quick Start: Using Standard ROS Control Tools (Sliders)

Step-by-step guide to control the robot using GUI sliders and standard ROS tools.

---

## Step 1: Rebuild Docker Image (If Needed)

**Why**: The Dockerfile was updated to include standard ROS control tools.

**Check if you need to rebuild**:
```bash
cd ~/llm_robot/llm-pick-me-bots
docker images | grep noetic-gazebo-rosa
```

**If the image doesn't exist or is old, rebuild it**:
```bash
./setup.sh
```

**Wait for**: Build to complete (15-30 minutes). You'll see "✓ Docker image built successfully!"

**Tell me when**: Build is complete.

---

## Step 2: Start the Simulation

**Why**: We need Gazebo and RViz running before we can control the robot.

**Run this**:
```bash
cd ~/llm_robot/llm-pick-me-bots
./run_simulation.sh
```

**What happens**:
- Gazebo window opens with the robot
- RViz window opens with MoveIt interface
- Terminal shows ROS launch output

**Important**: Keep this terminal open! The simulation is running here.

**Tell me when**: You see Gazebo and RViz windows open with the robot visible.

---

## Step 3: Find the Container ID

**Why**: We need the container ID to access the container and run control tools.

**Open a NEW terminal** (keep the simulation terminal running).

**Run this**:
```bash
docker ps
```

**What to look for**: You'll see a container with image `noetic-gazebo-rosa`

**Example output**:
```
CONTAINER ID   IMAGE                COMMAND                  CREATED          STATUS
abc123def456   noetic-gazebo-rosa   "/ros_entrypoint.sh …"   2 minutes ago    Up 2 minutes
```

**Copy the CONTAINER ID** (first column, like `abc123def456`)

**Tell me when**: You have the container ID.

---

## Step 4: Control Mobile Base with Sliders

**Why**: This gives you smooth GUI control of the car/base.

**In your NEW terminal, run**:
```bash
docker exec -it <CONTAINER_ID> /bin/bash
```

**Replace `<CONTAINER_ID>`** with the actual ID you copied (e.g., `abc123def456`)

**Inside the container, run**:
```bash
rosrun rqt_robot_steering rqt_robot_steering
```

**What happens**:
- A GUI window opens with two sliders
- Left slider: Forward/Backward
- Right slider: Left/Right rotation

**How to use**:
- Move the sliders up/down to control the robot
- The robot moves in real-time in Gazebo
- Release slider to stop

**Tell me when**: The slider window is open and you can move the robot.

---

## Step 5: Control Arm with Joint Sliders (Optional)

**Why**: This gives you GUI control of each arm joint.

**In the SAME container terminal** (or open another one):

**Run this**:
```bash
rosrun joint_state_publisher_gui joint_state_publisher_gui
```

**What happens**:
- A GUI window opens with sliders for each joint
- You'll see 6 sliders (one for each arm joint)

**How to use**:
- Adjust each slider to move that joint
- The arm moves in real-time in Gazebo

**Note**: This works best for visualization. For real control, use MoveIt in RViz.

**Tell me when**: The joint slider window is open.

---

## Step 6: Use MoveIt in RViz (For Arm Planning)

**Why**: MoveIt provides high-level motion planning with collision avoidance.

**In RViz** (already open from Step 2):

1. **Find the "Motion Planning" panel** (usually on the left side)

2. **Use interactive markers**:
   - You'll see colored markers on the robot arm
   - Click and drag these markers to set target positions
   - The arm model in RViz will show the target pose

3. **Plan a trajectory**:
   - Click the "Plan" button
   - You'll see the planned path in RViz

4. **Execute the trajectory**:
   - Click the "Execute" button
   - The arm moves in Gazebo following the planned path

**Tell me when**: You've successfully moved the arm using MoveIt.

---

## Step 7: Keyboard Control for Base (Alternative)

**Why**: Some people prefer keyboard over sliders.

**In a container terminal, run**:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**Control keys**:
```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i = forward
, = backward
j = left
l = right
k = stop
```

**Tell me when**: You can control the robot with keyboard.

---

## Summary of Control Methods

| Method | What It Controls | How to Run |
|--------|-----------------|------------|
| `rqt_robot_steering` | Mobile Base | `rosrun rqt_robot_steering rqt_robot_steering` |
| `joint_state_publisher_gui` | Arm Joints | `rosrun joint_state_publisher_gui joint_state_publisher_gui` |
| MoveIt (RViz) | Arm (Planning) | Already running, use RViz panel |
| `teleop_twist_keyboard` | Mobile Base | `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` |
| `robot_controller.py` | Both | Custom script (see CONTROLLER_USAGE.md) |

---

## Troubleshooting

**Problem**: "command not found" when running rosrun
- **Solution**: Make sure you're inside the container and ROS is sourced

**Problem**: Slider window doesn't open
- **Solution**: Check DISPLAY is set: `echo $DISPLAY`
- **Solution**: Make sure X11 forwarding is working

**Problem**: Robot doesn't move with sliders
- **Solution**: Check if simulation is running (Gazebo should be open)
- **Solution**: Check topics: `rostopic list | grep cmd_vel`

---

## Next Steps

Once you're comfortable with standard tools:
1. Try combining base and arm control simultaneously
2. Use MoveIt for complex arm movements
3. Integrate ROSA for natural language control (future)

