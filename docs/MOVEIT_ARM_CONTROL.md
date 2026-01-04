# MoveIt Arm Control Guide

## Overview

MoveIt is the **standard and recommended way** to control the arm in this simulation. It was part of the original `limo_cobot_sim` repository and provides high-level motion planning with collision avoidance.

## Using MoveIt in RViz

### Step 1: Make sure RViz is open

When you start the simulation with:
```bash
./run_simulation.sh
```

RViz should automatically open with the MoveIt interface. If it doesn't, you can launch it manually:
```bash
# Inside the container
roslaunch limo_cobot_moveit demo_gazebo.launch
```

### Step 2: Use the Motion Planning Panel

1. **In RViz**, look for the **"Motion Planning"** panel (usually on the left side)
2. If you don't see it, go to: **Panels → Motion Planning**

### Step 3: Control the Arm

**Method A: Interactive Markers (Easiest)**
1. In the Motion Planning panel, you'll see **"Planning"** and **"Execute"** buttons
2. You can **drag the interactive markers** (colored arrows/rings) on the arm in the 3D view
3. This sets a target pose for the end-effector
4. Click **"Plan"** to see the planned trajectory (orange line)
5. Click **"Execute"** to move the arm

**Method B: Joint Sliders**
1. In the Motion Planning panel, look for **"Query"** tab
2. You'll see sliders for each joint
3. Adjust sliders to set target joint positions
4. Click **"Plan"** then **"Execute"**

**Method C: Goal States**
1. In the Motion Planning panel, go to **"Planning"** tab
2. Use **"Goal States"** dropdown to select predefined poses
3. Click **"Plan"** then **"Execute"**

## Why joint_state_publisher_gui Doesn't Work

The `joint_state_publisher_gui` tool publishes to `/joint_states`, which is a **read-only** topic that reports the current state. It doesn't send commands.

For **real controllers in Gazebo**, the arm needs:
- **Topic**: `/arm_controller/command` (trajectory_msgs/JointTrajectory)
- **Action**: `/arm_controller/follow_joint_trajectory` (FollowJointTrajectoryAction)

MoveIt automatically handles this by:
1. Planning the trajectory
2. Converting it to the correct format
3. Sending it to the controller via the action server

## Alternative: Use the Custom Controller Script

If you want keyboard control, use the custom script:
```bash
# Inside container, deactivate ROSA venv first
deactivate
python3 /root/catkin_ws/src/limo_cobot_sim/.../robot_controller.py
```

This script publishes JointTrajectory messages directly to `/arm_controller/command`.

## Troubleshooting

**RViz not showing MoveIt panel?**
- Make sure the simulation is running with `demo_gazebo.launch`
- Check if MoveIt nodes are running: `rosnode list | grep moveit`

**Arm not moving when executing?**
- Check controller status: `rosservice call /controller_manager/list_controllers`
- Verify arm_controller is running: Should see `arm_controller` in the list

**Planning fails?**
- The target pose might be unreachable or in collision
- Try a different target pose
- Check joint limits in the Motion Planning panel

