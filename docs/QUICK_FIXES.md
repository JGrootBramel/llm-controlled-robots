# Quick Fixes for Common Issues

## Issue 1: teleop_twist_keyboard not found

**Problem**: `[rospack] Error: package 'teleop_twist_keyboard' not found`

**Solution**: The ROSA virtual environment is active. Deactivate it first:

```bash
# Inside container
deactivate  # Exit ROSA venv
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**Why**: The ROSA venv doesn't have ROS packages. ROS packages are in the system ROS installation, not in the Python venv.

---

## Issue 2: Arm not moving with joint_state_publisher_gui

**Problem**: Sliders show "Centering" and "Randomizing" but arm doesn't move in Gazebo.

**Solution**: `joint_state_publisher_gui` doesn't work with real Gazebo controllers. Use MoveIt instead:

1. **In RViz** (should already be open):
   - Open the **Motion Planning** panel
   - Use interactive markers or joint sliders
   - Click **Plan** then **Execute**

2. **Or use the custom controller script**:
   ```bash
   deactivate  # Exit ROSA venv
   python3 /root/catkin_ws/src/limo_cobot_sim/.../robot_controller.py
   ```

**Why**: `joint_state_publisher_gui` publishes to `/joint_states` (read-only state topic). Real controllers need commands on `/arm_controller/command` (JointTrajectory messages).

---

## Issue 3: How to use MoveIt planning in RViz

**Answer**: Yes! MoveIt is part of the original `limo_cobot_sim` repo and should be running.

**How to use**:

1. **Check if RViz is open** (it should open automatically with the simulation)

2. **Open Motion Planning panel**:
   - In RViz: **Panels → Motion Planning**
   - Or look for it in the left sidebar

3. **Control the arm**:
   - **Drag interactive markers** (colored arrows/rings) on the arm
   - Or use **joint sliders** in the "Query" tab
   - Click **"Plan"** to see trajectory
   - Click **"Execute"** to move the arm

**See**: `docs/MOVEIT_ARM_CONTROL.md` for detailed instructions.

---

## General Rule: When to Deactivate ROSA Venv

**Always deactivate ROSA venv when using ROS tools**:
```bash
deactivate
```

**Activate ROSA venv only when using ROSA library**:
```bash
source /opt/rosa-venv/bin/activate
```

The ROSA venv is for Python packages (like `jpl-rosa`), not for ROS system packages.

