# Robot Controller Usage Guide

## Overview

The `robot_controller.py` script provides keyboard control for both the mobile base and robotic arm of the LIMO Cobot simulation.

## Features

- **Mobile Base Control**: Smooth continuous movement using `/cmd_vel` topic
- **Arm Control**: Joint-by-joint control using `/arm_controller/command` trajectory controller
- **Auto-discovery**: Automatically discovers joint names from the robot
- **Real-time Control**: 20 Hz publishing rate for smooth movement

## Usage

### Inside Docker Container

1. Start the simulation:
   ```bash
   ./run_simulation.sh
   ```

2. In a new terminal, access the container:
   ```bash
   docker exec -it <container_id> /bin/bash
   ```

3. Navigate and run the controller:
   ```bash
   cd /root
   deactivate  # Exit ROSA venv if active
   python3 /root/catkin_ws/src/limo_cobot_sim/.../robot_controller.py
   ```
   
   Or copy the script into the container first:
   ```bash
   docker cp docs/robot_controller.py <container_id>:/root/robot_controller.py
   python3 /root/robot_controller.py
   ```

## Controls

### Mobile Base
- **W** - Move forward
- **S** - Move backward
- **A** - Turn left
- **D** - Turn right
- **X** - Stop

### Arm (6 joints)
- **1-6** - Increase joint 1-6 angle
- **Q, E** - Decrease joint 1, 2
- **R, F** - Decrease joint 3, 4
- **T, G** - Decrease joint 5, 6
- **Z** - Reset all joints to zero

### Other
- **+** - Increase speed
- **-** - Decrease speed
- **H** - Show help
- **ESC** - Quit

## Technical Details

### Control Methods
- **Base**: Uses `geometry_msgs/Twist` on `/cmd_vel` topic
- **Arm**: Uses `trajectory_msgs/JointTrajectory` on `/arm_controller/command` topic

### Joint Names
The controller automatically discovers joint names:
- joint2_to_joint1
- joint3_to_joint2
- joint4_to_joint3
- joint5_to_joint4
- joint6_to_joint5
- joint6output_to_joint6

### Future Improvements
For smoother motion, consider:
- Using action client instead of topic publisher
- Increasing trajectory duration (currently 0.1s)
- Adding velocity/acceleration constraints
- Using MoveIt for high-level planning

## Standard Control Methods

For production use, consider:
1. **MoveIt**: High-level motion planning via services/actions
2. **rqt_robot_steering**: GUI control for base
3. **teleop_twist_keyboard**: Standard keyboard control for base
4. **ROSA**: Natural language control (future integration)

