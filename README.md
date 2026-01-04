# llm-pick-me-bots

ROS-based simulation environment for LIMO Cobot robots with LLM integration capabilities.

## Quick Start

1. **Install Docker** (if not already installed):
   ```bash
   sudo apt update && sudo apt install -y docker.io
   sudo systemctl start docker
   sudo usermod -aG docker $USER
   # Log out and log back in
   ```

2. **Run the setup script**:
   ```bash
   ./setup.sh
   ```

3. **Start the simulation**:
   ```bash
   ./run_simulation.sh
   ```

4. **Run the ROSA natural-language controller (inside the container)**:
   - Open a new terminal (keep the simulation terminal running)
   - Find the container id: `sudo docker ps`
   - Enter the container: `sudo docker exec -it <CONTAINER_ID> /bin/bash`
   - Run:
     ```bash
     bash /workspace/llm-pick-me-bots/start_rosa.sh
     ```

**Windows note**: Use WSL2 (recommended). See the Windows section in `docs/ROSA_INTEGRATION.md`.

## Documentation

- **[Installation Guide](INSTALL.md)** - Detailed setup instructions
- **[ROSA Integration](docs/ROSA_INTEGRATION.md)** - Natural language robot control with ROSA
- **[Standard Control Methods](docs/STANDARD_CONTROL_METHODS.md)** - Use GUI sliders and standard ROS tools
- **[Controller Usage](docs/CONTROLLER_USAGE.md)** - Custom keyboard controller
- **[Simulation Guide](docs/Simulation%20in%20ROS1%20and%20Gazebo%20Classic.md)** - ROS1 and Gazebo Classic simulation
- **[Troubleshooting](docs/troubleshooting/)** - Common issues and solutions

## ROS Distribution

- **ROS Noetic (ROS1)** - Required for this project
  - Uses Docker to run on Ubuntu 20.04 (required for ROS 1)
  - Works on Ubuntu 22.04 host via Docker
  - Gazebo Classic simulation
  - MoveIt motion planning
  - ROSA (jpl-rosa) Python package for LLM control

## Features

- Pre-configured Docker image with all dependencies
- LIMO mobile robot simulation
- MyCobot robotic arm integration
- Gazebo Classic simulation
- MoveIt motion planning
- **ROSA (jpl-rosa)** - Natural language robot control with LLM integration
- **ROSA (jpl-rosa)** - Natural language robot control (✅ Integrated)
- **Standard ROS control tools**:
  - `rqt_robot_steering` - GUI sliders for base control
  - `teleop_twist_keyboard` - Keyboard control for base
  - `joint_state_publisher_gui` - GUI sliders for arm joints
  - MoveIt in RViz - Interactive arm planning
- Custom robot controller script for keyboard control

## Project Structure

```
llm-pick-me-bots/
├── Dockerfile.noetic      # ROS Noetic + Gazebo Classic + ROSA
├── setup.sh               # Automated setup script
├── setup_env.sh           # Setup API keys for ROSA
├── run_simulation.sh      # Run simulation script
├── start_rosa.sh          # Start ROSA controller
├── cleanup_and_start.sh   # Cleanup and restart script
├── INSTALL.md             # Installation guide
├── README.md              # This file
└── docs/                  # Documentation
    ├── robot_controller.py        # Keyboard controller for robot
    ├── rosa_robot_controller.py   # ROSA natural language controller
    ├── diagnose_robot.py          # Diagnostic tool
    ├── REPOSITORY_NOTES.md        # Notes from original repos
    ├── ROSA_INTEGRATION.md        # ROSA integration guide
    └── troubleshooting/           # Troubleshooting guides
```

## Requirements

- **Host OS**: Ubuntu 20.04+ (tested with 22.04)
  - Note: ROS 1 Noetic requires Ubuntu 20.04, but Docker allows running it on Ubuntu 22.04
- **Docker**: Required for containerized ROS environment
- **X11 display**: Required for GUI applications (Gazebo, RViz)
- **Python 3.9+**: For ROSA library (installed in Docker container)

## Purpose

This project provides a simulation environment for the LIMO Cobot robot to enable:
- Development and testing of LLM-controlled robot behaviors
- Working on robot control algorithms without physical robot access
- Preparing code that can later be applied to the physical robot in the lab

The simulation uses ROS 1 Noetic with Gazebo Classic, and includes ROSA for natural language control integration. You can control the robot using commands like "move forward", "turn left", or "what is the robot's current state?".

## License

See [LICENSE](LICENSE) file for details.