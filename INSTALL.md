# Installation Guide

This guide will help you set up the complete simulation environment for llm-pick-me-bots.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04+ (tested with Ubuntu 22.04)
- **Python**: 3.10+ (already installed ✓)
- **Docker**: Required for running simulations

### Installing Docker

#### For Windows Subsystem for Linux (WSL)

If you're using WSL, you need Docker Desktop for Windows:

1. **Install Docker Desktop**:
   - Download from: https://www.docker.com/products/docker-desktop
   - Install and start Docker Desktop

2. **Configure WSL Integration**:
   - Open Docker Desktop
   - Go to Settings → Resources → WSL Integration
   - Enable "Use the WSL 2 based engine"
   - Enable integration with your WSL distro (e.g., Ubuntu-22.04)
   - Click "Apply & Restart"

3. **Verify in WSL**:
   ```bash
   docker --version
   docker run hello-world
   ```

#### For Native Linux (Ubuntu)

If Docker is not installed, run the following commands:

```bash
# Update package list
sudo apt update

# Install Docker
sudo apt install -y docker.io

# Start Docker service
sudo systemctl start docker
sudo systemctl enable docker

# Add your user to the docker group (to run without sudo)
sudo usermod -aG docker $USER

# Log out and log back in for group changes to take effect
```

After logging back in, verify Docker is working:
```bash
docker --version
docker run hello-world
```

## Quick Setup

1. **Run the setup script** (this will build the Docker image):
   ```bash
   chmod +x setup.sh
   ./setup.sh
   ```

3. **Start the simulation**:
   ```bash
   chmod +x run_simulation.sh
   ./run_simulation.sh
   ```

## Manual Setup

### Building Docker Image

Build the ROS Noetic Docker image:
```bash
docker build -t noetic-gazebo-rosa -f ./Dockerfile.noetic .
```

**Note**: The build process may take 15-30 minutes depending on your internet connection, as it downloads and compiles many packages.

### Running the Simulation

The simulation automatically starts Gazebo Classic and RViz:
```bash
xhost +local:root
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  noetic-gazebo-rosa
```

Or use the provided script (recommended):
```bash
./run_simulation.sh
```

## What's Included

The Docker image includes:
- **Ubuntu 20.04** + **ROS Noetic (ROS 1)**
- **Gazebo Classic** - 3D physics simulator
- **MoveIt** - Motion planning framework
- **RViz** - 3D visualization tool
- **ROSA (jpl-rosa)** - Python package for LLM integration (in separate venv)
- **LIMO Cobot simulation packages** - Pre-built catkin workspace
- **Standard ROS control tools**:
  - `rqt_robot_steering` - GUI sliders for base control
  - `teleop_twist_keyboard` - Keyboard control for base
  - `joint_state_publisher_gui` - GUI sliders for visualization
- **Pre-built and ready to run**

## Troubleshooting

### X11 Display Issues
If you get display errors:
```bash
# Allow X11 connections
xhost +local:root

# Check DISPLAY variable
echo $DISPLAY
```

### Docker Permission Denied
If you get permission errors:
```bash
# Add user to docker group
sudo usermod -aG docker $USER
# Log out and log back in, or run:
newgrp docker
```

### Build Failures
If Docker build fails:
- Check your internet connection (builds download many packages)
- Ensure you have enough disk space (images are ~5-10GB)
- Try building with more verbose output: `docker build --progress=plain -t ...`

## Next Steps

After setup, see:
- [README.md](README.md) - Project overview and quick start
- [Standard Control Methods](docs/STANDARD_CONTROL_METHODS.md) - How to control the robot
- [MoveIt Arm Control](docs/MOVEIT_ARM_CONTROL.md) - Using MoveIt for arm control
- [Simulation Documentation](docs/Simulation%20in%20ROS1%20and%20Gazebo%20Classic.md) - Detailed simulation guide

