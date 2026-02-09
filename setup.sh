#!/bin/bash
# Setup script for llm-pick-me-bots simulation environment

set -e

echo "========================================="
echo "LLM Pick Me Bots - Setup Script"
echo "========================================="

# Detect if running on WSL
IS_WSL=false
if grep -qi microsoft /proc/version 2>/dev/null || [ -f /proc/sys/fs/binfmt_misc/WSLInterop ]; then
    IS_WSL=true
    echo "Detected: Windows Subsystem for Linux (WSL)"
fi

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo ""
    echo "Docker is not installed."
    echo ""
    if [ "$IS_WSL" = true ]; then
        echo "For WSL, you need Docker Desktop for Windows:"
        echo "1. Install Docker Desktop from: https://www.docker.com/products/docker-desktop"
        echo "2. Enable 'Use the WSL 2 based engine' in Docker Desktop settings"
        echo "3. Enable integration with your WSL distro in Docker Desktop settings"
        echo "4. Restart Docker Desktop"
        echo "5. Run this script again"
    else
        echo "Please run the following commands:"
        echo "  sudo apt update"
        echo "  sudo apt install -y docker.io"
        echo "  sudo systemctl start docker"
        echo "  sudo systemctl enable docker"
        echo "  sudo usermod -aG docker $USER"
        echo ""
        echo "After adding yourself to the docker group, log out and log back in, then run this script again."
    fi
    exit 1
fi

# Check Docker daemon status first (using systemctl, doesn't require permissions)
DOCKER_DAEMON_RUNNING=false
if systemctl is-active --quiet docker 2>/dev/null || sudo systemctl is-active --quiet docker 2>/dev/null; then
    DOCKER_DAEMON_RUNNING=true
fi

# Check Docker daemon and permissions
DOCKER_ACCESSIBLE=false
if docker info &> /dev/null; then
    DOCKER_ACCESSIBLE=true
elif [ "$DOCKER_DAEMON_RUNNING" = true ] && sudo docker info &> /dev/null 2>&1; then
    echo ""
    echo "Docker is running but you don't have permission to access it."
    echo ""
    if [ "$IS_WSL" = true ]; then
        echo "For WSL with Docker Desktop:"
        echo "1. Make sure Docker Desktop is running on Windows"
        echo "2. Check Docker Desktop settings -> Resources -> WSL Integration"
        echo "3. Ensure your WSL distro is enabled"
        echo "4. Restart Docker Desktop if needed"
    else
        echo "You need to be added to the docker group:"
        echo "  sudo usermod -aG docker $USER"
        echo ""
        echo "Then refresh your session:"
        echo "  - Log out and log back in, OR"
        echo "  - Run: newgrp docker, OR"
        echo "  - Close and reopen your terminal"
        echo ""
        echo "Alternatively, you can use sudo (not recommended):"
        read -p "Continue with sudo? [y/N]: " use_sudo
        if [[ "$use_sudo" =~ ^[Yy]$ ]]; then
            DOCKER_CMD="sudo docker"
            DOCKER_ACCESSIBLE=true
        else
            exit 1
        fi
    fi
else
    if [ "$DOCKER_DAEMON_RUNNING" = false ]; then
        echo ""
        echo "Docker daemon is not running."
        echo ""
        if [ "$IS_WSL" = true ]; then
            echo "For WSL:"
            echo "1. Start Docker Desktop on Windows"
            echo "2. Wait a few seconds for it to initialize"
            echo "3. Run this script again"
        else
            echo "Please start Docker with:"
            echo "  sudo systemctl start docker"
            echo "  sudo systemctl enable docker  # to start on boot"
        fi
        exit 1
    else
        # Daemon is running but we can't access it
        echo ""
        echo "Docker daemon is running but you don't have permission to access it."
        echo ""
        if [ "$IS_WSL" = true ]; then
            echo "For WSL with Docker Desktop:"
            echo "1. Make sure Docker Desktop is running on Windows"
            echo "2. Check Docker Desktop settings -> Resources -> WSL Integration"
            echo "3. Ensure your WSL distro is enabled"
            echo "4. Restart Docker Desktop if needed"
        else
            echo "You need to be added to the docker group:"
            echo "  sudo usermod -aG docker $USER"
            echo ""
            echo "Then refresh your session:"
            echo "  - Log out and log back in, OR"
            echo "  - Run: newgrp docker, OR"
            echo "  - Close and reopen your terminal"
            echo ""
            echo "Alternatively, you can use sudo (not recommended):"
            read -p "Continue with sudo? [y/N]: " use_sudo
            if [[ "$use_sudo" =~ ^[Yy]$ ]]; then
                DOCKER_CMD="sudo docker"
                DOCKER_ACCESSIBLE=true
            else
                exit 1
            fi
        fi
        if [ "$DOCKER_ACCESSIBLE" = false ]; then
            exit 1
        fi
    fi
fi

# Set docker command
if [ -z "$DOCKER_CMD" ]; then
    DOCKER_CMD="docker"
fi

# Check if user is in docker group (for non-WSL, non-sudo cases)
if [ "$IS_WSL" = false ] && [ "$DOCKER_CMD" = "docker" ] && ! groups | grep -q docker; then
    echo ""
    echo "Note: You're not in the docker group, but Docker is accessible."
    echo "This might be because you're using sudo or Docker Desktop integration."
    echo ""
fi

echo "Docker is installed and accessible ✓"

# Use ROS Noetic (ROS1) - Required for this project
echo ""
echo "Building ROS Noetic (ROS1) simulation environment..."
echo "Note: This project uses ROS 1 Noetic with Gazebo Classic for simulation."
DOCKERFILE="Dockerfile.noetic"
IMAGE_NAME="noetic-gazebo-rosa"
ROS_DISTRO="noetic"

echo ""
echo "Building Docker image: $IMAGE_NAME"
echo "This may take 15-30 minutes depending on your internet connection..."
echo ""

# Build the Docker image
$DOCKER_CMD build -t $IMAGE_NAME -f ./$DOCKERFILE .

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "✓ Docker image built successfully!"
    echo "========================================="
    echo ""
    echo "Image name: $IMAGE_NAME"
    echo "ROS distribution: $ROS_DISTRO"
    echo ""
    echo "To run the simulation, use:"
    echo "  ./run_simulation.sh"
    echo ""
    echo "Or manually:"
    echo "  xhost +local:root"
    echo "  docker run -it --rm \\"
    echo "    --net=host \\"
    echo "    -e DISPLAY=\$DISPLAY \\"
    echo "    -e QT_X11_NO_MITSHM=1 \\"
    echo "    -v /tmp/.X11-unix:/tmp/.X11-unix \\"
    echo "    $IMAGE_NAME"
else
    echo ""
    echo "========================================="
    echo "✗ Docker build failed!"
    echo "========================================="
    exit 1
fi

