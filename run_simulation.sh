#!/bin/bash
# Run script for llm-controlled-robots simulation

set -e

# Detect if we need sudo for docker
if docker ps >/dev/null 2>&1; then
    DOCKER_CMD="docker"
elif sudo docker ps >/dev/null 2>&1; then
    DOCKER_CMD="sudo docker"
    echo "Note: Using sudo for Docker commands"
else
    echo "Error: Cannot access Docker. Please check Docker installation and permissions."
    exit 1
fi

# Detect which Docker images are available
if $DOCKER_CMD images | grep -q "noetic-gazebo-rosa"; then
    IMAGE_NAME="noetic-gazebo-rosa"
    ROS_DISTRO="noetic"
elif $DOCKER_CMD images | grep -q "humble-limo-cobot"; then
    IMAGE_NAME="humble-limo-cobot"
    ROS_DISTRO="humble"
elif $DOCKER_CMD images | grep -q "foxy-limo-cobot"; then
    IMAGE_NAME="foxy-limo-cobot"
    ROS_DISTRO="foxy"
else
    echo "No Docker image found. Please run ./setup.sh first to build the image."
    exit 1
fi

echo "========================================="
echo "Starting LLM Controlled Robots Simulation"
echo "========================================="
echo "Using image: $IMAGE_NAME"
echo "ROS distribution: $ROS_DISTRO"
echo ""

# Check if DISPLAY is set
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY environment variable is not set."
    echo "This is required for GUI applications (Gazebo, RViz)."
    echo "If you're using SSH, you may need X11 forwarding."
    exit 1
fi

# Allow X11 connections
echo "Setting up X11 forwarding..."
xhost +local:root 2>/dev/null || echo "Warning: Could not set xhost. GUI may not work."

# Run the Docker container
echo "Starting Docker container..."
echo ""

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
echo "Mounting project into container at /workspace/llm-controlled-robots"
echo "Project root: $PROJECT_ROOT"
echo ""

if [ "$ROS_DISTRO" = "noetic" ]; then
    # ROS Noetic - runs Gazebo Classic simulation automatically
    $DOCKER_CMD run -it --rm \
        --net=host \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$PROJECT_ROOT:/workspace/llm-controlled-robots" \
        $IMAGE_NAME
else
    # ROS 2 (Humble/Foxy) - starts interactive shell
    $DOCKER_CMD run -it --rm \
        --net=host \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$PROJECT_ROOT:/workspace/llm-controlled-robots" \
        $IMAGE_NAME
fi

