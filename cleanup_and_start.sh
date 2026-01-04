#!/bin/bash
# Cleanup and fresh start script - Step by step

echo "========================================="
echo "Step 1: Cleaning Up Old Containers"
echo "========================================="
echo "Why: We need to stop all old containers so we start fresh"
echo ""

# Stop all running containers with our image
echo "Stopping all running containers..."
CONTAINERS=$(sudo docker ps -q --filter ancestor=noetic-gazebo-rosa 2>/dev/null)

if [ -z "$CONTAINERS" ]; then
    echo "✓ No containers running"
else
    echo "Found containers: $CONTAINERS"
    for container in $CONTAINERS; do
        echo "Stopping container $container..."
        sudo docker stop $container
    done
    echo "✓ All containers stopped"
fi

echo ""
echo "Removing stopped containers..."
sudo docker container prune -f --filter ancestor=noetic-gazebo-rosa
echo "✓ Cleanup complete"
echo ""

echo "========================================="
echo "Step 2: Verifying Docker is Ready"
echo "========================================="
echo "Why: We need to make sure Docker is working before starting"
echo ""

if sudo docker ps &> /dev/null; then
    echo "✓ Docker is accessible"
else
    echo "✗ Cannot access Docker. Please check permissions."
    exit 1
fi

echo ""
echo "========================================="
echo "Step 3: Starting Fresh Simulation"
echo "========================================="
echo "Why: Now we start ONE clean simulation instance"
echo ""

# Check if DISPLAY is set
if [ -z "$DISPLAY" ]; then
    echo "⚠ Warning: DISPLAY not set. GUI may not work."
fi

# Allow X11
xhost +local:root 2>/dev/null

echo "Starting simulation container..."
echo "This will open Gazebo and RViz"
echo ""

sudo docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  noetic-gazebo-rosa

