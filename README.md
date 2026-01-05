```markdown
# LIMO Office Simulation Setup Guide

## Step 1: Start the Docker Container (Host Machine)

**Where to run this:** Open a standard terminal on your Ubuntu computer.

We need to launch Docker with specific permissions to allow it to access your graphics card (to prevent freezing) and your display (to show the simulation window).

```bash
# 1. Allow Docker to access your local screen
xhost +local:root

# 2. Launch the container
# --device /dev/dri: Enables hardware acceleration (Crucial for Gazebo)
# LIBGL_ALWAYS_SOFTWARE=1: Prevents driver conflicts if GPU fails
sudo docker run -it \
    --net=host \
    --privileged \
    --device /dev/dri \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="LIBGL_ALWAYS_SOFTWARE=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    limo_office \
    /bin/bash
```

## Step 2: Create the "Safe Launch" Script (Inside Docker)

**Where to run this:** Inside the Docker container (your prompt should look like `root@hostname...`).

The standard office world file has "Shadows" enabled, which causes Docker to crash or freeze on many computers. We will create a script that automatically:

1. Downloads the office environment.
2. Disables shadows to fix the crashing.
3. Spawns the robot safely.

Copy and paste this entire code block into your terminal:

```bash
cat << 'EOF' > start_simulation.sh
#!/bin/bash
set -e

# --- 1. SETUP & CLEANUP ---
source /root/catkin_ws/devel/setup.bash
echo ">>> Killing old Gazebo processes..."
pkill -f gazebo || true
pkill -f rosmaster || true
sleep 2

# --- 2. DOWNLOAD OFFICE ASSETS ---
# Automatically downloads the map if you don't have it
if [ ! -d "/root/gazebo_assets" ]; then
    echo ">>> Downloading Office Map..."
    git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git /root/gazebo_assets
fi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/gazebo_assets/models

# --- 3. CREATE SAFE WORLD FILE ---
# We create a new world file that forces Shadows=False
cat << 'XML' > /root/gazebo_assets/worlds/safe_office.world
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <include>
        <uri>model://office_env_w</uri>
        <pose>0 0 0 0 0 0</pose>
    </include>
    <scene>
        <shadows>false</shadows>
        <ambient>0.5 0.5 0.5 1</ambient>
    </scene>
    <physics type='ode'>
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
  </world>
</sdf>
XML

# --- 4. LAUNCH GAZEBO (PAUSED) ---
echo ">>> Launching World (Paused to prevent crash)..."
roslaunch gazebo_ros empty_world.launch world_name:=/root/gazebo_assets/worlds/safe_office.world paused:=true gui:=true verbose:=true &
GAZEBO_PID=$!

echo ">>> Waiting 15 seconds for world to load..."
sleep 15

# --- 5. SPAWN ROBOT ---
echo ">>> Spawning Robot..."
ROBOT_FILE=$(find /root/catkin_ws/src -name "limo_mycobot.xacro" | head -n 1)
rosparam set robot_description "$(xacro $ROBOT_FILE)"

# Spawn at X=2.0, Y=2.0 (Open Hallway)
/usr/bin/python3 /opt/ros/noetic/lib/gazebo_ros/spawn_model -urdf -param robot_description -model limo_cobot -x 2.0 -y 2.0 -z 0.2

# --- 6. START CONTROLLERS ---
echo ">>> Starting Robot Controllers..."
CONFIG_FILE=$(find /root/catkin_ws/src -name "limo_cobot_control.yaml" | head -n 1)
if [ -z "$CONFIG_FILE" ]; then
    CONFIG_FILE=$(find /root/catkin_ws/src -name "limo_control.yaml" | head -n 1)
fi
rosparam load "$CONFIG_FILE"
rosrun controller_manager spawner joint_state_controller limo_state_controller limo_arm_controller limo_base_controller

echo "SUCCESS! Go to Gazebo and press PLAY."
wait $GAZEBO_PID
EOF

# Make the script executable
chmod +x start_simulation.sh
```

## Step 3: Run the Simulation (Inside Docker)

Now that the script is created, run it to start the simulation:

```bash
./start_simulation.sh
```

### Crucial Troubleshooting

**"Not Responding" is Normal:** When the window first opens, Ubuntu might say "Application is not responding" for 10-20 seconds. Do not force close it. It is just loading the heavy office textures.

**The Robot is Missing/Broken?** The simulation launches PAUSED. The robot will not appear correctly until you click the Play Button at the bottom left of the Gazebo window.

## Step 4: Drive the Robot (Host Machine)

**Where to run this:** Open a new terminal window on your Ubuntu computer (Host).

Since your first terminal is busy running the simulation, you need a second terminal to send keyboard commands.

```bash
# 1. Log into the running container
sudo docker exec -it $(sudo docker ps -q | head -n 1) bash

# 2. Start the Keyboard Driver
source /root/catkin_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
```
