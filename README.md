# llm-pick-me-bots
LIMO Robot Office Simulation (Ubuntu 22.04)

This guide documents how to launch the LIMO Cobot inside the custom GitHub Office Environment without freezing or crashing.

Tested OS: Ubuntu 22.04.5 LTS Prerequisites: Docker installed, X11 Display Server (Standard on Ubuntu).
 Step 1: Start the Docker Container

Run these commands on your Host Machine (Ubuntu Terminal) to start the container with hardware acceleration enabled.
Bash

# 1. Allow Docker to access your local screen
xhost +local:root

# 2. Launch the container (Hardware Accel + Software Fallback)
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

 Step 2: Create the Launch Script

Once you are inside the container (root@...), copy and paste this entire block once. This creates a file named start_simulation.sh that handles the downloading, fixing, and launching automatically.
Bash

cat << 'EOF' > start_simulation.sh
#!/bin/bash
set -e

# --- 1. SETUP & CLEANUP ---
source /root/catkin_ws/devel/setup.bash
echo ">>> Killing old Gazebo processes..."
pkill -f gazebo
pkill -f rosmaster
sleep 2

# --- 2. DOWNLOAD OFFICE ASSETS ---
# Checks if map exists, if not, downloads it from GitHub
if [ ! -d "/root/gazebo_assets" ]; then
    echo ">>> Downloading Office Map..."
    git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git /root/gazebo_assets
fi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/gazebo_assets/models

# --- 3. CREATE SAFE WORLD FILE ---
# Creates a shadow-less world file to prevent 'Not Responding' freeze
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
# Fallback if specific config not found
if [ -z "$CONFIG_FILE" ]; then
    CONFIG_FILE=$(find /root/catkin_ws/src -name "limo_control.yaml" | head -n 1)
fi
rosparam load "$CONFIG_FILE"
rosrun controller_manager spawner joint_state_controller limo_state_controller limo_arm_controller limo_base_controller

echo " SUCCESS! Go to Gazebo and press PLAY ."
wait $GAZEBO_PID
EOF

# Make the script executable
chmod +x start_simulation.sh

 Step 3: Run the Simulation

Now simply run the script you just created:
Bash

./start_simulation.sh

 Important:

    Wait: The window might say "Not Responding" for 10-20 seconds while loading textures. Do not close it.

    Play: The robot will not appear correctly until you click the Play Button  at the bottom of the Gazebo window.

Step 4: Drive the Robot

To move the robot, open a New Terminal on your Host Machine (Ubuntu) and run:
Bash

# 1. Log into the running container
sudo docker exec -it $(sudo docker ps -q | head -n 1) bash

# 2. Start Keyboard Control
source /root/catkin_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

