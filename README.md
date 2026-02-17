# LLM-Controlled LIMO Cobot (ROS1 + ROSA)

This repository contains everything required to reproduce our project:\
controlling an AgileX LIMO Cobot (ROS1 Noetic, Ubuntu 20.04, Python
3.8)\
using an LLM (GPT-4 class model) via ROSA (NASA JPL) running on a remote
PC.

The system is intentionally split into:

-   🟢 Robot side (ROS1 Noetic, Python 3.8) → hardware, drivers,
    perception, capability APIs\
-   🔵 Remote side (Python 3.9+) → ROSA tools, LLM orchestration,
    high-level task logic\
-   🟡 Simulation (Gazebo Classic via Docker) → reproducible testing
    without hardware

------------------------------------------------------------------------

# 1. Architecture Overview

## Deployment View

Robot: - Drivers & Controllers - Sensors (Camera, LiDAR, IMU) -
Perception Nodes - Capability Interfaces (services/actions/topics) - ROS
Master

Remote PC: - ROSA Tools - LLM Orchestrator - Monitoring / UI

Design Principles: - Robot exports capabilities, not raw perception. -
Remote performs high-level reasoning. - High-rate perception runs on
robot. - ROSA tools call structured ROS services/actions.

------------------------------------------------------------------------

# 2. Repository Structure
```
llm-controlled-robots/ 
├─ catkin_ws/   # Robot-side ROS1 workspace 
├─ src/         # Remote PC (ROSA + LLM application) 
├─ sim/         # Gazebo Classic + Docker setup 
├─ scripts/     # Bootstrap & run scripts 
├─ docs/        #Architecture and interface documentation 
├─ .env.example 
└─ README.md
```

------------------------------------------------------------------------

# 3. System Requirements

## Robot

-   Ubuntu 20.04
-   ROS Noetic
-   Python 3.8

## Remote PC

-   Python 3.9+
-   Network access to robot
-   OpenAI API key

------------------------------------------------------------------------

# 4. Quick Start

## Robot

cd catkin_ws\
catkin_make\
source devel/setup.bash

roslaunch limo_bringup bringup.launch

------------------------------------------------------------------------

## Remote

cd src\
python3.9 -m venv venv\
source venv/bin/activate\
pip install -r requirements.txt

Configure .env:

OPENAI_API_KEY=your_key\
ROS_MASTER_URI=http://`<robot_ip>`{=html}:11311\
ROS_IP=`<remote_pc_ip>`{=html}

Run:

python -m limo_llm_control.main --profile robot

------------------------------------------------------------------------

## Simulation

cd sim/docker\
docker compose up --build

Then:

python -m limo_llm_control.main --profile sim

------------------------------------------------------------------------

# 5. Communication Options

Option A --- Native ROS1 Networking (Recommended) - Full compatibility
(topics/services/actions) - Requires correct ROS networking
configuration

Option B --- rosbridge - Easier across network boundaries - Not ideal
for high-rate streams

------------------------------------------------------------------------

# 6. Capability API Contract

Example interfaces:

/detect_objects (Service) Returns list of detected objects with pose.

/go_to_pose (Action) Input: PoseStamped

/cmd_vel (Topic) Velocity control fallback.

------------------------------------------------------------------------

# 7. Reproducibility Checklist

-   Build catkin_ws
-   Verify ROS networking
-   Install remote dependencies
-   Configure .env
-   Verify topics/services
-   Run remote entrypoint
-   Test motion
-   Test perception

------------------------------------------------------------------------

# 8. Security Note

ROS1 has no built-in security.\
Use only in trusted LAN environments.

------------------------------------------------------------------------

# 9. License

Specify your license here.
