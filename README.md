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

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

### Online SLAM + mapping (gmapping)

For live mapping with `gmapping` and autonomy tools:

```bash
roslaunch limo_rosa_bridge rosa_bridge.launch
```

This starts:

- LIMO base drivers (`limo_start.launch`)
- `limo_gmapping.launch` (online SLAM, publishes `/map`)
- `limo_move_base.launch` (navigation)
- Camera + rosbridge
- Core autonomy nodes (`autonomy_core.launch`, `autonomy_perception.launch`)

### Static map navigation (saved map + AMCL)

After you have built and saved a map (e.g. `limo_lab_map.yaml` in `limo_rosa_bridge/launch`),
you can restart the robot using the static map navigation launch:

```bash
roslaunch limo_rosa_bridge test_map.launch
```

This starts:

- LIMO base drivers
- `map_server` with your saved map
- `limo_amcl.launch` (localization on the static map)
- `limo_move_base.launch` (navigation)
- Camera + rosbridge + autonomy tools

------------------------------------------------------------------------

## Remote

```bash
cd ~/llm-controlled-robots
python3.9 -m venv venv
source venv/bin/activate
pip install -r requirements-remote.txt
```

Configure `.env` in the repo root:

```bash
OPENAI_API_KEY=your_key
ROS_MASTER_URI=http://<robot_ip>:11311
ROS_IP=<remote_pc_ip>
```

Run the ROSA LIMO agent (robot profile):

```bash
cd ~/llm-controlled-robots
python src/launch_rosa.py
```

------------------------------------------------------------------------

## Simulation

### Docker Compose (robot + remote)

Replicates the two-machine setup in Docker: one container runs the robot stack (Gazebo + bridge), the other runs the ROSA/LLM app and connects via ROS.

1. **From repo root**, create `.env` from the example and set your API key:
   ```bash
   cp .example.env .env
   # Edit .env and set OPENAI_API_KEY
   ```

2. **Build and start** (robot and remote services):
   ```bash
   docker compose -f sim/docker/docker-compose.yml up --build
   ```
   - **robot** container: Gazebo + limo_cobot_sim + limo_rosa_bridge (ROS master).
   - **remote** container: waits for roscore, then starts `python -m limo_llm_control.main` (ROSA chat).

3. **Optional (GUI)**: To show Gazebo/RViz, run with X11:
   - Linux: ensure `xhost +local:root`, then in `sim/docker/docker-compose.yml` uncomment the `DISPLAY`, `QT_X11_NO_MITSHM`, and `/tmp/.X11-unix` volume for the `robot` service; pass `-e DISPLAY` when needed.
   - Windows: use WSL2 and Docker Desktop with WSL integration; same X11 forwarding as in [docs/quickstart](docs/quickstart.md).

4. **Single-container (legacy)**  
   From repo root: `./setup.sh` then `./run_simulation.sh`. Then in another terminal, `docker exec -it <container> bash` and run `bash /workspace/llm-controlled-robots/start_rosa.sh`.  
   Or: `python -m limo_llm_control.main --profile sim` inside that container.

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
