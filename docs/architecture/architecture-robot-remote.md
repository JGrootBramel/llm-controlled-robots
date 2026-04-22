# Robot vs Remote PC Architecture (ROSA + ROS1 Noetic)

## Purpose
This document clarifies which components should run on the **LIMO Cobot (Robot)** versus the **Remote PC**, and discusses communication options between them.

Constraints:
- Robot: ROS 1 Noetic on Ubuntu 20.04, Python 3.8 (cannot upgrade)
- Remote PC: can run Python 3.9+ and hosts ROSA + LLM orchestration

Design goals:
- Keep robot-side control stable and vendor-compatible
- Keep LLM/ROSA logic off-robot
- Avoid streaming high-rate sensor data through fragile links when possible
- Expose “capabilities” as clean ROS interfaces (services/actions/topics)

---

## Recommended Diagram Types

### 1) Deployment Diagram (What runs where)
Use this to document placement: robot vs remote PC, processes, and network connections.

**Mermaid (Deployment View):**
```mermaid
flowchart LR
  subgraph Robot["LIMO Cobot (ROS1 Noetic, Python 3.8)"]
    Drivers[limo_bringup + astra_camera + pymycobot]
    Nav[gmapping / amcl + move_base]
    CAM[cam_coverage_node]
    FX[frontier_explorer_node]
    RCD[red_cube_detector_node]
    APP[approach_object_node]
    ARM[arm_control_node]
    BRIDGE[rosbridge_server]
    RM[(roscore)]
  end

  subgraph Remote["Remote PC (Python 3.9+, ROSA + LLM)"]
    ROSA[ROSA tools: motion, navigation, perception, manipulation, mission]
    LLM[LLM orchestrator]
    UI[rviz / logs / rqt]
  end

  Remote ---|network| Robot
  ROSA -->|std_srvs.SetBool| FX
  ROSA -->|std_srvs.Trigger| APP
  ROSA -->|std_srvs.Trigger| ARM
  ROSA -->|std_srvs.SetBool / Trigger| RCD
  ROSA -->|PoseStamped| Nav
  CAM --> FX
  FX --> Nav
  RCD --> APP
  RCD --> ARM
  APP --> Nav
  ARM --> Drivers
  Drivers --- RM
  Nav --- RM
  CAM --- RM
  FX --- RM
  RCD --- RM
  APP --- RM
  ARM --- RM
  BRIDGE --- RM
```

See [`node-contracts.md`](./node-contracts.md) for the full per-node API.

## Component Placement (Guideline)
**Prefer on the Robot** (or on a nearby compute node on the same LAN using native ROS)

These components are bandwidth/latency sensitive and often require tight integration with drivers and TF:

- Hardware drivers and low-level controllers (base/arm/gripper)
- Sensor drivers (camera, lidar, imu)
- TF tree publishers and robot state publisher
- SLAM/localization if used during navigation
- Perception that consumes high-rate streams (object detection, tracking, depth processing)
- Anything safety-critical (emergency stop logic, collision stops, watchdogs)

**Prefer on the Remote PC**

These components are compute-flexible and benefit from rapid iteration:

- ROSA + LLM orchestration and planning
- High-level task logic (“fetch object”, “navigate to room”)
- Calling robot-side capabilities via services/actions
- UI/teleop/monitoring (rviz, dashboards), logging/metrics
- Tooling that queries state and triggers discrete capabilities (not continuous streaming)

Key principle: the remote PC should call capabilities (services/actions), not pull raw streams unless necessary.

## Docker replication of robot–remote

The same split (robot = Noetic / Python 3.8, remote = Python 3.9+ with jpl-rosa) can be run in Docker using Compose. Two containers: **robot** (Gazebo + limo_cobot_sim + limo_rosa_bridge) and **remote** (ROSA + limo_llm_control). The remote connects to the robot via `ROS_MASTER_URI=http://robot:11311`. See [README](../../README.md) (Simulation → Docker Compose) and [sim/docker/docker-compose.yml](../../sim/docker/docker-compose.yml).

## Communication Options (Robot ↔ Remote PC)
