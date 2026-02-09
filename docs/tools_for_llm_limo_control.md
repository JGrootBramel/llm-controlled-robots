## Tools for LLM-Based Limo Cobot Control

- **ROSA core tools**: High-level task APIs, action clients/servers, and robot capability abstractions used by the LLM to issue semantically rich commands (e.g., navigation, arm control, perception queries).
- **Perception / object finding**: RGB-D or stereo camera drivers, object detection/segmentation models (e.g., YOLO/Mask R-CNN wrappers), 3D pose estimation, and scene understanding services exposed as ROSA tools for "find object X" or "pick nearest graspable object".
- **Navigation and localization**: Mapping and localization stack (e.g., SLAM, AMCL), path planning, obstacle avoidance, and waypoint management tools that let the LLM request motion like "go to the table" or "navigate to pose (x, y, θ)".
- **Manipulation and grasping**: Motion planning (MoveIt or equivalent), grasp synthesis, IK/trajectory generation, and gripper control tools for "pick", "place", and "handover" primitives.
- **World and task memory**: A lightweight world model / blackboard service, key–value memory, and task history logging so the LLM can track object locations, goals, and intermediate states across turns.
- **Safety and supervision**: Safety checkers (joint/velocity limits, collision checks), emergency stop and pause/resume tools, human-in-the-loop approval hooks, and sandbox/simulation tools for dry‑running plans.
- **Debugging and monitoring**: Telemetry streaming (joint states, battery, diagnostics), log and replay tools, visualization hooks (e.g., RViz-like views or web dashboards), and scripted test scenarios for validating LLM behaviors.

