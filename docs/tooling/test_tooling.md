# Testing ROSA Tooling

This guide explains how to test the ROSA tooling workflow end-to-end for the LIMO cobot project.

## Prerequisites

- ROS master is running and reachable (`ROS_MASTER_URI` set correctly).
- Python environment has required packages installed (for example `rospy`, `tf2_ros`, `langchain_openai`, and robot-specific dependencies).
- `OPENAI_API_KEY` is set.
- You are in the project root: `llm-pick-me-bots`.

## 1) Start ROSA and verify tool discovery

Run:

```powershell
python .\src\launch_rosa.py
```

`launch_rosa.py` prints loaded tools. Confirm your expected tool names appear in the output.

## 2) Smoke-test tools from the ROSA chat

In the chat prompt, run one command at a time:

1. `Start cam coverage with range 1.5 meters.`
2. `Get autonomy status.`
3. `Reset camera coverage.`
4. `Start object finder with prompt 'a water bottle' and threshold 0.2.`
5. `Update object query to 'a blue cube'.`
6. `Stop all autonomy nodes.`

Expected result:

- Each tool call returns a success/failure message.
- Status calls return a readable state payload (often JSON-like text).

## 3) Validate ROS graph in parallel terminal

Use a second terminal while ROSA is running:

```powershell
rostopic list
rosservice list
```

Check that expected topics/services appear when nodes are started (for example coverage/object-related topics and reset service).

## 4) Validate logs and node health

- Review tool/node logs created by your tooling layer.
- If a tool reports startup failure, inspect its log first.
- Confirm nodes stay alive after startup and stop cleanly when requested.

## 5) Integration flow test (recommended)

Run this sequence:

1. Start coverage node.
2. Start exactly one exploration planner.
3. Start object finder.
4. Poll status until object flags update.
5. Stop all nodes.

Then run blue-cube grasp mode separately to isolate failures:

1. Start blue-cube grasper.
2. Observe behavior and logs.
3. Stop that node.

## 6) Common failure checks

- Missing Python dependency in active environment.
- TF frame mismatch (`map`, `base_link`, camera frame names).
- Incorrect topic names or camera info topic.
- Robot connectivity/permissions issues.
- Hardware dependency missing (for example arm serial device, gripper access).

## Quick test prompt set

Copy/paste one-by-one in ROSA chat:

```text
Start cam coverage with range 1.5 meters.
Get autonomy status.
Start object finder with prompt "a blue cube".
Update object query to "a blue cube".
Get autonomy status.
Stop all autonomy nodes.
```

## Notes

- Keep tests incremental. Start with discovery, then smoke tests, then integration.
- Avoid running multiple planners simultaneously in first validation pass.
- If behavior is inconsistent, restart ROSA and repeat with fewer active nodes.
