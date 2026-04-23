# Map Update ROSA Tool

This document describes the new ROSA map-update functionality that lets the robot explore and save a fresh occupancy map.

## What was added

- ROSA tool: `build_and_save_map(...)` in `src/limo_llm_control/tools/navigation.py`
- Robot-side save service node: `map_update_manager_node.py` in `catkin_ws/src/limo_rosa_bridge/scripts/`
- Service endpoint: `/map_update_manager/save_map` (`std_srvs/Trigger`)
- Launch integration: `autonomy_core.launch` now starts the map update manager node

## How it works

`build_and_save_map(...)` orchestrates the map update flow:

1. Optionally reset camera coverage and exploration state.
2. Enable frontier exploration.
3. Wait for `exploration_duration_s` while the robot explores.
4. Disable exploration.
5. Set `/map_update_manager/next_map_name`.
6. Call `/map_update_manager/save_map`.

The save service runs:

- `rosrun map_server map_saver -f <output_prefix> map:=/map`

and writes:

- `<output_prefix>.yaml`
- `<output_prefix>.pgm`

Default output directory is:

- `catkin_ws/src/limo_rosa_bridge/maps`

## Usage from ROSA

Example:

```text
build_and_save_map(map_name="lab_v2", exploration_duration_s=180)
```

Tool parameters:

- `map_name`: base filename for the saved map (letters, numbers, `_`, `-`)
- `exploration_duration_s`: exploration time before saving
- `reset_coverage_first`: clear coverage map first
- `reset_explorer_goal_first`: reset frontier goal selection first

## Prerequisites

- Run a live-SLAM launch that publishes `/map` (for example `rosa_bridge.launch`).
- Ensure `autonomy_core.launch` is active so `/map_update_manager/save_map` exists.
- Build and source after changes:

```bash
cd ~/llm-controlled-robots/catkin_ws
catkin_make
source devel/setup.bash
```

## Notes and limitations

- This updates maps by exploration + save; it does not merge maps post-hoc.
- If `/map` is not being published, map saving will fail or time out.
- Saved maps are reused later with static-map launches by passing `map_file:=.../your_map.yaml`.
