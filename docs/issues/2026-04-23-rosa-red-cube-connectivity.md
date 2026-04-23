# ROSA Red Cube Detection False Negative and Service Reachability

## Date
2026-04-23

## Summary
ROSA reported that no red cube was detected and could not enable the red-cube detector service, even while robot-side components were running and RViz showed red-cube markers.

## Symptoms
- `healthcheck_autonomy_stack` returned READY.
- ROSA responses said no red-cube detection message had been received.
- `enable_red_cube_detector` intermittently returned service unavailable.
- ROS graph inspection showed topic/service presence, but endpoint URIs included hostnames like `master` that may not resolve from the remote ROSA PC.

## Impact
- False-negative perception answers in ROSA.
- Mission flow interruptions when enabling detector or reading detection status.
- Operator confusion because RViz/robot state and ROSA status disagreed.

## Root Cause
Two issues combined:

1. **Perception status logic gap**
   - `is_red_cube_found` could return `found=False` when `/red_cubes/found` was false, even if `/red_cubes/latest_pose` was available and usable.

2. **ROS1 network endpoint reachability**
   - ROS master graph visibility alone was insufficient.
   - Some node/service XML-RPC endpoints were advertised with non-LAN hostnames (for example `master`), causing service/topic connection failures from the remote machine despite appearing healthy in master state.

## Fixes Applied
1. **Perception tool hardening**
   - Updated `is_red_cube_found` to cross-check both:
     - `/red_cubes/found`
     - `/red_cubes/latest_pose`
   - A valid latest pose now yields a positive result even during transient `found` flicker.

2. **Connectivity diagnostics**
   - Added host-resolution hints in perception tools for topic/service failures.
   - Failure responses now explain when peer hosts are not resolvable and suggest setting robot `ROS_IP`/`ROS_HOSTNAME` to LAN IP.

3. **Robot launch environment normalization**
   - Updated robot startup scripts to set:
     - `ROS_IP` to robot LAN IP (if unset)
     - `ROS_HOSTNAME` to same value (if unset)
   - This prevents publishing unreachable XML-RPC endpoint hostnames for remote clients.

## Files Changed
- `src/limo_llm_control/tools/perception.py`
- `sync_git_main_to_robot.sh`
- `scripts/start_robot_rosa_full_clean.sh`
- `src/rosa_agent.py` (startup healthcheck integration)
- `src/limo_llm_control/tools/diagnostics.py` (healthcheck tool)

## Verification Steps
1. Relaunch robot stack using updated startup path.
2. Start ROSA with `./start_rosa.sh`.
3. Confirm startup health line reports READY.
4. In ROSA:
   - Ask detection status (`is_red_cube_found` path).
   - Enable detector (`enable_red_cube_detector` path).
5. If failure persists, inspect returned connectivity hint for unresolvable endpoint hostnames.

## Follow-up Recommendation
Add a dedicated ROSA tool to report endpoint hostnames for key publishers/services (`lookupNode` / `lookupService`) so networking misconfiguration is visible before mission execution.
