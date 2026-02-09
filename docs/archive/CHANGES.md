## December 2025 integration notes (what changed and why)

### 1) Repo mount into Docker
- **Change**: `run_simulation.sh` now mounts the repo into the container at `/workspace/llm-pick-me-bots`.
- **Why**: Running controllers from host paths (e.g., `/home/student/...`) fails inside Docker. Mounting makes scripts available and editable from the host while running in-container.

### 2) `start_rosa.sh` made robust
- **Change**: `start_rosa.sh` now:
  - Activates `/opt/rosa-venv`
  - Sources ROS Noetic + the workspace
  - Auto-installs missing Python deps into the venv (`rospkg`, `catkin_pkg`, `importlib-metadata`, and provider SDKs)
  - Prompts for missing API keys without echo
  - Prefers running the controller from `/workspace/llm-pick-me-bots/docs/rosa_robot_controller.py`
- **Why**: The ROSA venv was missing ROS python deps and provider SDKs, causing runtime import failures.

### 3) OpenAI support (standard key) added/confirmed
- **Change**: OpenAI path in `docs/rosa_robot_controller.py` uses `openai_api_key=...` and supports `OPENAI_MODEL` (default: `gpt-4o-mini`).
- **Why**: Standard OpenAI should work similarly to Gemini for testing. OpenAI is also more tolerant of tool schemas than Gemini.

### 4) Gemini tool schema failures addressed
- **Observed runtime issue**: Gemini rejected ROSA’s full tool list with 400 errors referencing tool JSON schema fields (`items` / `v__args`).
- **Change**: `start_rosa.sh` now supports `ROSA_GEMINI_TOOL_MODE`:
  - `safe` (default): only binds the robot-control tools for Gemini (most reliable)
  - `all`: attempts to use full ROSA toolset with Gemini (may still fail due to strict schema validation)
- **Why**: Gemini validates tool schemas strictly; many default ROSA tools have schemas that are fine for OpenAI but rejected by Gemini.

### 5) Safer base motion behavior
- **Change**: Base motion tools publish `/cmd_vel` for a short duration and then auto-stop (default duration 1s), and `stop_base` publishes stop repeatedly for reliability.
- **Why**: One-shot `/cmd_vel` publishes can leave the robot turning/moving until another command stops it.



