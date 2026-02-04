# ROSA Integration Guide

Complete guide for using ROSA (Robot Operating System Agent) to control the LIMO Cobot with natural language.

## Overview

ROSA enables natural language control of the robot. You can give commands like:
- "Move the robot forward"
- "Turn left"
- "Stop the robot"
- "What is the current state of the robot?"

## Prerequisites

1. **Simulation must be running**
   - Gazebo Classic
   - RViz with MoveIt
   - ROS nodes active

2. **LLM API Key**
   - OpenAI API key (for GPT-4) OR
   - Google Gemini API key (for Gemini 2.5 Flash)
   - Get keys from:
     - OpenAI: https://platform.openai.com/api-keys
     - Google: https://makersuite.google.com/app/apikey

3. **ROSA Virtual Environment**
   - ROSA is installed in `/opt/rosa-venv`
   - Must activate before use

## Windows / WSL2 notes (for future teammates)

This project runs ROS1 + Gazebo/RViz inside Linux Docker containers. On Windows, this usually works best via **WSL2**.

### Recommended (Windows 11)
- Install **Docker Desktop** and enable **WSL2 integration**
- Install **Ubuntu** via WSL2
- Use **WSLg** (built into Windows 11) so Gazebo/RViz can display from WSL
- Clone this repo inside WSL and run the normal steps:
  ```bash
  ./setup.sh
  ./run_simulation.sh
  ```

### Windows 10
- Use WSL2 + Docker Desktop, but you’ll likely need an X server (e.g., VcXsrv/Xming) and correct `DISPLAY` forwarding.
- If Gazebo/RViz do not appear, it’s almost always an X11/WSL display issue, not ROSA.

### Quick verification checklist
1. When running `./run_simulation.sh`, you should see:
   - `Mounting project into container at /workspace/llm-pick-me-bots`
2. Inside the container:
   ```bash
   ls /workspace/llm-pick-me-bots/start_rosa.sh
   ```
3. If that path exists, you can run:
   ```bash
   bash /workspace/llm-pick-me-bots/start_rosa.sh
   ```

## Quick Start

### Step 1: Start the Simulation

In one terminal:
```bash
./run_simulation.sh
```

Wait for Gazebo and RViz to open.

### Step 2: Access the Container

In a **new terminal**:
```bash
# Get container ID
sudo docker ps

# Access container
sudo docker exec -it <CONTAINER_ID> /bin/bash
```

**Note:** `run_simulation.sh` mounts this repo into the container at:

```bash
/workspace/llm-pick-me-bots
```

### Step 3: Activate ROSA Environment

**Inside the container:**
```bash
# ROSA venv is already activated in .bashrc, but verify:
source /opt/rosa-venv/bin/activate

# Verify ROSA is installed
python3 -c "from rosa import ROSA; print('ROSA OK')"
```

### Step 4: Set API Key

**Option A: Use setup script (Recommended)**
```bash
# Run the setup script (works in container or host)
./setup_env.sh
```

This will:
- Prompt you to select a provider (OpenAI or Gemini)
- Securely save your API key to `~/.rosa_env`
- Load the environment variables

**Option B: Set manually**

**For OpenAI:**
```bash
export OPENAI_API_KEY='your-api-key-here'
```

**For Google Gemini:**
```bash
export GOOGLE_API_KEY='your-api-key-here'
```

**Note:** The `start_rosa.sh` script automatically loads `~/.rosa_env` if it exists.

### Step 5: Run ROSA Controller

**Use the quick start script (recommended):**
```bash
bash /workspace/llm-pick-me-bots/start_rosa.sh
```

This script automatically:
- Activates ROSA venv
- Sources ROS environment
- Loads API keys from `~/.rosa_env`
- Installs missing Python deps into the ROSA venv if needed (rospkg/catkin_pkg, provider SDKs)
- Supports both OpenAI and Google Gemini
- Auto-detects provider from available API keys

### Provider configuration

**OpenAI (standard key)**

```bash
export ROSA_LLM_PROVIDER="openai"
export OPENAI_API_KEY="..."
# optional:
export OPENAI_MODEL="gpt-4o-mini"
bash /workspace/llm-pick-me-bots/start_rosa.sh
```

**Gemini**

```bash
export ROSA_LLM_PROVIDER="gemini"
export GOOGLE_API_KEY="..."
```

Gemini has strict tool schema validation. Choose a tool mode:

- `safe` (default): binds only the robot-control tools (most reliable)
- `all`: tries the full ROSA toolset (may error with Gemini tool schema 400s)

```bash
export ROSA_GEMINI_TOOL_MODE="safe"   # or "all"
bash /workspace/llm-pick-me-bots/start_rosa.sh
```

## Usage Examples

Once the ROSA controller is running, you can use natural language commands:

### Mobile Base Control

```
You: Move the robot forward
ROSA: Moving forward at 0.3 m/s for 1.0s

You: Turn left
ROSA: Turning left at 0.5 rad/s for 1.0s

You: Stop the robot
ROSA: Stopped the mobile base

You: Move backward slowly
ROSA: Moving backward at 0.2 m/s
```

### Robot State Queries

```
You: What is the current state of the robot?
ROSA: Robot State:
  joint2_to_joint1: 0.123 rad
  joint3_to_joint2: -0.456 rad
  ...
```

### Arm Control

```
You: Move the arm to a neutral position
ROSA: Moving arm to positions: {'joint2_to_joint1': 0.0, ...}
```

## Available Tools

ROSA has access to these custom tools:

1. **move_base_forward(speed, duration_s)** - Move forward for a duration then stop
2. **move_base_backward(speed, duration_s)** - Move backward for a duration then stop
3. **turn_base_left(angular_speed, duration_s)** - Turn left for a duration then stop
4. **turn_base_right(angular_speed, duration_s)** - Turn right for a duration then stop
5. **stop_base()** - Stop immediately
6. **get_robot_state()** - Get current joint positions
7. **move_arm_to_position(joint_name, position)** - Move a single arm joint

Plus all standard ROSA tools:
- `rostopic_list()` - List topics
- `rostopic_pub()` - Publish to topics
- `rostopic_echo()` - Subscribe to topics
- `rosservice_list()` - List services
- `rosservice_call()` - Call services
- `rosnode_list()` - List nodes
- And more...

## Configuration

### LLM Providers

**OpenAI (Default if OPENAI_API_KEY is set and ROSA_LLM_PROVIDER=openai)**
- Model: set via `OPENAI_MODEL` (default: `gpt-4o-mini`)
- Set: `export OPENAI_API_KEY='your-key'` or use `./setup_env.sh`
- Usage: `python3 rosa_robot_controller.py --provider openai`

**Google Gemini**
- Model: gemini-2.5-flash
- Set: `export GOOGLE_API_KEY='your-key'` or use `./setup_env.sh`
- Usage: `python3 rosa_robot_controller.py --provider gemini`

### Customization

You can modify `rosa_robot_controller.py` to:
- Add custom tools
- Change default speeds
- Add new robot capabilities
- Integrate with MoveIt services

## Troubleshooting

### "ROSA or LangChain not found"
- **Solution**: Activate ROSA venv
  ```bash
  source /opt/rosa-venv/bin/activate
  ```

### "API key required"
- **Solution**: Set environment variable or use setup script
  ```bash
  # Option 1: Use setup script (recommended)
  ./setup_env.sh
  
  # Option 2: Set manually
  export OPENAI_API_KEY='your-key'      # For OpenAI
  # or
  export GOOGLE_API_KEY='your-key'      # For Gemini
  ```

### "ROS node not initialized"
- **Solution**: Make sure ROS is sourced
  ```bash
  source /opt/ros/noetic/setup.bash
  source /root/catkin_ws/devel/setup.bash
  ```

### "Action server not available"
- **Solution**: Wait for simulation to fully start
- Check: `rostopic list | grep arm_controller`

### Robot doesn't move
- **Solution**: 
  1. Check if simulation is running
  2. Verify topics exist: `rostopic list`
  3. Check for errors in ROSA output

### Gemini tool schema 400 errors
If you see errors like `GenerateContentRequest.tools... missing field`, switch Gemini tool mode to safe:

```bash
export ROSA_GEMINI_TOOL_MODE="safe"
bash /workspace/llm-pick-me-bots/start_rosa.sh
```

## Advanced Usage

### Using ROSA for Diagnostics

ROSA can also be used for system inspection:

```python
from rosa import ROSA
from langchain_openai import ChatOpenAI
from langchain_google_genai import ChatGoogleGenerativeAI

# For OpenAI
llm = ChatOpenAI(model="gpt-4", api_key=os.getenv("OPENAI_API_KEY"))

# Or for Gemini
# llm = ChatGoogleGenerativeAI(model="gemini-2.5-flash", google_api_key=os.getenv("GOOGLE_API_KEY"))

agent = ROSA(ros_version=1, llm=llm)

# Ask ROSA questions
response = agent.invoke("Show me all topics that have publishers but no subscribers")
print(response)

response = agent.invoke("What services are available for the arm controller?")
print(response)
```

### Programmatic Control

You can also use ROSA programmatically:

```python
from rosa_robot_controller import ROSARobotController

# With OpenAI
controller = ROSARobotController(llm_provider="openai")

# Or with Gemini
# controller = ROSARobotController(llm_provider="gemini")

response = controller.agent.invoke("Move forward for 2 seconds then stop")
```

## Next Steps

1. **Experiment with commands** - Try different natural language phrases
2. **Add custom tools** - Extend functionality for your use case
3. **Integrate MoveIt** - Use MoveIt services for complex arm motions
4. **Add safety checks** - Implement collision avoidance and limits

## Resources

- **ROSA Documentation**: https://github.com/nasa-jpl/rosa
- **LangChain Tools**: https://python.langchain.com/docs/modules/tools/
- **ROS 1 Noetic**: http://wiki.ros.org/noetic

---

**Status**: ✅ ROSA Integration Complete  
**Last Updated**: December 2025

