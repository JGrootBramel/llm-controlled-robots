#!/bin/bash
# Quick start script for ROSA integration

echo "============================================================================"
echo "ROSA Robot Controller - Quick Start"
echo "============================================================================"
echo ""

# Check if we're in a Docker container
if [ -f /.dockerenv ]; then
    echo "✓ Running inside Docker container"
    CONTAINER_MODE=true
else
    echo "⚠️  Not in Docker container. This script should be run inside the container."
    echo "   Access container first: docker exec -it <CONTAINER_ID> /bin/bash"
    exit 1
fi

# Check ROSA venv
if [ -z "$VIRTUAL_ENV" ] || [[ "$VIRTUAL_ENV" != *"rosa-venv"* ]]; then
    echo "Activating ROSA virtual environment..."
    source /opt/rosa-venv/bin/activate
fi

# Ensure ROS python deps exist inside the venv (rospy -> roslib -> rospkg)
python3 -c "import rospkg" >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "Installing missing ROS python deps into ROSA venv (rospkg, catkin_pkg)..."
    python3 -m pip install --quiet --disable-pip-version-check rospkg catkin_pkg || {
        echo "❌ Failed to install rospkg/catkin_pkg. Check network access inside container."
        echo "   You can try manually: python3 -m pip install rospkg catkin_pkg"
        exit 1
    }
    echo "✓ Installed rospkg/catkin_pkg"
fi

# Python 3.9 compatibility: some libs expect importlib.metadata.packages_distributions (Py3.10+)
python3 - <<'PY' >/dev/null 2>&1
import importlib.metadata as m
raise SystemExit(0 if hasattr(m, "packages_distributions") else 1)
PY
if [ $? -ne 0 ]; then
    echo "Installing Python compatibility backport (importlib-metadata)..."
    python3 -m pip install --quiet --disable-pip-version-check 'importlib-metadata>=6' || {
        echo "❌ Failed to install importlib-metadata backport."
        exit 1
    }
    echo "✓ Installed importlib-metadata"
fi

# Load API keys from .rosa_env if it exists
if [ -f "$HOME/.rosa_env" ]; then
    echo "Loading API keys from $HOME/.rosa_env..."
    source "$HOME/.rosa_env"
    echo "✓ Environment variables loaded"
fi

# Check ROS setup
if [ -z "$ROS_DISTRIBUTION" ]; then
    echo "Sourcing ROS environment..."
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash
fi

# Check API key
if [ -z "$OPENAI_API_KEY" ] && [ -z "$GOOGLE_API_KEY" ]; then
    echo ""
    echo "⚠️  WARNING: No LLM API key found!"
    echo ""
    echo "Set one of the following:"
    echo "  export OPENAI_API_KEY='your-key'      # For OpenAI"
    echo "  export GOOGLE_API_KEY='your-key'       # For Google Gemini"
    echo ""
    echo "Or run: ./setup_env.sh"
    echo ""
    read -p "Do you want to continue anyway? (y/N): " continue
    if [[ ! "$continue" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Find ROSA controller script
SCRIPT_PATH=""
# Prefer the mounted workspace copy so we always run the latest code.
if [ -f "/workspace/llm-controlled-robots/docs/rosa_robot_controller.py" ]; then
    SCRIPT_PATH="/workspace/llm-controlled-robots/docs/rosa_robot_controller.py"
elif [ -f "/root/catkin_ws/src/limo_cobot_sim/docs/rosa_robot_controller.py" ]; then
    SCRIPT_PATH="/root/catkin_ws/src/limo_cobot_sim/docs/rosa_robot_controller.py"
elif [ -f "./docs/rosa_robot_controller.py" ]; then
    SCRIPT_PATH="./docs/rosa_robot_controller.py"
elif [ -f "rosa_robot_controller.py" ]; then
    SCRIPT_PATH="./rosa_robot_controller.py"
else
    echo "❌ ROSA controller script not found!"
    echo "   Please ensure rosa_robot_controller.py is available"
    exit 1
fi

echo "✓ Found ROSA controller script: $SCRIPT_PATH"
echo ""

# Determine provider (use env var if set, otherwise auto-detect)
if [ -n "$ROSA_LLM_PROVIDER" ]; then
    PROVIDER="$ROSA_LLM_PROVIDER"
elif [ -n "$OPENAI_API_KEY" ] && [ -z "$GOOGLE_API_KEY" ]; then
    PROVIDER="openai"
elif [ -n "$GOOGLE_API_KEY" ] && [ -z "$OPENAI_API_KEY" ]; then
    PROVIDER="gemini"
else
    # Both keys available, default to gemini
    PROVIDER="gemini"
fi

echo "Starting ROSA controller with provider: $PROVIDER"
echo ""

# Tag this run in the debug log (log file can't be cleared in this environment)
export ROSA_DEBUG_RUN_ID="run-$(date +%s)"

# Gemini schema workaround: optionally force tool list to our known-good robot tools only.
# - Set ROSA_GEMINI_TOOL_MODE=safe to force custom-only tools (avoids Gemini schema 400s).
# - Set ROSA_GEMINI_TOOL_MODE=all to attempt full ROSA toolset (may fail on Gemini strict schemas).
if [ "$PROVIDER" = "gemini" ]; then
    if [ -z "$ROSA_GEMINI_TOOL_MODE" ]; then
        # Default: keep Gemini reliable unless the user explicitly opts into "all".
        ROSA_GEMINI_TOOL_MODE="safe"
    fi
    export ROSA_GEMINI_TOOL_MODE
    if [ "$ROSA_GEMINI_TOOL_MODE" = "safe" ]; then
        export ROSA_GEMINI_FORCE_CUSTOM_TOOLS_ONLY=1
        echo "Gemini tool mode: safe (custom robot tools only)."
        echo "  To try all ROSA tools on Gemini: export ROSA_GEMINI_TOOL_MODE=all"
    else
        unset ROSA_GEMINI_FORCE_CUSTOM_TOOLS_ONLY
        echo "Gemini tool mode: all (full ROSA toolset)."
        echo "  If Gemini errors with tool schema 400s: export ROSA_GEMINI_TOOL_MODE=safe"
    fi
fi

# If key is missing, prompt once (avoids accidentally exporting an empty var)
if [ "$PROVIDER" = "gemini" ] && [ -z "$GOOGLE_API_KEY" ]; then
    read -s -p "GOOGLE_API_KEY: " GOOGLE_API_KEY
    echo
    export GOOGLE_API_KEY
fi
if [ "$PROVIDER" = "openai" ] && [ -z "$OPENAI_API_KEY" ]; then
    read -s -p "OPENAI_API_KEY: " OPENAI_API_KEY
    echo
    export OPENAI_API_KEY
fi

# Ensure provider-specific LangChain integration is installed in the venv
if [ "$PROVIDER" = "gemini" ]; then
    python3 -c "import langchain_google_genai" >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "Installing missing Gemini integration (langchain-google-genai)..."
        python3 -m pip install --quiet --disable-pip-version-check langchain-google-genai || {
            echo "❌ Failed to install langchain-google-genai. Check network access inside container."
            echo "   You can try manually: python3 -m pip install langchain-google-genai"
            exit 1
        }
        echo "✓ Installed langchain-google-genai"
    fi
elif [ "$PROVIDER" = "openai" ]; then
    python3 -c "import langchain_openai" >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "Installing missing OpenAI integration (langchain-openai)..."
        python3 -m pip install --quiet --disable-pip-version-check langchain-openai || {
            echo "❌ Failed to install langchain-openai. Check network access inside container."
            echo "   You can try manually: python3 -m pip install langchain-openai"
            exit 1
        }
        echo "✓ Installed langchain-openai"
    fi
fi

# Run the controller
python3 "$SCRIPT_PATH" --provider "$PROVIDER"

