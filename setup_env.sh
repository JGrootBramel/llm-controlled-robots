#!/bin/bash
# Environment setup for ROSA API keys
# This script helps you set API keys without hardcoding them

echo "============================================================================"
echo "ROSA API Key Setup"
echo "============================================================================"
echo ""

# Check if .env file exists
ENV_FILE="$HOME/.rosa_env"

if [ -f "$ENV_FILE" ]; then
    echo "Found existing .rosa_env file"
    read -p "Do you want to update it? (y/N): " update
    if [[ ! "$update" =~ ^[Yy]$ ]]; then
        echo "Using existing configuration."
        source "$ENV_FILE"
        echo "✓ Environment variables loaded from $ENV_FILE"
        exit 0
    fi
fi

echo "Select your LLM provider:"
echo "1. OpenAI (GPT-4)"
echo "2. Google (Gemini 2.5 Flash)"
echo "3. Skip (set manually)"
read -p "Enter choice [1-3]: " choice

case $choice in
    1)
        read -p "Enter your OpenAI API key: " api_key
        if [ -z "$api_key" ]; then
            echo "No API key provided. Exiting."
            exit 1
        fi
        echo "export OPENAI_API_KEY='$api_key'" > "$ENV_FILE"
        echo "export ROSA_LLM_PROVIDER='openai'" >> "$ENV_FILE"
        ;;
    2)
        read -p "Enter your Google Gemini API key: " api_key
        if [ -z "$api_key" ]; then
            echo "No API key provided. Exiting."
            exit 1
        fi
        echo "export GOOGLE_API_KEY='$api_key'" > "$ENV_FILE"
        echo "export ROSA_LLM_PROVIDER='gemini'" >> "$ENV_FILE"
        ;;
    3)
        echo "Skipping. You can set environment variables manually:"
        echo "  export OPENAI_API_KEY='your-key'"
        echo "  export GOOGLE_API_KEY='your-key'"
        exit 0
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

# Make file readable only by user
chmod 600 "$ENV_FILE"

echo ""
echo "✓ API key saved to $ENV_FILE (readable only by you)"
echo ""
echo "To use these keys, run:"
echo "  source $ENV_FILE"
echo ""
echo "Or add to your ~/.bashrc:"
echo "  source $ENV_FILE"
echo ""

# Load the environment
source "$ENV_FILE"

echo "✓ Environment variables are now set for this session"

