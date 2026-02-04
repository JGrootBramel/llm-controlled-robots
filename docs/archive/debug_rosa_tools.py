#!/usr/bin/env python3
"""
Debug script to inspect ROSA tools and their schemas for Gemini compatibility
"""

import os
import sys
import json
import rospy
from langchain_google_genai import ChatGoogleGenerativeAI

# Initialize ROS (minimal)
rospy.init_node('debug_rosa', anonymous=True)

# Setup Gemini
api_key = os.getenv("GOOGLE_API_KEY")
if not api_key:
    print("Error: GOOGLE_API_KEY not set")
    sys.exit(1)

llm = ChatGoogleGenerativeAI(
    model="gemini-2.5-flash", 
    temperature=0, 
    google_api_key=api_key
)

# Import ROSA
from rosa import ROSA

print("="*70)
print("Debugging ROSA Tools for Gemini Compatibility")
print("="*70)

# Try to get ROSA's tools
try:
    # Initialize ROSA without custom tools to see what it provides
    agent = ROSA(ros_version=1, llm=llm, tools=[])
    
    # Access the agent's tools
    if hasattr(agent, 'tools'):
        tools = agent.tools
        print(f"\nFound {len(tools)} ROSA tools")
        
        # Inspect each tool
        for i, tool in enumerate(tools):
            print(f"\n--- Tool {i}: {tool.name if hasattr(tool, 'name') else 'Unknown'} ---")
            
            # Get tool schema
            if hasattr(tool, 'args_schema'):
                print(f"Args schema: {tool.args_schema}")
            if hasattr(tool, 'description'):
                print(f"Description: {tool.description}")
            
            # Try to get JSON schema
            try:
                if hasattr(tool, 'args_schema'):
                    schema = tool.args_schema.schema() if hasattr(tool.args_schema, 'schema') else str(tool.args_schema)
                    print(f"Schema: {json.dumps(schema, indent=2) if isinstance(schema, dict) else schema}")
            except Exception as e:
                print(f"Error getting schema: {e}")
    
    # Try to see what happens when we initialize with tools
    print("\n" + "="*70)
    print("Testing tool conversion to Gemini format...")
    print("="*70)
    
    # Check if we can access the agent's internal tool conversion
    if hasattr(agent, 'agent_executor'):
        executor = agent.agent_executor
        if hasattr(executor, 'tools'):
            print(f"Agent executor has {len(executor.tools)} tools")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "="*70)
print("Debug complete")
print("="*70)

