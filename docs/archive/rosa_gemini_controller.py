#!/usr/bin/env python3
"""
ROSA Gemini Controller - Natural Language Control for LIMO Cobot
This version works around Gemini's tool schema limitations by using direct command parsing.
"""

import os
import sys
import json
import re

# Debug logging setup (Debug Mode expects logs INSIDE the repo at `.cursor/debug.log`)
DEBUG_LOG_PATH = os.path.abspath(
    os.getenv(
        "ROSA_DEBUG_LOG_PATH",
        os.path.join(os.path.dirname(__file__), "..", ".cursor", "debug.log"),
    )
)

def debug_log(location, message, data=None, hypothesis_id=None, run_id="run1"):
    """Write NDJSON debug log entry (never log secrets)."""
    try:
        import time
        run_id = os.getenv("ROSA_DEBUG_RUN_ID", run_id)
        log_entry = {
            "timestamp": int(time.time() * 1000),
            "location": location,
            "message": message,
            "data": data or {},
            "sessionId": "debug-session",
            "runId": run_id,
            "hypothesisId": hypothesis_id,
        }
        with open(DEBUG_LOG_PATH, "a") as f:
            f.write(json.dumps(log_entry) + "\n")
    except Exception:
        pass

# region agent log
# Patch importlib.metadata.packages_distributions on Python 3.9 from backport if needed.
try:
    import importlib.metadata as _ilm
    if not hasattr(_ilm, "packages_distributions"):
        try:
            import importlib_metadata as _ilm_backport  # pip package: importlib-metadata
            _ilm.packages_distributions = _ilm_backport.packages_distributions  # type: ignore[attr-defined]
            debug_log("rosa_gemini_controller.py:preflight", "Patched importlib.metadata.packages_distributions from backport", {
                "python_version": sys.version.split()[0],
            }, "H9")
        except Exception as e:
            debug_log("rosa_gemini_controller.py:preflight", "Could not patch importlib.metadata.packages_distributions", {
                "error_type": type(e).__name__,
                "error_message": str(e),
            }, "H9")
except Exception as e:
    debug_log("rosa_gemini_controller.py:preflight", "importlib.metadata preflight failed", {
        "error_type": type(e).__name__,
        "error_message": str(e),
    }, "H9")
# endregion

# region agent log
debug_log("rosa_gemini_controller.py:preflight", "Python preflight before rospy import", {
    "executable": sys.executable,
    "python_version": sys.version.split()[0],
    "virtual_env": os.getenv("VIRTUAL_ENV", ""),
    "cwd": os.getcwd(),
    "sys_path_head": sys.path[:5],
}, "H7")
# endregion

# ROS imports (can fail if ROS python deps aren't installed in current venv)
try:
    import rospy
    from geometry_msgs.msg import Twist
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from sensor_msgs.msg import JointState
except Exception as e:
    # region agent log
    debug_log("rosa_gemini_controller.py:preflight", "Failed importing rospy / ROS deps", {
        "error_type": type(e).__name__,
        "error_message": str(e),
        "hint": "If ModuleNotFoundError: rospkg, install into venv: python3 -m pip install rospkg catkin_pkg",
    }, "H7")
    # endregion
    raise

# Gemini imports
try:
    from langchain_google_genai import ChatGoogleGenerativeAI
except ImportError as e:
    print("Error: langchain-google-genai not found!")
    print("Install with: pip install langchain-google-genai")
    sys.exit(1)


class GeminiRobotController:
    """Gemini-based natural language robot controller (workaround for ROSA tool limitations)"""
    
    def __init__(self, api_key=None):
        """Initialize Gemini robot controller"""
        # Initialize ROS node
        rospy.init_node('gemini_robot_controller', anonymous=True)
        
        # Robot control publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.arm_trajectory_pub = rospy.Publisher('/arm_controller/command', 
                                                   JointTrajectory, queue_size=10)
        
        # Joint state subscriber
        self.joint_states = {}
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, 
                                                 self.joint_state_callback)
        
        # Wait for joint states
        rospy.sleep(1.0)
        
        # Initialize Gemini LLM
        api_key = api_key or os.getenv("GOOGLE_API_KEY")
        if not api_key:
            raise ValueError("Google API key required. Set GOOGLE_API_KEY environment variable.")

        # #region agent log
        debug_log("rosa_gemini_controller.py:__init__", "Gemini controller init", {
            "env_has_google_api_key": bool(os.getenv("GOOGLE_API_KEY")),
            "api_key_provided_arg": bool(api_key),
            "model": "gemini-2.5-flash",
        }, "H2")
        # #endregion
        
        self.llm = ChatGoogleGenerativeAI(
            model="gemini-2.5-flash", 
            temperature=0, 
            google_api_key=api_key
        )
        
        rospy.loginfo("Gemini Robot Controller initialized!")
    
    def joint_state_callback(self, msg):
        """Update joint states"""
        for i, name in enumerate(msg.name):
            self.joint_states[name] = msg.position[i] if i < len(msg.position) else 0.0
    
    def move_base_forward(self, speed=0.3):
        """Move the mobile base forward"""
        cmd = Twist()
        cmd.linear.x = max(0.0, min(1.0, float(speed)))
        self.cmd_vel_pub.publish(cmd)
        return f"Moving forward at {speed} m/s"
    
    def move_base_backward(self, speed=0.3):
        """Move the mobile base backward"""
        cmd = Twist()
        cmd.linear.x = -max(0.0, min(1.0, float(speed)))
        self.cmd_vel_pub.publish(cmd)
        return f"Moving backward at {speed} m/s"
    
    def turn_base_left(self, angular_speed=0.5):
        """Turn the mobile base left"""
        cmd = Twist()
        cmd.angular.z = max(0.0, min(1.0, float(angular_speed)))
        self.cmd_vel_pub.publish(cmd)
        return f"Turning left at {angular_speed} rad/s"
    
    def turn_base_right(self, angular_speed=0.5):
        """Turn the mobile base right"""
        cmd = Twist()
        cmd.angular.z = -max(0.0, min(1.0, float(angular_speed)))
        self.cmd_vel_pub.publish(cmd)
        return f"Turning right at {angular_speed} rad/s"
    
    def stop_base(self):
        """Stop the mobile base"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        return "Stopped the mobile base"
    
    def get_robot_state(self):
        """Get current robot state"""
        if not self.joint_states:
            return "No joint state information available yet"
        
        state_info = "Robot State:\n"
        for joint_name, position in self.joint_states.items():
            state_info += f"  {joint_name}: {position:.3f} rad\n"
        
        return state_info
    
    def parse_command(self, user_input):
        """Use Gemini to parse natural language command and return action"""
        prompt = f"""You are a robot controller. Parse this command and respond with ONLY a JSON object:
{{
  "action": "move_forward|move_backward|turn_left|turn_right|stop|get_state",
  "speed": 0.3,
  "angular_speed": 0.5
}}

User command: "{user_input}"

Rules:
- "move forward", "go forward", "drive forward" -> {{"action": "move_forward", "speed": 0.3}}
- "move backward", "go back", "reverse" -> {{"action": "move_backward", "speed": 0.3}}
- "turn left", "rotate left" -> {{"action": "turn_left", "angular_speed": 0.5}}
- "turn right", "rotate right" -> {{"action": "turn_right", "angular_speed": 0.5}}
- "stop", "halt" -> {{"action": "stop"}}
- "state", "status", "what is" -> {{"action": "get_state"}}
- Extract speed/angular_speed numbers if mentioned (default: 0.3 for linear, 0.5 for angular)

Respond with ONLY the JSON, no other text."""

        try:
            # #region agent log
            debug_log("rosa_gemini_controller.py:parse_command", "Before llm.invoke", {
                "user_input_len": len(user_input),
            }, "H5")
            # #endregion
            response = self.llm.invoke(prompt)
            response_text = response.content.strip()
            # #region agent log
            debug_log("rosa_gemini_controller.py:parse_command", "After llm.invoke", {
                "response_len": len(response_text),
                "has_brace": ("{" in response_text and "}" in response_text),
            }, "H5")
            # #endregion
            
            # Extract JSON from response
            json_match = re.search(r'\{[^}]+\}', response_text)
            if json_match:
                command = json.loads(json_match.group())
                return command
            else:
                # Try parsing entire response as JSON
                return json.loads(response_text)
        except Exception as e:
            rospy.logerr(f"Error parsing command: {e}")
            # #region agent log
            debug_log("rosa_gemini_controller.py:parse_command", "parse_command exception", {
                "error_type": type(e).__name__,
                "error_message": str(e),
            }, "H6")
            # #endregion
            return None
    
    def execute_command(self, command):
        """Execute parsed command"""
        if not command:
            return "Could not parse command"
        
        action = command.get("action", "").lower()
        
        if action == "move_forward":
            speed = command.get("speed", 0.3)
            return self.move_base_forward(speed)
        elif action == "move_backward":
            speed = command.get("speed", 0.3)
            return self.move_base_backward(speed)
        elif action == "turn_left":
            angular_speed = command.get("angular_speed", 0.5)
            return self.turn_base_left(angular_speed)
        elif action == "turn_right":
            angular_speed = command.get("angular_speed", 0.5)
            return self.turn_base_right(angular_speed)
        elif action == "stop":
            return self.stop_base()
        elif action == "get_state":
            return self.get_robot_state()
        else:
            return f"Unknown action: {action}"
    
    def run_interactive(self):
        """Run interactive command loop"""
        print("\n" + "="*70)
        print("Gemini Robot Controller - Natural Language Control")
        print("="*70)
        print("\nThe robot is ready! You can control it using natural language.")
        print("\nExample commands:")
        print("  - 'Move the robot forward'")
        print("  - 'Turn left'")
        print("  - 'Stop the robot'")
        print("  - 'What is the current state of the robot?'")
        print("\nType 'quit' or 'exit' to stop.")
        print("="*70 + "\n")
        
        while not rospy.is_shutdown():
            try:
                user_input = input("\nYou: ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("\nStopping controller...")
                    break
                
                if not user_input:
                    continue
                
                # Parse and execute command
                print("\nProcessing command...")
                command = self.parse_command(user_input)
                result = self.execute_command(command)
                print(f"\nRobot: {result}")
                
            except KeyboardInterrupt:
                print("\n\nStopping controller...")
                break
            except Exception as e:
                print(f"\nError: {e}")
                rospy.logerr(f"Error processing command: {e}")
        
        # Stop robot before exiting
        self.stop_base()
        print("\nRobot controller stopped.")


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Gemini Robot Controller')
    parser.add_argument('--api-key', type=str, help='Google API key (or set GOOGLE_API_KEY env var)')
    
    args = parser.parse_args()
    
    try:
        controller = GeminiRobotController(api_key=args.api_key)
        controller.run_interactive()
        
    except ValueError as e:
        print(f"\nConfiguration Error: {e}")
        print("\nSet your Google API key:")
        print("  export GOOGLE_API_KEY='your-api-key'")
        sys.exit(1)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

