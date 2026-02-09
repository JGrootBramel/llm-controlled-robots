#!/usr/bin/env python3
"""
LIMO Cobot Controller - Version 3
- Uses trajectory controller for arm (correct method)
- Smooth continuous movement for base
"""

import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RobotController:
    def __init__(self):
        rospy.init_node('limo_cobot_controller', anonymous=True)
        
        # Mobile base publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Arm trajectory publisher (correct method for this robot)
        self.arm_trajectory_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        
        # Discover arm joints
        print("Discovering robot joints...")
        self.discover_joints()
        
        # Control parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.arm_speed = 0.1
        
        # Current state
        self.base_cmd = Twist()
        self.arm_joints = {name: 0.0 for name in self.arm_joint_names}
        
        # Setup terminal
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        # Wait for publishers to connect
        rospy.sleep(0.5)
        
        self.print_controls()
    
    def discover_joints(self):
        """Discover actual joint names from the robot"""
        try:
            print("  Waiting for /joint_states topic...")
            msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
            
            # Filter out wheel joints and other non-arm joints
            arm_joint_keywords = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            self.arm_joint_names = []
            
            for name in msg.name:
                # Skip wheel joints
                if 'wheel' in name.lower():
                    continue
                # Skip gripper joint
                if 'grasping' in name.lower():
                    continue
                # Include arm joints
                if any(kw in name.lower() for kw in arm_joint_keywords):
                    self.arm_joint_names.append(name)
            
            # Limit to first 6 joints (typical for 6-DOF arm)
            self.arm_joint_names = self.arm_joint_names[:6]
            
            print(f"  Found {len(self.arm_joint_names)} arm joints:")
            for i, name in enumerate(self.arm_joint_names):
                print(f"    {i+1}. {name}")
            print("  Using /arm_controller/command (trajectory controller)")
            
        except rospy.ROSException:
            print("  ERROR: Could not get joint states. Using default joint names.")
            self.arm_joint_names = [
                'joint2_to_joint1',
                'joint3_to_joint2', 
                'joint4_to_joint3',
                'joint5_to_joint4',
                'joint6_to_joint5',
                'joint6output_to_joint6'
            ]
    
    def print_controls(self):
        num_joints = len(self.arm_joint_names)
        print("\n" + "="*60)
        print("LIMO Cobot Controller - Version 3 (Trajectory Control)")
        print("="*60)
        print("\nMOBILE BASE:")
        print("  W    - Forward    S    - Backward")
        print("  A    - Left       D    - Right")
        print("  X    - Stop")
        print(f"\nARM ({num_joints} joints):")
        print("  1-6  - Increase joint 1-6")
        print("  Q,E  - Decrease joint 1,2")
        print("  R,F  - Decrease joint 3,4")
        if num_joints > 4:
            print("  T,G  - Decrease joint 5,6")
        print("  Z    - Reset all joints")
        print("\nOTHER:")
        print("  +    - Increase speed")
        print("  -    - Decrease speed")
        print("  H    - Help")
        print("  ESC  - Quit")
        print("="*60 + "\n")
    
    def move_base(self, linear_x, angular_z):
        """Control mobile base"""
        self.base_cmd.linear.x = linear_x
        self.base_cmd.angular.z = angular_z
    
    def publish_arm_trajectory(self):
        """Publish arm trajectory command"""
        traj = JointTrajectory()
        traj.joint_names = self.arm_joint_names
        traj.header.stamp = rospy.Time.now()
        
        point = JointTrajectoryPoint()
        point.positions = [self.arm_joints[name] for name in self.arm_joint_names]
        point.velocities = [0.0] * len(self.arm_joint_names)
        point.time_from_start = rospy.Duration(0.1)  # Short duration for immediate movement
        
        traj.points = [point]
        self.arm_trajectory_pub.publish(traj)
    
    def move_arm_joint(self, joint_idx, delta):
        """Move a specific arm joint"""
        if 0 <= joint_idx < len(self.arm_joint_names):
            joint_name = self.arm_joint_names[joint_idx]
            self.arm_joints[joint_name] += delta
            # Limit to reasonable range
            self.arm_joints[joint_name] = max(-3.14, min(3.14, self.arm_joints[joint_name]))
            
            # Publish trajectory command
            self.publish_arm_trajectory()
            
            print(f"Joint {joint_idx+1} ({joint_name}): {self.arm_joints[joint_name]:.2f} rad")
    
    def reset_arm(self):
        """Reset all arm joints to zero"""
        for joint_name in self.arm_joint_names:
            self.arm_joints[joint_name] = 0.0
        self.publish_arm_trajectory()
        print("Arm reset to home position")
    
    def get_key(self):
        """Get a single keypress (non-blocking)"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None
    
    def run(self):
        """Main control loop with smooth continuous movement"""
        rate = rospy.Rate(20)  # 20 Hz for smooth movement
        
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key:
                    key = key.lower()
                    
                    # Mobile base controls
                    if key == 'w':
                        self.move_base(self.linear_speed, 0.0)
                        print("Moving forward")
                    elif key == 's':
                        self.move_base(-self.linear_speed, 0.0)
                        print("Moving backward")
                    elif key == 'a':
                        self.move_base(0.0, self.angular_speed)
                        print("Turning left")
                    elif key == 'd':
                        self.move_base(0.0, -self.angular_speed)
                        print("Turning right")
                    elif key == 'x':
                        self.move_base(0.0, 0.0)
                        print("Stopped")
                    
                    # Arm controls
                    elif key in '123456':
                        joint_idx = int(key) - 1
                        if joint_idx < len(self.arm_joint_names):
                            self.move_arm_joint(joint_idx, self.arm_speed)
                    
                    # Decrease joint angles
                    elif key == 'q':
                        if len(self.arm_joint_names) > 0:
                            self.move_arm_joint(0, -self.arm_speed)
                    elif key == 'e':
                        if len(self.arm_joint_names) > 1:
                            self.move_arm_joint(1, -self.arm_speed)
                    elif key == 'r':
                        if len(self.arm_joint_names) > 2:
                            self.move_arm_joint(2, -self.arm_speed)
                    elif key == 'f':
                        if len(self.arm_joint_names) > 3:
                            self.move_arm_joint(3, -self.arm_speed)
                    elif key == 't':
                        if len(self.arm_joint_names) > 4:
                            self.move_arm_joint(4, -self.arm_speed)
                    elif key == 'g':
                        if len(self.arm_joint_names) > 5:
                            self.move_arm_joint(5, -self.arm_speed)
                    elif key == 'z':
                        self.reset_arm()
                    elif key == 'h':
                        self.print_controls()
                    elif key == '+' or key == '=':
                        self.linear_speed = min(1.0, self.linear_speed + 0.1)
                        print(f"Speed: {self.linear_speed:.2f}")
                    elif key == '-':
                        self.linear_speed = max(0.1, self.linear_speed - 0.1)
                        print(f"Speed: {self.linear_speed:.2f}")
                    elif key == '\x1b':  # ESC to quit
                        break
                
                # Continuously publish base command for smooth movement
                self.cmd_vel_pub.publish(self.base_cmd)
                
                rate.sleep()
        
        except KeyboardInterrupt:
            pass
        finally:
            # Cleanup - stop robot
            self.move_base(0.0, 0.0)
            self.cmd_vel_pub.publish(self.base_cmd)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print("\nController stopped. Robot stopped.")

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

