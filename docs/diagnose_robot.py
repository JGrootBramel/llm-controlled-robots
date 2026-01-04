#!/usr/bin/env python3
"""
Diagnostic script to find joint names and control topics
"""
import rospy
from sensor_msgs.msg import JointState

def diagnose_robot():
    rospy.init_node('diagnose_robot', anonymous=True)
    
    print("\n" + "="*70)
    print("ROBOT DIAGNOSTICS - Finding Joint Names and Control Topics")
    print("="*70)
    
    # 1. Get joint names from /joint_states
    print("\n1. JOINT NAMES FROM /joint_states:")
    print("-" * 70)
    try:
        msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
        print(f"   Total joints found: {len(msg.name)}\n")
        
        arm_joints = []
        wheel_joints = []
        other_joints = []
        
        for name in msg.name:
            if 'wheel' in name.lower():
                wheel_joints.append(name)
            elif any(kw in name.lower() for kw in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']):
                arm_joints.append(name)
            else:
                other_joints.append(name)
        
        print("   ARM JOINTS (likely):")
        for i, name in enumerate(arm_joints, 1):
            print(f"      {i}. {name}")
        
        if wheel_joints:
            print("\n   WHEEL JOINTS (skipping):")
            for name in wheel_joints[:3]:
                print(f"      - {name}")
            if len(wheel_joints) > 3:
                print(f"      ... and {len(wheel_joints)-3} more")
        
        if other_joints:
            print("\n   OTHER JOINTS:")
            for name in other_joints:
                print(f"      - {name}")
                
    except rospy.ROSException as e:
        print(f"   ERROR: {e}")
        arm_joints = []
    
    # 2. List all topics with 'joint' or 'arm' in name
    print("\n\n2. AVAILABLE CONTROL TOPICS:")
    print("-" * 70)
    topics = rospy.get_published_topics()
    
    joint_topics = []
    arm_topics = []
    controller_topics = []
    
    for topic_name, topic_type in topics:
        topic_lower = topic_name.lower()
        if 'command' in topic_lower:
            if 'joint' in topic_lower:
                joint_topics.append((topic_name, topic_type))
            elif 'arm' in topic_lower:
                arm_topics.append((topic_name, topic_type))
            elif 'controller' in topic_lower:
                controller_topics.append((topic_name, topic_type))
    
    if joint_topics:
        print("   JOINT COMMAND TOPICS:")
        for topic_name, topic_type in joint_topics:
            print(f"      {topic_name} ({topic_type})")
    
    if arm_topics:
        print("\n   ARM COMMAND TOPICS:")
        for topic_name, topic_type in arm_topics:
            print(f"      {topic_name} ({topic_type})")
    
    if controller_topics:
        print("\n   CONTROLLER COMMAND TOPICS:")
        for topic_name, topic_type in controller_topics[:10]:  # Limit to first 10
            print(f"      {topic_name} ({topic_type})")
        if len(controller_topics) > 10:
            print(f"      ... and {len(controller_topics)-10} more")
    
    # 3. List all topics (for reference)
    print("\n\n3. ALL TOPICS WITH 'joint' OR 'arm' (for reference):")
    print("-" * 70)
    relevant_topics = [(name, typ) for name, typ in topics 
                      if 'joint' in name.lower() or 'arm' in name.lower()]
    
    for topic_name, topic_type in relevant_topics[:20]:  # Limit to first 20
        print(f"   {topic_name} ({topic_type})")
    if len(relevant_topics) > 20:
        print(f"   ... and {len(relevant_topics)-20} more")
    
    # 4. Summary and recommendations
    print("\n\n4. SUMMARY AND RECOMMENDATIONS:")
    print("-" * 70)
    if arm_joints:
        print(f"   ✓ Found {len(arm_joints)} arm joints")
        print("   ✓ Use these joint names in the controller:")
        print("      joint_names = [")
        for name in arm_joints:
            print(f"          '{name}',")
        print("      ]")
    else:
        print("   ✗ No arm joints found!")
    
    if joint_topics or arm_topics:
        print(f"\n   ✓ Found {len(joint_topics) + len(arm_topics)} potential control topics")
        print("   ✓ Try using these topics for arm control")
    else:
        print("\n   ✗ No obvious control topics found")
        print("   → May need to use MoveIt or joint_state_publisher")
    
    print("\n" + "="*70 + "\n")

if __name__ == '__main__':
    try:
        diagnose_robot()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

