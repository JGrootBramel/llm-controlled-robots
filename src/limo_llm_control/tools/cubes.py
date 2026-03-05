import rospy
import math
from geometry_msgs.msg import Twist, PointStamped
from std_srvs.srv import Trigger
from langchain.tools import tool
import actionlib
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

@tool
def scan_for_blue_cubes(duration_seconds: int = 25) -> str:
    """
    Spins the robot in a circle to scan the room for blue cubes.
    Returns a list of the X, Y coordinates of all unique cubes found.
    """
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    found_cubes = set()
    
    def cube_cb(msg: PointStamped):
        # Round to 1 decimal place (~10cm resolution) to group duplicates
        x = round(msg.point.x, 1)
        y = round(msg.point.y, 1)
        found_cubes.add((x, y))

    sub = rospy.Subscriber('/blue_cube_grasper/cube_map_pose', PointStamped, cube_cb)
    
    # Spin the robot slowly
    tw = Twist()
    tw.angular.z = 0.2
    
    end_time = rospy.Time.now() + rospy.Duration(duration_seconds)
    rate = rospy.Rate(10)
    
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        cmd_pub.publish(tw)
        rate.sleep()
        
    # Stop spinning
    cmd_pub.publish(Twist())
    sub.unregister()
    
    if not found_cubes:
        return "Scan complete. No blue cubes detected."
        
    result = "Scan complete. Found blue cubes at the following map coordinates:\n"
    for idx, (x, y) in enumerate(found_cubes):
        result += f"- Cube {idx+1}: x={x}, y={y}\n"
    return result


@tool
def fetch_and_store_cube(cube_x: float, cube_y: float) -> str:
    """
    Drives the robot to a blue cube, picks it up, and places it on the tray.
    Provide the exact cube_x and cube_y coordinates found from the scan.
    """
    # 0. Import move_base message types lazily so rosa_agent can start
    # even on machines without the ROS move_base_msgs package installed.
    try:
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # type: ignore[import-not-found]
    except Exception as exc:  # pragma: no cover
        return f"Error: move_base_msgs is not available in this environment: {exc}"

    # 1. We must calculate a standoff position so the robot doesn't run over the cube!
    # Get current robot position using TF
    import tf2_ros
    tfbuf = tf2_ros.Buffer()
    tfl = tf2_ros.TransformListener(tfbuf)
    rospy.sleep(0.5) # Give TF time to buffer
    
    try:
        trans = tfbuf.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
    except Exception:
        return "Error: Could not determine current robot position to calculate approach."

    # Calculate vector from robot to cube
    dx = cube_x - rx
    dy = cube_y - ry
    distance = math.hypot(dx, dy)
    
    if distance < 0.1:
        return "Error: Robot is too close to calculate a safe standoff trajectory."

    # Calculate standoff goal (e.g. 0.35 meters away from the cube)
    standoff = 0.35 
    goal_x = cube_x - (standoff * (dx / distance))
    goal_y = cube_y - (standoff * (dy / distance))
    goal_yaw = math.atan2(dy, dx)
    
    # 2. Command navigation (you can reuse your existing go_to_pose tool logic here)
    # Assuming you have a standard action client for move_base setup:
    
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    q = quaternion_from_euler(0, 0, goal_yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    
    client.send_goal(goal)
    client.wait_for_result()
    
    if client.get_state() != GoalStatus.SUCCEEDED:
        return "Error: Failed to navigate to the cube's location."
        
    # 3. Trigger the arm grasp sequence
    rospy.wait_for_service('/blue_cube_grasper/execute_grasp', timeout=5.0)
    try:
        grasp_trigger = rospy.ServiceProxy('/blue_cube_grasper/execute_grasp', Trigger)
        resp = grasp_trigger()
        return "Navigation successful. Grasp sequence initiated. " + resp.message
    except rospy.ServiceException as e:
        return f"Navigation successful, but failed to trigger arm grasp: {e}"