agilex@master:~/catkin_ws/src/limo_llm$ roslaunch limo_llm pick_cube.launch
... logging to /home/agilex/.ros/log/cfd6f424-f076-11f0-9163-3c6d660d71c0/roslaunch-master-4634.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:40849/

SUMMARY
========

PARAMETERS
 * /limo_base_node/base_frame: base_link
 * /limo_base_node/odom_frame: odom
 * /limo_base_node/port_name: ttyTHS0
 * /limo_base_node/pub_odom_tf: False
 * /limo_base_node/use_mcnamu: False
 * /rosdistro: noetic
 * /rosversion: 1.16.0
 * /ydlidar_lidar_publisher/abnormal_check_count: 4
 * /ydlidar_lidar_publisher/angle_max: 100.0
 * /ydlidar_lidar_publisher/angle_min: -100.0
 * /ydlidar_lidar_publisher/auto_reconnect: True
 * /ydlidar_lidar_publisher/baudrate: 230400
 * /ydlidar_lidar_publisher/device_type: 0
 * /ydlidar_lidar_publisher/frame_id: laser_link
 * /ydlidar_lidar_publisher/frequency: 6.0
 * /ydlidar_lidar_publisher/ignore_array: 
 * /ydlidar_lidar_publisher/intensity: True
 * /ydlidar_lidar_publisher/intensity_bit: 8
 * /ydlidar_lidar_publisher/invalid_range_is_inf: False
 * /ydlidar_lidar_publisher/inverted: True
 * /ydlidar_lidar_publisher/isSingleChannel: False
 * /ydlidar_lidar_publisher/lidar_type: 1
 * /ydlidar_lidar_publisher/point_cloud_preservative: False
 * /ydlidar_lidar_publisher/port: /dev/ydlidar
 * /ydlidar_lidar_publisher/range_max: 16.0
 * /ydlidar_lidar_publisher/range_min: 0.02
 * /ydlidar_lidar_publisher/resolution_fixed: True
 * /ydlidar_lidar_publisher/reversion: False
 * /ydlidar_lidar_publisher/sample_rate: 4
 * /ydlidar_lidar_publisher/support_motor_dtr: False

NODES
  /
    base_link_to_camera_link (tf/static_transform_publisher)
    base_link_to_imu_link (tf/static_transform_publisher)
    base_link_to_laser_link (tf/static_transform_publisher)
    limo_base_node (limo_base/limo_base_node)
    ydlidar_lidar_publisher (ydlidar_ros_driver/ydlidar_ros_driver_node)

auto-starting new master
process[master]: started with pid [4647]
ERROR: Unable to start XML-RPC server, port 11311 is already in use
Exception ignored in thread started by: <bound method XmlRpcNode.run of <rosgraph.xmlrpc.XmlRpcNode object at 0xffff9d68e040>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rosgraph/xmlrpc.py", line 252, in run
    self._run()
  File "/opt/ros/noetic/lib/python3/dist-packages/rosgraph/xmlrpc.py", line 321, in _run
    self._run_init()
  File "/opt/ros/noetic/lib/python3/dist-packages/rosgraph/xmlrpc.py", line 271, in _run_init
    self.server = ThreadingXMLRPCServer((bind_address, port), log_requests)
  File "/opt/ros/noetic/lib/python3/dist-packages/rosgraph/xmlrpc.py", line 149, in __init__
    SimpleXMLRPCServer.__init__(self, addr, SilenceableXMLRPCRequestHandler, log_requests)
  File "/usr/lib/python3.8/xmlrpc/server.py", line 605, in __init__
    socketserver.TCPServer.__init__(self, addr, requestHandler, bind_and_activate)
  File "/usr/lib/python3.8/socketserver.py", line 452, in __init__
    self.server_bind()
  File "/usr/lib/python3.8/socketserver.py", line 466, in server_bind
    self.socket.bind(self.server_address)
OSError: [Errno 98] Address already in use
RLException: ERROR: could not contact master [http://139.6.65.144:11311]
The traceback for the exception was written to the log file
[master] killing on exit
