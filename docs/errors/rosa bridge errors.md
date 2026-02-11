# Launchfile cant find xacro or udrf

```bash
agilex@master:~/llm-controlled-robots/catkin_ws$ roslaunch limo_rosa_bridge rosa_bridge.launch
... logging to /home/agilex/.ros/log/9da1af78-074e-11f1-ab38-3c6d660d71c0/roslaunch-master-27066.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

No such file or directory: /home/agilex/agilex_ws/src/limo_description/urdf/limo.urdf.xacro [Errno 2] No such file or directory: '/home/agilex/agilex_ws/src/limo_description/urdf/limo.urdf.xacro'
RLException: Invalid <param> tag: Cannot load command parameter [robot_description]: command [['/opt/ros/noetic/lib/xacro/xacro', '/home/agilex/agilex_ws/src/limo_description/urdf/limo.urdf.xacro']] returned with code [2]. 

Param xml is <param name="robot_description" command="$(find xacro)/xacro $(find limo_description)/urdf/limo.urdf.xacro"/>
The traceback for the exception was written to the log file
```

