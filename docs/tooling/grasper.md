# Grasper

To grasp objects we need the orocos kynematics dynamics (PyKDL) project from github. We need to install it in the catkin package on the remote pc.

To install PyKDL we followed their [installation guide on github](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md)

## Install Prerequsits

```bash
sudo apt-get update # Optional
sudo apt-get install libeigen3-dev libcppunit-dev
sudo apt-get install libeigen3-dev libcppunit-dev
sudo apt-get install doxygen graphviz # Optional
```

## Install PyKDL

```bash
cd ~/llm-controlled-robots/catkin_ws/src/
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
cd ~/llm-controlled-robots/catkin_ws/
catkin_make
```

This installation failed.

We are exloring to run PyKDL on the robot directly or create a different server.

We changed the limo_rosa_bridge.launch file to include PyKDL. Which requierd `ros-noetic-kdl-parser-py` to be installed.

```bash
sudo apt install ros-noetic-kdl-parser-py
```
