# Limo Quickstart

Quickly prepare the limo cobot to listen to commands from a remote pc.

Loginto the limo cobot with no machine. Start terminal and execute:

```bash
roslaunch limo_rosa_bridge rosa_bridge.launch
```

If the package cannot be found try to rebuild the package:
```bash
cd ~/llm-controlled-robots/catkin_ws
rm -rf build devel   # optional but strongly recommended after moves
catkin_make
source devel/setup.bash
```
