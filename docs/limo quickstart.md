# Limo Quickstart

Quickly prepare the LIMO cobot to listen to commands from a remote PC,
either in **online SLAM mode (gmapping)** or using a **saved static map**.

## 1. Build and source on the robot

Log into the LIMO cobot, open a terminal and run:

```bash
cd ~/llm-controlled-robots/catkin_ws
rm -rf build devel   # optional but strongly recommended after moves
catkin_make
source devel/setup.bash
```

## 2. Launch options

### A) Online mapping with gmapping (default exploration)

Use this when you want to build or update a map:

```bash
roslaunch limo_rosa_bridge rosa_bridge.launch
```

This starts base drivers, `gmapping`, `move_base`, camera, rosbridge,
and the core autonomy stack.

### B) Static map navigation with saved map

After you have used gmapping + `map_saver` to create a map file
`limo_lab_map.yaml` in `~/llm-controlled-robots/catkin_ws/src/limo_rosa_bridge/launch/`,
you can start navigation on that fixed map:

```bash
roslaunch limo_rosa_bridge test_map.launch
```

This starts base drivers, `map_server` with your saved map, `amcl`,
`move_base`, camera, rosbridge, and the autonomy stack, so ROSA can
send goals in the `map` frame (e.g. via the `go_to_map_pose` tool).
