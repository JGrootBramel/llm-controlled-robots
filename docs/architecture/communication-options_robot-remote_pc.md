# Communication Options (Robot ↔ Remote PC)

## Option A — Native ROS1 networking (recommended if feasible)

Remote PC runs ROS nodes (or Python tools using rospy) that connect directly to the robot’s ROS master.

**Pros**

- Best compatibility with ROS1 (topics, services, actions, TF)
- Lower overhead and better reliability than websockets for many use cases
- Works naturally with RViz/rqt and standard tooling

**Cons**

- Requires correct network configuration (ROS_MASTER_URI, ROS_IP/ROS_HOSTNAME)
- Can be fragile across subnets/VPN/NAT
- Security is minimal unless network is isolated

**When to use**

- You control the LAN and can place remote PC on the same subnet
- You need actions/TF/tools to work “as designed”

## Option B — rosbridge (WebSocket JSON bridge)

Remote PC talks to the robot via rosbridge (typically on port 9090) using roslibpy, etc.

**Pros**

- Easier to traverse network boundaries than native ROS1 in some setups
- Language-agnostic client ecosystem
- Useful for web UIs and simple command/control

**Cons**

- High-rate messages (images/point clouds) are often problematic
- Actions/TF support can be more complex or less robust depending on client
- Adds another moving part (rosbridge server configuration, message size limits)

**When to use**

- You cannot reliably join the robot ROS master with native ROS1 networking
- Your interactions are discrete and low-bandwidth (commands, status queries)

**Option C — Hybrid (common and pragmatic)**

- Use native ROS1 for high-rate/real-time and core robotics interfaces
- Use rosbridge only for UI/telemetry or limited remote calls
- Keep perception on-robot and expose a clean service/action API

**Recommended pattern**

- Robot publishes detection results as /detections and/or offers /detect_objects service
- Remote ROSA tool calls /detect_objects and receives structured output
- Remote does not subscribe to /camera/image_raw unless debugging