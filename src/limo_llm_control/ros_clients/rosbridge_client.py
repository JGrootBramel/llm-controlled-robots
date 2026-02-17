"""
Optional rosbridge (WebSocket) client for when native ROS1 networking is not feasible.

Use when: remote PC cannot join robot's ROS master (e.g. across subnets/VPN).
Prefer ros1_client (rospy) when on same LAN; use rosbridge for discrete, low-bandwidth calls.
"""

# Stub: implement with roslibpy when rosbridge is required.
# Tools in limo_llm_control/tools use rospy via ros1_client by default.
