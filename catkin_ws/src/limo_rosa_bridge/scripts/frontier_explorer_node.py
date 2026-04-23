#!/usr/bin/env python3
"""Frontier-based exploration planner.

Responsibilities (single concern: **explore unseen space**):

* Subscribe to ``/cam_coverage`` (from ``cam_coverage_node``) and to the
  global costmap of ``move_base``.
* Select the next frontier cell (seen camera cell with unknown neighbour)
  that is free in the costmap and reachable via the global planner.
* Publish the chosen goal on ``/move_base_simple/goal`` so ``move_base``
  drives there.
* Expose an ``/exploration_enabled`` ``std_srvs/SetBool`` service so ROSA
  (or any other client) can pause/resume exploration without killing the
  node.

Out of scope (moved elsewhere after the refactor):

* Reacting to ``/object_found`` and driving to an object — this lives in
  ``approach_object_node`` now.
* Close-in / yaw align / manipulation — also lives in dedicated nodes.
"""

import math

import numpy as np
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


class FrontierExplorer:
    """Frontier-goal selector. Pure exploration; no object handling."""

    def __init__(self):
        self._load_params()
        self._init_state()
        self._init_ros_comm()

        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        # Lazy ServiceProxy: does not connect until first call, and failures
        # are caught in is_reachable(). This lets the node come up cleanly
        # even if move_base is still starting.
        self.make_plan = rospy.ServiceProxy(
            "/move_base/GlobalPlanner/make_plan", GetPlan
        )

        self.timer = rospy.Timer(rospy.Duration(1.0), self.tick)

        if self.enabled:
            rospy.loginfo(
                "[frontier_explorer] Exploration ENABLED at start "
                "(enabled_at_start=true)."
            )
        else:
            rospy.loginfo(
                "[frontier_explorer] Idle. Waiting for a client to call "
                "/exploration_enabled with data=true to start exploring."
            )

    # ----------------------------------------------------------------- init
    def _load_params(self):
        self.cov_topic = rospy.get_param("~coverage_topic", "/cam_coverage")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.global_costmap_topic = rospy.get_param(
            "~global_costmap_topic", "/move_base/global_costmap/costmap"
        )
        self.goal_reached_radius = float(rospy.get_param("~goal_reached_radius", 0.5))
        self.min_frontier_spacing_m = float(
            rospy.get_param("~min_frontier_spacing_m", 0.1)
        )
        self.min_frontier_dist_m = float(rospy.get_param("~min_frontier_dist_m", 0.3))
        self.cost_free_threshold = int(rospy.get_param("~cost_free_threshold", 10))
        self.plan_tolerance = float(rospy.get_param("~plan_tolerance", 0.3))
        self.max_plan_tries = int(rospy.get_param("~max_plan_tries", 100))
        self.frame_robot = rospy.get_param("~frame_robot", "base_link")
        self.safety_radius_m = float(rospy.get_param("~safety_radius_m", 0.2))
        self.max_candidates_eval = int(rospy.get_param("~max_candidates_eval", 100))
        # If True, start exploration right after launch. Otherwise wait until
        # a client calls /exploration_enabled set_bool(True). We default to
        # False so launching rosa_bridge.launch does not kick the robot into
        # autonomous exploration until ROSA (or another client) asks for it.
        self.enabled_at_start = bool(rospy.get_param("~enabled_at_start", False))

    def _init_state(self):
        self.costmap_msg = None
        self.cov_msg = None
        self.current_goal = None
        self.plan_fail_count = 0
        self.last_goal_distance = None
        self.last_progress_time = rospy.Time.now()
        self.enabled = bool(self.enabled_at_start)

    def _init_ros_comm(self):
        self.map_sub = rospy.Subscriber(
            self.global_costmap_topic, OccupancyGrid, self.costmap_cb, queue_size=1
        )
        self.costmap_update_sub = rospy.Subscriber(
            "/move_base/global_costmap/costmap_updates",
            OccupancyGridUpdate,
            self.costmap_update_cb,
            queue_size=10,
        )
        self.cov_sub = rospy.Subscriber(
            self.cov_topic, OccupancyGrid, self.cov_cb, queue_size=1
        )
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=1
        )
        self.frontier_pub = rospy.Publisher(
            "~frontier_markers", MarkerArray, queue_size=1
        )
        # Pause/resume exploration via ROSA.
        self.enable_srv = rospy.Service(
            "/exploration_enabled", SetBool, self._handle_enable
        )
        # Reset exploration state (drop current goal).
        self.reset_srv = rospy.Service(
            "/exploration_reset", Trigger, self._handle_reset
        )

    # ------------------------------------------------------------ callbacks
    def costmap_cb(self, msg):
        if self.costmap_msg is None:
            self.costmap_msg = msg

    def costmap_update_cb(self, upd):
        if self.costmap_msg is None:
            return
        cm = self.grid_to_array(self.costmap_msg)
        H, W = cm.shape
        ux, uy, uw, uh = upd.x, upd.y, upd.width, upd.height
        if ux < 0 or uy < 0 or ux + uw > W or uy + uh > H:
            rospy.logwarn(
                "[frontier_explorer] costmap update OOB x=%d y=%d w=%d h=%d",
                ux, uy, uw, uh,
            )
            return
        block = np.asarray(upd.data, dtype=np.int8).reshape((uh, uw))
        cm[uy:uy + uh, ux:ux + uw] = block
        self.costmap_msg.data = cm.flatten().astype(np.int8).tolist()

    def cov_cb(self, msg):
        self.cov_msg = msg

    def _handle_enable(self, req):
        was = self.enabled
        self.enabled = bool(req.data)
        if self.enabled and not was:
            # Coming out of idle: drop any stale progress tracking so the
            # next tick selects a fresh goal based on the current map.
            self.current_goal = None
            self.plan_fail_count = 0
            self.last_goal_distance = None
            self.last_progress_time = rospy.Time.now()
        if not self.enabled and was:
            # Actively cancel move_base so the robot stops right away.
            try:
                self.cancel_pub.publish(GoalID())
            except Exception:
                pass
            self.current_goal = None
        msg = "enabled" if self.enabled else "disabled"
        rospy.loginfo("[frontier_explorer] exploration %s", msg)
        return SetBoolResponse(success=True, message=f"exploration {msg}")

    def _handle_reset(self, _req):
        self.current_goal = None
        self.plan_fail_count = 0
        self.last_goal_distance = None
        try:
            self.cancel_pub.publish(GoalID())
        except Exception:
            pass
        return TriggerResponse(success=True, message="exploration state reset")

    # -------------------------------------------------------------- main loop
    def tick(self, _evt):
        if not self.enabled:
            return
        if not self._is_ready_for_tick():
            return

        try:
            T = self.tfbuf.lookup_transform(
                self.costmap_msg.header.frame_id,
                self.frame_robot,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        except Exception:
            return
        rx = T.transform.translation.x
        ry = T.transform.translation.y

        if self.current_goal is not None:
            if self._update_current_goal_status(rx, ry):
                return

        if self.current_goal is None:
            goal = self.select_next_goal((rx, ry))
            if goal is not None:
                self.current_goal = goal
                self.goal_pub.publish(goal)
                rospy.loginfo(
                    "[frontier_explorer] New goal (%.2f, %.2f)",
                    goal.pose.position.x, goal.pose.position.y,
                )

    def _is_ready_for_tick(self):
        if self.costmap_msg is None or self.cov_msg is None:
            return False
        if self.costmap_msg.header.frame_id != self.cov_msg.header.frame_id:
            rospy.logwarn_throttle(
                5.0,
                "[frontier_explorer] Frame mismatch costmap=%s coverage=%s",
                self.costmap_msg.header.frame_id,
                self.cov_msg.header.frame_id,
            )
            return False
        return True

    def _update_current_goal_status(self, rx, ry):
        gx = self.current_goal.pose.position.x
        gy = self.current_goal.pose.position.y
        dist = math.hypot(gx - rx, gy - ry)

        if dist <= self.goal_reached_radius:
            rospy.loginfo("[frontier_explorer] Goal reached.")
            self.current_goal = None
            self.plan_fail_count = 0
            self.last_goal_distance = None
            return True

        now = rospy.Time.now()
        if self.last_goal_distance is None or dist < self.last_goal_distance - 0.05:
            self.last_goal_distance = dist
            self.last_progress_time = now

        infeasible = (now - self.last_progress_time).to_sec() > 8.0
        if not self.is_goal_region_free(gx, gy, radius_m=0.05):
            infeasible = True
        if infeasible:
            rospy.logwarn("[frontier_explorer] Goal dropped (no progress / blocked).")
            self.current_goal = None
            self.plan_fail_count = 0
            self.last_goal_distance = None
            return True
        return False

    # ------------------------------------------------------- frontier selection
    def select_next_goal(self, robot_xy):
        cm = self.grid_to_array(self.costmap_msg)
        cov = self.grid_to_array(self.cov_msg)
        if cm is None or cov is None:
            return None
        info = self.costmap_msg.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        H, W = cm.shape

        free_cm = (cm >= 0) & (cm <= self.cost_free_threshold)
        seen = (cov == 100)
        unknown = (cov < 0)

        neigh = _count_unknown_neighbors(unknown)
        frontier = seen & (neigh > 0)
        cy_all, cx_all = np.where(frontier)
        if cy_all.size == 0:
            return None

        mask = free_cm[cy_all, cx_all]
        cx_all, cy_all = cx_all[mask], cy_all[mask]
        if cy_all.size == 0:
            return None

        xs = ox + (cx_all.astype(np.float32) + 0.5) * res
        ys = oy + (cy_all.astype(np.float32) + 0.5) * res
        rx, ry = robot_xy
        dists = np.hypot(xs - rx, ys - ry)
        keep = dists >= self.min_frontier_dist_m
        if not np.any(keep):
            return None
        cx_all, cy_all, dists = cx_all[keep], cy_all[keep], dists[keep]

        # Greedy spatial thinning
        min_space_cells = max(1, int(self.min_frontier_spacing_m / max(res, 1e-6)))
        order = np.argsort(dists)
        limit = min(self.max_candidates_eval, int(dists.size))
        chosen = []
        sel_for_space = []
        for idx in order[:limit]:
            cy = int(cy_all[idx])
            cx = int(cx_all[idx])
            if not _is_far_from_all((cy, cx), sel_for_space, min_space_cells):
                continue
            chosen.append((cy, cx, idx))
            sel_for_space.append((cy, cx))
        if not chosen:
            return None

        # Clearance via integral image
        free_int = np.zeros((H + 1, W + 1), dtype=np.int32)
        free_int[1:, 1:] = np.cumsum(np.cumsum(free_cm.astype(np.uint8), axis=0), axis=1)
        cell_r = max(1, int(self.safety_radius_m / max(res, 1e-6)))

        cands = []
        for (cy, cx, idx) in chosen:
            clr = self._clearance(cx, cy, cell_r, free_int, W, H)
            cands.append((clr, float(dists[idx]), cy, cx))
        cands.sort(key=lambda c: (-c[0], c[1]))

        self._publish_frontier_markers([c[2] for c in cands], [c[3] for c in cands])

        tried = 0
        for (_, _d, cy, cx) in cands:
            pose, yaw = self._cell_to_goal_pose(cx, cy, res, unknown)
            mm = self.world_to_map(pose.position.x, pose.position.y, self.costmap_msg)
            if mm is None:
                continue
            if self.is_reachable(pose):
                return _to_ps(pose, self.costmap_msg.header.frame_id)
            tried += 1
            if tried >= self.max_plan_tries:
                break
        return None

    # --------------------------------------------------- low-level helpers

    def _clearance(self, mx, my, cell_r, free_int, w, h):
        y0, y1 = max(0, my - cell_r), min(h - 1, my + cell_r)
        x0, x1 = max(0, mx - cell_r), min(w - 1, mx + cell_r)
        free = (
            free_int[y1 + 1, x1 + 1]
            - free_int[y0, x1 + 1]
            - free_int[y1 + 1, x0]
            + free_int[y0, x0]
        )
        data = self.costmap_msg.data
        total_known = 0
        for iy in range(y0, y1 + 1):
            base = iy * w
            for ix in range(x0, x1 + 1):
                if data[base + ix] >= 0:
                    total_known += 1
        if total_known == 0:
            return 0.0
        return float(free) / float(total_known)

    @staticmethod
    def grid_to_array(grid):
        data = np.asarray(grid.data, dtype=np.int16)
        return data.reshape((grid.info.height, grid.info.width))

    def _cell_to_goal_pose(self, cx, cy, res, unknown_mask):
        ox = self.costmap_msg.info.origin.position.x
        oy = self.costmap_msg.info.origin.position.y
        x = ox + (cx + 0.5) * res
        y = oy + (cy + 0.5) * res
        yaw = _face_unknown_yaw(cx, cy, unknown_mask)
        q = _yaw_to_quat(yaw)
        return Pose(Point(x, y, 0.0), q), yaw

    def world_to_map(self, x, y, grid):
        info = grid.info
        res = info.resolution
        mx = int((x - info.origin.position.x) / res)
        my = int((y - info.origin.position.y) / res)
        if 0 <= mx < info.width and 0 <= my < info.height:
            return mx, my
        return None

    def is_goal_region_free(self, gx, gy, radius_m=0.30):
        if self.costmap_msg is None:
            return True
        info = self.costmap_msg.info
        res = info.resolution
        w, h = info.width, info.height
        data = self.costmap_msg.data
        mm = self.world_to_map(gx, gy, self.costmap_msg)
        if mm is None:
            return False
        mx, my = mm
        cell_r = max(1, int(radius_m / max(res, 1e-6)))
        for iy in range(max(0, my - cell_r), min(h, my + cell_r + 1)):
            for ix in range(max(0, mx - cell_r), min(w, mx + cell_r + 1)):
                if (ix - mx) ** 2 + (iy - my) ** 2 > cell_r ** 2:
                    continue
                v = data[iy * w + ix]
                if v < 0 or v > self.cost_free_threshold:
                    return False
        return True

    def is_reachable(self, goal_pose):
        try:
            T = self.tfbuf.lookup_transform(
                self.costmap_msg.header.frame_id,
                self.frame_robot,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        except Exception:
            return False
        start = PoseStamped()
        start.header.frame_id = self.costmap_msg.header.frame_id
        start.pose.position.x = T.transform.translation.x
        start.pose.position.y = T.transform.translation.y
        start.pose.orientation = Quaternion(0, 0, 0, 1)
        goal = PoseStamped()
        goal.header.frame_id = self.costmap_msg.header.frame_id
        goal.pose = goal_pose
        req = GetPlanRequest()
        req.start = start
        req.goal = goal
        req.tolerance = self.plan_tolerance
        try:
            resp = self.make_plan(req)
            return len(resp.plan.poses) > 0
        except rospy.ServiceException:
            return False

    def _publish_frontier_markers(self, cys, cxs):
        if self.cov_msg is None:
            return
        arr = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)
        info = self.cov_msg.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        for i, (cy, cx) in enumerate(zip(cys, cxs)):
            m = Marker()
            m.header = self.cov_msg.header
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = ox + (cx + 0.5) * res
            m.pose.position.y = oy + (cy + 0.5) * res
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.g = 0.7
            m.color.b = 1.0
            m.color.a = 1.0
            arr.markers.append(m)
        self.frontier_pub.publish(arr)


# --------------------------------------------------------------- free functions


def _count_unknown_neighbors(unknown_mask):
    H, W = unknown_mask.shape
    cnt = np.zeros_like(unknown_mask, dtype=np.int16)
    dirs = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    for dy, dx in dirs:
        y0 = max(0, -dy); y1 = min(H, H - dy)
        x0 = max(0, -dx); x1 = min(W, W - dx)
        src = unknown_mask[y0:y1, x0:x1]
        cnt[y0 + dy:y1 + dy, x0 + dx:x1 + dx] += src.astype(np.int16)
    return cnt


def _is_far_from_all(cell, chosen, min_spacing_cells):
    cy, cx = cell
    for (py, px) in chosen:
        if abs(py - cy) <= min_spacing_cells and abs(px - cx) <= min_spacing_cells:
            if (py - cy) ** 2 + (px - cx) ** 2 <= (min_spacing_cells ** 2):
                return False
    return True


def _face_unknown_yaw(cx, cy, unknown_mask):
    H, W = unknown_mask.shape
    dirs = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    vx, vy = 0.0, 0.0
    for dy, dx in dirs:
        ny, nx = cy + dy, cx + dx
        if 0 <= ny < H and 0 <= nx < W and unknown_mask[ny, nx]:
            vx += dx
            vy += dy
    if vx == 0.0 and vy == 0.0:
        return 0.0
    return math.atan2(vy, vx)


def _yaw_to_quat(yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)


def _to_ps(pose, frame_id):
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = frame_id
    ps.pose = pose
    return ps


if __name__ == "__main__":
    rospy.init_node("frontier_explorer")
    FrontierExplorer()
    rospy.loginfo("[frontier_explorer] ready")
    rospy.spin()
