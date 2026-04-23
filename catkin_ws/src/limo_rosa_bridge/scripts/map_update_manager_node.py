#!/usr/bin/env python3
"""Map update helper: expose a service to persist the current /map to disk."""

from __future__ import annotations

import os
import re
import subprocess
from typing import Tuple

import rospy
from std_srvs.srv import Trigger, TriggerResponse


_SAFE_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


class MapUpdateManager:
    def __init__(self) -> None:
        self.output_dir = rospy.get_param("~output_dir", "")
        self.default_map_name = rospy.get_param("~default_map_name", "limo_lab_map")
        self.next_map_name_param = rospy.get_param("~next_map_name_param", "~next_map_name")
        self.save_timeout_s = float(rospy.get_param("~save_timeout_s", 90.0))
        self.map_topic = rospy.get_param("~map_topic", "/map")

        if not self.output_dir:
            self.output_dir = os.path.expanduser("~/llm-controlled-robots/catkin_ws/src/limo_rosa_bridge/maps")
        self.output_dir = os.path.abspath(os.path.expanduser(self.output_dir))

        rospy.Service("~save_map", Trigger, self._handle_save_map)
        rospy.loginfo("map_update_manager ready: output_dir=%s", self.output_dir)

    def _resolve_map_name(self) -> Tuple[bool, str]:
        next_name = rospy.get_param(self.next_map_name_param, "").strip()
        name = next_name or self.default_map_name
        if not _SAFE_NAME_RE.fullmatch(name):
            return False, (
                "Invalid map name. Use only letters, numbers, underscore, or hyphen; "
                f"got '{name}'."
            )
        return True, name

    def _handle_save_map(self, _req: Trigger) -> TriggerResponse:
        ok, name_or_msg = self._resolve_map_name()
        if not ok:
            return TriggerResponse(success=False, message=name_or_msg)
        map_name = name_or_msg

        try:
            os.makedirs(self.output_dir, exist_ok=True)
        except Exception as exc:
            return TriggerResponse(success=False, message=f"Failed to create map dir: {exc}")

        map_prefix = os.path.join(self.output_dir, map_name)
        cmd = ["rosrun", "map_server", "map_saver", "-f", map_prefix, "map:=" + self.map_topic]

        rospy.loginfo("Saving map with command: %s", " ".join(cmd))
        try:
            completed = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(1.0, self.save_timeout_s),
            )
        except subprocess.TimeoutExpired:
            return TriggerResponse(
                success=False,
                message=(
                    f"map_saver timed out after {self.save_timeout_s:.1f}s. "
                    f"Ensure topic '{self.map_topic}' is being published."
                ),
            )
        except Exception as exc:
            return TriggerResponse(success=False, message=f"map_saver launch failed: {exc}")

        if completed.returncode != 0:
            stderr = (completed.stderr or "").strip()
            stdout = (completed.stdout or "").strip()
            detail = stderr or stdout or "unknown error"
            return TriggerResponse(success=False, message=f"map_saver failed: {detail}")

        yaml_path = map_prefix + ".yaml"
        pgm_path = map_prefix + ".pgm"
        return TriggerResponse(
            success=True,
            message=f"Saved map: {yaml_path} (image: {pgm_path})",
        )


def main() -> None:
    rospy.init_node("map_update_manager")
    MapUpdateManager()
    rospy.spin()


if __name__ == "__main__":
    main()
