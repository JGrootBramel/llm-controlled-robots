"""Pixel -> 3D world-frame projection helpers.

The ROS-specific parts (``tf2_geometry_msgs.do_transform_point``) are kept
thin so that the geometric math stays unit-testable with a fake transform
buffer.

The API deliberately accepts/returns plain tuples so callers (and tests)
don't need to construct ``PointStamped`` objects.
"""

from __future__ import annotations

from typing import Callable, Optional, Tuple


# ---- camera intrinsics ------------------------------------------------------


def pixel_to_camera(
    u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float
) -> Tuple[float, float, float]:
    """Back-project a pinhole pixel at depth ``z`` into the camera frame.

    Returns ``(X, Y, Z)`` in metres in the optical/camera frame (X right,
    Y down, Z forward — the ROS convention used by ``camera_info``).
    """
    x = (float(u) - float(cx)) * float(z) / float(fx)
    y = (float(v) - float(cy)) * float(z) / float(fy)
    return x, y, float(z)


# ---- TF-aware projection helpers -------------------------------------------


# Small seam so tests can supply a fake transform callback instead of a ROS TF
# buffer. ``transform_point_fn`` takes ``(point_xyz, source_frame, target_frame,
# stamp)`` and returns a transformed ``(x, y, z)`` tuple or raises.
TransformPointFn = Callable[[Tuple[float, float, float], str, str, object], Tuple[float, float, float]]


def camera_pixel_to_frame(
    u: float,
    v: float,
    z: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    source_frame: str,
    target_frame: str,
    transform_point_fn: TransformPointFn,
    stamp: object = None,
) -> Optional[Tuple[float, float, float]]:
    """End-to-end: pinhole (u,v,z) in ``source_frame`` -> point in ``target_frame``.

    The actual TF lookup is delegated to ``transform_point_fn`` so this
    function has no ROS dependencies and can be unit tested with a plain
    Python callable.
    """
    cam_xyz = pixel_to_camera(u, v, z, fx, fy, cx, cy)
    try:
        return transform_point_fn(cam_xyz, source_frame, target_frame, stamp)
    except Exception:
        return None


# ---- tf2 adapter -----------------------------------------------------------


def make_tf2_transform_point_fn(tf_buffer, lookup_timeout_s: float = 0.3) -> TransformPointFn:
    """Return a ``TransformPointFn`` that uses a ``tf2_ros.Buffer`` instance.

    This is the only function that imports ROS message types, and it does so
    lazily so the helper stays importable on CI hosts without ROS.
    """
    import rospy  # type: ignore
    import tf2_geometry_msgs  # type: ignore  # noqa: F401  (registers do_transform_point)
    from geometry_msgs.msg import PointStamped  # type: ignore

    def _fn(point_xyz, source_frame, target_frame, stamp):
        ps = PointStamped()
        ps.header.frame_id = source_frame
        ps.header.stamp = stamp if stamp is not None else rospy.Time(0)
        ps.point.x = float(point_xyz[0])
        ps.point.y = float(point_xyz[1])
        ps.point.z = float(point_xyz[2])
        t = tf_buffer.lookup_transform(
            target_frame, source_frame, rospy.Time(0), rospy.Duration(lookup_timeout_s)
        )
        out = tf2_geometry_msgs.do_transform_point(ps, t).point
        return (float(out.x), float(out.y), float(out.z))

    return _fn
