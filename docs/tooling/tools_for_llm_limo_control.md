# Tools for LLM-Based LIMO Cobot Control

This document summarizes the ROSA-facing tool APIs in `src/limo_llm_control/tools/`
with focus on perception/scanning workflows.

## Perception and Object Discovery

### Object-query workflow (generic objects)

- `start_object_finder_node(prompt=...)`
  - Starts OWL-ViT based object finder and publishes detections to `/object_pose`.
  - Use this for semantic categories such as bottles, pens, or cups.

- `update_object_query(query=...)`
  - Updates target object at runtime via `/object_query` without restarting detector.

- `scan_for_objects(object_query="a red cube", duration_seconds=25, spin=True, merge_distance_m=0.08)`
  - Canonical scan API.
  - Updates `/object_query`, listens on `/object_pose`, and returns deduplicated map coordinates.

- `grasp_detected_object(service_name="/object_finder/grasp_detected_object")`
  - Triggers a grasp attempt for the currently detected object via ROS `Trigger` service.
  - Reuses the existing object_finder grasp pipeline (`_compute_grasp`/`do_grasp`).

## Recommended Usage Patterns

### Find all red cubes in a room

1. Start color detector:
   - `start_object_finder_node(prompt="a red cube")`
2. Scan while rotating:
   - `scan_for_objects(object_query="a red cube", duration_seconds=30, spin=True)`

### Detect then grasp an object

1. Start detector:
   - `start_object_finder_node(prompt="a red cube")`
2. Detect and localize:
   - `scan_for_objects(object_query="a red cube", duration_seconds=20, spin=True)`
3. Trigger grasp:
   - `grasp_detected_object()`

### Find all bottles (or pens) in a room

1. Start generic detector:
   - `start_object_finder_node(prompt="a bottle")`
2. Run generic scan:
   - `scan_for_objects(object_query="a bottle", duration_seconds=30, spin=True)`

For pens, only change query string to `"a pen"` (or `"a black pen"`).

## Design Notes for Extensibility

- `scan_for_objects(...)` is the preferred entrypoint for future classes of objects.
- The scan backend uses object_finder map-frame output (`/object_pose`).
- New detector logic should feed object_finder-compatible map-frame object poses.
