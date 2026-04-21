# Tools for LLM-Based LIMO Cobot Control

This document summarizes the ROSA-facing tool APIs in `src/limo_llm_control/tools/`
with focus on perception/scanning workflows.

## Perception and Item Discovery

### Object-query workflow (generic items)

- `start_object_finder_node(prompt=...)`
  - Starts OWL-ViT based object finder and publishes detections to `/object_pose`.
  - Use this for semantic categories such as bottles, pens, or cups.

- `update_object_query(query=...)`
  - Updates target object at runtime via `/object_query` without restarting detector.

- `scan_for_items(item_query="a bottle", source="object_pose", duration_seconds=25, spin=True, merge_distance_m=0.08)`
  - Generic scan API for extensible item types.
  - If `source="object_pose"`, the tool updates `/object_query` using `item_query`.
  - Deduplicates nearby detections and returns merged map coordinates.

### Color-cube workflow (HSV)

- `start_color_cube_grasper_node(target_color="red", ...)`
  - Starts HSV-based cube detector/grasper with configurable color presets.
  - Presets:
    - `red` -> `H=[0,15]`
    - `blue` -> `H=[95,135]`
    - `green` -> `H=[40,90]`
  - Optional overrides: `h_low`, `h_high`, `s_low`, `v_low`.

- `scan_for_red_cubes(duration_seconds=25, spin=True)`
  - Convenience wrapper to scan red cubes from `/blue_cube_grasper/cube_map_pose`.

- `scan_for_blue_cubes(duration_seconds=25, spin=True)`
  - Legacy-compatible cube scan wrapper using same scan backend.

## Recommended Usage Patterns

### Find all red cubes in a room

1. Start color detector:
   - `start_color_cube_grasper_node(target_color="red")`
2. Scan while rotating:
   - `scan_for_red_cubes(duration_seconds=30, spin=True)`

### Find all bottles (or pens) in a room

1. Start generic detector:
   - `start_object_finder_node(prompt="a bottle")`
2. Run generic scan:
   - `scan_for_items(item_query="a bottle", source="object_pose", duration_seconds=30, spin=True)`

For pens, only change query string to `"a pen"` (or `"a black pen"`).

## Design Notes for Extensibility

- `scan_for_items(...)` is the preferred entrypoint for future classes of objects.
- The scan backend supports both:
  - `PointStamped` sources (e.g., cube pose topic)
  - `PoseStamped` sources (e.g., object pose topic)
- New detectors can be integrated by publishing to a compatible pose topic and reusing the same scan tool.
