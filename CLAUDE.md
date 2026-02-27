# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

As the codebase evolves, any agent making non-trivial changes should also update this document so that node names, topics, workflows, configuration descriptions, and other details stay in sync with the current implementation.

## Workflow Orchestration

### 1. Plan Mode Default

- Enter plan mode for any non-trivial task (3+ steps or architectural decisions).
- If something goes sideways, stop and re-plan immediately — don't keep pushing.
- Use plan mode for verification steps, not just building.
- Write detailed specs upfront to reduce ambiguity.

### 2. Subagent Strategy

- Use subagents liberally to keep the main context window clean.
- Offload research, exploration, and parallel analysis to subagents.
- For complex problems, throw more compute at it via subagents.
- One task per subagent for focused execution.

### 3. Self-Improvement Loop

- After any correction from the user, update `tasks/lessons.md` with the pattern.
- Write rules for yourself that prevent the same mistake.
- Ruthlessly iterate on these lessons until the mistake rate drops.
- Review lessons at session start for the relevant project.

### 4. Verification Before Done

- Never mark a task complete without proving it works.
- Diff behavior between main and your changes when relevant.
- Ask yourself: "Would a staff engineer approve this?"
- Run tests, check logs, and demonstrate correctness.

### 5. Demand Elegance (Balanced)

- For non-trivial changes, pause and ask "is there a more elegant way?"
- If a fix feels hacky, ask: "Knowing everything I know now, what is the elegant solution?"
- Skip this for simple, obvious fixes — don't over-engineer.
- Challenge your own work before presenting it.

### 6. Autonomous Bug Fixing

- When given a bug report, just fix it — don't ask for hand-holding.
- Point at logs, errors, and failing tests — then resolve them.
- Avoid causing extra context switching for the user.
- Fix failing tests or CI issues without needing detailed instructions.

## Task Management

1. **Plan First**: Write a plan to `tasks/todo.md` with checkable items.
2. **Verify Plan**: Check in with the user before starting implementation when plans are non-trivial or ambiguous.
3. **Track Progress**: Mark items complete as you go.
4. **Explain Changes**: Provide a high-level summary of changes at each major step.
5. **Document Results**: Add a brief review section to `tasks/todo.md` describing what was verified.
6. **Capture Lessons**: Update `tasks/lessons.md` after corrections so future work improves.

## Core Principles

- **Simplicity First**: Make every change as simple as possible. Touch minimal code.
- **No Laziness**: Find root causes; avoid band-aid fixes. Hold to senior engineer standards.
- **Minimal Impact**: Changes should only touch what's necessary and must avoid introducing new bugs.

## Project Overview

Buckeye Vertical Core (`bv_core`) — a ROS 2 (Humble) Python package for autonomous drone missions using PX4 via MAVROS. The drone performs waypoint laps, object detection scanning, geolocation/filtering, payload delivery, and image stitching.

## Build & Run Commands

```bash
# Build (from workspace root ~/bv_ws)
colcon build
source install/setup.bash

# Launch full mission stack
ros2 launch bv_core mission.launch.py

# Run individual nodes
ros2 run bv_core mission_node
ros2 run bv_core vision_node
ros2 run bv_core filtering_node
ros2 run bv_core stitching_node

# Replay recorded data
ros2 bag play <bag_path>
```

## Architecture

**Mission state machine** (`bv_core/mission.py`): TAKEOFF → LAP → SCAN → [LOCALIZE → DELIVER → DEPLOY] × N → RTL. Each detected object triggers: loiter → localize → fly to object → deploy payload via servo → resume scan.

**main ROS 2 nodes** launched together via `launch/mission.launch.py`:

- **mission_node** — FSM controller. Pushes PX4 waypoints, arms, sets mode. Subscribes to MAVROS state/position/waypoint-reached. Calls `get_object_locations` service from filtering. Publishes `/mission_state` (string) consumed by all other nodes.
- **vision_node** — Captures frames during scan state, runs RF-DETR inference in batches, publishes `/obj_dets` (bv_msgs/ObjectDetections). Signals queue readiness on `/queue_state`.
- **filtering_node** — Fuses detections with drone pose/GPS, projects pixel coordinates to lat/lon using camera intrinsics. Provides `get_object_locations` service.
- **stitching_node** — Captures images at waypoints, runs OpenCV stitcher for aerial map.

**Inter-node communication**: mission_node broadcasts state via `/mission_state` topic. Other nodes react to state transitions. Vision publishes detections → filtering accumulates and geolocates → mission queries filtering via service call after scan completes.

**Custom messages**: defined in separate `bv_msgs` package (sibling repo in `src/`). Key types: `ObjectDetections`, `ObjectLocations`, `LocalizeObject` (service).

## Topics

- **Core mission coordination**
  - **`/mission_state` (`std_msgs/String`)**: Published by `mission_node`; subscribed by `vision_node`, `filtering_node`, `stitching_node`, and `bv_viz_node`. Drives all nodes' high-level behavior.
  - **`/obj_dets` (`bv_msgs/ObjectDetections`)**: Published by `vision_node`; subscribed by `filtering_node` and `bv_viz_node`. Carries per-frame detector outputs in the camera frame.
  - **`/queue_state` (`std_msgs/Int8`)**: Published by `vision_node`; subscribed by `bv_viz_node`. Encodes internal vision queue/throughput state for HUD display.

- **Object confirmation and deployment**
  - **`/global_obj_dets` (`std_msgs/Int8`)**: Published by `filtering_node` after 3-frame confirmation; subscribed by `mission_node`. Each message is the confirmed COCO `class_id` that should trigger a localize/deliver/deploy sequence.
  - **`/deployed_object_locations` (`bv_msgs/ObjectLocations`)**: Published by `mission_node` after a successful payload deployment; subscribed by `filtering_node` so it can ignore future detections near already-serviced locations.

- **Visualization**
  - **`/viz/markers` (`visualization_msgs/MarkerArray`)**: Published by `bv_viz_node` and visualized in RViz. Aggregates drone pose, trajectory, mission waypoints, detections, and HUD information.

- **Shared MAVROS topics**
  - **`/mavros/mission/reached` (`mavros_msgs/WaypointReached`)**: Subscribed by `mission_node`, `vision_node`, `stitching_node`, and `bv_viz_node`. Signals when PX4 reports a waypoint as reached.
  - **`/mavros/mission/waypoints` (`mavros_msgs/WaypointList`)**: Subscribed by `bv_viz_node` to visualize the planned mission path.
  - **`/mavros/global_position/global` (`sensor_msgs/NavSatFix`)**: Subscribed by `mission_node`, `vision_node`, `filtering_node`, and `bv_viz_node`. Provides GPS fixes used for home position, localization, and visualization.
  - **`/mavros/global_position/rel_alt` (`std_msgs/Float64`)**: Subscribed by `vision_node` and `filtering_node` for altitude used in localization.
  - **`/mavros/local_position/pose` (`geometry_msgs/PoseStamped`)**: Subscribed by `vision_node`, `filtering_node`, and `bv_viz_node`; used for local-frame pose, localization, and visualization.

- **Camera topics used by pipelines**
  - **`/image_compressed` (`sensor_msgs/CompressedImage`)**: Optionally published by `CameraPipeline` (when recording is enabled) and consumed by `RosCamPipeline` or other ROS tools for replay and debugging.
  - **`/image_raw` (`sensor_msgs/Image`)**: Published by `camera_pipeline_test_node` to expose raw camera frames to ROS tools and RViz during pipeline debugging.

## Configuration and Modes

YAML config files in `config/` define **mission behavior**, **vision/detector settings**, and **localization geometry**. Use these, not code changes, to tune behavior or switch modes.

- **`mission_params.yaml`** — mission layout and speed/tolerance tuning
  - **Waypoints**: `points`, `scan_points`, `deliver_points`, optional `stitch_points` — define the lap path, scan leg, delivery locations, and stitching waypoints.
  - **Velocities**: `Lap_velocity`, `Scan_velocity`, `Deliver_velocity` — command PX4 cruise speeds for each phase via mission FSM.
  - **Tolerances**: `Lap_tolerance`, `Scan_tolerance`, `Deliver_tolerance` — distance thresholds for treating a waypoint as “reached” in each phase.
  - **Servos/payload**: `Default_servo_pwms`, `Deliver_initial_pwms`, `Deliver_second_pwms` — PWM values used for payload deployment sequences.

- **`vision_params.yaml`** — detector, camera pipeline, and scan behavior
  - **Detector config**:
    - `detector_type`: `"gazebo_bbox"` (sim ground truth) vs `"ml"` (RF-DETR). Controls which `BaseDetector` subclass is constructed by `create_detector(...)`.
    - `batch_size`, `detection_threshold`, `resolution`, `overlap` — RF‑DETR tiling/batching and confidence threshold settings.
  - **Camera pipeline / mode selection**:
    - `pipeline_type`: `'sim'` (Gazebo), `'real'` (physical camera via GStreamer), `'ros'` (rosbag / ROS camera feed). Controls which `VisionPipeline` subclass is built by `create_pipeline(...)`.
    - `gz_topic`, `ros_image_topic` — topics for Gazebo or ROS image sources.
    - `gst_pipeline`, `camera_fps`, `record_video` — GStreamer string and timing for real cameras, plus optional recording to `/image_compressed`.
  - **Scan behavior**:
    - `capture_interval`, `estimated_camera_latency`, `num_scan_wp` — control how often frames are captured during scan and how many waypoints are considered part of the scan leg.

- **`filtering_params.yaml`** — camera intrinsics and localization tuning
  - **Intrinsics/distortion**:
    - `c_matrix` — 3×3 camera intrinsic matrix (flattened list); must match the active camera model and resolution.
    - `dist_coefficients` — distortion coefficients; used to undistort pixels before geolocation.
  - **Camera pose**:
    - `camera_orientation` — camera mount orientation (Euler, radians) relative to the vehicle body; critical for mapping rays into world coordinates.
  - **Filtering**:
    - `deployed_ignore_radius_deg` — angular radius around previously deployed locations within which new detections are ignored.

- **General guidance**
  - Prefer editing these YAMLs over touching node logic when changing speeds, paths, camera sources, or detectors.
  - Do not hardcode topics, modes, or thresholds inside nodes; use the existing factories and parameters instead.

## Camera Pipelines and Detectors

- **Camera pipelines** (in `bv_core/pipelines/`):
  - Implemented as `VisionPipeline` subclasses (`camera_pipeline.py`, `ros_cam_pipeline.py`, `gz_transport_pipeline.py`) that all expose the same minimal API: `start()`, `stop()`, and `get_frame()`. This lets `vision_node` and `stitching_node` treat them interchangeably.
  - The factory function `create_pipeline(...)` in `bv_core/pipelines/__init__.py` selects the concrete pipeline based on `pipeline_type` and other kwargs from `vision_params.yaml`:
    - `'real'` / `'camera'` → `CameraPipeline` (GStreamer physical camera).
    - `'ros'` → `RosCamPipeline` (subscribes to a ROS `CompressedImage` topic, e.g. rosbag replay).
    - `'sim'` / `'gazebo'` → `GzTransportPipeline` (reads images directly from Gazebo transport).
  - **To add a new pipeline**, follow the pattern:
    - Create a new module under `bv_core/pipelines/` that subclasses `VisionPipeline` and implements the same public methods.
    - Extend `create_pipeline(...)` to map a new `pipeline_type` string to your class.
    - Add any new config fields to `vision_params.yaml` and keep topic/frame conventions consistent.

- **Detectors** (in `bv_core/detectors/`):
  - Implemented as `BaseDetector` subclasses (`ml_detector.py`, `gazebo_bbox_detector.py`) that consume numpy frames and return `supervision.Detections` with consistent fields (boxes, scores, class IDs).
  - The factory function `create_detector(...)` in `bv_core/detectors/__init__.py` selects the concrete detector based on `detector_type` from `vision_params.yaml`:
    - `"ml"` → `MLDetector` (RF-DETR model; uses tiling, batching, and GPU when available).
    - `"gazebo_bbox"` → `GazeboBBoxDetector` (reads ground-truth bounding boxes from Gazebo transport).
  - **To add a new detector**, follow the pattern:
    - Implement a new `BaseDetector` subclass with `start()`, `stop()`, and a `predict()`/call method that returns `supervision.Detections` in the same format.
    - Extend `create_detector(...)` to recognize a new `detector_type` string and construct your class.
    - Add corresponding config keys to `vision_params.yaml` (e.g. thresholds or model paths) instead of hardcoding them in code.

Using these factories keeps node code agnostic to specific camera backends or models. Prefer adding new `pipeline_type` / `detector_type` options and wiring them through the factories over branching on modes inside the nodes.

## Package Structure

- `bv_core/` — Python package with all node implementations
- `bv_core/detectors/` — Detection model wrappers
- `bv_core/pipelines/` — camera pipelines
- `launch/` — ROS 2 launch files
- `config/` — Parameter YAML files
- `container/` — Docker build files (Dockerfile.arm, Dockerfile.x86)

## Key Dependencies

- ROS 2 Humble, MAVROS, PX4
- `bv_msgs` (sibling package — must be built alongside)
- Python: supervision, geographiclib, numpy<2
- Build system: `ament_python` (no CMake — pure Python ROS 2 package)

