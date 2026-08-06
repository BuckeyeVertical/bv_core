# Human-in-the-loop approval — mission wiring (phase 5)

**Date:** 2026-08-05
**Status:** Design approved, not yet implemented
**Scope:** `bv_core` + one line in `bv_gcs`. Phases 1–4 (interfaces, `approval_node`,
frontend) are already complete and verified.

## Context

`bv_core` flies fully autonomously today: `filtering_node` confirms a detection across
three frames, `mission_node` loiters, localizes, flies to the object, and deploys a
payload with no human involvement. A false positive costs a payload — and there are
exactly as many payloads as targets, so each one is a scoring loss.

This phase adds an optional operator gate. When enabled, the drone holds in
`AUTO.LOITER` after localizing and waits for a human to approve or reject, judging from
an annotated crop of the frame that produced the fix.

The gate must never make the aircraft worse off than it is today. Disabled, the flight
path is unchanged. Enabled, every failure mode resolves by continuing the mission.

## Decisions

**The gate sits after LOCALIZE, before DELIVER.** `_handle_localize_request`
(`vision_node.py:475`) is what captures the frame and runs the detector — it *is* the
image-producing step. Gating earlier means the operator has nothing to look at, and
localization costs ~71 ms in practice, so doing it first is free.

**No new FSM state.** The drone stays in `STATE_LOCALIZE` with the gate holding the
pending. `vision_node.py:379` only keeps the camera pipeline alive for
`('scan', 'localize')`, and `filtering_node.py:341` keys off a fixed state list — a new
state string would silently change both nodes' behavior for a cosmetic gain.

**The timeout fails open**, at `Approval_timeout_sec: 180.0`. On expiry the drone
deploys and continues, so the degraded mode is exactly today's autonomous behavior.
Approve and timeout converge on the same `enter_deliver_state()` call — the timeout is
not a separate branch that can rot.

**The timer lives in `mission_node`, never in `approval_node`.** The most likely thing
to fail is the radio link or the GCS process; a timer that dies with the process it is
protecting against is not a safety net.

**Rejection suppresses on location *and* class together, never class-wide.** A rejected
"person at L" blocks only future person detections near L. A *tent* detection at the
same spot still earns its own approval, and a person detected 50 m away is untouched.
The operator rejects a specific claim, not the ground under it — and with two targets
and two payloads, over-suppressing forfeits deliveries.

**`deployed_ignore_radius_deg` is dead config and stays that way.** It is declared in
`filtering_params.yaml:17` and logged by `mission.py:789`, but `filtering_node.py` never
reads it — deployed suppression is class-state-wide today. Wiring it up would *narrow*
that suppression to a radius and risk re-delivering to an already-serviced class, so
this work adds a separate `rejected_ignore_radius_deg` and only corrects the misleading
comment.

**Approach: extract two helpers rather than inline everything.** `mission.py` is already
1,151 lines and is the file where a mistake flies the aircraft somewhere wrong. Keeping
its delta to ~20 lines has safety value beyond tidiness, and both extracted units are
testable without ROS or a simulator.

## Architecture

Everything up to and including localization is unchanged. The gate is a branch at a
single point: `on_vision_localization_complete` (`mission.py:963`).

```
filtering ──/global_obj_dets──> mission ──LocalizeObject──> vision
                                    │                          │
                                    │      (+ annotated_crop)  │
                                    │<─────────────────────────┘
                                    │
                    Approval_required?
                    ├── false ──> enter_deliver_state()     [today's path, untouched]
                    └── true  ──> gate.request(...)
                                    │
                                    ├──/pending_obj_dets──> approval_node ──WS──> operator
                                    │                            │
                                    │<──srv /detection_decision──┘
                                    │
                    ┌───────────────┼────────────────┐
                 approve         reject          timeout (180 s)
                    │               │                │
          enter_deliver_state()     │        enter_deliver_state()
                                    │        + APPROVAL_TIMEOUT log
                    ┌───────────────┘
                    ▼
          publish /rejected_object_locations ──> filtering (spatial suppression)
          clear target, enter_scan_state()
```

Reject reuses the abandon logic that already exists at `mission.py:925-935` — clear
coords, clear `confirmed_detection_class_id`, `enter_scan_state()` — with the rejected
location published first.

`/pending_obj_dets` is published `TRANSIENT_LOCAL` depth 1 so a restarted
`approval_node`, or a Herelink link that drops and recovers mid-decision, immediately
gets the live pending instead of showing an empty dashboard while the drone counts down.

### Concurrency

`mission.py` runs under `rclpy.spin()` (single-threaded, mutually-exclusive callbacks),
and every service call is `call_async` + `add_done_callback` with no blocking waits.
`on_vision_localization_complete` already calls `enter_deliver_state()` from a callback
context in flight today, so the gate's service handler may do the same. No deferred-flag
indirection is needed.

## Components

### New: `bv_core/approval_gate.py` (~110 lines)

Modelled on `mission_logger.py` — a focused helper the node composes in.

```python
class ApprovalGate:
    def __init__(self, node, timeout_sec, log)
    def request(self, *, class_id, lat, lon, alt, confidence,
                drone_lat, drone_lon, annotated_crop,
                on_approve, on_reject) -> str   # returns detection_id
    def is_pending(self) -> bool
    def cancel(self, reason)                     # abandon without deciding
```

Owns the `/pending_obj_dets` publisher, the `/detection_decision` service, and a
one-shot timer. `request()` mints a `uuid4`, publishes, and arms the timer when
`timeout_sec > 0`. Every clear publishes an empty `PendingDetection` so the dashboard
resets.

| Condition | `accepted` | Effect |
| --- | --- | --- |
| nothing pending | `false` | message says so; no callback |
| `detection_id` mismatch | `false` | message names the active ID |
| approved | `true` | clear, cancel timer, `on_approve()` |
| rejected | `true` | clear, cancel timer, `on_reject()` |
| timer expires | — | log `APPROVAL_TIMEOUT`, then `on_approve()` |

Takes the node as a constructor argument so a stub exposing `create_publisher`,
`create_service`, and `create_timer` can drive the whole table in a unit test.

### New: `bv_core/detection_crop.py` (~70 lines, pure numpy/cv2, no ROS)

```python
build_annotated_crop(frame, detections, index, cfg) -> bytes   # JPEG
```

In this order:

1. Take `detections.xyxy[index]`, expand about its centre by `crop_margin_factor`
2. Widen to at least `crop_min_px` so a small sim detection still gets context
3. Clamp to frame bounds, **shifting** the window rather than shrinking it, so an
   edge detection keeps its full context
4. Crop
5. Downscale so the longest side is at most `crop_max_px`
6. **Then** draw the box and `"person 0.94"` label, in output coordinates
7. `cv2.imencode('.jpg', ..., crop_jpeg_quality)`

Drawing after the downscale is deliberate: annotating first means the 4640→1024 resize
thins the lines into near-invisibility. This way line weight is specified in the pixels
the operator actually sees.

Sizing rationale — at 15 m AGL the real camera (`fx=3582.9`, 4640 px) has a ~4.2 mm/px
ground sample distance, making a 1.8 m person ~430 px tall; the sim (`fx=410.9`,
1280 px) is ~36.5 mm/px, making the same person ~49 px. A single fixed crop size cannot
serve both, which is why the rule is adaptive.

### Deltas to existing files

**`vision_node.py`** (~10 lines changed, plus the crop call). `_handle_localize_request`
drops the detection index at line 579:

```python
matched = [c for c in coords if int(c[2]) == target_cls]   # index lost
```

`localizer.get_lat_lon` (`localizer.py:52-68`) appends one result per input in strict
order, so the index is recoverable. Carry `(i, coord)` pairs through the class filter,
then `detections.xyxy[best_index]` is the correct box for the crop.

**`mission.py`** (~20 lines). Load the two config keys, construct the gate when enabled,
branch in `on_vision_localization_complete`, add the `/rejected_object_locations`
publisher (reusing the existing `ObjectLocations` message — no new interface), and
`gate.cancel()` in `enter_rtl_state`.

**`filtering_node.py`** (~15 lines). Load `rejected_ignore_radius_deg`, subscribe
`/rejected_object_locations`, add `_is_rejected(lat, lon, cls)`, and add one clause to
the existing `filtered_detections` comprehension at `filtering_node.py:194-198`. The
reset at `filtering_node.py:345-349` is deliberately **not** touched — resetting
non-deployed targets to `undetected` on scan re-entry is exactly the desired behavior
now that suppression is spatial.

**`mission.launch.py`**. Read `mission_params.yaml` at generation time using the same
`get_package_share_directory('bv_core')` path `mission.py:115` uses, and conditionally
`IncludeLaunchDescription` on `bv_gcs`'s `gcs.launch.py`. Reuses the existing launch
file rather than duplicating the node spec. `bv_gcs` is only referenced when the flag is
set, so an unbuilt `bv_gcs` cannot break an autonomous launch.

**`bv_gcs/approval_node.py`** (one line). Subscription durability to `TRANSIENT_LOCAL`
to match the new publisher.

## Configuration

| File | Keys |
| --- | --- |
| `mission_params.yaml` | `Approval_required: false`, `Approval_timeout_sec: 180.0` |
| `filtering_params.yaml` | `rejected_ignore_radius_deg: 0.0001` (~11 m); corrected comment on the dead `deployed_ignore_radius_deg` |
| `vision_params.yaml` | `crop_margin_factor: 2.5`, `crop_min_px: 320`, `crop_max_px: 1024`, `crop_jpeg_quality: 85` |

`0.0001 deg` ≈ 11.1 m, which comfortably covers the ~4 m localization error observed in
sim while leaving a genuine second target 20 m away still findable.

## Error handling

The governing rule: nothing in the gate may strand the aircraft, and nothing may make
the autonomous path worse.

| Failure | Behavior |
| --- | --- |
| Localization fails | No pending published. Existing retry×5 → abandon path, untouched. |
| Crop construction fails | Log a warning, send the pending with an empty `annotated_crop`, still gate. |
| `approval_node` not running | Pending goes nowhere; timeout fires and deploys. Warn at `request()` if the publisher has zero subscribers. |
| Browser disconnects mid-decision | Timeout fires. The GCS also clears its own pending on socket close, so a stale card cannot be clicked. |
| Stale or duplicate verdict | `accepted=false`, with the active ID in the message. |
| Verdict arrives after timeout fired | Nothing pending → `accepted=false`. |
| RTL while a pending is open | `gate.cancel()` publishes the empty pending; dashboard clears. |
| Timer fires after the FSM moved on | Guarded twice: `is_pending()` in the gate, and `current_state == STATE_LOCALIZE` in mission's `on_approve`. |

The crop-failure row is a deliberate choice. Failing the localization instead would mean
a camera-encoding bug silently converts into the drone abandoning real targets — much
worse than showing a card with no picture. The GCS already renders an explicit "no image
received — judge with caution, or reject and let the drone keep scanning" state.

## Verification

### Automated, no simulator

- **`detection_crop` unit tests:** bbox flush against each frame edge (window shifts,
  stays full size, stays in bounds), bbox larger than `crop_max_px`, bbox smaller than
  `crop_min_px`, degenerate zero-area bbox, output decodes as valid JPEG within the cap.
- **`ApprovalGate` unit tests:** the full decision table against a stub node, plus
  timeout invoking `on_approve` and `TRANSIENT_LOCAL` redelivering to a late subscriber.
- **`filtering_node` suppression, live ROS, no PX4.** `filtering_node` needs only
  `/mavros/global_position/global`, `/mavros/local_position/pose`,
  `/mavros/global_position/rel_alt`, `/obj_dets`, and `/mission_state` — all plain
  topics a test script can publish. Drive a synthetic detection stream, publish a
  `/rejected_object_locations`, and assert the same detection stops reaching 3-frame
  confirmation. This is the loop the whole design exists to prevent.
- **Full GCS path** via the existing `fake_pending`, extended to carry a real crop from
  `detection_crop`.

### Requires the simulator (operator-run)

1. **Regression** — `Approval_required: false`; the autonomous mission flies identically
   to the baseline log.
2. **Approve** — loiter → approve → `DELIVER` → `DEPLOY`.
3. **Timeout in flight** — `Approval_timeout_sec: 15`, no response, deploys with
   `APPROVAL_TIMEOUT` logged.

To keep that session short, ship `bv_core/test/sim_approval_check.py`: a node run
alongside the mission that watches `/mission_state`, `/pending_obj_dets`,
`/rejected_object_locations`, and `/mavros/mission/reached`, then prints a per-scenario
verdict:

```
[sim-check] scenario=approve
  ✓ pending published while state=localize
  ✓ pending carried a non-empty crop (84 KB)
  ✓ decision accepted, state -> deliver within 2.0s
  ✓ deploy completed, servo CH1 cycled
  RESULT: PASS
```

Of these three, run 1 carries the most weight — it proves the autonomous path is intact.
The reject scenario, which proves the feature does what it exists to do, is covered
automatically by the `filtering_node` suppression test above and does not need sim time.

## Out of scope

- Wiring `deployed_ignore_radius_deg` (see Decisions).
- Any change to 3-frame confirmation, localization geometry, or stitching.
- Operator actions beyond approve and reject — no re-localize button, no manual
  coordinate entry, no class re-assignment.

## Open items for the operator

1. The exact sim launch invocation, so `sim_approval_check.py` targets the right setup.
2. Confirmation that the uncommitted changes in `mission_params.yaml`,
   `vision_params.yaml`, and `filtering_params.yaml` are the values flown in sim — tests
   should not be written against stale waypoints.
