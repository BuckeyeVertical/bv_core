# Debug video stream in the GCS

**Date:** 2026-08-06
**Status:** Design approved, not yet implemented
**Scope:** `bv_core` (capture-side encode) + `bv_gcs` (relay and UI). No `bv_msgs` changes.

## Context

There is no way to see what the vision system sees while the aircraft is flying.
Debugging a missed or spurious detection means landing, pulling `raw_frames/`, and
inferring what happened. The approval gate added an annotated crop, but only for
detections that already passed 3-frame confirmation — it says nothing about the
frames where nothing fired.

This adds a low-bitrate live preview to the existing `bv_gcs` web UI, with detection
markers overlaid, for debugging and testing. It is explicitly **not** flight
infrastructure: it touches no FSM code, adds no failure path for the aircraft, and
its worst failure is a blank video panel.

### Constraints that shaped the design

- **The ground laptop is on Herelink WiFi**, so the stream shares a constrained RF
  link with telemetry. Bitrate is the binding constraint, which rules out MJPEG.
- **The companion computer is a Jetson** with NVENC, so H.264 encoding is nearly
  free. (`container/Dockerfile.arm` says Raspberry Pi; it is stale.)
- **Detection runs at 0.67 Hz** (`capture_interval: 1.5e9`), so annotated frames
  cannot be the video — they would be a slideshow. Video and detections must be
  decoupled.
- **`cv2.VideoCapture` owns the capture GStreamer pipeline** (`camera_pipeline.py:35`)
  and exposes no handle to its elements, so a `tee` branch cannot be gated at
  runtime. The preview is fed from the numpy frames Python already holds.
- **OpenCV cannot be relied on for the encode.** The dev PC's `opencv-python` wheel
  reports `GStreamer: NO`, so `cv2.VideoWriter(..., CAP_GSTREAMER)` does not exist
  there. It works on the Jetson only because that OpenCV is built with GStreamer —
  which is also why `CameraPipeline` runs only under `pipeline_type: 'real'` while sim
  uses `GzTransportPipeline`. Depending on an OpenCV build flag that varies per machine
  is the fragile path.

## Decisions

**The hook lives in `VisionPipeline._enqueue_frame`, not in `CameraPipeline`.** All
three pipelines funnel through it — `camera_pipeline.py:85`,
`gz_transport_pipeline.py:72`, `ros_cam_pipeline.py:70` — so sim, real camera and
rosbag replay all get the stream with zero per-pipeline code. The feature can be
developed and tested entirely in Gazebo.

**Encoding goes through `gi.repository.Gst` directly, not `cv2`.** PyGObject and
GStreamer 1.20 are already installed and verified working on both the dev PC and the
Jetson, so the same code runs everywhere regardless of how OpenCV was built. It also
gives an `appsink`, so encoded bytes come straight back into Python — which removes
the localhost TCP socket an earlier draft needed purely to work around
`cv2.VideoWriter` being write-only.

**The encoder element is chosen by probing, in a three-tier ladder.** Measured on the
target machines:

| Tier | Element | Where |
| --- | --- | --- |
| 1 | `nvv4l2h264enc` + `nvvidconv` | Jetson (Tegra NVENC, NVMM memory) |
| 2 | `nvh264enc` | desktop NVIDIA GPUs |
| 3 | `x264enc` | anywhere, software |

Tier 1 and 2 differ by more than the element name: `nvvidconv` produces
`video/x-raw(memory:NVMM)`, which only the Jetson encoder consumes, so each tier
carries its own caps.

**Selection is by trying, not by asking.** Registry presence is not sufficient —
`nvh264enc` is present on the dev PC and still fails to initialise. `start()` walks the
tiers in order and accepts the first that actually reaches `PLAYING`. Details under
Components.

Software encoding is not a compromise on the dev machine: `x264enc` was measured at
roughly 125 fps for 1024×576 on the RTX 3070 Ti PC — over 30× the 4 fps default, and
the default output is smaller still at 640 wide. (`nvh264enc` is present there but
fails to initialise under WSL2 with "Could not configure supporting library"; tier 3
covers it and nothing is lost.)

**Transport is H.264 → fragmented MP4 → a dedicated WebSocket → MSE.** It reuses
`approval_node`'s aiohttp server and port, so there is no new port and no ICE, but
video gets its **own endpoint (`/video`) separate from control (`/ws`)**. Sharing one
socket would let a backlog of H.264 delay a verdict — the aircraft would still be safe,
since the timeout lives in `mission_node`, but the operator would be clicking Approve
into a stalled pipe exactly when it matters. Latency lands around 300–600 ms, which is
acceptable for debugging. The encode half is identical to a WebRTC design, so if
latency ever matters the browser transport can be swapped without touching the
pipeline.

**The operator toggles it explicitly in the GCS**, defaulting off at startup. Because
the vision pipeline only runs during `scan` and `localize`, a forgotten "on" is
bounded to those phases.

**Preview framerate is independent of capture framerate.** The real camera may run at
24 fps; the preview decimates to `preview_fps`. This is the single knob for trading
image smoothness against both CPU and link bandwidth, tunable in the field without
code changes.

**Detections are drawn browser-side on a canvas overlay**, not baked into the video.
No extra encode cost, crisp at display resolution, toggleable, and decoupled from the
0.67 Hz detector — markers persist between updates rather than making video stutter.

**`/image_compressed` is left alone.** `record_video: True` currently JPEG-encodes
every full 4640×3480 frame. That is a real cost, but `RosCamPipeline` replays from it,
so downscaling would change what the detector sees on bag replay and silently
invalidate comparisons against past runs. Out of scope; noted below.

## Architecture

```
vision_node (bv_core)                         approval_node (bv_gcs)         browser
─────────────────────                         ──────────────────────         ───────
capture loop
  │
  ├─→ detection queue          (unchanged)
  ├─→ /image_compressed        (unchanged)
  │
  └─→ VisionPipeline._enqueue_frame
        └─→ PreviewStream.offer(frame)
              │  (returns immediately; drops if busy)
              ▼
         encoder thread
          decimate → resize → appsrc
          → [probed encoder] → h264parse
          → mp4mux(fragmented) → appsink
              │
              └─→ /preview_stream ──────────→ subscribe (drop if busy)
                  (std_msgs/UInt8MultiArray)     └──binary on /video──→ MSE <video>
                                                                            ▲
/obj_dets ─────────────────────────────────→ normalise ──JSON on /ws──→ canvas overlay
/preview_enabled ←──────────────────────────  toggle
```

The process boundary between `bv_core` and `bv_gcs` is a plain ROS topic carrying
fMP4 chunks, which matches how every other cross-node path in this stack works and
needs no socket lifecycle management. `std_msgs/UInt8MultiArray` avoids a new
`bv_msgs` interface. QoS is BEST_EFFORT depth 2: a dropped chunk should be skipped,
never retried, since late video is worse than missing video.

## Components

### New: `bv_core/preview_stream.py` (~120 lines)

```python
@dataclass
class PreviewConfig:
    width: int = 640
    fps: float = 4.0
    bitrate_bps: int = 400_000

class PreviewStream:
    def __init__(self, cfg: PreviewConfig, on_chunk, log=None)
    def start(self) -> None      # probe encoder, build the Gst pipeline, spawn thread
    def stop(self) -> None       # release; idempotent
    def is_running(self) -> bool
    def offer(self, frame) -> None   # non-blocking, latest-wins, drops when busy
    def stats(self) -> dict      # frames offered / encoded / dropped
```

`offer()` is called from the capture thread and must never block. It timestamps and
writes into a single slot under a lock, then returns. The encode thread wakes, takes
whatever is in the slot, and encodes it; anything that arrived while it was busy is
simply overwritten and never seen. Under load the preview loses framerate and capture
is untouched.

**Decimate before resize.** The dominant per-frame cost is scaling a 48 MB frame, not
the encode. Strided decimation (`frame[::n, ::n]`, effectively a strided copy) reduces
to roughly the target size first, then a small `cv2.resize` finishes the job — an
order of magnitude cheaper than resizing the full frame directly. `n` is derived from
`frame.shape[1] // cfg.width`, clamped to at least 1.

**Rate limiting** happens in `offer()`: if less than `1/fps` has elapsed since the last
accepted frame, return immediately. This is what makes a 24 fps camera cost the same
as an 8 fps one.

**The Jetson pipeline is the one that matters.** It is the deployment target and the
only machine where encode cost competes with flight work; the other tiers exist so the
feature can be developed against sim and must not be optimised for.

Tier 1 — **Jetson**, the shipping path. Scaling and colour conversion happen in NVMM
memory on the hardware converter, so the CPU never touches a full-size frame after the
decimation step:
```
appsrc ! video/x-raw,format=BGR ! videoconvert ! nvvidconv
  ! video/x-raw(memory:NVMM),width=<w>,height=<h>
  ! nvv4l2h264enc bitrate=<bitrate> insert-sps-pps=true iframeinterval=<2*fps>
    maxperf-enable=true
  ! h264parse ! mp4mux fragment-duration=200 streamable=true
  ! appsink emit-signals=true sync=false max-buffers=8 drop=true
```

Tier 2 — desktop NVIDIA. `nvh264enc` takes ordinary system memory, so the NVMM caps
must not be applied:
```
appsrc ! video/x-raw,format=BGR ! videoconvert ! videoscale
  ! video/x-raw,width=<w>,height=<h>
  ! nvh264enc bitrate=<kbps> preset=low-latency-hq ! h264parse ! mp4mux … ! appsink
```

Tier 3 — software, any machine: `videoconvert ! videoscale ! x264enc tune=zerolatency
speed-preset=ultrafast bitrate=<kbps>`, same tail.

**Selection is by trying, not by asking.** Registry presence is not sufficient:
`nvh264enc` is present in the registry on the dev PC and still fails with "Could not
configure supporting library". So `start()` walks the tiers in order, and for each one
builds the pipeline and waits for it to actually reach `PLAYING` (with a short timeout,
since `set_state` returns `ASYNC`). Only a tier that reaches `PLAYING` is used; failures
tear down and fall through to the next.

The chosen tier is logged. **On the Jetson, seeing tier 2 or 3 in the log means the
hardware path silently regressed** — most likely the flight image missing the Tegra
multimedia plugins — and that is the failure most worth noticing, because otherwise it
shows up only as an unexplained hot CPU.

### Edits: `bv_core/pipelines/Vision_Pipeline.py` (~8 lines)

```python
self._preview = None                      # in __init__

def set_preview(self, preview):           # None disables
    self._preview = preview

def _enqueue_frame(self, frame):
    if self._preview is not None:         # the whole cost when disabled
        self._preview.offer(frame)
    ...existing body unchanged...
```

When disabled this is one attribute comparison per frame — no thread, no pipeline, no
allocation. Behaviour is identical to today.

### Edits: `bv_core/vision_node.py` (~25 lines)

Construct `PreviewStream` from config with an `on_chunk` callback that publishes to
`/preview_stream`, subscribe `/preview_enabled` (`std_msgs/Bool`), and start/stop it on
toggle, attaching it to the pipeline via `set_preview`. Starting is a no-op unless the
pipeline is running, so the stream naturally follows the existing scan/localize
lifecycle without touching it.

### Edits: `bv_gcs/bv_gcs/approval_node.py` (~70 lines)

- Publish `/preview_enabled` (`std_msgs/Bool`, latched) from a WS message
  `{"type": "preview", "enabled": bool}`.
- Serve video on a **separate WebSocket endpoint, `/video`**, distinct from the control
  socket at `/ws`. Subscribe `/preview_stream` (`std_msgs/UInt8MultiArray`, BEST_EFFORT
  depth 2) and broadcast each chunk as a binary frame to `/video` clients only.
- **Never `await` a video send behind a full buffer.** If a client's transport is
  already busy, drop the chunk. Video is expendable; the next keyframe recovers it.
  Chunks arriving with no `/video` client are dropped outright, so nothing crosses the
  radio link when nobody is watching.
- Subscribe `/obj_dets`, **normalise centres to 0–1** against the source frame
  dimensions, and broadcast
  `{"type":"detections","dets":[{"x":0.42,"y":0.31,"class_id":0,"class_name":"person"}],"stamp":…}`.
  Normalising in the node means the browser never needs to know the sensor resolution.

Existing JSON message handling is unaffected; binary frames are distinguished by
`WSMsgType.BINARY` on the client.

### Edits: `bv_gcs/web/` (~90 lines)

- `src/net/videoClient.ts` — a second `WebSocket` to `/video`, opened only while the
  toggle is on and closed when it is off, kept entirely separate from the control
  client in `net/client.ts` so neither can stall the other.
- `src/components/VideoPanel.tsx` — a `<video>` fed by `MediaSource` +
  `SourceBuffer.appendBuffer` on each binary frame, with a `<canvas>` overlaid at the
  same size. Draws a marker and class label per detection, holding the last set until
  the next arrives.
- A toggle control that sends the `preview` message, plus a stream-status line
  (`off` / `connecting` / `live`).
- Store additions for `previewEnabled`, `detections`, `streamState`.

### Layout

Three columns. The live feed and the approval crop are **side by side and both
visible** — the gate never replaces the video, because the operator wants to see what
the aircraft is looking at *now* while judging a frame captured a moment ago.

```
┌──────────┬───────────────────────────┬──────────────────────┐
│ MISSION  │  LIVE FEED                │  PENDING DETECTION   │
│          │  ┌─────────────────────┐  │  ┌────────────────┐  │
│ state    │  │                     │  │  │ annotated crop │  │
│ drone    │  │   <video> + canvas  │  │  │  (native res)  │  │
│ lat/lon  │  │   detection markers │  │  └────────────────┘  │
│          │  │                     │  │  PERSON      94%     │
│ ── ── ── │  └─────────────────────┘  │  lat/lon/alt/dist    │
│ STREAM   │  live · 640×480 · 4fps    │  auto-deploys 2:41   │
│ [ on/off]│                           │  [APPROVE] [REJECT]  │
└──────────┴───────────────────────────┴──────────────────────┘
   260px            flex: 1                    360px
```

**With no pending detection** the right column collapses and the live feed takes the
full width — so an operator who is only watching the stream gets the biggest possible
picture, and the approval card arriving is itself a visible signal.

**With the stream toggled off** the centre column shows the toggle and a short "stream
off" message. The approval workflow is completely unchanged from today for an operator
who never turns video on: same crop, same fields, same countdown, same shortcuts.

The `[A]`/`[R]` keyboard shortcuts stay bound to the approval card regardless of what
the video is doing, and the video element is never focusable, so it cannot swallow
them.

### Configuration — `config/vision_params.yaml`

| Key | Default | Purpose |
| --- | --- | --- |
| `preview_width` | `640` | Output width; height follows source aspect |
| `preview_fps` | `4.0` | Decimation target; the knob for CPU and bitrate |
| `preview_bitrate_bps` | `400000` | Encoder target bitrate |

**The defaults are deliberately conservative because the Herelink budget is unknown.**
The operator does not care about picture quality — this is a debug feed, not a camera
downlink — so the defaults are set well under any plausible ceiling and raised only if
measurement shows headroom. Starting low and raising beats discovering contention in
flight.

At 640 wide, a 1.8 m person that occupies ~430 px in the 4640 px sensor still occupies
~59 px in the preview: unmistakable. 4 fps is stepped but shows where the aircraft is.
Roughly 400 kbps fits even if Herelink's native video downlink is running at the same
time.

**Aspect ratio differs between sources.** The real sensor is 4640×3480 — **4:3**, not
16:9 — so `preview_width: 640` gives 640×480. The Gazebo camera is 1280×720 (16:9) and
gives 640×360. Nothing derives height from width; both are handled from the actual
frame shape.

Both `preview_width` and `preview_fps` are field-tunable without code changes. Raise
them once the link has been measured.

## Isolation from the mission and the approval gate

This is the property the feature is judged on, so it is stated explicitly rather than
left implied.

**`mission.py` is not modified.** No FSM change, no new state, no new callback, no new
failure path. Neither are `approval_gate.py` or `filtering_node.py`. The only edit to
flight-path code is the two-line hook in `Vision_Pipeline._enqueue_frame`.

**Disabled cost is one branch per frame.** `self._preview is None` short-circuits before
anything else happens — no thread, no pipeline, no allocation. To be exact rather than
approximate: this is not byte-identical to today. The comparison executes (tens per
second), `vision_node` holds one idle subscription, and a dormant `PreviewStream` object
exists. None of it is measurable, but the claim is "indistinguishable", not "literally
unchanged".

**The mission cannot be affected by link congestion.** The approval timeout lives in
`mission_node`, not in `approval_node` or the browser, so a saturated link, a wedged
ground station, or a killed `approval_node` all resolve the same way the design already
guarantees: the timeout fires and the aircraft deploys and continues. This was verified
in flight in sim run 3.

**The approval gate's UI is protected by socket separation.** Video and control share a
radio link but not a WebSocket: control is `/ws`, video is `/video`. A backlog of H.264
cannot delay an outbound `decision_ack` or an inbound Approve click, and video sends
drop rather than queue. Without this split, an operator could click Approve and wait
seconds behind megabytes of video — the aircraft would still be safe, but the
interaction would be bad exactly when it matters most.

## Error handling

The governing rule: **the preview may never degrade capture, detection, or the
mission.** Every failure resolves by the video stopping.

| Failure | Behaviour |
| --- | --- |
| Encoder slower than capture | Frames dropped in `offer()`. Preview loses fps; capture unaffected. |
| Gst pipeline fails to reach PLAYING | Log an error naming the tier that was tried, `is_running()` stays false, toggle reports failure to the UI. Vision continues. On the Jetson this is the signal that the hardware encoder is unavailable and the flight image needs checking. |
| `offer()` raises | Caught inside `offer()` and logged once per N failures; never propagates into `_enqueue_frame`. |
| No chunks arriving on `/preview_stream` | UI shows `connecting` while the toggle is on. Distinguishes "encoder failed" from "not in scan yet" by whether `is_running()` was reported true. |
| Last browser disconnects | The toggle is authoritative and stays on — it is an operator control, not a viewer-presence signal. `approval_node` drops incoming chunks while there are no WS clients, so nothing crosses the radio link; the encoder keeps running on the Jetson until the operator toggles off. Reconnecting resumes on the next keyframe. |
| Pipeline stops (leaving scan/localize) | No frames arrive, so no chunks are produced. The encoder stays alive and resumes on the next scan. |
| MSE buffer grows unbounded | Client evicts appended ranges behind `currentTime` and seeks to live if it falls more than 2 s behind. |

## Testing

**Unit, no ROS, no GStreamer** — `PreviewStream`'s frame-management logic is the part
worth testing and is separable from the encoder:
- `offer()` returns immediately and never raises when the encoder is absent
- rate limiting: at `fps=8`, offering 24 frames in one second accepts ~8
- latest-wins: offering three frames while the encoder is busy encodes only the last
- decimation factor derivation for 4640→1024 (real camera), 1280→1024 (Gazebo), and a
  source already narrower than the target
- `stop()` is idempotent; `offer()` after `stop()` is a no-op

**Integration, in sim** — the whole point of hooking the base class is that Gazebo
exercises the real path. Toggle on during a scan, confirm video appears, confirm
detection markers land on the objects, toggle off, confirm the encoder stops.

**Load check on the real camera** — with `record_video: True` and the real 4640×3480
source, confirm capture fps and detection cadence are unchanged with the preview on
versus off. This is the measurement that matters; the design's whole claim is that it
costs nothing that the mission notices.

## Out of scope

- Changing `/image_compressed` or `record_video` (see Decisions).
- Bounding boxes in the overlay. `/obj_dets` carries only centres —
  `vision_node._publish_detections` computes `(x1+x2)/2, (y1+y2)/2` and discards the
  box. Boxes would require extending `ObjectDetections`, which `filtering_node` also
  consumes. Markers are adequate for debugging; revisit if they prove not to be.
- WebRTC. The encode path is deliberately transport-agnostic so this stays cheap to
  add later.
- Recording the preview stream to disk.
- Any change to `mission.py`, the FSM, or the approval gate.

## Open items

1. Confirm `nvv4l2h264enc` and `nvvidconv` are present in the flight container — the
   ARM Dockerfile is stale and says Raspberry Pi, so the actual Jetson image needs
   checking before the fallback path is assumed to be dev-only.
2. Measure real-camera capture fps with the preview on, to validate the decimation
   approach at 24 fps source.
