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

## Decisions

**The hook lives in `VisionPipeline._enqueue_frame`, not in `CameraPipeline`.** All
three pipelines funnel through it — `camera_pipeline.py:85`,
`gz_transport_pipeline.py:72`, `ros_cam_pipeline.py:70` — so sim, real camera and
rosbag replay all get the stream with zero per-pipeline code. The feature can be
developed and tested entirely in Gazebo.

**Transport is H.264 → fragmented MP4 → the existing WebSocket → MSE.** It reuses
`approval_node`'s WebSocket, so there is no new port, no ICE, and no new Python
dependency. Latency lands around 300–600 ms, which is acceptable for debugging. The
encode half is identical to a WebRTC design, so if latency ever matters the browser
transport can be swapped without touching the pipeline.

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
          → nvv4l2h264enc → h264parse
          → mp4mux(fragmented)
          → tcpserversink 127.0.0.1:5001
                                    │
                                    └──TCP──→ relay task
                                                └──binary WS frames──→ MSE <video>
                                                                            ▲
/obj_dets ─────────────────────────────────→ normalise ──JSON WS──→ canvas overlay
/preview_enabled ←──────────────────────────  toggle
```

The localhost TCP socket is the process boundary between `bv_core` and `bv_gcs`. It
avoids a `gi.repository.Gst` dependency: `cv2.VideoWriter` with `CAP_GSTREAMER` can
drive an `appsrc → … → tcpserversink` pipeline, and `cv2` is already a dependency.

## Components

### New: `bv_core/preview_stream.py` (~120 lines)

```python
@dataclass
class PreviewConfig:
    width: int = 1024
    fps: float = 8.0
    bitrate_bps: int = 1_200_000
    port: int = 5001

class PreviewStream:
    def __init__(self, cfg: PreviewConfig, log=None)
    def start(self) -> None      # build the VideoWriter, spawn the encode thread
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

GStreamer pipeline (Jetson):
```
appsrc ! video/x-raw,format=BGR ! videoconvert ! nvvidconv
  ! nvv4l2h264enc bitrate=<bitrate> insert-sps-pps=true iframeinterval=<2*fps>
  ! h264parse ! mp4mux fragment-duration=200 streamable=true
  ! tcpserversink host=127.0.0.1 port=<port> sync=false recover-policy=keyframe
```
On a machine without NVENC (dev laptops, sim) `nvvidconv`/`nvv4l2h264enc` are absent;
the module falls back to `videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast`.
Selection is by probing element availability once at `start()`, not by config.

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

Construct `PreviewStream` from config, subscribe `/preview_enabled` (`std_msgs/Bool`),
and start/stop it on toggle, attaching it to the pipeline via `set_preview`. Starting
is a no-op unless the pipeline is running, so the stream naturally follows the existing
scan/localize lifecycle without touching it.

### Edits: `bv_gcs/bv_gcs/approval_node.py` (~70 lines)

- Publish `/preview_enabled` (`std_msgs/Bool`, latched) from a WS message
  `{"type": "preview", "enabled": bool}`.
- When enabled, an asyncio task connects to `127.0.0.1:5001`, reads fMP4 chunks, and
  broadcasts them as **binary** WebSocket frames. On disconnect it retries with backoff
  while the toggle is on, and stops when it is off.
- Subscribe `/obj_dets`, **normalise centres to 0–1** against the source frame
  dimensions, and broadcast
  `{"type":"detections","dets":[{"x":0.42,"y":0.31,"class_id":0,"class_name":"person"}],"stamp":…}`.
  Normalising in the node means the browser never needs to know the sensor resolution.

Existing JSON message handling is unaffected; binary frames are distinguished by
`WSMsgType.BINARY` on the client.

### Edits: `bv_gcs/web/` (~90 lines)

- `src/components/VideoPanel.tsx` — a `<video>` fed by `MediaSource` +
  `SourceBuffer.appendBuffer` on each binary frame, with a `<canvas>` overlaid at the
  same size. Draws a marker and class label per detection, holding the last set until
  the next arrives.
- A toggle control that sends the `preview` message, plus a stream-status line
  (`off` / `connecting` / `live`).
- Store additions for `previewEnabled`, `detections`, `streamState`.

The panel is collapsed by default so the approval workflow is unchanged for an
operator who does not want video.

### Configuration — `config/vision_params.yaml`

| Key | Default | Purpose |
| --- | --- | --- |
| `preview_width` | `1024` | Output width; height follows source aspect |
| `preview_fps` | `8.0` | Decimation target; the knob for CPU and bitrate |
| `preview_bitrate_bps` | `1200000` | NVENC target bitrate |
| `preview_port` | `5001` | localhost TCP port between the nodes |

**Aspect ratio differs between sources and this affects the bitrate.** The real
sensor is 4640×3480 — **4:3**, not 16:9 — so `preview_width: 1024` gives 1024×768.
The Gazebo camera is 1280×720 (16:9) and gives 1024×576, about 25% fewer pixels for
the same width. The default is 1024 rather than 1280 so the real camera's 4:3 output
lands near 0.8 MP and comfortably inside the 1.2 Mbps budget; at 1280 the real camera
would produce 1280×960 (1.2 MP) and want roughly 1.6 Mbps. Raise it if the link
proves to have headroom — this is the second field-tunable knob after `preview_fps`.

## Error handling

The governing rule: **the preview may never degrade capture, detection, or the
mission.** Every failure resolves by the video stopping.

| Failure | Behaviour |
| --- | --- |
| Encoder slower than capture | Frames dropped in `offer()`. Preview loses fps; capture unaffected. |
| `VideoWriter` fails to open (no encoder, port in use) | Log an error, `is_running()` stays false, toggle reports failure to the UI. Vision continues. |
| `offer()` raises | Caught inside `offer()` and logged once per N failures; never propagates into `_enqueue_frame`. |
| `approval_node` cannot connect to 5001 | Retry with backoff while the toggle is on; UI shows `connecting`. |
| Last browser disconnects | The toggle is authoritative and stays on — it is an operator control, not a viewer-presence signal. The relay stops reading the socket while there are no WS clients, so nothing crosses the radio link; the encoder keeps running on the Jetson until the operator toggles off. Reconnecting a browser resumes mid-stream on the next keyframe. |
| Pipeline stops (leaving scan/localize) | No frames arrive, so no chunks. The socket stays open and resumes on the next scan. |
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
