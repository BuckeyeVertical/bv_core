# 3-of-5 confirmation + GCS status panel

## Why

`filtering_node` confirmed a detection only when a class appeared in **all 3** of the
last 3 frames. Frames with no detection are appended too, so a single miss resets the
run. Measured against 16 detection opportunities per crossing:

| rule | p=0.3 | p=0.5 | p=0.7 |
| --- | --- | --- | --- |
| 3-of-3 (old) | 25.2% | 70.2% | 96.4% |
| 3-of-5 (new) | 58.6% | 94.8% | 99.9% |

The strictness was not buying much: it only rejects *flickering* false positives, which
the spatial proximity gate already rejects. A *persistent* false positive (a rock, a
shadow) recurs in the same spot and satisfies 3-of-3 as easily as 3-of-5.

## Tasks

- [x] Extract the confirmation predicate to a pure, testable module-level function
- [x] Write tests first, including the old behaviour as a regression guard
- [x] Make hits/window configurable (`confirmation_hits`, `confirmation_window`)
- [x] Verify config keys are actually read — no more dead knobs
- [x] Publish the live window on `/confirmation_window` for the GCS
- [x] Relay it through `approval_node`
- [x] `ConfirmationPanel` in the GCS left column
- [x] Full test suite green

## Review (verified 2026-08-07)

**Behaviour change, same input, proven both ways.** Window `[hit][miss][hit][hit]`:
old 3-of-3 rule → not confirmed; new 3-of-5 → confirmed. Spatially scattered hits
(`0.01 deg` apart) still rejected, so the proximity gate is intact.

**Tests.** 13 new in `test/test_confirmation_window.py` covering: consecutive hits,
two hits refused, the gap case, hits spread across five frames, spatial rejection,
empty window, wrong-class isolation, two classes tracked independently, configurable
hit count, and that empty frames contribute no position. Suite: **217 passed**
(was 204), plus 18 in `bv_gcs`.

**Chain verified with real ROS messages**, not just unit tests: latched delivery to a
late-joining GCS, relay through `approval_node`, duplicate suppression, and malformed
JSON surviving without raising.

**Panel verified visually** at 1500x900 against a live `approval_node`: header
`3 of 5`, `PERSON 2/3` with `[on][off][on][off][pending]`, `TENT confirmed` in green,
no console errors. Dims when mission state is not `scan`, since the window only
advances during scan.

**Risk containment in `filtering_node`.** The confirmation predicate moved to pure
module-level functions (`evaluate_window`, `window_presence`) with no ROS dependency.
`_publish_window` is wrapped in try/except and publishes only on change — a debug view
can never disturb what the aircraft chases.

### Not done / follow-ups

- Confirmation remains **class-level**, not object-level: with two people in view the
  window sees `{person}` per frame and the proximity check takes the *first* match per
  frame, so it can compare person A against person B. Fixing it properly means
  per-object spatial clustering. Flagged, not attempted.
- Not exercised against a live flying mission — verified with synthetic windows and
  the unit suite only.
