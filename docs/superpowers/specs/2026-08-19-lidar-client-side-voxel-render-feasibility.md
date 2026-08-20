# LIDAR thin-client: client-side isometric voxel rendering — feasibility spike

**Date:** 2026-08-19
**Status:** Proposed — research spike only. No implementation, no protocol changes yet.
**Question:** Instead of streaming rendered pixel frames (JPEG today, H.264 spiked
separately), can the rig send compact raw scene state — a coarse voxel/occupancy grid —
and let the P4 render its own stylized fixed-angle isometric view locally? Does that
actually reduce bandwidth need for the common case, and is it cheap enough to render on
the panel?
**Related:** `2026-08-18-lidar-thin-frame-bandwidth-protocol.md` (shipped v2 JPEG +
credit protocol) and `2026-08-19-lidar-video-delta-streaming-feasibility.md` (H.264
spike). This spec is a **different axis of change** from both — it moves *where
rendering happens*, not which codec compresses a rendered image. Read both before
approving this; several of their conclusions (the orbit failure mode, the
independent-vs-referenced-frame flow-control problem) are the baseline this idea is
being compared against.

## The core insight

Both existing approaches compute their "what changed" signal in **pixel space** — after
the rig has already projected the 3D scene through a camera. That's why camera orbit is
the worst case for both: a `thin_orbit` drag changes every pixel even though nothing in
the actual scene moved, so pixel-space diffing (rejected tile-diff idea,
`2026-08-18-...md`'s "Rejected alternatives") and even motion-compensated video (this
spec's sibling spike) both spend the most bytes exactly when the user is interacting.

If instead the delta is computed in **scene space** — a compact model of the actual
LIDAR/SLAM world state, held persistently on the panel — and the panel does its own
(deliberately simple, fixed-angle) projection locally, then **camera motion becomes a
local recompute that costs zero network bytes.** Only genuine content changes (new
returns, something moving through the scanned space) need to cross the link at all, and
those really do look like "a few cells changing a little" — which is the case this
whole exploration started from. This is architecturally the strongest of the three
options on paper, for exactly that reason: it's the one where interaction (the thing
users actually do most on this panel) doesn't cost bandwidth.

It also sidesteps the flow-control objection that weighs down the H.264 idea. A voxel
update can be written as an **idempotent absolute value** (`"cell (x,y,z) = occupied,
color C"`) rather than a diff against a specific prior frame, so losing one update in
transit just leaves that cell stale until the next thing touches it, or a periodic full
resync passes over it — nothing like the corrupted-macroblock cascade a dropped P-frame
causes downstream of a missing reference. The reliability bar for this transport is
lower than for either pixel-stream alternative.

## What this deliberately gives up

This is not an attempt to reproduce the rig's actual photoreal/arbitrary-perspective
point-cloud or SLAM render on the panel. It's a **stylized, reduced-fidelity view**: a
coarse voxel grid, likely a fixed or simply-stepped isometric camera (à la classic
isometric tile-engine games), not real perspective 3D. That's a real fidelity cut versus
what `PK_LIDAR` shows today — worth naming plainly rather than discovering after the
fact. It also happens to fit this prop's cassette-futurism aesthetic (blocky, stylized,
deliberately retro) arguably *better* than a photoreal render would, which is a genuine
bonus, not just a compute compromise — but it's a creative call, not just an engineering
one, and should be signed off as such before real work starts.

## Open questions (in priority order — #1 blocks everything else)

1. **What raw representation does the rig's SLAM/point-cloud pipeline actually have
   available to export?** Point cloud, voxel grid, height map, occupancy grid — unknown
   from this workspace; `submodules/lidar-roomscanner` is not checked out here
   (`docs/referenceDesign`-style external submodule, per `CLAUDE.md`). Nothing else in
   this spec can be sized or scoped without answering this first. This is an
   **investigation task, not a benchmark** — cheaper to start than the H.264 spike's
   decode microbenchmark, and should happen first regardless of which spike concludes
   faster.
2. **Real-world delta size**, once #1 is known — validated against actual capture data,
   not assumption. "A few voxels change per tick" needs checking against what this
   system's actual motion looks like: a moving robot/operator continually adds new LIDAR
   returns, which may make "small delta" true only some of the time (e.g., idle
   stationary scanning) and not others (active sweeping of new area). Back-of-envelope
   only after real data: e.g. a 32×32×16 grid (16,384 cells) at 1 byte/cell (occupancy +
   palette index) is 16 KB full-state — comparable to a JPEG frame — so the whole
   argument for this approach rests on deltas being small *in practice*, not just in
   principle.
3. **Server-side delta/resync machinery.** Unlike JPEG's "discard and send newest" model,
   voxel state is cumulative and needs a reliable, ordered (or self-healing via periodic
   full resync) delta stream — new machinery on the rig, comparable in kind to but
   simpler than the encoder-side credit-aware frame selection the H.264 idea needs (see
   that spec's §The flow-control problem), because idempotent writes make loss
   self-correcting rather than corrupting. Still real work, still rig-side
   (`lidar-roomscanner`, out of scope for this repo to build).
4. **Client-side render cost.** A simple fixed-angle isometric voxel rasterizer on the
   P4 — plausibly modest given `prop_ui.c` already draws far more complex vector content
   at 20 Hz, and LVGL's PPA (scale/rotate/blend hardware block, already used elsewhere —
   see the PPA research spec) may accelerate tile blitting. Still needs a real prototype
   against whatever grid size #2 lands on, not an assumption — this is a materially
   different (and probably easier to reason about) compute question than software H.264
   decode, since it's directly comparable to LVGL's own existing per-panel render budget
   rather than an opaque third-party codec's cost.
5. **Scope: additive mode, not replacement.** `thin_mode` already supports switching
   between point_cloud/SLAM/IR views (`main/prop_lidar.c`); recommend this ships as a
   new mode alongside the existing ones, not a replacement, at least until fidelity #the
   "what this gives up" tradeoff above is validated as acceptable.

## Recommended next step

Answer open question 1 before anything else — it's a reading task, not code:

1. Check out `submodules/lidar-roomscanner` and read its scanning/SLAM pipeline for
   what internal representation it actually holds (point cloud? voxelized already for
   its own rendering? something else?) and whether any export path already exists.
2. Once known, do a **paper design** of a candidate wire delta-format and a rough
   bytes-per-tick estimate **against a real capture** from the rig (not synthetic data)
   — this is what actually validates or kills open question 2, and it's cheap (log
   scene-state changes over a real session, compute what a delta encoding of that log
   would have cost) compared to writing a renderer first and finding out the deltas
   aren't actually small.
3. Only after both of those hold up is a client-side render prototype (open question 4)
   worth building.

## How this compares to the other two options

This is likely the **highest-ceiling** option — it's the only one where interaction
(orbit) doesn't cost bandwidth, which directly targets this link's actual constraint —
but it's also the **highest-uncertainty** one, because it depends on data the rig may or
may not currently expose in a usable form, and it requires genuinely new render code on
the panel rather than reusing existing rig rendering + an off-the-shelf codec the way
both JPEG (shipped) and H.264 (spiked) do. The H.264 spike's decode-cost question and
this spec's data-availability question are independent and can be investigated in
parallel; neither blocks the other.

## Non-goals for this spike

- No changes to `main/prop_lidar.c`, `main/prop_ui.c`, or the `/ws-thin` wire protocol.
- No rig-side (`lidar-roomscanner`) implementation — reading its existing pipeline to
  answer open question 1 is in scope; building an export path is not, yet.
- Not attempting photoreal or arbitrary-perspective parity with the current rendered
  view — that is explicitly not the goal (see "What this deliberately gives up").

## Rejected framing

- **Building the client-side renderer first, against synthetic/assumed data.** If the
  real delta sizes turn out large (open question 2), the renderer was wasted effort; the
  data-availability and real-capture sizing questions are strictly cheaper to answer and
  gate everything downstream.
- **Treating this as a drop-in replacement for the current LIDAR view.** The fidelity
  cut is real and worth a deliberate decision, not something to discover after shipping.
