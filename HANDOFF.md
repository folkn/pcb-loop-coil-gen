# Handoff: parametric loop-coil antenna PCB generator

## What this is

A script that builds a multi-segment, capacitor-loaded loop-antenna PCB entirely through the
KiCad IPC API (kicad-python / `kipy`) — PCB editor only, no schematic. It draws a ring of trace
segments around a large center hole, bridges the gaps between segments with parallel capacitor
sets, and feeds the loop through an edge-mount SMA connector at the bottom.

Read `generate_loop_coil.py`'s module docstring first — it describes the net topology (which
segment is which net, how the feed gap's two capacitors and "centerpad" work, why one segment
shares the GND net). This document is about environment/process, not re-explaining that.

## Directory layout

```
/home/folkn/folk-shared/kicad-coil/
├── generate_loop_coil.py   # the generator script -- all user parameters are at the top
├── footprint_loader.py     # generic .kicad_mod parser + FootprintInstance builder (see below)
├── libraries/              # the 4 footprints this project uses, as flat .kicad_mod files
│   ├── CAP_1111B-hand.kicad_mod
│   ├── CAP_Knowles_47273.kicad_mod
│   ├── C_Trimmer_Voltronics_JZ.kicad_mod   # copied out of KiCad's stock Capacitor_SMD.pretty
│   └── SMA_J669.kicad_mod
├── project/
│   └── loop_coil.kicad_pcb # the actual board; this is what the script edits and what you open
├── kicad-python/           # upstream kicad-python repo, cloned + built here (has its own venv)
│   └── .env/               # the venv -- always run scripts through kicad-python/.env/bin/python
└── backups/<timestamp>/    # snapshots of generate_loop_coil.py + footprint_loader.py taken
                             # before risky edits -- there is no git repo here, so this is the
                             # only history; make a new timestamped one before a nontrivial change
```

## Why `footprint_loader.py` exists

The KiCad IPC API (as shipped in this kicad-python / KiCad 10.0.4) has **no RPC to instantiate a
footprint from a library by name** — it only lets a client create raw board items (pads, graphic
shapes, tracks, footprint definitions built from scratch). `footprint_loader.py` works around this
by parsing a `.kicad_mod` S-expression file directly and rebuilding it as a `FootprintInstance`
made of `Pad` / `BoardSegment` / `BoardArc` / `BoardCircle` / `BoardRectangle` items.

This was verified empirically against the live API (not just derived on paper) — see "How to
verify changes" below for the method, which is worth reusing if you touch this file.

Key functions:
- `load_footprint(path) -> FootprintTemplate`: parses pads (front-copper only — back-side pad
  copies are dropped, since this project is top-layer-only) and cosmetic front graphics
  (silkscreen/fab/courtyard).
- `instantiate_footprint(template, position_mm, rotation_deg, reference, net_by_pad=...)`: builds
  the `FootprintInstance`. Pads/graphics are added at their *local* (file) coordinates, then
  placed via `inst.position = ...` followed by `inst.orientation = Angle.from_degrees(...)` — in
  that order, because `FootprintInstance.orientation`'s setter rotates all children about
  `self.position`, so position must already be final before rotating.
- `global_pad_position(...)` / `world_bbox_corners(...)`: pure-Python reimplementations of kipy's
  own rotation formula (`Vector2.rotate`, confirmed by reading `kipy/geometry.py`), used so the
  main script can precompute exact pad/track endpoints without round-tripping to the server.

**Rotation convention** (this tripped me up once — re-derive from the same source rather than
guessing if something looks flipped): kipy's `Vector2.rotate(angle, center)` uses
`new_x = y*sin(a) + x*cos(a); new_y = y*cos(a) - x*sin(a)`. Working through this, a footprint
orientation of `φ` maps a *local* direction at angle `L` (0=local+X, measured the same way as the
`polar()` helper: 0=+X, 90=+Y, ...) to *global* angle `L - φ`. The script's
`orientation_pointing_local_dir(local_dir_deg, target_theta_deg)` helper wraps the inverse of
that (`(local_dir_deg - target_theta_deg) % 360`) — use it rather than hand-deriving angles again.

## Environment / how to run it

KiCad 10.0.4 is installed as an AppImage at `~/Downloads/kicad-10.0.4-x86_64.AppImage`. It is a
"sharun"-wrapped AppImage, **not** a normal one — you must pass an explicit subcommand as the
first argument, e.g. `<appimage> pcbnew <file>` or `<appimage> kicad <file>` (bare
`<appimage> <file>` fails with "Failed to check ELF class").

Important discovery: the plain `kicad` (project-manager) binary, even given a `.kicad_pcb` path
directly, does **not** register the IPC API's board-editing command handlers
(`GetOpenDocuments` etc. fail with "no handler available") unless a PCB editor frame is actually
open. Launching the **standalone `pcbnew` binary** directly with the file as an argument does
work reliably:

```sh
/home/folkn/Downloads/kicad-10.0.4-x86_64.AppImage pcbnew /home/folkn/folk-shared/kicad-coil/project/loop_coil.kicad_pcb &
```

The API server must be enabled (Preferences > Plugins > Enable KiCad API, or set
`"api": {"enable_server": true, ...}` in `~/.config/kicad/10.0/kicad_common.json` and restart —
this was off by default). Once running, the socket appears at `/tmp/kicad/api.sock` and
`kipy.KiCad()` connects to it with no arguments.

To run the generator:
```sh
cd /home/folkn/folk-shared/kicad-coil
kicad-python/.env/bin/python generate_loop_coil.py
```
It edits the currently-open board in place (wrapped in a single `begin_commit`/`push_commit`, so
it's one undo step). It does **not** call `board.save()` itself — call that separately (see
below) or use Ctrl+S in the GUI if you have a display attached.

If you re-run the script against a board that already has a previous generation on it, **clear it
first** or you'll get a duplicate coil on top of the old one:
```python
from kipy import KiCad
board = KiCad().get_board()
board.remove_items(list(board.get_footprints()) + list(board.get_tracks()) + list(board.get_shapes()))
```

## How to verify changes

Don't trust low-res SVG renders for small parts — I initially misread the render and thought the
`TRIMMER` capacitor wasn't being placed in parallel gaps, when in fact it was; the two parts (a
1111B and a much smaller trimmer) sit only ~0.6 mm apart edge-to-edge and merge into what looks
like one blob at typical zoom/export resolutions. Ground truth is always the API, not the image.
Two techniques that worked well together:

1. **Topology / connectivity**: query `board.get_nets()` and, for each footprint,
   `[(p.number, p.net.name) for p in fp.definition.pads]`. Compare against the expected net
   assignments in the module docstring. This catches wiring bugs immediately and is cheap.
2. **Geometry**: for anything spatial (are two parts actually separated, does a rotation point the
   right way), pull exact coordinates from the API and plot them yourself with matplotlib
   (`kicad-python/.env/bin/pip install matplotlib numpy` if not already present) — plot tracks as
   lines, pads as rectangles annotated with `f"{ref}.{pad.number}"`, using `to_mm()` throughout.
   This sidesteps all SVG-scale-guessing and rendering-resolution ambiguity and was what caught
   that the earlier visual read was wrong.

If you do want a quick visual sanity check anyway: `kicad-cli` works standalone (doesn't need the
IPC session), and is more reliable than the API's own `export_svg`/`export_render` (those failed
with "no handler available" when run against the standalone-`pcbnew`-opened document — job-export
RPCs seem to need the full `kicad` app process, not standalone `pcbnew`):
```sh
/tmp/.mount_kicad*/bin/kicad-cli pcb export svg --layers "F.Cu,F.SilkS,Edge.Cuts" \
  --output preview.svg project/loop_coil.kicad_pcb
```
(`librsvg2-bin`'s `rsvg-convert` is installed and can turn that into a PNG for viewing.) The
`/tmp/.mount_kicad*` path is an ephemeral AppImage FUSE mount that only exists while some AppImage
process is running — glob for it fresh each time, don't hardcode the suffix.

## Current parameters / design choices worth knowing about

- Default capacitor set per non-feed gap is now `["CAP_1111B", "TRIMMER"]` — both in parallel,
  stacked radially (pitch = larger of the two footprints' bbox depth + `CAP_STACK_CLEARANCE_MM`).
  Order in the list = innermost to outermost.
- `SMA_CABLE_FLIP_DEG` (default 180) is applied on top of the SMA's base "points away from the
  coil" orientation — it's a named, adjustable offset rather than a hardcoded angle, precisely so
  a future flip request doesn't require re-deriving the rotation math.
- The board outline is a donut: `BOARD_ANNULUS_WIDTH_MM` (default 5 mm) sets the radial distance
  from the raw hole edge to the outer Edge.Cuts boundary, and a tab at the feed gap covers the SMA
  connector down to the GND strap. The outer boundary is **one continuous closed loop, not two
  overlapping closed shapes** — an earlier version drew a full `BoardCircle` plus a
  separately-closed `BoardPolygon` rectangle and relied on KiCad to reconcile the overlap, which is
  the wrong way to add a tab to a board edge. The current version instead: computes where the
  tab's two side walls (straight lines running radially, at constant tangential offset) cross the
  outer circle, replaces that small arc of the circle with three `BoardSegment` lines (side wall,
  tab outer edge, side wall), and represents the remaining major arc as two `BoardArc`s split at
  the point opposite the feed. All of this is built in (radial, tangential) coordinates about
  `theta_feed`, so it stays correct if `FEED_ANGLE_DEG` is ever changed away from the default 90°
  (straight down). Verified by pulling the exact start/end coordinates of every edge shape via the
  API and confirming consecutive endpoints coincide to sub-micron precision (see "How to verify
  changes").
- The tab's outer edge is anchored to `gnd_strap_outer_edge_radius_mm` (the GND strap's far copper
  edge), **not** the SMA's full mechanical bounding box — the part of the connector beyond the
  strap (it's an edge-mount part) is expected to hang off the board. Get this radius via the
  *radial projection* of a strap endpoint onto `radial_unit`, not `math.hypot(...)` distance from
  center — the strap endpoints also carry a tangential offset (they're either side of the center
  pin), so a plain distance overshoots by however much that tangential offset contributes (this
  was an actual bug caught mid-session: `math.hypot(...)` put the tab ~0.2 mm further out than
  intended). If the tab position ever looks off by a small amount again, suspect this class of
  mistake first.
- `HOLE_TRACE_CLEARANCE_MM` (default 0.5 mm) keeps the coil trace's inner copper edge strictly
  outside the raw hole radius. This is *not* sufficient on its own for capacitors, though: a
  capacitor centered on `coil_radius_mm` clips the hole if its radial half-height exceeds
  `TRACE_WIDTH_MM/2` (true for both CAP_1111B and the trimmer at the current trace width). Every
  capacitor placement (feed caps and each generic-gap stack) is therefore anchored so its
  *innermost* body's inner edge lands exactly at the trace's own inner edge
  (`coil_radius_mm - TRACE_WIDTH_MM/2`) rather than being centered on `coil_radius_mm` — see
  `base_radial_offset` in the generic-gap loop and `feed_left_radial_offset`/
  `feed_right_radial_offset` for the feed caps. This is a real geometric proof, not a tuned
  constant: reducing `HOLE_DIAMETER_CM` alone cannot fix hole-clipping, because `coil_radius_mm`
  is defined as `hole_radius_mm + constant`, so the offset between a capacitor's reach and the
  hole edge is invariant to hole size. If clipping ever reappears, look for a capacitor placement
  that bypassed this anchoring, not a hole-size tweak.
- `BOARD_ANNULUS_WIDTH_MM = 5` is tight enough that the outer-stacked capacitor (whichever is last
  in `GAP_CAPACITORS`) currently pokes past the outer boundary — the script's own
  `max_extent_mm > outer_radius_mm` warning fires for this at the current defaults. This is
  expected/known, not a bug: the annulus width and the capacitor stack were set independently per
  explicit request. If it needs to actually fit inside the outer circle, either widen
  `BOARD_ANNULUS_WIDTH_MM` back up or reduce `GAP_CAPACITORS` to fewer/smaller parts.
- The GND strap between the two SMA ground pads is routed by offsetting each pad *radially
  outward* (away from the coil) before joining them, rather than a hardcoded-axis L-route — this
  keeps it correct regardless of `FEED_ANGLE_DEG` or `SMA_CABLE_FLIP_DEG`.

## Known simplifications / good next steps

- The feed-tab's SMA bounding box uses the footprint's parsed local bbox (pads + courtyard
  graphics). If you swap in a different SMA footprint whose courtyard doesn't tightly represent
  its true mechanical edge, the tab will still hug whatever the courtyard says — worth eyeballing
  after a footprint swap.
- Solder-mask margin / thermal-relief overrides on the source footprints (e.g. the Knowles part's
  `solder_mask_margin`, `thermal_bridge_angle`) are not replicated by `footprint_loader.py` —
  cosmetic/manufacturing fine-tuning only, not required for the topology to be correct, but a
  fabrication-readiness pass should double check these.
- `NUM_SEGMENTS == 1` is handled (no generic gaps, single segment = GND net throughout, feed caps
  both still present) but is an unusual/untested-visually edge case — sanity check it if it
  becomes relevant.
- No DRC pass is run by the script. Given the tight default clearances (e.g. the two feed
  capacitors are only `FEED_CENTER_GAP_MM` apart, `STUB_LENGTH_MM` stubs), it'd be worth running
  KiCad's DRC (via the GUI, or look for an API path) before fabrication, especially after
  changing trace width / capacitor choices.
