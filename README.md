# pcb-loop-coil-gen

Generates a parametric, multi-segment, capacitor-loaded loop-antenna PCB in KiCad — entirely
through the KiCad IPC API ([kicad-python](https://gitlab.com/kicad/code/kicad-python) / `kipy`).
PCB editor only; no schematic is involved.

It draws a ring of copper trace segments around a large center hole, bridges the gaps between
segments with parallel capacitor sets, and feeds the loop through an edge-mount SMA connector.
The board outline is a donut with a rectangular tab at the feed that borders the SMA connector.

## Requirements

- KiCad 10.x with the IPC API server enabled (Preferences > Plugins > Enable KiCad API)
- Python 3.9+ and [kicad-python](https://gitlab.com/kicad/code/kicad-python) built against your
  KiCad install (see kicad-python's own `COMPILING.md`)
- The four footprints under `libraries/` are already included (`.kicad_mod` files) — two custom
  hand-made parts, a Knowles part, and KiCad's stock trimmer capacitor copied out locally

## Usage

1. Open `project/loop_coil.kicad_pcb` in KiCad's PCB editor (or any blank `.kicad_pcb`), with the
   IPC API server running.
2. Edit the parameters at the top of `generate_loop_coil.py` (hole diameter, trace width, number
   of segments, which capacitors to place, SMA orientation, etc).
3. Run it against the open board:
   ```sh
   python generate_loop_coil.py
   ```
   This edits the currently open board in place as a single undo step. It does not save the file
   itself — save from the GUI (Ctrl+S) or call `board.save()` yourself.

If re-running against a board that already has a previous generation on it, clear it first:
```python
from kipy import KiCad
board = KiCad().get_board()
board.remove_items(list(board.get_footprints()) + list(board.get_tracks()) + list(board.get_shapes()))
```

## Topology

The loop is a circle of N trace segments and N gaps, evenly distributed by angle. One gap is the
"feed" gap and contains the SMA connector; the other N-1 gaps each carry a user-chosen, identical
set of parallel capacitors bridging the two neighbouring segments.

At the feed gap: `coilLeft -- FEED_CAP_LEFT -- centerpad -- FEED_CAP_RIGHT -- coilRight`.
`coilRight` is tied directly (copper, no capacitor) to the SMA ground pads; `centerpad` drops
straight down to the SMA center pin. One segment therefore shares the GND net with the SMA ground
pads, while every other segment gets its own unique net (isolated by the capacitor gaps on both
sides).

See the module docstring in `generate_loop_coil.py` for the full picture, and `HANDOFF.md` for
implementation notes, environment quirks, and known simplifications (worth reading before making
nontrivial changes).

## Layout

```
generate_loop_coil.py   - the generator script; all user parameters are at the top
footprint_loader.py     - generic .kicad_mod parser + FootprintInstance builder (see HANDOFF.md
                          for why this exists: the IPC API has no "load footprint by name" call)
libraries/              - the .kicad_mod footprints this project uses
project/                - the KiCad project + board file
HANDOFF.md              - developer notes: environment setup, verification method, design
                          decisions and the reasoning behind them, known limitations
```

## Verifying changes

Don't trust low-resolution renders for fine detail — small parts placed a fraction of a
millimeter apart can look merged. Ground truth is the API: query `board.get_nets()` and each
footprint's `[(pad.number, pad.net.name) for pad in fp.definition.pads]` to check wiring, and pull
exact coordinates to check geometry (see `HANDOFF.md` for the matplotlib-based technique used
during development). For a quick visual check, `kicad-cli pcb export svg` works well.
