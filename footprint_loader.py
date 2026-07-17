#!/usr/bin/env python3
"""Generic .kicad_mod loader for kicad-python (kipy).

The KiCad IPC API (as of kicad-python 0.8.0 / KiCad 10) has no RPC to instantiate a footprint
from a library by name -- it only lets a client create raw board items (pads, graphic shapes,
tracks, etc). This module parses a plain .kicad_mod S-expression file and rebuilds it as a kipy
FootprintInstance made of Pad / BoardSegment / BoardArc / BoardCircle / BoardRectangle items, so
that footprints from arbitrary .pretty libraries can be dropped onto a board purely through the
API.

Simplifications (all footprints used by this project are SMD-only):
    * Only front-side items are kept (F.Cu / F.SilkS / F.Fab / F.CrtYd). Back-layer copies of a
      pad (as seen in double-sided edge-launch connector footprints) are dropped, since this
      project is single (top) layer only.
    * Per-pad solder mask/paste margin overrides and thermal relief settings are not replicated
      (cosmetic/manufacturing fine-tuning only, not required for function).
    * Footprint text items ("${REFERENCE}" helper texts etc.) are not replicated; only the
      Reference/Value fields are, using the position found in the source file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Union

from kipy.board_types import (
    BoardArc,
    BoardCircle,
    BoardLayer,
    BoardRectangle,
    BoardSegment,
    FootprintInstance,
    Pad,
    PadStackShape,
    PadType,
)
from kipy.geometry import Angle, Vector2
from kipy.util.units import from_mm

Sexp = Union[str, list]


# ---------------------------------------------------------------------------
# S-expression parsing
# ---------------------------------------------------------------------------

def _tokenize(text: str) -> list:
    tokens: list = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c == "(":
            tokens.append("(")
            i += 1
        elif c == ")":
            tokens.append(")")
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while j < n and text[j] != '"':
                if text[j] == "\\" and j + 1 < n:
                    buf.append(text[j + 1])
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            tokens.append(("str", "".join(buf)))
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in " \t\r\n()":
                j += 1
            tokens.append(text[i:j])
            i = j
    return tokens


def _parse_tokens(tokens: list) -> Sexp:
    pos = 0

    def parse_expr():
        nonlocal pos
        tok = tokens[pos]
        if tok == "(":
            pos += 1
            lst: list = []
            while tokens[pos] != ")":
                lst.append(parse_expr())
            pos += 1
            return lst
        pos += 1
        return tok[1] if isinstance(tok, tuple) else tok

    return parse_expr()


def parse_sexp(text: str) -> Sexp:
    return _parse_tokens(_tokenize(text))


def _children(node: Sexp, tag: str) -> list:
    """Returns all direct child lists of `node` whose first element equals `tag`."""
    if not isinstance(node, list):
        return []
    return [c for c in node if isinstance(c, list) and c and c[0] == tag]


def _child(node: Sexp, tag: str) -> Optional[list]:
    found = _children(node, tag)
    return found[0] if found else None


def _f(x) -> float:
    return float(x)


# ---------------------------------------------------------------------------
# In-memory footprint template
# ---------------------------------------------------------------------------

@dataclass
class PadDef:
    number: str
    shape: PadStackShape.ValueType
    x_mm: float
    y_mm: float
    rot_deg: float
    w_mm: float
    h_mm: float
    roundrect_ratio: float = 0.0


@dataclass
class GraphicDef:
    kind: str  # 'segment' | 'rect' | 'circle' | 'arc'
    layer: BoardLayer.ValueType
    width_mm: float
    filled: bool = False
    points: list = field(default_factory=list)  # list of (x_mm, y_mm) tuples, meaning depends on kind


@dataclass
class FootprintTemplate:
    name: str
    pads: list  # list[PadDef]
    graphics: list  # list[GraphicDef]
    ref_pos_mm: tuple
    value_pos_mm: tuple

    def bbox_mm(self) -> tuple:
        """Returns (min_x, min_y, max_x, max_y) in local (unrotated) mm, covering pads+graphics."""
        xs, ys = [], []
        for p in self.pads:
            half_w, half_h = p.w_mm / 2, p.h_mm / 2
            for dx in (-half_w, half_w):
                for dy in (-half_h, half_h):
                    xs.append(p.x_mm + dx)
                    ys.append(p.y_mm + dy)
        for g in self.graphics:
            for (x, y) in g.points:
                xs.append(x)
                ys.append(y)
        if not xs:
            return (0.0, 0.0, 0.0, 0.0)
        return (min(xs), min(ys), max(xs), max(ys))


_FRONT_LAYERS = {"F.Cu", "F.SilkS", "F.Fab", "F.CrtYd", "F.Mask", "F.Paste"}

_SHAPE_MAP = {
    "rect": PadStackShape.PSS_RECTANGLE,
    "roundrect": PadStackShape.PSS_ROUNDRECT,
    "circle": PadStackShape.PSS_CIRCLE,
    "oval": PadStackShape.PSS_OVAL,
    "trapezoid": PadStackShape.PSS_TRAPEZOID,
}


def load_footprint(path: str) -> FootprintTemplate:
    with open(path, "r", encoding="utf-8") as fh:
        text = fh.read()
    tree = parse_sexp(text)
    assert isinstance(tree, list) and tree and tree[0] == "footprint"
    name = tree[1] if isinstance(tree[1], str) else "footprint"

    pads: list = []
    for pad_node in _children(tree, "pad"):
        # (pad "<number>" smd <shape> (at x y [rot]) (size w h) (layers ...) (roundrect_rratio r))
        number = pad_node[1]
        shape_kw = pad_node[3]
        at = _child(pad_node, "at")
        size = _child(pad_node, "size")
        layers_node = _child(pad_node, "layers")
        layer_names = layers_node[1:] if layers_node else []
        if "F.Cu" not in layer_names:
            continue  # top-layer-only project: skip back-side pad copies

        x_mm, y_mm = _f(at[1]), _f(at[2])
        rot_deg = _f(at[3]) if len(at) > 3 else 0.0
        w_mm, h_mm = _f(size[1]), _f(size[2])
        rr_node = _child(pad_node, "roundrect_rratio")
        rr = _f(rr_node[1]) if rr_node else 0.0

        pads.append(
            PadDef(
                number=number,
                shape=_SHAPE_MAP.get(shape_kw, PadStackShape.PSS_RECTANGLE),
                x_mm=x_mm,
                y_mm=y_mm,
                rot_deg=rot_deg,
                w_mm=w_mm,
                h_mm=h_mm,
                roundrect_ratio=rr,
            )
        )

    graphics: list = []
    for tag, kind in (("fp_line", "segment"), ("fp_rect", "rect"), ("fp_circle", "circle"), ("fp_arc", "arc")):
        for node in _children(tree, tag):
            layer_node = _child(node, "layer")
            layer_name = layer_node[1] if layer_node else ""
            if layer_name not in _FRONT_LAYERS or layer_name in ("F.Cu", "F.Mask", "F.Paste"):
                continue  # only keep cosmetic front graphics: silkscreen/fab/courtyard

            stroke = _child(node, "stroke")
            width_mm = _f(_child(stroke, "width")[1]) if stroke and _child(stroke, "width") else 0.1
            fill_node = _child(node, "fill")
            filled = bool(fill_node and fill_node[1] == "yes")

            layer = getattr(BoardLayer, "BL_" + layer_name.replace(".", "_"))

            if kind == "segment":
                start = _child(node, "start")
                end = _child(node, "end")
                pts = [(_f(start[1]), _f(start[2])), (_f(end[1]), _f(end[2]))]
            elif kind == "rect":
                start = _child(node, "start")
                end = _child(node, "end")
                pts = [(_f(start[1]), _f(start[2])), (_f(end[1]), _f(end[2]))]
            elif kind == "circle":
                center = _child(node, "center")
                end = _child(node, "end")
                pts = [(_f(center[1]), _f(center[2])), (_f(end[1]), _f(end[2]))]
            else:  # arc
                start = _child(node, "start")
                mid = _child(node, "mid")
                end = _child(node, "end")
                pts = [(_f(start[1]), _f(start[2])), (_f(mid[1]), _f(mid[2])), (_f(end[1]), _f(end[2]))]

            graphics.append(GraphicDef(kind=kind, layer=layer, width_mm=width_mm, filled=filled, points=pts))

    ref_pos = (0.0, 0.0)
    value_pos = (0.0, 0.0)
    for prop in _children(tree, "property"):
        prop_name = prop[1]
        at = _child(prop, "at")
        if not at:
            continue
        pos = (_f(at[1]), _f(at[2]))
        if prop_name == "Reference":
            ref_pos = pos
        elif prop_name == "Value":
            value_pos = pos

    return FootprintTemplate(name=name, pads=pads, graphics=graphics, ref_pos_mm=ref_pos, value_pos_mm=value_pos)


# ---------------------------------------------------------------------------
# World-space geometry helpers (mirrors kipy's own Vector2.rotate formula, so results match
# exactly what the server computes for FootprintInstance.position/orientation)
# ---------------------------------------------------------------------------

def _rotate_xy(x_mm: float, y_mm: float, rotation_deg: float) -> tuple:
    rot = math.radians(rotation_deg)
    s, c = math.sin(rot), math.cos(rot)
    return (y_mm * s + x_mm * c, y_mm * c - x_mm * s)


def global_pad_position(
    template: FootprintTemplate,
    pad_number: str,
    position_mm: tuple,
    rotation_deg: float,
    occurrence: int = 0,
) -> tuple:
    """World position (mm) of the `occurrence`-th pad named `pad_number` (pad numbers may repeat,
    e.g. multiple ground pads), for a footprint placed at `position_mm`/`rotation_deg`."""
    matches = [p for p in template.pads if p.number == pad_number]
    p = matches[occurrence]
    dx, dy = _rotate_xy(p.x_mm, p.y_mm, rotation_deg)
    return (position_mm[0] + dx, position_mm[1] + dy)


def world_bbox_corners(template: FootprintTemplate, position_mm: tuple, rotation_deg: float) -> list:
    """World positions (mm) of the 4 corners of `template`'s local bounding box, for a footprint
    placed at `position_mm`/`rotation_deg`. Useful for conservative extent/clearance checks."""
    min_x, min_y, max_x, max_y = template.bbox_mm()
    corners = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]
    result = []
    for (x, y) in corners:
        dx, dy = _rotate_xy(x, y, rotation_deg)
        result.append((position_mm[0] + dx, position_mm[1] + dy))
    return result


# ---------------------------------------------------------------------------
# Instantiation onto a board
# ---------------------------------------------------------------------------

def instantiate_footprint(
    template: FootprintTemplate,
    position_mm: tuple,
    rotation_deg: float,
    reference: str,
    value: Optional[str] = None,
    net_by_pad: Optional[dict] = None,
) -> FootprintInstance:
    """Builds a FootprintInstance for `template`, placed at `position_mm` (board-space, mm) and
    rotated by `rotation_deg` (kipy/pcbnew orientation convention).

    `net_by_pad` maps pad number strings (as found in the source .kicad_mod, e.g. "1", "2") to
    kipy Net objects; pads without an entry are left netless.
    """
    net_by_pad = net_by_pad or {}

    inst = FootprintInstance()
    inst.layer = BoardLayer.BL_F_Cu
    inst.definition.id.library = "generated"
    inst.definition.id.name = template.name

    for p in template.pads:
        pad = Pad()
        pad.number = p.number
        pad.pad_type = PadType.PT_SMD
        pad.position = Vector2.from_xy_mm(p.x_mm, p.y_mm)
        pad.padstack.layers = [BoardLayer.BL_F_Cu, BoardLayer.BL_F_Mask, BoardLayer.BL_F_Paste]
        copper = pad.padstack.copper_layer(BoardLayer.BL_F_Cu)
        assert copper is not None
        copper.shape = p.shape
        copper.size = Vector2.from_xy_mm(p.w_mm, p.h_mm)
        if p.shape == PadStackShape.PSS_ROUNDRECT:
            copper.corner_rounding_ratio = p.roundrect_ratio
        if p.rot_deg:
            pad.padstack.angle = Angle.from_degrees(p.rot_deg)
        net = net_by_pad.get(p.number)
        if net is not None:
            pad.net = net
        inst.definition.add_item(pad)

    for g in template.graphics:
        shape: Union[BoardSegment, BoardRectangle, BoardCircle, BoardArc]
        if g.kind == "segment":
            shape = BoardSegment()
            shape.start = Vector2.from_xy_mm(*g.points[0])
            shape.end = Vector2.from_xy_mm(*g.points[1])
        elif g.kind == "rect":
            shape = BoardRectangle()
            shape.top_left = Vector2.from_xy_mm(*g.points[0])
            shape.bottom_right = Vector2.from_xy_mm(*g.points[1])
            shape.attributes.fill.filled = g.filled
        elif g.kind == "circle":
            shape = BoardCircle()
            shape.center = Vector2.from_xy_mm(*g.points[0])
            shape.radius_point = Vector2.from_xy_mm(*g.points[1])
            shape.attributes.fill.filled = g.filled
        else:
            shape = BoardArc()
            shape.start = Vector2.from_xy_mm(*g.points[0])
            shape.mid = Vector2.from_xy_mm(*g.points[1])
            shape.end = Vector2.from_xy_mm(*g.points[2])
        shape.layer = g.layer
        shape.attributes.stroke.width = from_mm(g.width_mm)
        inst.definition.add_item(shape)

    inst.reference_field.text.value = reference
    inst.reference_field.text.layer = BoardLayer.BL_F_SilkS
    inst.reference_field.text.position = Vector2.from_xy_mm(*template.ref_pos_mm)
    inst.reference_field.text.attributes.size = Vector2.from_xy_mm(1.0, 1.0)
    inst.reference_field.text.attributes.stroke_width = from_mm(0.15)

    inst.value_field.text.value = value if value is not None else template.name
    inst.value_field.text.layer = BoardLayer.BL_F_Fab
    inst.value_field.text.position = Vector2.from_xy_mm(*template.value_pos_mm)
    inst.value_field.text.attributes.size = Vector2.from_xy_mm(1.0, 1.0)
    inst.value_field.text.attributes.stroke_width = from_mm(0.15)
    inst.value_field.visible = False

    # Place: translate to position, then rotate about that same point (matches KiCad's own
    # placement transform, and matches how FootprintInstance.orientation applies its rotation).
    inst.position = Vector2.from_xy_mm(*position_mm)
    if rotation_deg:
        inst.orientation = Angle.from_degrees(rotation_deg)

    return inst
