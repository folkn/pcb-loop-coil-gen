#!/usr/bin/env python3
"""Parametric multi-segment, capacitor-loaded loop-antenna PCB generator.

Builds the whole board directly through the KiCad IPC API (kicad-python / kipy) -- PCB editor
only, no schematic involved. Run with KiCad's PCB editor open on an (empty) board and the API
server enabled in Preferences > Plugins:

    kicad-python/.env/bin/python generate_loop_coil.py

Topology
--------
The loop is a circle of N trace segments and N gaps, evenly distributed by angle. Gap 0 is the
"feed" gap (at the bottom by default) and contains the SMA connector; the other N-1 gaps each
carry a user-chosen, identical set of parallel capacitors bridging the two neighbouring segments.

Going around the loop in increasing angle (clockwise on screen, since KiCad's Y axis points
down): ... seg[N] -> gap[0] (feed) -> seg[1] -> gap[1] -> seg[2] -> ... -> seg[N] ...

At the feed gap: coilLeft (seg[1] side) -- FEED_CAP_LEFT -- centerpad -- FEED_CAP_RIGHT --
coilRight (seg[N] side). coilRight is tied directly (copper, no capacitor) to the SMA ground
pads; centerpad drops straight down to the SMA center pin. Because of this, seg[N] shares the
GND net with the SMA ground pads, seg[1..N-1] each get their own unique net (isolated by the
capacitor gaps on both sides), and the feed node is its own "FEED" net.
"""

import math
import os

from kipy import KiCad
from kipy.board_types import ArcTrack, BoardArc, BoardCircle, BoardLayer, BoardSegment, Net, Track
from kipy.geometry import Vector2
from kipy.util.units import from_mm

from footprint_loader import global_pad_position, instantiate_footprint, load_footprint, world_bbox_corners

# ---------------------------------------------------------------------------
# USER PARAMETERS
# ---------------------------------------------------------------------------

HOLE_DIAMETER_CM = 5.0     # raw diameter of the large center hole (the coil trace is kept clear
                           # of this by HOLE_TRACE_CLEARANCE_MM below -- it never overlaps/cuts it)
TRACE_WIDTH_MM = 2.0       # loop trace width
NUM_SEGMENTS = 6           # number of trace segments (and gaps) around the loop, >= 1

LIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libraries")
CAP_FOOTPRINTS = {
    "CAP_1111B": os.path.join(LIB_DIR, "CAP_1111B-hand.kicad_mod"),
    "CAP_KNOWLES": os.path.join(LIB_DIR, "CAP_Knowles_47273.kicad_mod"),
    "TRIMMER": os.path.join(LIB_DIR, "C_Trimmer_Voltronics_JZ.kicad_mod"),
}
SMA_FOOTPRINT_PATH = os.path.join(LIB_DIR, "SMA_J669.kicad_mod")

# Capacitor set placed in parallel across every non-feed gap (order = innermost to outermost).
# Repeat an entry for multiple identical parallel caps, e.g. ["CAP_1111B", "CAP_1111B"].
GAP_CAPACITORS = ["CAP_1111B", "TRIMMER"]

# The feed gap's two capacitors: coilLeft -- FEED_CAP_LEFT -- centerpad -- FEED_CAP_RIGHT -- coilRight
FEED_CAP_LEFT = "CAP_1111B"
FEED_CAP_RIGHT = "CAP_1111B"

SMA_TRACE_WIDTH_MM = 3.0       # trace width from centerpad down to the SMA center pin
GND_STRAP_WIDTH_MM = 1.5       # trace width tying the SMA ground pads together
SMA_CABLE_FLIP_DEG = 180.0     # extra rotation applied to the SMA on top of its base "points away
                               # from the coil" orientation -- 180 flips which way the cable exits

STUB_LENGTH_MM = 0.6              # straight stub connecting an arc end to a nearby capacitor pad
CAP_STACK_CLEARANCE_MM = 0.5      # spacing between capacitors stacked in parallel in one gap
FEED_CENTER_GAP_MM = 1.5          # gap between the two feed capacitors (spanned by the centerpad)
SMA_STANDOFF_CLEARANCE_MM = 3.0   # clearance between the SMA's reach-toward-coil and the coil trace

HOLE_TRACE_CLEARANCE_MM = 0.5   # guaranteed clearance between the center hole edge and the trace
BOARD_ANNULUS_WIDTH_MM = 5.0    # radial distance from the center hole edge to the board's outer
                                 # Edge.Cuts edge (the board is a donut of this width, plus a
                                 # rectangular tab at the feed -- see SMA_EDGE_MARGIN_MM)
SMA_EDGE_MARGIN_MM = 0.0         # extra clearance added beyond the GND strap when cutting the
                                 # feed-side tab's outer edge (0 = it ends exactly at the strap)

LOOP_CENTER_MM = (150.0, 100.0)   # coil center position on the page
FEED_ANGLE_DEG = 90.0             # 0=right, 90=down, 180=left, 270=up (screen convention, Y-down)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def polar(center_mm, radius_mm, theta_deg):
    th = math.radians(theta_deg)
    return (center_mm[0] + radius_mm * math.cos(th), center_mm[1] + radius_mm * math.sin(th))


def orientation_pointing_local_dir(local_dir_deg, target_theta_deg):
    """kipy/pcbnew orientation angle that rotates a footprint so that its local direction
    `local_dir_deg` (0=local +X, 180=local -X, ...) ends up pointing at global angle
    `target_theta_deg` (same 0=right/90=down/... convention as `polar`)."""
    return (local_dir_deg - target_theta_deg) % 360.0


def orientation_for_forward(theta_deg):
    """kipy/pcbnew orientation angle that points a footprint's local +X axis (its pad1->pad2
    axis, for all capacitor footprints used here) along the tangent of increasing theta at angle
    `theta_deg` -- i.e. towards the "next" segment when walking the loop forward."""
    return orientation_pointing_local_dir(0.0, theta_deg + 90.0)


def angle_span(length_mm, radius_mm):
    return math.degrees(length_mm / radius_mm)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    kicad = KiCad()
    board = kicad.get_board()

    hole_radius_mm = HOLE_DIAMETER_CM * 10.0 / 2.0  # raw hole radius, never touched by the trace
    coil_radius_mm = hole_radius_mm + TRACE_WIDTH_MM / 2.0 + HOLE_TRACE_CLEARANCE_MM
    outer_radius_mm = hole_radius_mm + BOARD_ANNULUS_WIDTH_MM
    n = NUM_SEGMENTS

    if coil_radius_mm + TRACE_WIDTH_MM / 2.0 >= outer_radius_mm:
        raise ValueError(
            "BOARD_ANNULUS_WIDTH_MM is too small to contain the coil trace; increase it."
        )

    cap_templates = {key: load_footprint(path) for key, path in CAP_FOOTPRINTS.items()}
    sma_template = load_footprint(SMA_FOOTPRINT_PATH)

    def bbox_width(template):
        min_x, _, max_x, _ = template.bbox_mm()
        return max_x - min_x

    def bbox_height(template):
        _, min_y, _, max_y = template.bbox_mm()
        return max_y - min_y

    generic_templates = [cap_templates[k] for k in GAP_CAPACITORS]
    generic_half_span_mm = (max(bbox_width(t) for t in generic_templates) / 2.0) if generic_templates else 1.0
    generic_half_angle_deg = angle_span(generic_half_span_mm + STUB_LENGTH_MM, coil_radius_mm)

    feed_left_t = cap_templates[FEED_CAP_LEFT]
    feed_right_t = cap_templates[FEED_CAP_RIGHT]
    feed_left_half = bbox_width(feed_left_t) / 2.0
    feed_right_half = bbox_width(feed_right_t) / 2.0
    # Same hole-clipping fix as the generic-gap stack below: anchor each feed capacitor so its
    # inner edge lands no further in than the trace's own inner edge, instead of centering it on
    # coil_radius_mm (which clips the hole if the capacitor's radial half-height > TRACE_WIDTH_MM/2).
    feed_left_radial_offset = bbox_height(feed_left_t) / 2.0 - TRACE_WIDTH_MM / 2.0
    feed_right_radial_offset = bbox_height(feed_right_t) / 2.0 - TRACE_WIDTH_MM / 2.0
    center_gap_half = FEED_CENTER_GAP_MM / 2.0
    feed_total_half_mm = center_gap_half + max(feed_left_half, feed_right_half) + STUB_LENGTH_MM
    # (both sides use the larger of the two so the gap is symmetric enough to place either cap)
    feed_half_angle_deg = angle_span(feed_total_half_mm, coil_radius_mm)

    def half_angle_for_gap(k):
        return feed_half_angle_deg if k % n == 0 else generic_half_angle_deg

    if 2 * (feed_half_angle_deg + generic_half_angle_deg) >= 2 * (360.0 / n):
        raise ValueError(
            "Gaps are too wide for the requested NUM_SEGMENTS at this hole size; "
            "increase HOLE_DIAMETER_CM, reduce NUM_SEGMENTS, or use smaller/fewer capacitors."
        )

    theta = [FEED_ANGLE_DEG + k * 360.0 / n for k in range(n + 1)]

    seg_nets = {}
    gnd_net = Net(name="GND")
    feed_net = Net(name="FEED")
    for i in range(1, n):
        seg_nets[i] = Net(name=f"SEG_{i}")
    seg_nets[n] = gnd_net

    all_items = []
    max_extent_mm = coil_radius_mm + TRACE_WIDTH_MM / 2.0
    ref_counter = [1]

    def next_cap_ref():
        r = f"C{ref_counter[0]}"
        ref_counter[0] += 1
        return r

    def track_extent(position_mm, rotation_deg, template):
        nonlocal max_extent_mm
        for corner in world_bbox_corners(template, position_mm, rotation_deg):
            d = math.hypot(corner[0] - LOOP_CENTER_MM[0], corner[1] - LOOP_CENTER_MM[1])
            max_extent_mm = max(max_extent_mm, d)

    def place_capacitor(key, position_mm, rotation_deg, backward_net, forward_net):
        template = cap_templates[key]
        inst = instantiate_footprint(
            template,
            position_mm=position_mm,
            rotation_deg=rotation_deg,
            reference=next_cap_ref(),
            net_by_pad={"1": backward_net, "2": forward_net},
        )
        all_items.append(inst)
        track_extent(position_mm, rotation_deg, template)
        pad1 = global_pad_position(template, "1", position_mm, rotation_deg)
        pad2 = global_pad_position(template, "2", position_mm, rotation_deg)
        return pad1, pad2

    def add_track(start_mm, end_mm, width_mm, net, layer=BoardLayer.BL_F_Cu):
        tr = Track()
        tr.start = Vector2.from_xy_mm(*start_mm)
        tr.end = Vector2.from_xy_mm(*end_mm)
        tr.width = from_mm(width_mm)
        tr.layer = layer
        tr.net = net
        all_items.append(tr)

    def add_arc(start_mm, mid_mm, end_mm, width_mm, net, layer=BoardLayer.BL_F_Cu):
        arc = ArcTrack()
        arc.start = Vector2.from_xy_mm(*start_mm)
        arc.mid = Vector2.from_xy_mm(*mid_mm)
        arc.end = Vector2.from_xy_mm(*end_mm)
        arc.width = from_mm(width_mm)
        arc.layer = layer
        arc.net = net
        all_items.append(arc)

    # --- segments ---------------------------------------------------------
    for i in range(1, n + 1):
        theta_start = theta[i - 1] + half_angle_for_gap(i - 1)
        theta_end = theta[i] - half_angle_for_gap(i)
        theta_mid = (theta_start + theta_end) / 2.0
        add_arc(
            polar(LOOP_CENTER_MM, coil_radius_mm, theta_start),
            polar(LOOP_CENTER_MM, coil_radius_mm, theta_mid),
            polar(LOOP_CENTER_MM, coil_radius_mm, theta_end),
            TRACE_WIDTH_MM,
            seg_nets[i],
        )

    # --- generic (capacitor-only) gaps -------------------------------------
    for k in range(1, n):
        theta_gap = theta[k]
        backward_net = seg_nets[k]
        forward_net = seg_nets[k + 1]
        rot = orientation_for_forward(theta_gap)

        edge_back = polar(LOOP_CENTER_MM, coil_radius_mm, theta_gap - half_angle_for_gap(k))
        edge_fwd = polar(LOOP_CENTER_MM, coil_radius_mm, theta_gap + half_angle_for_gap(k))

        stack = GAP_CAPACITORS
        heights = [bbox_height(cap_templates[key]) for key in stack]
        pitch = (max(heights) + CAP_STACK_CLEARANCE_MM) if len(stack) > 1 else 0.0
        # Anchor the stack so its innermost capacitor's inner edge lands no further in than the
        # trace's own inner edge (coil_radius_mm - TRACE_WIDTH_MM/2), instead of centering the
        # stack on coil_radius_mm. The trace's inner edge is already guaranteed clear of the hole
        # by HOLE_TRACE_CLEARANCE_MM, so anchoring here keeps every stacked capacitor clear too,
        # regardless of how much radially taller than the trace they are (previously, a capacitor
        # centered on coil_radius_mm with half-height > TRACE_WIDTH_MM/2 could reach in far enough
        # to clip the hole).
        base_radial_offset = heights[0] / 2.0 - TRACE_WIDTH_MM / 2.0
        for j, key in enumerate(stack):
            radial_offset = base_radial_offset + j * pitch
            pos = polar(LOOP_CENTER_MM, coil_radius_mm + radial_offset, theta_gap)
            pad1, pad2 = place_capacitor(key, pos, rot, backward_net, forward_net)
            add_track(edge_back, pad1, TRACE_WIDTH_MM, backward_net)
            add_track(pad2, edge_fwd, TRACE_WIDTH_MM, forward_net)

    # --- feed gap -----------------------------------------------------------
    theta_feed = theta[0]
    rot_feed = orientation_for_forward(theta_feed)

    edge_gnd_side = polar(LOOP_CENTER_MM, coil_radius_mm, theta_feed - half_angle_for_gap(0))  # -> seg[N], GND
    edge_seg1_side = polar(LOOP_CENTER_MM, coil_radius_mm, theta_feed + half_angle_for_gap(0))  # -> seg[1]

    left_offset = center_gap_half + feed_left_half
    right_offset = center_gap_half + feed_right_half
    pos_left = polar(
        LOOP_CENTER_MM, coil_radius_mm + feed_left_radial_offset, theta_feed + angle_span(left_offset, coil_radius_mm)
    )
    pos_right = polar(
        LOOP_CENTER_MM, coil_radius_mm + feed_right_radial_offset, theta_feed - angle_span(right_offset, coil_radius_mm)
    )

    # capA (feed-left, towards seg[1]): pad1 (backward/inward) -> FEED, pad2 (forward/outward) -> seg[1]
    capA_pad1, capA_pad2 = place_capacitor(FEED_CAP_LEFT, pos_left, rot_feed, feed_net, seg_nets[1])
    add_track(capA_pad2, edge_seg1_side, TRACE_WIDTH_MM, seg_nets[1])

    # capB (feed-right, towards seg[N]/GND): pad1 (backward/outward) -> GND, pad2 (forward/inward) -> FEED
    capB_pad1, capB_pad2 = place_capacitor(FEED_CAP_RIGHT, pos_right, rot_feed, gnd_net, feed_net)
    add_track(edge_gnd_side, capB_pad1, TRACE_WIDTH_MM, gnd_net)

    # bridge across the two feed caps' inner (FEED) pads, through the "centerpad" point
    add_track(capA_pad1, capB_pad2, TRACE_WIDTH_MM, feed_net)

    # --- SMA connector -------------------------------------------------------
    sma_reach_mm = sma_template.bbox_mm()[2]  # max local x = reach towards the coil once rotated
    standoff_mm = sma_reach_mm + SMA_STANDOFF_CLEARANCE_MM
    sma_position = polar(LOOP_CENTER_MM, coil_radius_mm + standoff_mm, theta_feed)
    # The SMA footprint's local -X axis is its cable-exit / mating-face direction (pad 1 sits at
    # the local origin, the connector body reaches out towards local +X). Point local -X (180)
    # at theta_feed so the connector's base orientation "points" away from the coil (down at the
    # default FEED_ANGLE_DEG=90), then apply SMA_CABLE_FLIP_DEG on top (180 = cable exits the
    # other way).
    sma_rotation = (orientation_pointing_local_dir(180.0, theta_feed) + SMA_CABLE_FLIP_DEG) % 360.0

    sma_net_by_pad = {"1": feed_net, "2": gnd_net}
    sma_inst = instantiate_footprint(
        sma_template, position_mm=sma_position, rotation_deg=sma_rotation, reference="J1", net_by_pad=sma_net_by_pad
    )
    all_items.append(sma_inst)
    # (not tracked via track_extent/max_extent_mm: the SMA is expected to extend past the plain
    # annulus -- it is covered by the rectangular tab built below instead)

    sma_pin1 = global_pad_position(sma_template, "1", sma_position, sma_rotation)
    num_gnd_pads = sum(1 for p in sma_template.pads if p.number == "2")
    gnd_positions = [
        global_pad_position(sma_template, "2", sma_position, sma_rotation, occurrence=idx)
        for idx in range(num_gnd_pads)
    ]

    # The "centerpad" point: exact midpoint of the feed bridge (capA_pad1 <-> capB_pad2). Using
    # the actual pad positions (rather than assuming radius == coil_radius_mm) keeps this correct
    # now that the feed capacitors are anchored off coil_radius_mm by their own radial offsets.
    feed_bottom = ((capA_pad1[0] + capB_pad2[0]) / 2.0, (capA_pad1[1] + capB_pad2[1]) / 2.0)
    add_track(feed_bottom, sma_pin1, SMA_TRACE_WIDTH_MM, feed_net)

    # Route the ground strap around the center pin (pad "1") rather than straight through it: from
    # each ground pad, detour radially outward (away from the coil) by past the pin's reach plus
    # clearance, then run the strap between the two detour points, parallel to the original
    # pad-to-pad line. This stays correct regardless of FEED_ANGLE_DEG / SMA_CABLE_FLIP_DEG.
    theta_feed_rad = math.radians(theta_feed)
    radial_unit = (math.cos(theta_feed_rad), math.sin(theta_feed_rad))
    pin1 = next(p for p in sma_template.pads if p.number == "1")
    pin1_reach_mm = max(pin1.w_mm, pin1.h_mm) / 2.0
    detour_offset_mm = pin1_reach_mm + GND_STRAP_WIDTH_MM / 2.0 + 1.0

    def radial_offset(pt, offset_mm):
        return (pt[0] + radial_unit[0] * offset_mm, pt[1] + radial_unit[1] * offset_mm)

    gnd_a, gnd_b = gnd_positions[0], gnd_positions[1]
    detour_a = radial_offset(gnd_a, detour_offset_mm)
    detour_b = radial_offset(gnd_b, detour_offset_mm)
    add_track(gnd_a, detour_a, GND_STRAP_WIDTH_MM, gnd_net)
    add_track(detour_a, detour_b, GND_STRAP_WIDTH_MM, gnd_net)
    add_track(detour_b, gnd_b, GND_STRAP_WIDTH_MM, gnd_net)

    # Radial position of the GND strap's outer copper edge -- the feed-side board tab is cut off
    # right here rather than reaching all the way out to the SMA's full mechanical extent (it is
    # an edge connector; the part of its body beyond the ground strap is expected to hang off the
    # board edge). Project onto radial_unit (not a plain distance-from-center) since the strap's
    # endpoints also carry a tangential offset (they sit either side of the center pin).
    gnd_strap_radius_mm = (
        (detour_a[0] - LOOP_CENTER_MM[0]) * radial_unit[0] + (detour_a[1] - LOOP_CENTER_MM[1]) * radial_unit[1]
    )
    gnd_strap_outer_edge_radius_mm = gnd_strap_radius_mm + GND_STRAP_WIDTH_MM / 2.0

    if max_extent_mm > outer_radius_mm:
        print(
            f"WARNING: some non-SMA components extend to {max_extent_mm:.2f} mm from center, "
            f"beyond the annulus outer radius of {outer_radius_mm:.2f} mm; increase "
            "BOARD_ANNULUS_WIDTH_MM."
        )

    # --- board outline: a donut (hole + outer boundary) with a notch at the feed gap replaced by
    # a rectangular tab bordering the SMA connector exactly (it is an edge-mount part) ------------
    #
    # The outer boundary must be ONE continuous closed loop -- not a full circle overlapped by a
    # separate closed rectangle, which leaves two independently-closed shapes for KiCad to
    # reconcile and is the wrong way to add a tab to a board edge. Instead: the small arc of the
    # outer circle that would pass through the tab's footprint is replaced by three straight
    # lines (the tab's two side walls and its outer edge), and the remaining major arc is split
    # in two at the point opposite the feed, per the requested "circle as two arcs" shape. The
    # hole remains a separate closed circle -- that's a normal, distinct inner cutout.
    hole = BoardCircle()
    hole.layer = BoardLayer.BL_Edge_Cuts
    hole.center = Vector2.from_xy_mm(*LOOP_CENTER_MM)
    hole.radius_point = Vector2.from_xy_mm(LOOP_CENTER_MM[0] + hole_radius_mm, LOOP_CENTER_MM[1])
    hole.attributes.stroke.width = from_mm(0.05)
    all_items.append(hole)

    # Work in (radial, tangential) coordinates about the feed angle so the tab's sides stay
    # parallel/perpendicular to the SMA regardless of FEED_ANGLE_DEG.
    tangent_unit = (-radial_unit[1], radial_unit[0])

    def to_radial_tangential(pt):
        dx, dy = pt[0] - LOOP_CENTER_MM[0], pt[1] - LOOP_CENTER_MM[1]
        return (dx * radial_unit[0] + dy * radial_unit[1], dx * tangent_unit[0] + dy * tangent_unit[1])

    def from_radial_tangential(r, t):
        return (
            LOOP_CENTER_MM[0] + r * radial_unit[0] + t * tangent_unit[0],
            LOOP_CENTER_MM[1] + r * radial_unit[1] + t * tangent_unit[1],
        )

    sma_rt = [to_radial_tangential(c) for c in world_bbox_corners(sma_template, sma_position, sma_rotation)]
    r_tab_outer = gnd_strap_outer_edge_radius_mm + SMA_EDGE_MARGIN_MM
    t_min = min(rt[1] for rt in sma_rt) - SMA_EDGE_MARGIN_MM
    t_max = max(rt[1] for rt in sma_rt) + SMA_EDGE_MARGIN_MM

    # Where the tab's two side walls (straight lines at constant t, running radially) cross the
    # outer circle: r^2 + t^2 = outer_radius_mm^2, so r = sqrt(outer_radius_mm^2 - t^2) there.
    r_at_t_min = math.sqrt(outer_radius_mm ** 2 - t_min ** 2)
    r_at_t_max = math.sqrt(outer_radius_mm ** 2 - t_max ** 2)
    notch_start_deg = theta_feed + math.degrees(math.asin(t_min / outer_radius_mm))
    notch_end_deg = theta_feed + math.degrees(math.asin(t_max / outer_radius_mm))
    opposite_deg = theta_feed + 180.0

    def add_edge_line(a_mm, b_mm):
        seg = BoardSegment()
        seg.layer = BoardLayer.BL_Edge_Cuts
        seg.start = Vector2.from_xy_mm(*a_mm)
        seg.end = Vector2.from_xy_mm(*b_mm)
        seg.attributes.stroke.width = from_mm(0.05)
        all_items.append(seg)

    def add_edge_arc(start_deg, end_deg):
        arc = BoardArc()
        arc.layer = BoardLayer.BL_Edge_Cuts
        mid_deg = (start_deg + end_deg) / 2.0
        arc.start = Vector2.from_xy_mm(*polar(LOOP_CENTER_MM, outer_radius_mm, start_deg))
        arc.mid = Vector2.from_xy_mm(*polar(LOOP_CENTER_MM, outer_radius_mm, mid_deg))
        arc.end = Vector2.from_xy_mm(*polar(LOOP_CENTER_MM, outer_radius_mm, end_deg))
        arc.attributes.stroke.width = from_mm(0.05)
        all_items.append(arc)

    # Notch: circle -> out to the tab's outer corner -> across -> back in to the circle.
    add_edge_line(from_radial_tangential(r_at_t_min, t_min), from_radial_tangential(r_tab_outer, t_min))
    add_edge_line(from_radial_tangential(r_tab_outer, t_min), from_radial_tangential(r_tab_outer, t_max))
    add_edge_line(from_radial_tangential(r_tab_outer, t_max), from_radial_tangential(r_at_t_max, t_max))

    # The rest of the circle, split into two arcs at the point opposite the feed.
    add_edge_arc(notch_end_deg, opposite_deg)
    add_edge_arc(opposite_deg, notch_start_deg + 360.0)

    # --- commit ---------------------------------------------------------------
    commit = board.begin_commit()
    try:
        board.create_items(all_items)
        board.push_commit(commit, "Generate loop coil")
    except Exception:
        board.drop_commit(commit)
        raise

    print(f"Created {len(all_items)} items: {n} segments, {n - 1} generic gap(s), feed gap, SMA J1.")
    print(f"Coil radius: {coil_radius_mm:.2f} mm, board outer radius: {outer_radius_mm:.2f} mm.")


if __name__ == "__main__":
    main()
