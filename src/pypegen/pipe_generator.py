"""
Parametric Pipe & Fitting Generator using CadQuery.

This module provides a complete system for generating 3D pipe runs and fittings
parametrically, without relying on pre-made STEP files.
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal

import cadquery as cq

from .fittings.npt_thread import get_npt_spec
from .fittings.socket_weld_elbow import ASME_B1611_ELBOW90_CLASS3000

# =============================================================================
# CONSTANTS
# =============================================================================

INCH = 25.4  # mm per inch
FT = 12 * INCH  # mm per foot

# =============================================================================
# DIRECTION VECTORS
# =============================================================================

Direction = Literal["EAST", "WEST", "NORTH", "SOUTH", "UP", "DOWN"]

DIR_VECTORS: dict[Direction, cq.Vector] = {
    "EAST": cq.Vector(1, 0, 0),
    "WEST": cq.Vector(-1, 0, 0),
    "NORTH": cq.Vector(0, 1, 0),
    "SOUTH": cq.Vector(0, -1, 0),
    "UP": cq.Vector(0, 0, 1),
    "DOWN": cq.Vector(0, 0, -1),
}

# =============================================================================
# CORE DATA STRUCTURES
# =============================================================================


@dataclass
class Move:
    """Represents a single move in a pipe route."""

    direction: Direction
    length: float  # mm


@dataclass
class Segment:
    """A pipe segment from start to end point."""

    start: cq.Vector
    end: cq.Vector

    @property
    def direction(self) -> cq.Vector:
        """Unit vector from start to end."""
        v = self.end - self.start
        return v.normalized()

    @property
    def length(self) -> float:
        """Length of the segment in mm."""
        return (self.end - self.start).Length


@dataclass
class BranchEvent:
    """Branch off the main run at distance s from start of a given segment."""

    segment_index: int  # which segment in the route
    s: float  # distance along that segment from its start (mm)
    direction: cq.Vector  # branch direction in world coords
    fitting_name: str  # key into fitting registry
    branch_line_id: str | None = None  # for a child line route


@dataclass
class PipeSpec:
    """Specification for a pipe size and schedule."""

    name: str
    od: float  # outer diameter (mm)
    id: float  # inner diameter (mm)
    material: str = "A312-TP316L"


# Pipe specifications table
PIPE_SPECS: dict[tuple[str, str], PipeSpec] = {
    ("1/2in", "40"): PipeSpec("NPS0.5_SCH40", od=0.840 * INCH, id=0.622 * INCH),
    ("3/4in", "40"): PipeSpec("NPS0.75_SCH40", od=1.050 * INCH, id=0.824 * INCH),
    ("1in", "40"): PipeSpec("NPS1_SCH40", od=1.315 * INCH, id=1.049 * INCH),
    ("1.5in", "40"): PipeSpec("NPS1.5_SCH40", od=1.900 * INCH, id=1.610 * INCH),
    ("2in", "40"): PipeSpec("NPS2_SCH40", od=2.375 * INCH, id=2.067 * INCH),
    ("2.5in", "40"): PipeSpec("NPS2.5_SCH40", od=2.875 * INCH, id=2.469 * INCH),
    ("3in", "40"): PipeSpec("NPS3_SCH40", od=3.500 * INCH, id=3.068 * INCH),
    ("4in", "40"): PipeSpec("NPS4_SCH40", od=4.500 * INCH, id=4.026 * INCH),
    ("6in", "40"): PipeSpec("NPS6_SCH40", od=6.625 * INCH, id=6.065 * INCH),
    ("8in", "40"): PipeSpec("NPS8_SCH40", od=8.625 * INCH, id=7.981 * INCH),
}


@dataclass
class PortDef:
    """Port definition for fitting connections."""

    name: str
    origin_local: cq.Vector  # location in fitting's local CS
    axis_local: cq.Vector  # unit vector for pipe direction (outward)
    radius: float  # mating radius (pipe OD/2 or socket bore)
    socket_depth: float = 0.0  # for socket welds (mm)
    socket_gap: float = 0.0  # weld gap (mm)
    kind: Literal["butt", "socket"] = "butt"


@dataclass
class FittingSpec:
    """Specification for a parametric fitting."""

    name: str
    type: Literal["elbow90", "coupling", "flange", "branch"]
    pipe_spec: PipeSpec
    params: dict
    ports: list[PortDef]
    make_solid: Callable[[PipeSpec, dict], cq.Shape]


# Global fitting registry
FITTINGS: dict[str, FittingSpec] = {}


# =============================================================================
# ROUTE BUILDING
# =============================================================================


def build_segments(
    start: tuple[float, float, float], moves: list[Move]
) -> list[Segment]:
    """Convert a list of moves into segments."""
    p = cq.Vector(*start)
    segs: list[Segment] = []
    for mv in moves:
        d = DIR_VECTORS[mv.direction]
        new_p = p + d * mv.length
        segs.append(Segment(start=p, end=new_p))
        p = new_p
    return segs


# =============================================================================
# PIPE GEOMETRY
# =============================================================================


def make_pipe_solid(seg: Segment, spec: PipeSpec) -> cq.Shape:
    """Create a straight pipe segment solid."""
    v = seg.end - seg.start
    length = v.Length
    if length <= 0:
        raise ValueError("Zero-length segment")

    axis = v.normalized()
    ro = spec.od / 2.0
    ri = spec.id / 2.0

    # Convert Vector to tuple for CadQuery API
    start_tuple = (seg.start.x, seg.start.y, seg.start.z)
    axis_tuple = (axis.x, axis.y, axis.z)

    outer = cq.Solid.makeCylinder(ro, length, pnt=cq.Vector(*start_tuple), dir=cq.Vector(*axis_tuple))
    if ri > 0:
        inner = cq.Solid.makeCylinder(ri, length, pnt=cq.Vector(*start_tuple), dir=cq.Vector(*axis_tuple))
        return outer.cut(inner)
    return outer


def trim_segment_for_socket(seg: Segment, at_start: bool, port: PortDef) -> Segment:
    """Trim segment for socket weld insertion."""
    if port.kind != "socket":
        return seg

    insertion = port.socket_depth - port.socket_gap
    if insertion <= 0:
        return seg

    dir_vec = seg.direction
    if at_start:
        new_start = seg.start + dir_vec * insertion
        return Segment(start=new_start, end=seg.end)
    else:
        new_end = seg.end - dir_vec * insertion
        return Segment(start=seg.start, end=new_end)


# NPS to ASME size mapping for threaded pipe
NPS_TO_ASME_THREADED: dict[str, str] = {
    "1/8in": "1/8",
    "1/4in": "1/4",
    "3/8in": "3/8",
    "1/2in": "1/2",
    "3/4in": "3/4",
    "1in": "1",
    "1.25in": "1-1/4",
    "1-1/4in": "1-1/4",
    "1.5in": "1-1/2",
    "1-1/2in": "1-1/2",
    "2in": "2",
    "2.5in": "2-1/2",
    "2-1/2in": "2-1/2",
    "3in": "3",
    "4in": "4",
}


def make_threaded_pipe(
    nps: str,
    length: float,
    thread_end: Literal["none", "inlet", "outlet", "both"] = "both",
    include_threads: bool = True,
) -> cq.Shape:
    """
    Create a pipe with NPT external threads on one or both ends.

    The pipe is oriented with:
    - Axis along +Z
    - Inlet (start) at Z=0
    - Outlet (end) at Z=length

    Uses an ADDITIVE approach: builds pipe from sections (threaded ends + plain middle)
    and fuses them together. This is more reliable than the subtractive approach
    which can fail due to OCCT Boolean operation issues.

    Args:
        nps: Nominal pipe size. Can be ASME format ("1/2", "1", "2")
             or inch format ("1/2in", "1in", "2in")
        length: Total length of pipe in mm
        thread_end: Which ends to thread:
            - "none": No threads (plain pipe)
            - "inlet": Thread only the Z=0 end
            - "outlet": Thread only the Z=length end
            - "both": Thread both ends (default)
        include_threads: If True, add actual NPT thread geometry.
                        If False, return plain pipe (same as thread_end="none")

    Returns:
        CadQuery Shape of the threaded pipe
    """
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer

    from .fittings.npt_thread import (
        NPT_HALF_ANGLE_DEG,
        NPT_TAPER_RATIO,
        _make_tapered_cylinder,
        _make_thread_solid,
    )

    def extract_solid(shape: cq.Shape) -> cq.Solid | None:
        """Extract the largest solid from a shape (handles Compounds)."""
        if isinstance(shape, cq.Solid):
            return shape
        if hasattr(shape, "wrapped"):
            exp = TopExp_Explorer(shape.wrapped, TopAbs_SOLID)
        else:
            exp = TopExp_Explorer(shape, TopAbs_SOLID)
        solids = []
        while exp.More():
            solids.append(cq.Solid(exp.Current()))
            exp.Next()
        if not solids:
            return None
        return max(solids, key=lambda s: s.Volume())

    # Normalize NPS format
    asme_nps = NPS_TO_ASME_THREADED.get(nps, nps)

    # Get NPT spec for dimensions
    npt_spec = get_npt_spec(asme_nps)

    # Pipe dimensions
    pipe_od = npt_spec.od
    wall_thickness = npt_spec.thread_depth * 2  # Approximate
    pipe_id = pipe_od - 2 * wall_thickness

    r_outer = pipe_od / 2.0
    r_inner = pipe_id / 2.0

    # If no threads requested, return plain pipe
    if not include_threads or thread_end == "none":
        outer_cyl = cq.Solid.makeCylinder(r_outer, length, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
        inner_cyl = cq.Solid.makeCylinder(r_inner, length, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
        result = outer_cyl.cut(inner_cyl)
        return extract_solid(result) or result

    thread_length = npt_spec.L2  # Effective thread length
    pitch = npt_spec.pitch_mm
    thread_depth = npt_spec.thread_depth

    inlet_threaded = thread_end in ("inlet", "both")
    outlet_threaded = thread_end in ("outlet", "both")

    # Calculate thread radii at small end (Z=0 for inlet, Z=length for outlet)
    apex_start = r_outer - (thread_length * NPT_TAPER_RATIO / 2.0)
    root_start = apex_start - thread_depth

    # ADDITIVE APPROACH: Build pipe from sections and fuse them together
    # This avoids OCCT Boolean cut issues with transformed thread geometry

    # Calculate section positions
    inlet_length = thread_length if inlet_threaded else 0
    outlet_length = thread_length if outlet_threaded else 0
    middle_start = inlet_length
    middle_length = length - inlet_length - outlet_length

    pipe = None

    # 1. Create inlet section (threaded or plain)
    if inlet_threaded:
        # Thread core (tapered cylinder from root to expanding)
        inlet_core = _make_tapered_cylinder(
            root_start, root_start + thread_length * NPT_TAPER_RATIO / 2.0, thread_length
        )
        inlet_bore = cq.Solid.makeCylinder(r_inner, thread_length, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
        inlet_core = extract_solid(inlet_core.cut(inlet_bore))

        # Thread teeth
        inlet_teeth = _make_thread_solid(
            apex_radius=apex_start,
            root_radius=root_start,
            pitch=pitch,
            length=thread_length,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("fade", "fade"),  # Use fade for proper thread runout
        )

        inlet_section = extract_solid(inlet_core.fuse(inlet_teeth))
        pipe = inlet_section
    else:
        # Plain inlet (if outlet-only threading)
        if inlet_length > 0:
            inlet_cyl = cq.Solid.makeCylinder(r_outer, inlet_length, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
            inlet_bore = cq.Solid.makeCylinder(r_inner, inlet_length, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
            pipe = extract_solid(inlet_cyl.cut(inlet_bore))

    # 2. Create middle section (plain cylinder)
    if middle_length > 0:
        middle_cyl = cq.Solid.makeCylinder(
            r_outer, middle_length, cq.Vector(0, 0, middle_start), cq.Vector(0, 0, 1)
        )
        middle_bore = cq.Solid.makeCylinder(
            r_inner, middle_length, cq.Vector(0, 0, middle_start), cq.Vector(0, 0, 1)
        )
        middle_section = extract_solid(middle_cyl.cut(middle_bore))

        if pipe is None:
            pipe = middle_section
        else:
            pipe = extract_solid(pipe.fuse(middle_section))

    # 3. Create outlet section (threaded or plain)
    if outlet_threaded:
        # Thread core (tapered cylinder from root to expanding)
        outlet_core = _make_tapered_cylinder(
            root_start, root_start + thread_length * NPT_TAPER_RATIO / 2.0, thread_length
        )
        outlet_bore = cq.Solid.makeCylinder(r_inner, thread_length, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
        outlet_core = extract_solid(outlet_core.cut(outlet_bore))

        # Thread teeth
        outlet_teeth = _make_thread_solid(
            apex_radius=apex_start,
            root_radius=root_start,
            pitch=pitch,
            length=thread_length,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("fade", "fade"),  # Use fade for proper thread runout
        )

        outlet_section = extract_solid(outlet_core.fuse(outlet_teeth))

        # Flip so free end (small diameter) is at Z=length
        outlet_section = outlet_section.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
        outlet_section = outlet_section.translate(cq.Vector(0, 0, length))

        if pipe is None:
            pipe = outlet_section
        else:
            pipe = extract_solid(pipe.fuse(outlet_section))

    return pipe


# =============================================================================
# FITTING GEOMETRY GENERATORS
# =============================================================================


def make_socket_coupling_solid(pipe: PipeSpec, params: dict) -> cq.Shape:
    """
    Create a socket coupling solid.

    params:
      body_len: total length of coupling
      socket_depth: depth of socket from each end
      wall_thickness: radial thickness beyond pipe OD
      chamfer: optional edge chamfer at ends
    """
    body_len = params["body_len"]
    socket_depth = params["socket_depth"]
    wall_thickness = params.get("wall_thickness", 0.25 * INCH)

    ro_pipe = pipe.od / 2.0
    ro_body = ro_pipe + wall_thickness
    r_socket = ro_pipe + 0.1  # small clearance

    # Base body centered on origin, aligned with X axis
    body = cq.Solid.makeCylinder(
        ro_body,
        body_len,
        cq.Vector(-body_len / 2, 0, 0),
        cq.Vector(1, 0, 0),
    )

    # Counterbore sockets from both ends
    socket_left = cq.Solid.makeCylinder(
        r_socket,
        socket_depth,
        cq.Vector(-body_len / 2, 0, 0),
        cq.Vector(1, 0, 0),
    )
    socket_right = cq.Solid.makeCylinder(
        r_socket,
        socket_depth,
        cq.Vector(body_len / 2, 0, 0),
        cq.Vector(-1, 0, 0),
    )

    coupling = body.cut(socket_left).cut(socket_right)
    return coupling


def define_socket_coupling(pipe: PipeSpec) -> FittingSpec:
    """Define a socket coupling fitting for the given pipe spec."""
    body_len = 2.0 * INCH
    socket_depth = 0.25 * INCH

    params = {
        "body_len": body_len,
        "socket_depth": socket_depth,
        "wall_thickness": 0.25 * INCH,
        "chamfer": 0.03 * INCH,
    }

    ro_pipe = pipe.od / 2.0

    ports = [
        PortDef(
            name="A",
            origin_local=cq.Vector(-body_len / 2 + socket_depth, 0, 0),
            axis_local=cq.Vector(1, 0, 0),
            radius=ro_pipe,
            socket_depth=socket_depth,
            socket_gap=0.06 * INCH,
            kind="socket",
        ),
        PortDef(
            name="B",
            origin_local=cq.Vector(body_len / 2 - socket_depth, 0, 0),
            axis_local=cq.Vector(-1, 0, 0),
            radius=ro_pipe,
            socket_depth=socket_depth,
            socket_gap=0.06 * INCH,
            kind="socket",
        ),
    ]

    return FittingSpec(
        name=f"{pipe.name}_SOCKET_COUPLING",
        type="coupling",
        pipe_spec=pipe,
        params=params,
        ports=ports,
        make_solid=make_socket_coupling_solid,
    )


# Map pipe spec names to ASME B16.11 size keys
PIPE_TO_ASME_SIZE: dict[str, str] = {
    "NPS0.5_SCH40": "1/2",
    "NPS0.75_SCH40": "3/4",
    "NPS1_SCH40": "1",
    "NPS1.5_SCH40": "1-1/2",
    "NPS2_SCH40": "2",
    "NPS2.5_SCH40": "2-1/2",
    "NPS3_SCH40": "3",
    "NPS4_SCH40": "4",
}


def make_asme_socket_elbow90_solid(_pipe: PipeSpec, params: dict) -> cq.Shape:
    """
    Create an ASME B16.11 socket weld 90-degree elbow solid.

    This creates a proper forged socket weld elbow with:
    - Correct center-to-end dimension (A)
    - Socket bore sized for pipe OD
    - Socket counterbores at each end
    - Proper swept bore through the fitting
    - Straight tangent legs at each end for the socket

    The elbow is oriented with:
    - Port A on the +X axis (pipe enters from +X direction)
    - Port B on the +Y axis (pipe enters from +Y direction)
    - The bend center at the origin

    params:
      center_to_end: A dimension - center to bottom of socket (mm)
      socket_bore: B dimension - socket bore diameter (mm)
      socket_depth: J dimension - depth of socket (mm)
      min_bore: D dimension - minimum flow bore diameter (mm)
      body_wall: G dimension - body wall thickness (mm)
    """
    import math

    A = params["center_to_end"]
    socket_bore = params["socket_bore"]
    socket_depth = params["socket_depth"]
    min_bore = params["min_bore"]
    body_wall = params["body_wall"]

    # Radii
    r_socket = socket_bore / 2.0  # Socket bore radius (accepts pipe OD)
    r_bore = min_bore / 2.0  # Flow bore radius
    r_outer = r_socket + body_wall  # Outer radius of fitting body

    # For a 90° elbow with center-to-end dimension A:
    # We'll use a swept circular profile along a 90° arc

    # The bend radius determines how "tight" the elbow is
    # For socket weld fittings, the bend is typically tight
    bend_radius = r_outer * 1.2

    # Ensure we have room for the socket
    leg_length = A - bend_radius
    if leg_length < socket_depth + 1:
        leg_length = socket_depth + 1
        bend_radius = A - leg_length

    # Ensure bend radius is at least as big as the outer radius
    if bend_radius < r_outer + 1:
        bend_radius = r_outer + 1

    # Build the elbow in the XY plane:
    # - Leg 1 runs along +X axis
    # - Leg 2 runs along +Y axis
    # - The bend connects them with a 90° arc
    # - The arc center is at the origin
    # - The arc centerline is at radius = bend_radius from origin

    # Straight leg along +X (from socket end toward the bend)
    leg_x_outer = cq.Solid.makeCylinder(
        r_outer,
        leg_length,
        cq.Vector(bend_radius, 0, 0),
        cq.Vector(1, 0, 0),
    )
    leg_x_bore = cq.Solid.makeCylinder(
        r_bore,
        leg_length + 1,
        cq.Vector(bend_radius - 0.5, 0, 0),
        cq.Vector(1, 0, 0),
    )

    # Straight leg along +Y (from socket end toward the bend)
    leg_y_outer = cq.Solid.makeCylinder(
        r_outer,
        leg_length,
        cq.Vector(0, bend_radius, 0),
        cq.Vector(0, 1, 0),
    )
    leg_y_bore = cq.Solid.makeCylinder(
        r_bore,
        leg_length + 1,
        cq.Vector(0, bend_radius - 0.5, 0),
        cq.Vector(0, 1, 0),
    )

    # Create the 90° bend by sweeping a circular profile along an arc path
    # The arc is in the XY plane, connecting the two legs
    mid_angle = math.pi / 4
    arc_path = (
        cq.Workplane("XY")
        .moveTo(bend_radius, 0)
        .threePointArc(
            (bend_radius * math.cos(mid_angle), bend_radius * math.sin(mid_angle)),
            (0, bend_radius)
        )
    )

    # The profile circle needs to be perpendicular to the arc at the start point.
    # At (bend_radius, 0), the arc tangent is in the +Y direction.
    # So the profile plane should be the XZ plane (perpendicular to Y).
    # The circle should be centered at (bend_radius, 0, 0).

    # Create profile for outer sweep
    outer_profile = (
        cq.Workplane("XZ")
        .transformed(offset=(bend_radius, 0, 0))
        .circle(r_outer)
    )

    # Sweep the profile along the arc
    outer_bend = outer_profile.sweep(arc_path, isFrenet=True)

    # Create profile for bore sweep
    bore_profile = (
        cq.Workplane("XZ")
        .transformed(offset=(bend_radius, 0, 0))
        .circle(r_bore)
    )

    bore_bend = bore_profile.sweep(arc_path, isFrenet=True)

    # Socket counterbores at the ends (larger diameter than bore to accept pipe OD)
    socket_x = cq.Solid.makeCylinder(
        r_socket,
        socket_depth,
        cq.Vector(A, 0, 0),
        cq.Vector(-1, 0, 0),
    )
    socket_y = cq.Solid.makeCylinder(
        r_socket,
        socket_depth,
        cq.Vector(0, A, 0),
        cq.Vector(0, -1, 0),
    )

    # Combine outer body: swept bend + straight legs
    outer_solid_obj = outer_bend.solids().val()
    assert isinstance(outer_solid_obj, cq.Solid)
    outer_body = outer_solid_obj.fuse(leg_x_outer).fuse(leg_y_outer)

    # Combine bore: swept bend + straight legs
    bore_solid_obj = bore_bend.solids().val()
    assert isinstance(bore_solid_obj, cq.Solid)
    bore_body = bore_solid_obj.fuse(leg_x_bore).fuse(leg_y_bore)

    # Create the elbow: outer body - bore - socket counterbores
    elbow = outer_body.cut(bore_body).cut(socket_x).cut(socket_y)

    return elbow


def define_asme_socket_elbow90(pipe: PipeSpec, pressure_class: int = 3000) -> FittingSpec:
    """
    Define an ASME B16.11 socket weld 90-degree elbow for the given pipe spec.

    Args:
        pipe: The pipe specification
        pressure_class: 3000 or 6000 (only 3000 currently implemented)
    """
    # Look up ASME dimensions
    asme_size = PIPE_TO_ASME_SIZE.get(pipe.name)
    if asme_size is None:
        raise ValueError(f"No ASME B16.11 dimensions for pipe {pipe.name}")

    if pressure_class == 3000:
        dims = ASME_B1611_ELBOW90_CLASS3000.get(asme_size)
    else:
        raise ValueError(f"Pressure class {pressure_class} not implemented")

    if dims is None:
        raise ValueError(f"No ASME B16.11 Class {pressure_class} dimensions for size {asme_size}")

    # Use average socket bore
    socket_bore = (dims.socket_bore_max + dims.socket_bore_min) / 2.0

    params = {
        "center_to_end": dims.center_to_end,
        "socket_bore": socket_bore,
        "socket_depth": dims.socket_depth,
        "min_bore": dims.min_bore,
        "body_wall": dims.body_wall_thickness,
        "socket_wall": dims.socket_wall_thickness,
    }

    # Port locations at the socket ends
    # Port A: at +X end, pipe enters in -X direction
    # Port B: at +Y end, pipe enters in -Y direction
    A = dims.center_to_end

    ports = [
        PortDef(
            name="A",
            origin_local=cq.Vector(A - dims.socket_depth, 0, 0),
            axis_local=cq.Vector(1, 0, 0),
            radius=pipe.od / 2.0,
            socket_depth=dims.socket_depth,
            socket_gap=1.5,  # ~1/16" gap per ASME
            kind="socket",
        ),
        PortDef(
            name="B",
            origin_local=cq.Vector(0, A - dims.socket_depth, 0),
            axis_local=cq.Vector(0, 1, 0),
            radius=pipe.od / 2.0,
            socket_depth=dims.socket_depth,
            socket_gap=1.5,
            kind="socket",
        ),
    ]

    return FittingSpec(
        name=f"{pipe.name}_ELBOW90_SW_CL{pressure_class}",
        type="elbow90",
        pipe_spec=pipe,
        params=params,
        ports=ports,
        make_solid=make_asme_socket_elbow90_solid,
    )


# Keep the old simplified elbow for backwards compatibility
def make_socket_elbow90_solid(pipe: PipeSpec, params: dict) -> cq.Shape:
    """
    Create a simplified socket 90-degree elbow solid (non-ASME).

    params:
      centerline_radius: radius of the bend centerline
      socket_depth: depth of socket in each leg
      wall_thickness: radial wall beyond pipe OD
    """
    r_cl = params["centerline_radius"]
    wall_thickness = params.get("wall_thickness", 0.25 * INCH)

    ro_pipe = pipe.od / 2.0
    ri_pipe = pipe.id / 2.0

    # Create the elbow using a sweep along a 90-degree arc
    # The arc is in the XY plane, centered at origin, from +X to +Y direction

    # Create a workplane for the path
    path = (
        cq.Workplane("XY")
        .moveTo(r_cl, 0)
        .threePointArc((r_cl * 0.7071, r_cl * 0.7071), (0, r_cl))
    )

    # Create the outer profile
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=r_cl)
        .circle(ro_pipe + wall_thickness)
        .sweep(path, isFrenet=True)
    )

    # Create the bore (inner) profile
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=r_cl)
        .circle(ri_pipe)
        .sweep(path, isFrenet=True)
    )

    elbow = outer.cut(bore)
    result = elbow.solids().val()
    assert isinstance(result, cq.Shape)
    return result


def define_socket_elbow90(pipe: PipeSpec) -> FittingSpec:
    """Define a simplified socket 90-degree elbow (non-ASME) for the given pipe spec."""
    r_cl = 1.5 * pipe.od  # centerline radius
    socket_depth = 0.25 * INCH
    wall_thickness = 0.25 * INCH

    params = {
        "centerline_radius": r_cl,
        "socket_depth": socket_depth,
        "wall_thickness": wall_thickness,
    }

    ro_pipe = pipe.od / 2.0

    ports = [
        PortDef(
            name="A",
            origin_local=cq.Vector(r_cl, 0, 0),
            axis_local=cq.Vector(1, 0, 0),
            radius=ro_pipe,
            socket_depth=socket_depth,
            socket_gap=0.06 * INCH,
            kind="socket",
        ),
        PortDef(
            name="B",
            origin_local=cq.Vector(0, r_cl, 0),
            axis_local=cq.Vector(0, 1, 0),
            radius=ro_pipe,
            socket_depth=socket_depth,
            socket_gap=0.06 * INCH,
            kind="socket",
        ),
    ]

    return FittingSpec(
        name=f"{pipe.name}_ELBOW90_SWT",
        type="elbow90",
        pipe_spec=pipe,
        params=params,
        ports=ports,
        make_solid=make_socket_elbow90_solid,
    )


def make_branch_half_coupling_solid(pipe: PipeSpec, params: dict) -> cq.Shape:
    """
    Create a branch half-coupling solid.

    params:
      pad_radius: radius of the round pad that sits on run pipe
      pad_thickness: thickness of pad
      socket_depth: depth of branch socket
      neck_length: length of socket neck beyond pad
      wall_thickness: wall thickness around branch OD
    """
    pad_radius = params["pad_radius"]
    pad_thickness = params["pad_thickness"]
    socket_depth = params["socket_depth"]
    neck_length = params["neck_length"]
    wall_thickness = params.get("wall_thickness", 0.25 * INCH)

    ro_pipe = pipe.od / 2.0
    r_branch_socket = ro_pipe + 0.1  # clearance
    r_branch_outer = r_branch_socket + wall_thickness

    # Pad: disc on XY plane, centered at origin, thickness along -Z
    pad = cq.Solid.makeCylinder(
        pad_radius,
        pad_thickness,
        cq.Vector(0, 0, -pad_thickness),
        cq.Vector(0, 0, 1),
    )

    # Socket + neck along +Z from pad top
    total_branch_len = socket_depth + neck_length

    outer = cq.Solid.makeCylinder(
        r_branch_outer,
        total_branch_len,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
    )
    inner = cq.Solid.makeCylinder(
        r_branch_socket,
        socket_depth,
        cq.Vector(0, 0, neck_length),
        cq.Vector(0, 0, 1),
    )

    branch_body = outer.cut(inner)

    # Through-hole for the bore
    bore = cq.Solid.makeCylinder(
        pipe.id / 2.0,
        total_branch_len + pad_thickness,
        cq.Vector(0, 0, -pad_thickness),
        cq.Vector(0, 0, 1),
    )

    combined = cq.Compound.makeCompound([pad, branch_body])
    return combined.cut(bore)


def define_branch_half_coupling(pipe: PipeSpec) -> FittingSpec:
    """Define a branch half-coupling for the given pipe spec."""
    socket_depth = 0.25 * INCH
    neck_length = 1.5 * INCH

    params = {
        "pad_radius": 1.5 * pipe.od,
        "pad_thickness": 0.5 * INCH,
        "socket_depth": socket_depth,
        "neck_length": neck_length,
        "wall_thickness": 0.25 * INCH,
    }

    ro_pipe = pipe.od / 2.0

    ports = [
        PortDef(
            name="BRANCH",
            origin_local=cq.Vector(0, 0, neck_length + socket_depth),
            axis_local=cq.Vector(0, 0, 1),
            radius=ro_pipe,
            socket_depth=socket_depth,
            socket_gap=0.06 * INCH,
            kind="socket",
        )
    ]

    return FittingSpec(
        name=f"{pipe.name}_HALF_COUPLING_BRANCH",
        type="branch",
        pipe_spec=pipe,
        params=params,
        ports=ports,
        make_solid=make_branch_half_coupling_solid,
    )


# =============================================================================
# FITTING PLACEMENT & TRANSFORMATION
# =============================================================================


def rotation_from_axes(xg: cq.Vector, yg: cq.Vector) -> cq.Matrix:
    """Create a rotation matrix from X and Y axis vectors."""
    xg = xg.normalized()
    yg = yg.normalized()
    zg = xg.cross(yg).normalized()

    # CadQuery Matrix constructor: row-major order
    return cq.Matrix(
        [
            [xg.x, yg.x, zg.x, 0],
            [xg.y, yg.y, zg.y, 0],
            [xg.z, yg.z, zg.z, 0],
            [0, 0, 0, 1],
        ]
    )


def branch_world_frame(
    seg: Segment, s: float, branch_dir: cq.Vector, run_od: float
) -> tuple[cq.Vector, cq.Matrix]:
    """
    Calculate world coordinate frame for a branch on a run pipe.

    Returns:
        surface_point: Point on the pipe surface where branch attaches
        rot: Rotation matrix for the branch orientation
    """
    run_dir = seg.direction
    branch_dir = branch_dir.normalized()

    cl_point = seg.start + run_dir * s
    surface_point = cl_point + branch_dir * (run_od / 2.0)

    z_global = branch_dir
    x_global = run_dir
    y_global = z_global.cross(x_global).normalized()

    rot = rotation_from_axes(x_global, y_global)
    return surface_point, rot


def transform_shape(shape: cq.Shape, _matrix: cq.Matrix, translation: cq.Vector) -> cq.Shape:
    """Transform a shape using a rotation matrix and translation."""
    # Create a location from the matrix and translation
    loc = cq.Location(cq.Vector(translation.x, translation.y, translation.z))
    return shape.moved(loc)


def place_branch_fitting(
    spec: FittingSpec, seg: Segment, event: BranchEvent, run_od: float
) -> cq.Shape:
    """Place a branch fitting on a run pipe."""
    surface_point, _rot = branch_world_frame(seg, event.s, event.direction, run_od)

    # Generate the solid
    solid = spec.make_solid(spec.pipe_spec, spec.params)

    # For now, we'll use a simpler placement approach
    # The fitting's Z axis should align with branch direction
    _branch_dir = event.direction.normalized()  # Reserved for future rotation

    # Create location for the fitting
    loc = cq.Location(cq.Vector(surface_point.x, surface_point.y, surface_point.z))

    return solid.moved(loc)


def cut_branch_hole(
    run_pipe_solid: cq.Shape,
    seg: Segment,
    event: BranchEvent,
    branch_od: float,
    extra: float = 2.0 * INCH,
) -> cq.Shape:
    """Cut a branch hole in the run pipe."""
    run_dir = seg.direction
    cl_point = seg.start + run_dir * event.s
    bdir = event.direction.normalized()

    start = cl_point - bdir * extra
    length = 2 * extra

    cut_cyl = cq.Solid.makeCylinder(
        branch_od / 2.0,
        length,
        cq.Vector(start.x, start.y, start.z),
        cq.Vector(bdir.x, bdir.y, bdir.z),
    )
    return run_pipe_solid.cut(cut_cyl)


# =============================================================================
# EXPORT
# =============================================================================


def export_spool_to_step(solids: list[cq.Shape], filename: str):
    """Export a list of solids to a STEP file."""
    compound = cq.Compound.makeCompound(solids)
    cq.exporters.export(compound, filename)


# =============================================================================
# END-TO-END EXAMPLE
# =============================================================================


def example_spool_with_branch():
    """Create an example spool with a branch connection."""
    # Pipe spec
    pipe = PIPE_SPECS[("2in", "40")]

    # Define fitting specs
    coupling_spec = define_socket_coupling(pipe)
    elbow_spec = define_socket_elbow90(pipe)
    branch_spec = define_branch_half_coupling(pipe)

    FITTINGS[coupling_spec.name] = coupling_spec
    FITTINGS[elbow_spec.name] = elbow_spec
    FITTINGS[branch_spec.name] = branch_spec

    # Route
    moves = [
        Move("UP", 3 * FT),
        Move("NORTH", 6 * FT),
        Move("EAST", 20 * FT),
    ]

    segments = build_segments(start=(0, 0, 0), moves=moves)

    # Straight pipe solids
    pipe_solids = [make_pipe_solid(seg, pipe) for seg in segments]

    # Branch on last segment
    last_seg_idx = len(segments) - 1
    last_seg = segments[last_seg_idx]

    branch_event = BranchEvent(
        segment_index=last_seg_idx,
        s=10 * FT,
        direction=cq.Vector(0, 0, 1),
        fitting_name=branch_spec.name,
    )

    # Cut hole
    pipe_with_hole = cut_branch_hole(
        run_pipe_solid=pipe_solids[last_seg_idx],
        seg=last_seg,
        event=branch_event,
        branch_od=pipe.od,
    )
    pipe_solids[last_seg_idx] = pipe_with_hole

    # Place branch fitting
    branch_fitting_solid = place_branch_fitting(
        spec=branch_spec,
        seg=last_seg,
        event=branch_event,
        run_od=pipe.od,
    )

    # Optional branch pipe extending from the fitting
    # _port = branch_spec.ports[0]  # Reserved for future port alignment
    surface_point, _rot = branch_world_frame(
        last_seg, branch_event.s, branch_event.direction, pipe.od
    )
    branch_dir = branch_event.direction.normalized()

    # Branch pipe starts from fitting port and extends upward
    branch_start = cq.Vector(
        surface_point.x,
        surface_point.y,
        surface_point.z + branch_spec.params["neck_length"] + branch_spec.params["socket_depth"],
    )
    branch_seg = Segment(
        start=branch_start,
        end=branch_start + branch_dir * (6 * FT),
    )
    branch_pipe_solid = make_pipe_solid(branch_seg, pipe)

    solids = pipe_solids + [branch_fitting_solid, branch_pipe_solid]
    export_spool_to_step(solids, "spool_with_branch.step")
    print("Exported spool_with_branch.step")


def example_simple_run():
    """Create a simple pipe run without branches."""
    pipe = PIPE_SPECS[("2in", "40")]

    moves = [
        Move("EAST", 10 * FT),
        Move("NORTH", 5 * FT),
        Move("UP", 3 * FT),
    ]

    segments = build_segments(start=(0, 0, 0), moves=moves)
    pipe_solids = [make_pipe_solid(seg, pipe) for seg in segments]

    export_spool_to_step(pipe_solids, "simple_run.step")
    print("Exported simple_run.step")


def example_asme_elbow():
    """Create and export an ASME B16.11 socket weld elbow."""
    pipe = PIPE_SPECS[("2in", "40")]

    # Create ASME elbow spec
    elbow_spec = define_asme_socket_elbow90(pipe, pressure_class=3000)

    # Generate the solid
    elbow_solid = elbow_spec.make_solid(pipe, elbow_spec.params)

    # Export just the elbow
    export_spool_to_step([elbow_solid], "asme_elbow_2in_cl3000.step")
    print("Exported asme_elbow_2in_cl3000.step")
    print(f"  Center-to-end: {elbow_spec.params['center_to_end']} mm")
    print(f"  Socket bore: {elbow_spec.params['socket_bore']:.1f} mm")
    print(f"  Socket depth: {elbow_spec.params['socket_depth']} mm")
    print(f"  Min bore: {elbow_spec.params['min_bore']} mm")


def example_all_asme_elbows():
    """Create ASME elbows for all available sizes."""
    solids = []
    offset = 0

    for (size, _sched), pipe in PIPE_SPECS.items():
        try:
            elbow_spec = define_asme_socket_elbow90(pipe, pressure_class=3000)
            solid = elbow_spec.make_solid(pipe, elbow_spec.params)
            # Offset each elbow so they don't overlap
            loc = cq.Location(cq.Vector(offset, 0, 0))
            solids.append(solid.moved(loc))
            print(f"Created ASME elbow for {size}: A={elbow_spec.params['center_to_end']}mm")
            offset += elbow_spec.params['center_to_end'] * 3
        except ValueError as e:
            print(f"Skipping {size}: {e}")

    if solids:
        export_spool_to_step(solids, "asme_elbows_all_sizes.step")
        print(f"Exported asme_elbows_all_sizes.step with {len(solids)} elbows")


if __name__ == "__main__":
    print("Running pipe generator examples...")
    example_simple_run()
    example_spool_with_branch()
    print("\nCreating ASME B16.11 elbows...")
    example_asme_elbow()
    example_all_asme_elbows()
    print("Done!")
