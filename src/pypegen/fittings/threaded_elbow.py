"""
ASME B16.11 NPT Threaded Elbow Generator using CadQuery.

This module creates NPT threaded elbows (90° and 45°) per ASME B16.11 dimensions.
Thread engagement follows ASME B1.20.1 for NPT pipe threads.

Sources:
- https://ferrobend.com/dimensions/ansi-asme/pipe-fitting/b16.11-threaded-class-3000-90-degree-elbow/
- https://ferrobend.com/dimensions/ansi-asme/pipe-fitting/b16.11-threaded-class-3000-45-degree-elbow/
- https://www.engineeringtoolbox.com/npt-national-pipe-taper-threads-d_750.html
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cadquery as cq

from pypegen.threads.npt_female import (
    NPT_TAPER_RATIO,
    apply_bore_id_chamfer,
    get_npt_spec,
    make_npt_internal_runout_cutter,
    make_npt_internal_thread_cutter,
)

# =============================================================================
# CONSTANTS
# =============================================================================

INCH = 25.4  # mm per inch


# =============================================================================
# DATA STRUCTURES
# =============================================================================


@dataclass
class ThreadedElbowDims:
    """ASME B16.11 Threaded Elbow dimensions (all in mm)."""

    center_to_end: float  # A or C - center to thread end
    band_od: float  # H - outside diameter of band/body
    wall_thickness: float  # G - minimum wall thickness
    min_thread_length: float  # B - minimum length of perfect thread


@dataclass
class NPTThreadSpec:
    """NPT thread specifications per ASME B1.20.1."""

    tpi: float  # Threads per inch
    engagement: float  # Hand+wrench makeup depth (mm)


# NPT Thread engagement depths (hand + wrench makeup)
# From ASME B1.20.1 / Engineering Toolbox
NPT_THREAD_ENGAGEMENT: dict[str, NPTThreadSpec] = {
    "1/8": NPTThreadSpec(tpi=27, engagement=6.4),
    "1/4": NPTThreadSpec(tpi=18, engagement=9.5),
    "3/8": NPTThreadSpec(tpi=18, engagement=9.5),
    "1/2": NPTThreadSpec(tpi=14, engagement=11.1),
    "3/4": NPTThreadSpec(tpi=14, engagement=12.7),
    "1": NPTThreadSpec(tpi=11.5, engagement=14.3),
    "1-1/4": NPTThreadSpec(tpi=11.5, engagement=14.3),
    "1-1/2": NPTThreadSpec(tpi=11.5, engagement=14.3),
    "2": NPTThreadSpec(tpi=11.5, engagement=15.9),
    "2-1/2": NPTThreadSpec(tpi=8, engagement=22.2),
    "3": NPTThreadSpec(tpi=8, engagement=25.4),
    "4": NPTThreadSpec(tpi=8, engagement=27.0),
}


# ASME B16.11 Class 3000 Threaded 90° Elbow dimension table
ASME_B1611_THREADED_ELBOW90_CLASS3000: dict[str, ThreadedElbowDims] = {
    "1/8": ThreadedElbowDims(
        center_to_end=21.0,
        band_od=22.0,
        wall_thickness=3.18,
        min_thread_length=6.4,
    ),
    "1/4": ThreadedElbowDims(
        center_to_end=25.0,
        band_od=25.0,
        wall_thickness=3.30,
        min_thread_length=8.1,
    ),
    "3/8": ThreadedElbowDims(
        center_to_end=28.0,
        band_od=33.0,
        wall_thickness=3.51,
        min_thread_length=9.1,
    ),
    "1/2": ThreadedElbowDims(
        center_to_end=33.0,
        band_od=38.0,
        wall_thickness=4.09,
        min_thread_length=10.9,
    ),
    "3/4": ThreadedElbowDims(
        center_to_end=38.0,
        band_od=46.0,
        wall_thickness=4.32,
        min_thread_length=12.7,
    ),
    "1": ThreadedElbowDims(
        center_to_end=44.0,
        band_od=56.0,
        wall_thickness=4.98,
        min_thread_length=14.7,
    ),
    "1-1/4": ThreadedElbowDims(
        center_to_end=51.0,
        band_od=62.0,
        wall_thickness=5.28,
        min_thread_length=17.0,
    ),
    "1-1/2": ThreadedElbowDims(
        center_to_end=60.0,
        band_od=75.0,
        wall_thickness=5.56,
        min_thread_length=17.8,
    ),
    "2": ThreadedElbowDims(
        center_to_end=64.0,
        band_od=84.0,
        wall_thickness=7.14,
        min_thread_length=19.0,
    ),
    "2-1/2": ThreadedElbowDims(
        center_to_end=83.0,
        band_od=102.0,
        wall_thickness=7.65,
        min_thread_length=23.6,
    ),
    "3": ThreadedElbowDims(
        center_to_end=95.0,
        band_od=121.0,
        wall_thickness=8.84,
        min_thread_length=25.9,
    ),
    "4": ThreadedElbowDims(
        center_to_end=114.0,
        band_od=152.0,
        wall_thickness=11.18,
        min_thread_length=27.7,
    ),
}


# ASME B16.11 Class 3000 Threaded 45° Elbow dimension table
ASME_B1611_THREADED_ELBOW45_CLASS3000: dict[str, ThreadedElbowDims] = {
    "1/8": ThreadedElbowDims(
        center_to_end=17.0,
        band_od=22.0,
        wall_thickness=3.18,
        min_thread_length=6.4,
    ),
    "1/4": ThreadedElbowDims(
        center_to_end=19.0,
        band_od=25.0,
        wall_thickness=3.30,
        min_thread_length=8.1,
    ),
    "3/8": ThreadedElbowDims(
        center_to_end=22.0,
        band_od=33.0,
        wall_thickness=3.51,
        min_thread_length=9.1,
    ),
    "1/2": ThreadedElbowDims(
        center_to_end=25.0,
        band_od=38.0,
        wall_thickness=4.09,
        min_thread_length=10.9,
    ),
    "3/4": ThreadedElbowDims(
        center_to_end=28.0,
        band_od=46.0,
        wall_thickness=4.32,
        min_thread_length=12.7,
    ),
    "1": ThreadedElbowDims(
        center_to_end=33.0,
        band_od=56.0,
        wall_thickness=4.98,
        min_thread_length=14.7,
    ),
    "1-1/4": ThreadedElbowDims(
        center_to_end=35.0,
        band_od=62.0,
        wall_thickness=5.28,
        min_thread_length=17.0,
    ),
    "1-1/2": ThreadedElbowDims(
        center_to_end=43.0,
        band_od=75.0,
        wall_thickness=5.56,
        min_thread_length=17.8,
    ),
    "2": ThreadedElbowDims(
        center_to_end=44.0,
        band_od=84.0,
        wall_thickness=7.14,
        min_thread_length=19.0,
    ),
    "2-1/2": ThreadedElbowDims(
        center_to_end=52.0,
        band_od=102.0,
        wall_thickness=7.65,
        min_thread_length=23.6,
    ),
    "3": ThreadedElbowDims(
        center_to_end=64.0,
        band_od=121.0,
        wall_thickness=8.84,
        min_thread_length=25.9,
    ),
    "4": ThreadedElbowDims(
        center_to_end=79.0,
        band_od=152.0,
        wall_thickness=11.18,
        min_thread_length=27.7,
    ),
}


# Pipe OD lookup (for bore calculation)
# Standard pipe OD per NPS
PIPE_OD: dict[str, float] = {
    "1/8": 10.3,
    "1/4": 13.7,
    "3/8": 17.1,
    "1/2": 21.3,
    "3/4": 26.7,
    "1": 33.4,
    "1-1/4": 42.2,
    "1-1/2": 48.3,
    "2": 60.3,
    "2-1/2": 73.0,
    "3": 88.9,
    "4": 114.3,
}


# =============================================================================
# ELBOW GEOMETRY
# =============================================================================


def _make_tapered_bore(
    bore_radius_at_opening: float,
    thread_length: float,
) -> cq.Shape:
    """Create a tapered cylinder for cutting bore.

    The bore is LARGEST at Z=0 (fitting face) and SMALLEST at Z=thread_length.
    This creates a flared entry that tapers down going into the fitting.

    Args:
        bore_radius_at_opening: Bore radius at the fitting face (Z=0)
        thread_length: Length of the tapered section

    Returns:
        Tapered cylinder shape with larger end at Z=0
    """
    taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
    bore_radius_at_end = bore_radius_at_opening - taper_delta  # Smaller going in

    # Create tapered cylinder using loft
    # LARGEST at opening (Z=0), SMALLEST going in (Z=thread_length)
    bore = (
        cq.Workplane("XY")
        .circle(bore_radius_at_opening)  # Larger at opening
        .workplane(offset=thread_length)
        .circle(bore_radius_at_end)  # Smaller going in
        .loft()
    )
    return cq.Shape(bore.val().wrapped)


def _make_elbow_body_90(r_outer: float, center_to_end: float) -> cq.Shape:
    """Create solid 90° elbow body.

    The elbow is oriented with:
    - Leg 1 along +X axis (opening at X=center_to_end)
    - Leg 2 along +Y axis (opening at Y=center_to_end)
    - Sphere at origin connects the two legs
    """
    A = center_to_end

    # Outer body: two cylinders + full sphere at origin
    leg1 = cq.Solid.makeCylinder(r_outer, A, cq.Vector(A, 0, 0), cq.Vector(-1, 0, 0))
    leg2 = cq.Solid.makeCylinder(r_outer, A, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))
    sphere = cq.Solid.makeSphere(
        r_outer,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=360,
    )

    elbow = leg1.fuse(leg2).fuse(sphere)
    return elbow


def make_threaded_elbow_90(
    nps: str,
    pressure_class: int = 3000,
    include_threads: bool = False,
) -> cq.Shape:
    """
    Create an ASME B16.11 90° NPT threaded elbow.

    The elbow is oriented with:
    - Inlet from +X direction toward origin
    - Outlet from origin toward +Y direction
    - A torus section at origin connects the two legs

    For threaded version, uses die-cut approach:
    1. Create solid outer elbow body (torus + legs)
    2. Cut tapered bore (smallest at opening, largest at center)
    3. Apply chamfer to bore ID edge
    4. Cut thread grooves using helical V-groove cutter

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        pressure_class: 3000 only currently implemented
        include_threads: If True, add actual NPT internal thread geometry

    Returns:
        CadQuery Shape of the elbow
    """
    if pressure_class != 3000:
        raise ValueError(f"Pressure class {pressure_class} not implemented")

    dims = ASME_B1611_THREADED_ELBOW90_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.11 Class {pressure_class} threaded 90° elbow dimensions for NPS {nps}")

    pipe_od = PIPE_OD.get(nps)
    if pipe_od is None:
        raise ValueError(f"No pipe OD for NPS {nps}")

    # Get dimensions
    A = dims.center_to_end  # Center to thread end
    r_outer = dims.band_od / 2.0  # Outer body radius

    # Create solid outer elbow body
    elbow = _make_elbow_body_90(r_outer, A)

    if not include_threads:
        # Simple version: just cut cylindrical bore using same approach
        r_bore = pipe_od / 2.0
        inner = _make_elbow_body_90(r_bore, A)
        elbow = elbow.cut(inner)
    else:
        # Threaded version: die-cut approach
        npt_spec = get_npt_spec(nps)
        thread_length = min(dims.min_thread_length, npt_spec.L2)
        thread_depth = npt_spec.thread_depth

        # Bore radius at opening (where external pipe thread crests contact)
        bore_radius_at_opening = npt_spec.od / 2.0
        taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
        bore_radius_at_end = bore_radius_at_opening - taper_delta  # Smaller going in

        # Step 1: Cut full bore through elbow at the SMALLEST radius
        # This ensures bore goes all the way through with clearance
        inner = _make_elbow_body_90(bore_radius_at_end, A)
        elbow = elbow.cut(inner)

        # Step 2: Cut tapered bore sections at each opening
        # Leg 1: opening at X=A, bore toward -X
        tapered_bore1 = _make_tapered_bore(bore_radius_at_opening, thread_length)
        tapered_bore1 = tapered_bore1.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))
        tapered_bore1 = tapered_bore1.moved(cq.Location(cq.Vector(A, 0, 0)))
        elbow = elbow.cut(tapered_bore1)

        # Leg 2: opening at Y=A, bore toward -Y
        tapered_bore2 = _make_tapered_bore(bore_radius_at_opening, thread_length)
        tapered_bore2 = tapered_bore2.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90))
        tapered_bore2 = tapered_bore2.moved(cq.Location(cq.Vector(0, A, 0)))
        elbow = elbow.cut(tapered_bore2)

        # Step 3: Apply chamfer to bore ID edges
        # Chamfer is 60° from vertical (steeper angle)
        radial_depth = thread_depth
        axial_depth = radial_depth * math.tan(math.radians(30))

        # Chamfer leg 1 bore (opening at X=A, axis points toward -X)
        elbow = apply_bore_id_chamfer(
            elbow,
            bore_axis=(-1.0, 0.0, 0.0),
            bore_opening=(A, 0.0, 0.0),
            radial_depth=radial_depth,
            axial_depth=axial_depth,
        )

        # Chamfer leg 2 bore (opening at Y=A, axis points toward -Y)
        elbow = apply_bore_id_chamfer(
            elbow,
            bore_axis=(0.0, -1.0, 0.0),
            bore_opening=(0.0, A, 0.0),
            radial_depth=radial_depth,
            axial_depth=axial_depth,
        )

        # Step 4: Create thread cutter with runout
        # Reserve space for runout at the end (going into fitting)
        runout_turns = 0.25  # 90° fade
        pitch = npt_spec.pitch_mm
        runout_length = pitch * runout_turns
        main_cutter_length = thread_length - runout_length

        if main_cutter_length < pitch:
            # Thread too short for proper runout, use full length
            main_cutter_length = thread_length
            runout_length = 0

        # Create main thread cutter
        thread_cutter = make_npt_internal_thread_cutter(nps, main_cutter_length)

        # Add runout section with fading depth
        full_cutter = thread_cutter
        if runout_length > 0:
            runout_cutter = make_npt_internal_runout_cutter(
                nps=nps,
                runout_turns=runout_turns,
                z_offset=main_cutter_length,
            )
            if runout_cutter is not None:
                try:
                    full_cutter = thread_cutter.fuse(runout_cutter)
                except Exception:
                    # Keep just the main cutter if fuse fails
                    pass

        # Cut threads for leg 1 (+X direction)
        # Thread cutter is generated along +Z axis
        # Rotate -90° around Y to align with -X axis (toward center from opening)
        cutter1 = full_cutter.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))
        cutter1 = cutter1.moved(cq.Location(cq.Vector(A, 0, 0)))
        elbow = elbow.cut(cutter1)

        # Cut threads for leg 2 (+Y direction)
        # Rotate +90° around X to align with -Y axis (toward center from opening)
        cutter2 = full_cutter.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90))
        cutter2 = cutter2.moved(cq.Location(cq.Vector(0, A, 0)))
        elbow = elbow.cut(cutter2)

    return elbow


def _make_elbow_body_45(r_outer: float, center_to_end: float) -> cq.Shape:
    """Create solid 45° elbow body.

    The elbow is oriented with:
    - Leg 1 along +X axis (opening at X=center_to_end)
    - Leg 2 along 135° direction (45° turn from inlet)
    - Sphere wedge at origin connects the two legs
    """
    A = center_to_end

    # Direction for 45° leg (135° from +X)
    angle_rad = math.radians(135)
    dir_x = math.cos(angle_rad)
    dir_y = math.sin(angle_rad)

    # Leg 1: inlet from +X direction
    leg1 = cq.Solid.makeCylinder(r_outer, A, cq.Vector(A, 0, 0), cq.Vector(-1, 0, 0))

    # Leg 2: outlet toward 135° direction
    leg2 = cq.Solid.makeCylinder(
        r_outer,
        A,
        cq.Vector(0, 0, 0),
        cq.Vector(dir_x, dir_y, 0),
    )

    # 90° sphere wedge at origin, rotated 180° to fill gap between legs
    sphere = cq.Solid.makeSphere(
        r_outer,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=90,  # 90° wedge
    )
    sphere = sphere.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 180))

    elbow = leg1.fuse(leg2).fuse(sphere)
    return elbow


def make_threaded_elbow_45(
    nps: str,
    pressure_class: int = 3000,
    include_threads: bool = False,
) -> cq.Shape:
    """
    Create an ASME B16.11 45° NPT threaded elbow.

    The elbow is oriented with:
    - Inlet from +X direction toward origin
    - Outlet from origin toward 135° direction (45° turn from inlet)

    For threaded version, uses die-cut approach:
    1. Create solid outer elbow body (torus + legs)
    2. Cut tapered bore (smallest at opening, largest at center)
    3. Apply chamfer to bore ID edge
    4. Cut thread grooves using helical V-groove cutter

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        pressure_class: 3000 only currently implemented
        include_threads: If True, add actual NPT internal thread geometry

    Returns:
        CadQuery Shape of the elbow
    """
    if pressure_class != 3000:
        raise ValueError(f"Pressure class {pressure_class} not implemented")

    dims = ASME_B1611_THREADED_ELBOW45_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.11 Class {pressure_class} threaded 45° elbow dimensions for NPS {nps}")

    pipe_od = PIPE_OD.get(nps)
    if pipe_od is None:
        raise ValueError(f"No pipe OD for NPS {nps}")

    # Get dimensions
    A = dims.center_to_end  # Center to thread end
    r_outer = dims.band_od / 2.0  # Outer body radius

    # Direction for 45° leg (45° turn from -X toward +Y)
    # Outlet direction: rotate -X by 45° counterclockwise = 135° from +X
    angle_rad = math.radians(135)
    dir_x = math.cos(angle_rad)  # ~-0.707
    dir_y = math.sin(angle_rad)  # ~0.707

    # Create solid outer elbow body
    elbow = _make_elbow_body_45(r_outer, A)

    if not include_threads:
        # Simple version: just cut cylindrical bore using same approach
        r_bore = pipe_od / 2.0
        inner = _make_elbow_body_45(r_bore, A)
        elbow = elbow.cut(inner)
    else:
        # Threaded version: die-cut approach
        npt_spec = get_npt_spec(nps)
        thread_length = min(dims.min_thread_length, npt_spec.L2)
        thread_depth = npt_spec.thread_depth

        # Bore radius at opening (where external pipe thread crests contact)
        bore_radius_at_opening = npt_spec.od / 2.0
        taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
        bore_radius_at_end = bore_radius_at_opening - taper_delta  # Smaller going in

        # Outlet end position
        outlet_end_x = A * dir_x
        outlet_end_y = A * dir_y

        # Step 1: Cut full bore through elbow at the SMALLEST radius
        inner = _make_elbow_body_45(bore_radius_at_end, A)
        elbow = elbow.cut(inner)

        # Step 2: Cut tapered bore sections at each opening
        # Leg 1: opening at X=A, bore toward -X
        tapered_bore1 = _make_tapered_bore(bore_radius_at_opening, thread_length)
        tapered_bore1 = tapered_bore1.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))
        tapered_bore1 = tapered_bore1.moved(cq.Location(cq.Vector(A, 0, 0)))
        elbow = elbow.cut(tapered_bore1)

        # Leg 2: opening at 135° end, bore toward center
        tapered_bore2 = _make_tapered_bore(bore_radius_at_opening, thread_length)
        # Rotate to align with the 135° leg axis (pointing toward center)
        tapered_bore2 = tapered_bore2.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))
        tapered_bore2 = tapered_bore2.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), -45))
        tapered_bore2 = tapered_bore2.moved(cq.Location(cq.Vector(outlet_end_x, outlet_end_y, 0)))
        elbow = elbow.cut(tapered_bore2)

        # Step 3: Apply chamfer to bore ID edges
        radial_depth = thread_depth
        axial_depth = radial_depth * math.tan(math.radians(30))

        # Chamfer leg 1 bore (opening at X=A, axis points toward -X)
        elbow = apply_bore_id_chamfer(
            elbow,
            bore_axis=(-1.0, 0.0, 0.0),
            bore_opening=(A, 0.0, 0.0),
            radial_depth=radial_depth,
            axial_depth=axial_depth,
        )

        # Chamfer leg 2 bore (opening at 135° end, axis points toward center)
        # Axis direction is opposite of outlet direction: (-dir_x, -dir_y, 0)
        elbow = apply_bore_id_chamfer(
            elbow,
            bore_axis=(-dir_x, -dir_y, 0.0),
            bore_opening=(outlet_end_x, outlet_end_y, 0.0),
            radial_depth=radial_depth,
            axial_depth=axial_depth,
        )

        # Step 4: Create thread cutter with runout
        # Reserve space for runout at the end (going into fitting)
        runout_turns = 0.25  # 90° fade
        pitch = npt_spec.pitch_mm
        runout_length = pitch * runout_turns
        main_cutter_length = thread_length - runout_length

        if main_cutter_length < pitch:
            # Thread too short for proper runout, use full length
            main_cutter_length = thread_length
            runout_length = 0

        # Create main thread cutter
        thread_cutter = make_npt_internal_thread_cutter(nps, main_cutter_length)

        # Add runout section with fading depth
        full_cutter = thread_cutter
        if runout_length > 0:
            runout_cutter = make_npt_internal_runout_cutter(
                nps=nps,
                runout_turns=runout_turns,
                z_offset=main_cutter_length,
            )
            if runout_cutter is not None:
                try:
                    full_cutter = thread_cutter.fuse(runout_cutter)
                except Exception:
                    # Keep just the main cutter if fuse fails
                    pass

        # Cut threads for leg 1 (+X direction)
        # Rotate -90° around Y to align with -X axis
        cutter1 = full_cutter.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))
        cutter1 = cutter1.moved(cq.Location(cq.Vector(A, 0, 0)))
        elbow = elbow.cut(cutter1)

        # Cut threads for leg 2 (135° direction)
        cutter2 = full_cutter.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))
        cutter2 = cutter2.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), -45))
        cutter2 = cutter2.moved(cq.Location(cq.Vector(outlet_end_x, outlet_end_y, 0)))
        elbow = elbow.cut(cutter2)

    return elbow


def get_threaded_elbow_90_dims(nps: str, pressure_class: int = 3000) -> ThreadedElbowDims:
    """Get dimensions for a threaded 90° elbow."""
    if pressure_class != 3000:
        raise ValueError(f"Pressure class {pressure_class} not implemented")
    dims = ASME_B1611_THREADED_ELBOW90_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No dimensions for NPS {nps}")
    return dims


def get_threaded_elbow_45_dims(nps: str, pressure_class: int = 3000) -> ThreadedElbowDims:
    """Get dimensions for a threaded 45° elbow."""
    if pressure_class != 3000:
        raise ValueError(f"Pressure class {pressure_class} not implemented")
    dims = ASME_B1611_THREADED_ELBOW45_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No dimensions for NPS {nps}")
    return dims


def get_npt_thread_engagement(nps: str) -> float:
    """Get NPT thread engagement depth (hand + wrench makeup) in mm."""
    spec = NPT_THREAD_ENGAGEMENT.get(nps)
    if spec is None:
        raise ValueError(f"No NPT thread spec for NPS {nps}")
    return spec.engagement


# =============================================================================
# EXPORT UTILITIES
# =============================================================================


def export_to_step(shape: cq.Shape, filename: str):
    """Export a shape to STEP file."""
    cq.exporters.export(shape, filename)


# =============================================================================
# EXAMPLES
# =============================================================================


def example_threaded_elbows():
    """Create example threaded elbows."""
    # 90° elbow
    elbow_90 = make_threaded_elbow_90("2")
    export_to_step(elbow_90, "threaded_elbow_90_2in.step")
    print("Exported threaded_elbow_90_2in.step")

    dims_90 = ASME_B1611_THREADED_ELBOW90_CLASS3000["2"]
    print(f"  Center-to-end: {dims_90.center_to_end} mm")
    print(f"  Band OD: {dims_90.band_od} mm")

    # 45° elbow
    elbow_45 = make_threaded_elbow_45("2")
    export_to_step(elbow_45, "threaded_elbow_45_2in.step")
    print("Exported threaded_elbow_45_2in.step")

    dims_45 = ASME_B1611_THREADED_ELBOW45_CLASS3000["2"]
    print(f"  Center-to-end: {dims_45.center_to_end} mm")


if __name__ == "__main__":
    print("ASME B16.11 NPT Threaded Elbow Generator")
    print("=" * 40)
    example_threaded_elbows()
    print("\nDone!")
