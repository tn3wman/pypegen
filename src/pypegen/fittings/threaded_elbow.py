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


def make_threaded_elbow_90(
    nps: str,
    pressure_class: int = 3000,
) -> cq.Shape:
    """
    Create an ASME B16.11 90° NPT threaded elbow.

    The elbow is oriented with:
    - Inlet from +X direction toward origin
    - Outlet from origin toward +Y direction
    - A sphere at origin connects the two legs

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        pressure_class: 3000 only currently implemented

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

    # Radii - threaded fittings have bore sized for pipe OD (threads are internal)
    r_bore = pipe_od / 2.0  # Bore accepts pipe OD
    r_outer = dims.band_od / 2.0  # Outer body radius

    # Outer body: two cylinders + full sphere at origin
    leg1_outer = cq.Solid.makeCylinder(r_outer, A, cq.Vector(A, 0, 0), cq.Vector(-1, 0, 0))
    leg2_outer = cq.Solid.makeCylinder(r_outer, A, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))
    sphere_outer = cq.Solid.makeSphere(
        r_outer,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=360,
    )

    outer = leg1_outer.fuse(leg2_outer).fuse(sphere_outer)

    # Inner bore: same shape with r_bore
    leg1_inner = cq.Solid.makeCylinder(r_bore, A, cq.Vector(A, 0, 0), cq.Vector(-1, 0, 0))
    leg2_inner = cq.Solid.makeCylinder(r_bore, A, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))
    sphere_inner = cq.Solid.makeSphere(
        r_bore,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=360,
    )

    inner = leg1_inner.fuse(leg2_inner).fuse(sphere_inner)

    # Cut inner from outer to create hollow elbow
    elbow = outer.cut(inner)

    # No socket counterbores for threaded fittings - the bore is sized
    # for the pipe OD and internal threads engage the pipe

    return elbow


def make_threaded_elbow_45(
    nps: str,
    pressure_class: int = 3000,
) -> cq.Shape:
    """
    Create an ASME B16.11 45° NPT threaded elbow.

    The elbow is oriented with:
    - Inlet from +X direction toward origin
    - Outlet from origin toward 135° direction (45° turn from inlet)

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        pressure_class: 3000 only currently implemented

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

    # Radii
    r_bore = pipe_od / 2.0  # Bore accepts pipe OD
    r_outer = dims.band_od / 2.0  # Outer body radius

    # Direction for 45° leg (45° turn from -X toward +Y)
    # Outlet direction: rotate -X by 45° counterclockwise = 135° from +X
    angle_rad = math.radians(135)
    dir_x = math.cos(angle_rad)  # ~-0.707
    dir_y = math.sin(angle_rad)  # ~0.707

    # Leg 1: inlet from +X direction
    leg1_outer = cq.Solid.makeCylinder(r_outer, A, cq.Vector(A, 0, 0), cq.Vector(-1, 0, 0))

    # Leg 2: outlet toward 135° direction
    leg2_outer = cq.Solid.makeCylinder(
        r_outer,
        A,
        cq.Vector(0, 0, 0),
        cq.Vector(dir_x, dir_y, 0),
    )

    # 90° sphere wedge at origin, rotated 180° to fill gap between legs
    # (Same approach as socket weld 45° elbow)
    sphere_outer = cq.Solid.makeSphere(
        r_outer,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=90,  # 90° wedge
    )
    sphere_outer = sphere_outer.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 180))

    outer = leg1_outer.fuse(leg2_outer).fuse(sphere_outer)

    # Inner bore
    leg1_inner = cq.Solid.makeCylinder(r_bore, A, cq.Vector(A, 0, 0), cq.Vector(-1, 0, 0))
    leg2_inner = cq.Solid.makeCylinder(
        r_bore,
        A,
        cq.Vector(0, 0, 0),
        cq.Vector(dir_x, dir_y, 0),
    )
    sphere_inner = cq.Solid.makeSphere(
        r_bore,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=90,
    )
    sphere_inner = sphere_inner.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 180))

    inner = leg1_inner.fuse(leg2_inner).fuse(sphere_inner)

    # Cut inner from outer
    elbow = outer.cut(inner)

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
