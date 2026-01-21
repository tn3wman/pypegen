"""
ASME B16.9 Butt Weld 45° Elbow Generator using CadQuery.

This module creates proper butt weld 45° elbows with swept geometry
per ASME B16.9 dimensions.

Note: Per ASME B16.9, 45° elbows are only available in Long Radius (1.5D).
There is no Short Radius option for 45° elbows.

Sources:
- https://www.zcpipefittings.com/45-degree-elbow.html
- ASME B16.9 Butt Welding Fittings Standard
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import cadquery as cq

# =============================================================================
# CONSTANTS
# =============================================================================

INCH = 25.4  # mm per inch


# =============================================================================
# DATA STRUCTURES
# =============================================================================


@dataclass
class ButtWeldElbow45Dims:
    """ASME B16.9 Butt Weld 45° Elbow dimensions (all in mm).

    Note: 45° elbows only come in Long Radius (1.5D).
    The center-to-end dimension (B) is less than the 90° elbow (A) due to
    the smaller bend angle.
    """

    od: float  # Outside diameter
    wall_thickness: float  # Wall thickness (schedule dependent)
    center_to_end: float  # B dimension for long radius (1.5D)


# ASME B16.9 45° Butt Weld Elbow dimension table
# Long Radius: bend radius = 1.5 x NPS
# Key is NPS size string
# Wall thicknesses from ASME B16.9 STD schedule (same as 90° elbows)
ASME_B169_ELBOW45: dict[str, ButtWeldElbow45Dims] = {
    "1/2": ButtWeldElbow45Dims(
        od=21.3,
        wall_thickness=2.78,  # STD
        center_to_end=16.0,
    ),
    "3/4": ButtWeldElbow45Dims(
        od=26.7,
        wall_thickness=2.87,  # STD
        center_to_end=19.0,
    ),
    "1": ButtWeldElbow45Dims(
        od=33.4,
        wall_thickness=3.38,  # STD
        center_to_end=22.0,
    ),
    "1-1/4": ButtWeldElbow45Dims(
        od=42.2,
        wall_thickness=3.56,  # STD
        center_to_end=25.0,
    ),
    "1-1/2": ButtWeldElbow45Dims(
        od=48.3,
        wall_thickness=3.68,  # STD
        center_to_end=29.0,
    ),
    "2": ButtWeldElbow45Dims(
        od=60.3,
        wall_thickness=3.91,  # STD
        center_to_end=35.0,
    ),
    "2-1/2": ButtWeldElbow45Dims(
        od=73.0,
        wall_thickness=5.16,  # STD
        center_to_end=44.0,
    ),
    "3": ButtWeldElbow45Dims(
        od=88.9,
        wall_thickness=5.49,  # STD
        center_to_end=51.0,
    ),
    "3-1/2": ButtWeldElbow45Dims(
        od=101.6,
        wall_thickness=5.74,  # STD
        center_to_end=57.0,
    ),
    "4": ButtWeldElbow45Dims(
        od=114.3,
        wall_thickness=6.02,  # STD
        center_to_end=64.0,
    ),
    "5": ButtWeldElbow45Dims(
        od=141.3,
        wall_thickness=6.55,  # STD
        center_to_end=79.0,
    ),
    "6": ButtWeldElbow45Dims(
        od=168.3,
        wall_thickness=7.11,  # STD
        center_to_end=95.0,
    ),
    "8": ButtWeldElbow45Dims(
        od=219.1,
        wall_thickness=8.18,  # STD
        center_to_end=127.0,
    ),
    "10": ButtWeldElbow45Dims(
        od=273.0,
        wall_thickness=9.27,  # STD
        center_to_end=159.0,
    ),
    "12": ButtWeldElbow45Dims(
        od=323.8,
        wall_thickness=9.52,  # STD
        center_to_end=190.0,
    ),
    "14": ButtWeldElbow45Dims(
        od=355.6,
        wall_thickness=9.52,  # STD
        center_to_end=222.0,
    ),
    "16": ButtWeldElbow45Dims(
        od=406.4,
        wall_thickness=9.52,  # STD
        center_to_end=254.0,
    ),
    "18": ButtWeldElbow45Dims(
        od=457.0,
        wall_thickness=9.52,  # STD
        center_to_end=286.0,
    ),
    "20": ButtWeldElbow45Dims(
        od=508.0,
        wall_thickness=9.52,  # STD
        center_to_end=318.0,
    ),
    "24": ButtWeldElbow45Dims(
        od=610.0,
        wall_thickness=9.52,  # STD
        center_to_end=381.0,
    ),
}


# =============================================================================
# ELBOW GEOMETRY
# =============================================================================


def make_butt_weld_elbow_45(
    nps: str,
    _schedule: str = "STD",  # Reserved for future wall thickness lookup
) -> cq.Shape:
    """
    Create an ASME B16.9 45° butt weld elbow.

    Note: 45° elbows are only available in Long Radius (1.5D).

    The elbow is oriented with:
    - One end on the +X axis (pipe would connect from +X direction)
    - Other end at 45° in the XY plane (pipe would connect from that direction)
    - The bend center at the origin

    The geometry uses the same approach as 90° elbows but with a 45° arc sweep
    instead of 90°.

    Args:
        nps: Nominal pipe size (e.g., "2", "4", "6")
        schedule: Pipe schedule (currently only "STD" implemented)

    Returns:
        CadQuery Shape of the elbow
    """
    dims = ASME_B169_ELBOW45.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.9 45° elbow dimensions for NPS {nps}")

    # Get dimensions
    od = dims.od
    wall = dims.wall_thickness
    id_ = od - 2 * wall

    # For a 45° long radius elbow, the bend radius (R) is 1.5 × NPS
    # The center-to-end (B) dimension is geometrically related to R:
    # B = R × tan(22.5°) where 22.5° = 45°/2
    # But we use the actual ASME B16.9 tabulated B values for accuracy
    center_to_end = dims.center_to_end

    # Calculate the bend radius from the center-to-end dimension
    # For a 45° arc: B = R × tan(45°/2) = R × tan(22.5°)
    # Therefore: R = B / tan(22.5°)
    angle_half_rad = math.radians(22.5)
    bend_radius = center_to_end / math.tan(angle_half_rad)

    # Radii for the pipe cross-section
    r_outer = od / 2.0
    r_inner = id_ / 2.0

    # Create the elbow by sweeping a circular annulus along a 45° arc
    #
    # Geometry setup (similar to 90° but with 45° arc):
    # - The elbow lies in the XY plane
    # - Bend center is at origin
    # - Arc goes from (bend_radius, 0, 0) 45° counterclockwise
    # - End point: (R*cos(45°), R*sin(45°), 0)
    #
    # At each point on the arc, the pipe cross-section is perpendicular
    # to the arc tangent direction.

    # Calculate arc endpoints
    start_x = bend_radius
    start_y = 0.0

    # End point at 45° from start
    end_angle_rad = math.radians(45)
    end_x = bend_radius * math.cos(end_angle_rad)
    end_y = bend_radius * math.sin(end_angle_rad)

    # Midpoint at 22.5° for three-point arc
    mid_angle_rad = math.radians(22.5)
    mid_x = bend_radius * math.cos(mid_angle_rad)
    mid_y = bend_radius * math.sin(mid_angle_rad)

    # Create the 45° arc path
    arc_path = (
        cq.Workplane("XY")
        .moveTo(start_x, start_y)
        .threePointArc((mid_x, mid_y), (end_x, end_y))
    )

    # Create the pipe cross-section profile at the start of the arc
    # At (bend_radius, 0, 0), the tangent is +Y
    # So the profile plane should be perpendicular to Y, which is the XZ plane
    # The profile center should be at (bend_radius, 0) in the XZ plane

    # Note: CadQuery's sweep with isFrenet=True rotates the profile to follow
    # the path's Frenet frame. For the profile to end up correctly positioned,
    # we place it in the XZ plane (perpendicular to the tangent at start)

    # Create outer profile at the start point
    outer_profile = (
        cq.Workplane("XZ")  # Plane perpendicular to Y axis (tangent direction)
        .center(bend_radius, 0)  # Move center to start of arc
        .circle(r_outer)
    )

    # Sweep outer along arc
    outer_sweep = outer_profile.sweep(arc_path, isFrenet=True)

    # Create inner profile (for the bore)
    inner_profile = (
        cq.Workplane("XZ")
        .center(bend_radius, 0)
        .circle(r_inner)
    )

    # Sweep inner along arc
    inner_sweep = inner_profile.sweep(arc_path, isFrenet=True)

    # Subtract inner from outer to get hollow pipe
    elbow = outer_sweep.cut(inner_sweep)

    result = elbow.solids().val()
    assert isinstance(result, cq.Shape)
    return result


def make_solid_elbow_45(
    od: float,
    wall_thickness: float,
    bend_radius: float,
) -> cq.Shape:
    """
    Create a 45° elbow with custom dimensions.

    This is a lower-level function for creating elbows with arbitrary dimensions.

    Args:
        od: Outside diameter (mm)
        wall_thickness: Wall thickness (mm)
        bend_radius: Bend radius (distance from origin to pipe centerline) (mm)

    Returns:
        CadQuery Shape of the elbow
    """
    r_outer = od / 2.0
    r_inner = (od - 2 * wall_thickness) / 2.0

    # Calculate arc endpoints
    start_x = bend_radius
    start_y = 0.0

    end_angle_rad = math.radians(45)
    end_x = bend_radius * math.cos(end_angle_rad)
    end_y = bend_radius * math.sin(end_angle_rad)

    mid_angle_rad = math.radians(22.5)
    mid_x = bend_radius * math.cos(mid_angle_rad)
    mid_y = bend_radius * math.sin(mid_angle_rad)

    # Create the 45° arc path
    arc_path = (
        cq.Workplane("XY")
        .moveTo(start_x, start_y)
        .threePointArc((mid_x, mid_y), (end_x, end_y))
    )

    # Create profiles in YZ plane at x = bend_radius
    outer_profile = (
        cq.Workplane("YZ")
        .transformed(offset=(bend_radius, 0, 0))
        .circle(r_outer)
    )

    inner_profile = (
        cq.Workplane("YZ")
        .transformed(offset=(bend_radius, 0, 0))
        .circle(r_inner)
    )

    # Sweep and subtract
    outer_sweep = outer_profile.sweep(arc_path, isFrenet=True)
    inner_sweep = inner_profile.sweep(arc_path, isFrenet=True)

    elbow = outer_sweep.cut(inner_sweep)
    result = elbow.solids().val()
    assert isinstance(result, cq.Shape)
    return result


# =============================================================================
# EXPORT UTILITIES
# =============================================================================


def export_to_step(shape: cq.Shape, filename: str):
    """Export a shape to STEP file."""
    cq.exporters.export(shape, filename)


# =============================================================================
# EXAMPLES
# =============================================================================


def example_single_elbow_45():
    """Create a single 2" long radius 45° elbow."""
    elbow = make_butt_weld_elbow_45("2")
    export_to_step(elbow, "butt_weld_elbow_45_2in.step")
    print("Exported butt_weld_elbow_45_2in.step")

    dims = ASME_B169_ELBOW45["2"]
    print(f"  OD: {dims.od} mm")
    print(f"  Wall: {dims.wall_thickness} mm")
    print(f"  Center-to-end (B): {dims.center_to_end} mm")


def example_compare_90_vs_45():
    """Create both 90° and 45° elbows for comparison."""
    from .butt_weld_elbow import ASME_B169_ELBOW90, make_butt_weld_elbow_90

    elbow_90 = make_butt_weld_elbow_90("4", radius_type="LR")
    elbow_45 = make_butt_weld_elbow_45("4")

    # Offset the 45° elbow so they don't overlap
    dims_90 = ASME_B169_ELBOW90["4"]
    dims_45 = ASME_B169_ELBOW45["4"]
    offset = dims_90.center_to_end_lr + dims_45.center_to_end + 50

    elbow_45_moved = elbow_45.moved(cq.Location(cq.Vector(offset, 0, 0)))

    # Combine and export
    compound = cq.Compound.makeCompound([elbow_90, elbow_45_moved])
    cq.exporters.export(compound, "butt_weld_elbow_4in_90_vs_45.step")
    print("Exported butt_weld_elbow_4in_90_vs_45.step")
    print(f"  4\" 90° LR center-to-end: {dims_90.center_to_end_lr} mm")
    print(f"  4\" 45° center-to-end: {dims_45.center_to_end} mm")


def example_all_sizes_45():
    """Create 45° elbows for all available sizes."""
    solids = []
    offset = 0

    for nps, dims in ASME_B169_ELBOW45.items():
        try:
            elbow = make_butt_weld_elbow_45(nps)
            loc = cq.Location(cq.Vector(offset, 0, 0))
            solids.append(elbow.moved(loc))
            print(f"Created {nps}\" 45° elbow: OD={dims.od}mm, B={dims.center_to_end}mm")
            offset += dims.center_to_end * 2 + 20
        except Exception as e:
            print(f"Error creating {nps}\" 45° elbow: {e}")

    if solids:
        compound = cq.Compound.makeCompound(solids)
        cq.exporters.export(compound, "butt_weld_elbows_45_all_sizes.step")
        print(f"\nExported butt_weld_elbows_45_all_sizes.step with {len(solids)} elbows")


if __name__ == "__main__":
    print("ASME B16.9 Butt Weld 45° Elbow Generator")
    print("=" * 40)
    example_single_elbow_45()
    print()
    example_compare_90_vs_45()
    print()
    example_all_sizes_45()
    print("\nDone!")
