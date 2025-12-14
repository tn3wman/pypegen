"""
ASME B16.9 Butt Weld Elbow Generator using CadQuery.

This module creates proper butt weld elbows with swept geometry
per ASME B16.9 dimensions.

Sources:
- https://www.zcpipefittings.com/90-degree-elbow.html
- https://blog.projectmaterials.com/category/products/piping/pipe-fittings/asme-b16-9-bw-fittings-sizes/
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
class ButtWeldElbowDims:
    """ASME B16.9 Butt Weld 90° Elbow dimensions (all in mm)."""

    od: float  # Outside diameter
    wall_thickness: float  # Wall thickness (schedule dependent)
    center_to_end_lr: float  # A dimension for long radius (1.5D)
    center_to_end_sr: float  # A dimension for short radius (1.0D)


# ASME B16.9 90° Butt Weld Elbow dimension table
# Long Radius: center-to-end = 1.5 x NPS
# Short Radius: center-to-end = 1.0 x NPS
# Key is NPS size string
ASME_B169_ELBOW90: dict[str, ButtWeldElbowDims] = {
    "1/2": ButtWeldElbowDims(
        od=21.3,
        wall_thickness=2.78,  # STD
        center_to_end_lr=38.1,
        center_to_end_sr=25.0,
    ),
    "3/4": ButtWeldElbowDims(
        od=26.7,
        wall_thickness=2.87,  # STD
        center_to_end_lr=38.1,  # Note: some sources show 28.6
        center_to_end_sr=25.0,
    ),
    "1": ButtWeldElbowDims(
        od=33.4,
        wall_thickness=3.38,  # STD
        center_to_end_lr=38.1,
        center_to_end_sr=25.0,
    ),
    "1-1/4": ButtWeldElbowDims(
        od=42.2,
        wall_thickness=3.56,  # STD
        center_to_end_lr=47.6,
        center_to_end_sr=32.0,
    ),
    "1-1/2": ButtWeldElbowDims(
        od=48.3,
        wall_thickness=3.68,  # STD
        center_to_end_lr=57.1,
        center_to_end_sr=38.0,
    ),
    "2": ButtWeldElbowDims(
        od=60.3,
        wall_thickness=3.91,  # STD
        center_to_end_lr=76.2,
        center_to_end_sr=51.0,
    ),
    "2-1/2": ButtWeldElbowDims(
        od=73.0,
        wall_thickness=5.16,  # STD
        center_to_end_lr=95.2,
        center_to_end_sr=64.0,
    ),
    "3": ButtWeldElbowDims(
        od=88.9,
        wall_thickness=5.49,  # STD
        center_to_end_lr=114.3,
        center_to_end_sr=76.0,
    ),
    "4": ButtWeldElbowDims(
        od=114.3,
        wall_thickness=6.02,  # STD
        center_to_end_lr=152.4,
        center_to_end_sr=102.0,
    ),
    "5": ButtWeldElbowDims(
        od=141.3,
        wall_thickness=6.55,  # STD
        center_to_end_lr=190.5,
        center_to_end_sr=127.0,
    ),
    "6": ButtWeldElbowDims(
        od=168.3,
        wall_thickness=7.11,  # STD
        center_to_end_lr=228.6,
        center_to_end_sr=152.0,
    ),
    "8": ButtWeldElbowDims(
        od=219.1,
        wall_thickness=8.18,  # STD
        center_to_end_lr=304.8,
        center_to_end_sr=203.0,
    ),
    "10": ButtWeldElbowDims(
        od=273.0,
        wall_thickness=9.27,  # STD
        center_to_end_lr=381.0,
        center_to_end_sr=254.0,
    ),
    "12": ButtWeldElbowDims(
        od=323.8,
        wall_thickness=9.52,  # STD
        center_to_end_lr=457.2,
        center_to_end_sr=305.0,
    ),
}


# =============================================================================
# ELBOW GEOMETRY
# =============================================================================


def make_butt_weld_elbow_90(
    nps: str,
    radius_type: Literal["LR", "SR"] = "LR",
    _schedule: str = "STD",  # Reserved for future wall thickness lookup
) -> cq.Shape:
    """
    Create an ASME B16.9 90° butt weld elbow.

    The elbow is oriented with:
    - One end on the +X axis (pipe would connect from +X direction)
    - Other end on the +Y axis (pipe would connect from +Y direction)
    - The bend center at the origin

    Args:
        nps: Nominal pipe size (e.g., "2", "4", "6")
        radius_type: "LR" for long radius (1.5D) or "SR" for short radius (1.0D)
        schedule: Pipe schedule (currently only "STD" implemented)

    Returns:
        CadQuery Shape of the elbow
    """
    dims = ASME_B169_ELBOW90.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.9 dimensions for NPS {nps}")

    # Get dimensions
    od = dims.od
    wall = dims.wall_thickness
    id_ = od - 2 * wall

    if radius_type == "LR":
        center_to_end = dims.center_to_end_lr
    else:
        center_to_end = dims.center_to_end_sr

    # The bend radius is the distance from the bend center to the pipe centerline
    # For a 90° elbow, this equals the center-to-end dimension
    bend_radius = center_to_end

    # Radii for the pipe cross-section
    r_outer = od / 2.0
    r_inner = id_ / 2.0

    # Create the elbow by sweeping a circular annulus along a 90° arc
    #
    # Geometry setup:
    # - The elbow lies in the XY plane
    # - Bend center is at origin
    # - Arc goes from (bend_radius, 0, 0) to (0, bend_radius, 0)
    # - At each point on the arc, the pipe cross-section is perpendicular
    #   to the arc tangent direction
    #
    # For a 90° arc in the XY plane centered at origin:
    # - Start point: (R, 0, 0), tangent direction: (0, 1, 0) = +Y
    # - End point: (0, R, 0), tangent direction: (-1, 0, 0) = -X
    #
    # The pipe cross-section (a circle) must be perpendicular to the tangent.
    # At the start, tangent is +Y, so cross-section is in the XZ plane.

    # Create the 90° arc path
    # Arc from (bend_radius, 0) to (0, bend_radius) via midpoint at 45°
    mid_x = bend_radius * math.cos(math.pi / 4)
    mid_y = bend_radius * math.sin(math.pi / 4)

    arc_path = (
        cq.Workplane("XY")
        .moveTo(bend_radius, 0)
        .threePointArc((mid_x, mid_y), (0, bend_radius))
    )

    # Create the pipe cross-section profile at the start of the arc
    # At (bend_radius, 0, 0), the tangent is +Y
    # So the profile plane is perpendicular to Y, which is the XZ plane
    # But we need to position it at x = bend_radius

    # The profile is an annulus (ring) - outer circle minus inner circle
    # CadQuery sweep works best with a single face, so we create the
    # annular profile as a face with a hole

    # Create outer profile at the start point
    outer_profile = (
        cq.Workplane("YZ")  # Plane perpendicular to X axis
        .transformed(offset=(bend_radius, 0, 0))  # Move to start of arc
        .circle(r_outer)
    )

    # Sweep outer along arc
    outer_sweep = outer_profile.sweep(arc_path, isFrenet=True)

    # Create inner profile (for the bore)
    inner_profile = (
        cq.Workplane("YZ")
        .transformed(offset=(bend_radius, 0, 0))
        .circle(r_inner)
    )

    # Sweep inner along arc
    inner_sweep = inner_profile.sweep(arc_path, isFrenet=True)

    # Subtract inner from outer to get hollow pipe
    elbow = outer_sweep.cut(inner_sweep)

    result = elbow.solids().val()
    assert isinstance(result, cq.Shape)
    return result


def make_solid_elbow_90(
    od: float,
    wall_thickness: float,
    bend_radius: float,
) -> cq.Shape:
    """
    Create a 90° elbow with custom dimensions.

    This is a lower-level function for creating elbows with arbitrary dimensions.

    Args:
        od: Outside diameter (mm)
        wall_thickness: Wall thickness (mm)
        bend_radius: Bend radius / center-to-end dimension (mm)

    Returns:
        CadQuery Shape of the elbow
    """
    r_outer = od / 2.0
    r_inner = (od - 2 * wall_thickness) / 2.0

    # Create the 90° arc path
    mid_x = bend_radius * math.cos(math.pi / 4)
    mid_y = bend_radius * math.sin(math.pi / 4)

    arc_path = (
        cq.Workplane("XY")
        .moveTo(bend_radius, 0)
        .threePointArc((mid_x, mid_y), (0, bend_radius))
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


def example_single_elbow():
    """Create a single 2" long radius elbow."""
    elbow = make_butt_weld_elbow_90("2", radius_type="LR")
    export_to_step(elbow, "butt_weld_elbow_2in_LR.step")
    print("Exported butt_weld_elbow_2in_LR.step")

    dims = ASME_B169_ELBOW90["2"]
    print(f"  OD: {dims.od} mm")
    print(f"  Wall: {dims.wall_thickness} mm")
    print(f"  Center-to-end (LR): {dims.center_to_end_lr} mm")


def example_compare_lr_sr():
    """Create both long radius and short radius elbows for comparison."""
    elbow_lr = make_butt_weld_elbow_90("4", radius_type="LR")
    elbow_sr = make_butt_weld_elbow_90("4", radius_type="SR")

    # Offset the SR elbow so they don't overlap
    dims = ASME_B169_ELBOW90["4"]
    offset = dims.center_to_end_lr + dims.center_to_end_sr + 50

    elbow_sr_moved = elbow_sr.moved(cq.Location(cq.Vector(offset, 0, 0)))

    # Combine and export
    compound = cq.Compound.makeCompound([elbow_lr, elbow_sr_moved])
    cq.exporters.export(compound, "butt_weld_elbow_4in_LR_vs_SR.step")
    print("Exported butt_weld_elbow_4in_LR_vs_SR.step")
    print(f"  4\" LR center-to-end: {dims.center_to_end_lr} mm")
    print(f"  4\" SR center-to-end: {dims.center_to_end_sr} mm")


def example_all_sizes():
    """Create elbows for all available sizes."""
    solids = []
    offset = 0

    for nps, dims in ASME_B169_ELBOW90.items():
        try:
            elbow = make_butt_weld_elbow_90(nps, radius_type="LR")
            loc = cq.Location(cq.Vector(offset, 0, 0))
            solids.append(elbow.moved(loc))
            print(f"Created {nps}\" elbow: OD={dims.od}mm, A={dims.center_to_end_lr}mm")
            offset += dims.center_to_end_lr * 2 + 20
        except Exception as e:
            print(f"Error creating {nps}\" elbow: {e}")

    if solids:
        compound = cq.Compound.makeCompound(solids)
        cq.exporters.export(compound, "butt_weld_elbows_all_sizes.step")
        print(f"\nExported butt_weld_elbows_all_sizes.step with {len(solids)} elbows")


if __name__ == "__main__":
    print("ASME B16.9 Butt Weld Elbow Generator")
    print("=" * 40)
    example_single_elbow()
    print()
    example_compare_lr_sr()
    print()
    example_all_sizes()
    print("\nDone!")
