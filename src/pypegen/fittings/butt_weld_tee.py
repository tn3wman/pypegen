"""
ASME B16.9 Butt Weld Tee Generator using CadQuery.

This module creates butt weld equal tees per ASME B16.9 dimensions.
The tee has three ports: inlet (run-in), run (run-out), and branch.

Sources:
- https://www.skylandmetal.in/dimensions/asme-b16-9-equal-tee.html
- https://www.wermac.org/fittings/dim_tees_eq.html
"""

from __future__ import annotations

from dataclasses import dataclass

import cadquery as cq

# =============================================================================
# DATA STRUCTURES
# =============================================================================


@dataclass
class ButtWeldTeeDims:
    """ASME B16.9 Butt Weld Equal Tee dimensions (all in mm).

    For an equal tee, the run and branch dimensions are the same.
    """

    od: float  # Outside diameter
    wall_thickness: float  # Wall thickness (STD schedule)
    center_to_end_run: float  # C - center to weld end (run direction)
    center_to_end_branch: float  # M - center to weld end (branch direction)


# ASME B16.9 Butt Weld Equal Tee dimension table
# Wall thickness based on STD schedule (same as butt weld elbow)
ASME_B169_TEE: dict[str, ButtWeldTeeDims] = {
    "1/2": ButtWeldTeeDims(
        od=21.3,
        wall_thickness=2.78,
        center_to_end_run=25.0,
        center_to_end_branch=25.0,
    ),
    "3/4": ButtWeldTeeDims(
        od=26.7,
        wall_thickness=2.87,
        center_to_end_run=29.0,
        center_to_end_branch=29.0,
    ),
    "1": ButtWeldTeeDims(
        od=33.4,
        wall_thickness=3.38,
        center_to_end_run=38.0,
        center_to_end_branch=38.0,
    ),
    "1-1/4": ButtWeldTeeDims(
        od=42.2,
        wall_thickness=3.56,
        center_to_end_run=48.0,
        center_to_end_branch=48.0,
    ),
    "1-1/2": ButtWeldTeeDims(
        od=48.3,
        wall_thickness=3.68,
        center_to_end_run=57.0,
        center_to_end_branch=57.0,
    ),
    "2": ButtWeldTeeDims(
        od=60.3,
        wall_thickness=3.91,
        center_to_end_run=64.0,
        center_to_end_branch=64.0,
    ),
    "2-1/2": ButtWeldTeeDims(
        od=73.0,
        wall_thickness=5.16,
        center_to_end_run=76.0,
        center_to_end_branch=76.0,
    ),
    "3": ButtWeldTeeDims(
        od=88.9,
        wall_thickness=5.49,
        center_to_end_run=86.0,
        center_to_end_branch=86.0,
    ),
    "3-1/2": ButtWeldTeeDims(
        od=101.6,
        wall_thickness=5.74,
        center_to_end_run=95.0,
        center_to_end_branch=95.0,
    ),
    "4": ButtWeldTeeDims(
        od=114.3,
        wall_thickness=6.02,
        center_to_end_run=105.0,
        center_to_end_branch=105.0,
    ),
    "5": ButtWeldTeeDims(
        od=141.3,
        wall_thickness=6.55,
        center_to_end_run=124.0,
        center_to_end_branch=124.0,
    ),
    "6": ButtWeldTeeDims(
        od=168.3,
        wall_thickness=7.11,
        center_to_end_run=143.0,
        center_to_end_branch=143.0,
    ),
    "8": ButtWeldTeeDims(
        od=219.1,
        wall_thickness=8.18,
        center_to_end_run=178.0,
        center_to_end_branch=178.0,
    ),
    "10": ButtWeldTeeDims(
        od=273.0,
        wall_thickness=9.27,
        center_to_end_run=216.0,
        center_to_end_branch=216.0,
    ),
    "12": ButtWeldTeeDims(
        od=323.8,
        wall_thickness=9.52,
        center_to_end_run=254.0,
        center_to_end_branch=254.0,
    ),
}


# =============================================================================
# TEE GEOMETRY
# =============================================================================


def make_butt_weld_tee(
    nps: str,
    _schedule: str = "STD",  # Reserved for future wall thickness lookup
) -> cq.Shape:
    """
    Create an ASME B16.9 butt weld equal tee.

    The tee is oriented with:
    - Inlet (run-in) from -X direction toward origin
    - Run (run-out) from origin toward +X direction
    - Branch from origin toward +Y direction

    The center of the tee body is at the origin.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        _schedule: Pipe schedule (currently only "STD" implemented)

    Returns:
        CadQuery Shape of the tee
    """
    dims = ASME_B169_TEE.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.9 dimensions for NPS {nps}")

    # Get dimensions from ASME B16.9
    od = dims.od
    wall = dims.wall_thickness
    C_run = dims.center_to_end_run  # Center to end (run)
    M_branch = dims.center_to_end_branch  # Center to end (branch)

    # Radii
    r_outer = od / 2.0  # Outer radius
    r_inner = (od - 2 * wall) / 2.0  # Inner radius (bore)

    # Build the tee body
    # The tee consists of:
    # 1. A horizontal cylinder (run) from -X to +X
    # 2. A vertical cylinder (branch) from origin to +Y
    # The intersection creates a natural saddle geometry

    # Run cylinder (along X axis)
    run_cylinder = cq.Solid.makeCylinder(
        r_outer,
        C_run * 2,  # Full length from -X to +X
        cq.Vector(-C_run, 0, 0),  # Start at -C_run
        cq.Vector(1, 0, 0),  # Along +X
    )

    # Branch cylinder (along Y axis)
    branch_cylinder = cq.Solid.makeCylinder(
        r_outer,
        M_branch,  # Length from origin to +Y
        cq.Vector(0, 0, 0),  # Start at origin
        cq.Vector(0, 1, 0),  # Along +Y
    )

    # Union the cylinders to form the tee body
    tee_body = run_cylinder.fuse(branch_cylinder)

    # Cut the flow bore through all three directions
    # Run bore (X axis) - extra length to ensure full cut
    run_bore = cq.Solid.makeCylinder(
        r_inner,
        C_run * 2 + 10,
        cq.Vector(-C_run - 5, 0, 0),
        cq.Vector(1, 0, 0),
    )

    # Branch bore (Y axis)
    branch_bore = cq.Solid.makeCylinder(
        r_inner,
        M_branch + 5,
        cq.Vector(0, -2, 0),  # Start slightly inside to ensure intersection cut
        cq.Vector(0, 1, 0),
    )

    # Cut the bores
    tee_shape = tee_body.cut(run_bore).cut(branch_bore)

    return tee_shape


def get_butt_weld_tee_dims(nps: str) -> ButtWeldTeeDims:
    """
    Get the dimensions for a butt weld tee.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")

    Returns:
        ButtWeldTeeDims with all dimensions

    Raises:
        ValueError: If NPS is not supported
    """
    dims = ASME_B169_TEE.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.9 dimensions for NPS {nps}")

    return dims


# =============================================================================
# EXPORT UTILITIES
# =============================================================================


def export_to_step(shape: cq.Shape, filename: str):
    """Export a shape to STEP file."""
    cq.exporters.export(shape, filename)


# =============================================================================
# EXAMPLES
# =============================================================================


def example_single_tee():
    """Create a single 2" butt weld tee."""
    tee = make_butt_weld_tee("2")
    export_to_step(tee, "butt_weld_tee_2in.step")
    print("Exported butt_weld_tee_2in.step")

    dims = ASME_B169_TEE["2"]
    print(f"  OD: {dims.od} mm")
    print(f"  Wall: {dims.wall_thickness} mm")
    print(f"  Center-to-end (run): {dims.center_to_end_run} mm")
    print(f"  Center-to-end (branch): {dims.center_to_end_branch} mm")


if __name__ == "__main__":
    print("ASME B16.9 Butt Weld Tee Generator")
    print("=" * 40)
    example_single_tee()
    print("\nDone!")
