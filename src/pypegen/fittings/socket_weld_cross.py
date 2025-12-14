"""
ASME B16.11 Socket Weld Cross Generator using CadQuery.

This module creates socket weld crosses (equal crosses) per ASME B16.11 dimensions.
The cross has four ports: inlet (run-in), run (run-out), branch_left, and branch_right.

Sources:
- https://www.ferrobend.com/dimensions/ansi-asme/pipe-fitting/b16.11-socket-weld-class-3000/
- ASME B16.11 Forged Fittings, Socket-Welding and Threaded
"""

from __future__ import annotations

from dataclasses import dataclass

import cadquery as cq

from .socket_weld_elbow import add_socket_counterbores

# =============================================================================
# CONSTANTS
# =============================================================================

INCH = 25.4  # mm per inch


# =============================================================================
# DATA STRUCTURES
# =============================================================================


@dataclass
class SocketWeldCrossDims:
    """ASME B16.11 Socket Weld Cross dimensions (all in mm).

    For an equal cross, all four ends have the same dimensions.
    """

    center_to_end: float  # C - center to bottom of socket (all directions)
    socket_bore_max: float  # B max - socket bore diameter (fits pipe OD)
    socket_bore_min: float  # B min
    socket_depth: float  # J - depth of socket
    min_bore: float  # D min - minimum flow bore diameter
    socket_wall_thickness: float  # C - socket wall thickness (avg)
    body_wall_thickness: float  # G - body wall thickness (min)


# ASME B16.11 Class 3000 Socket Weld Equal Cross dimension table
# Note: Cross dimensions are the same as tee dimensions for each NPS
# Socket dimensions (B, J, C, G) match 90° elbow and tee for same NPS
ASME_B1611_CROSS_CLASS3000: dict[str, SocketWeldCrossDims] = {
    "1/8": SocketWeldCrossDims(
        center_to_end=9.5,
        socket_bore_max=11.2,
        socket_bore_min=10.8,
        socket_depth=9.5,
        min_bore=6.1,
        socket_wall_thickness=3.18,
        body_wall_thickness=3.18,
    ),
    "1/4": SocketWeldCrossDims(
        center_to_end=11.0,
        socket_bore_max=14.6,
        socket_bore_min=14.2,
        socket_depth=9.5,
        min_bore=8.5,
        socket_wall_thickness=3.78,
        body_wall_thickness=3.78,
    ),
    "3/8": SocketWeldCrossDims(
        center_to_end=13.5,
        socket_bore_max=18.0,
        socket_bore_min=17.6,
        socket_depth=9.5,
        min_bore=11.8,
        socket_wall_thickness=4.01,
        body_wall_thickness=4.01,
    ),
    "1/2": SocketWeldCrossDims(
        center_to_end=15.5,
        socket_bore_max=22.2,
        socket_bore_min=21.8,
        socket_depth=10.0,
        min_bore=15.0,
        socket_wall_thickness=4.67,
        body_wall_thickness=3.75,
    ),
    "3/4": SocketWeldCrossDims(
        center_to_end=19.0,
        socket_bore_max=27.6,
        socket_bore_min=27.2,
        socket_depth=13.0,
        min_bore=20.2,
        socket_wall_thickness=4.90,
        body_wall_thickness=3.90,
    ),
    "1": SocketWeldCrossDims(
        center_to_end=22.5,
        socket_bore_max=34.3,
        socket_bore_min=33.9,
        socket_depth=13.0,
        min_bore=25.9,
        socket_wall_thickness=5.69,
        body_wall_thickness=4.55,
    ),
    "1-1/4": SocketWeldCrossDims(
        center_to_end=27.0,
        socket_bore_max=43.1,
        socket_bore_min=42.7,
        socket_depth=13.0,
        min_bore=34.3,
        socket_wall_thickness=6.07,
        body_wall_thickness=4.85,
    ),
    "1-1/2": SocketWeldCrossDims(
        center_to_end=32.0,
        socket_bore_max=49.2,
        socket_bore_min=48.8,
        socket_depth=13.0,
        min_bore=40.1,
        socket_wall_thickness=6.35,
        body_wall_thickness=5.10,
    ),
    "2": SocketWeldCrossDims(
        center_to_end=38.0,
        socket_bore_max=61.7,
        socket_bore_min=61.2,
        socket_depth=16.0,
        min_bore=51.7,
        socket_wall_thickness=6.93,
        body_wall_thickness=5.55,
    ),
    "2-1/2": SocketWeldCrossDims(
        center_to_end=41.0,
        socket_bore_max=74.4,
        socket_bore_min=73.9,
        socket_depth=16.0,
        min_bore=61.2,
        socket_wall_thickness=8.76,
        body_wall_thickness=7.00,
    ),
    "3": SocketWeldCrossDims(
        center_to_end=57.0,
        socket_bore_max=90.3,
        socket_bore_min=89.8,
        socket_depth=16.0,
        min_bore=76.4,
        socket_wall_thickness=9.52,
        body_wall_thickness=7.60,
    ),
    "4": SocketWeldCrossDims(
        center_to_end=66.5,
        socket_bore_max=115.7,
        socket_bore_min=115.2,
        socket_depth=19.0,
        min_bore=100.7,
        socket_wall_thickness=10.69,
        body_wall_thickness=8.55,
    ),
}


# =============================================================================
# CROSS GEOMETRY
# =============================================================================


def make_socket_weld_cross(
    nps: str,
    pressure_class: int = 3000,
) -> cq.Shape:
    """
    Create an ASME B16.11 socket weld equal cross.

    The cross is oriented with:
    - Inlet (run-in) from -X direction toward origin
    - Run (run-out) from origin toward +X direction
    - Branch Left from origin toward +Y direction
    - Branch Right from origin toward -Y direction

    The center of the cross body is at the origin.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        pressure_class: 3000 or 6000 (only 3000 currently implemented)

    Returns:
        CadQuery Shape of the cross
    """
    if pressure_class != 3000:
        raise ValueError(f"Pressure class {pressure_class} not implemented")

    dims = ASME_B1611_CROSS_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.11 Class {pressure_class} dimensions for NPS {nps}")

    # Get dimensions from ASME B16.11
    C = dims.center_to_end  # Center to end of socket (all directions)
    B = (dims.socket_bore_max + dims.socket_bore_min) / 2.0  # Socket bore diameter
    D = dims.min_bore  # Flow bore diameter
    G = dims.body_wall_thickness  # Body wall thickness
    J = dims.socket_depth  # Socket depth

    # Radii
    r_bore = D / 2.0  # Flow bore radius
    r_body = r_bore + G  # Body outer radius (at center)
    r_socket_outer = B / 2.0 + dims.socket_wall_thickness  # Socket outer radius
    r_socket_inner = B / 2.0  # Socket inner radius (pipe OD fit)

    # Build the cross body
    # The cross consists of:
    # 1. A horizontal cylinder (run) from -X to +X
    # 2. A vertical cylinder (branches) from -Y to +Y

    # Run cylinder (along X axis)
    run_cylinder = cq.Solid.makeCylinder(
        r_body,
        C * 2,  # Full length from -X to +X
        cq.Vector(-C, 0, 0),  # Start at -C
        cq.Vector(1, 0, 0),  # Along +X
    )

    # Branch cylinder (along Y axis) - full length from -Y to +Y
    branch_cylinder = cq.Solid.makeCylinder(
        r_body,
        C * 2,  # Full length from -Y to +Y
        cq.Vector(0, -C, 0),  # Start at -C
        cq.Vector(0, 1, 0),  # Along +Y
    )

    # Union the cylinders to form the cross body
    cross_body = run_cylinder.fuse(branch_cylinder)

    # Cut the flow bore through all four directions
    # Run bore (X axis)
    run_bore = cq.Solid.makeCylinder(
        r_bore,
        C * 2 + 10,  # Extra length to ensure full cut
        cq.Vector(-C - 5, 0, 0),
        cq.Vector(1, 0, 0),
    )

    # Branch bore (Y axis)
    branch_bore = cq.Solid.makeCylinder(
        r_bore,
        C * 2 + 10,
        cq.Vector(0, -C - 5, 0),
        cq.Vector(0, 1, 0),
    )

    # Cut the bores
    cross_shape = cross_body.cut(run_bore).cut(branch_bore)

    # Add socket counterbores at all four ends
    cross_shape = add_socket_counterbores(
        cross_shape,
        r_socket_outer=r_socket_outer,
        r_socket_inner=r_socket_inner,
        socket_depth=J,
        min_face_area=500.0,
    )

    return cross_shape


def get_cross_dims(nps: str, pressure_class: int = 3000) -> SocketWeldCrossDims:
    """
    Get the dimensions for a socket weld cross.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        pressure_class: 3000 or 6000 (only 3000 currently implemented)

    Returns:
        SocketWeldCrossDims with all dimensions

    Raises:
        ValueError: If NPS or pressure class is not supported
    """
    if pressure_class != 3000:
        raise ValueError(f"Pressure class {pressure_class} not implemented")

    dims = ASME_B1611_CROSS_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.11 Class {pressure_class} dimensions for NPS {nps}")

    return dims
