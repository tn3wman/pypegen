"""
ASME B16.11 Socket Weld Elbow Generator using CadQuery.

This module creates proper socket weld elbows with swept geometry
per ASME B16.11 dimensions.

Sources:
- https://www.wermac.org/fittings/dim_sw_elbows.html
- https://www.ferrobend.com/dimensions/ansi-asme/pipe-fitting/b16.11-socket-weld-class-3000-90-degree-elbow/
- https://www.marcelpiping.com/asme-b16-11-forged-socket-weld-90-degree-pipe-elbow/
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import cast

import cadquery as cq

# =============================================================================
# CONSTANTS
# =============================================================================

INCH = 25.4  # mm per inch


# =============================================================================
# GEOMETRY UTILITIES
# =============================================================================

def add_socket_counterbores(
    shape: cq.Shape,
    r_socket_outer: float,
    r_socket_inner: float,
    socket_depth: float,
    min_face_area: float = 500.0,
) -> cq.Shape:
    """
    Add socket counterbores to planar end faces of a shape.

    This function finds planar faces larger than min_face_area and adds
    socket cylinders extending outward along the face normal.

    Args:
        shape: The CadQuery shape to modify
        r_socket_outer: Outer radius of socket counterbore
        r_socket_inner: Inner radius (bore) of socket counterbore
        socket_depth: Depth of socket counterbore
        min_face_area: Minimum face area to consider (filters small faces)

    Returns:
        Modified shape with socket counterbores added
    """
    wp = cq.Workplane().add(shape)
    faces = wp.faces().vals()

    for face_obj in faces:
        face = cast(cq.Face, face_obj)
        if face.geomType() == "PLANE":
            center = face.Center()
            normal = face.normalAt(center)  # type: ignore[call-arg]
            area = face.Area()
            if area > min_face_area:
                sock_outer = cq.Solid.makeCylinder(
                    r_socket_outer, socket_depth,
                    cq.Vector(center.x, center.y, center.z),
                    cq.Vector(normal.x, normal.y, normal.z)
                )
                sock_inner = cq.Solid.makeCylinder(
                    r_socket_inner, socket_depth,
                    cq.Vector(center.x, center.y, center.z),
                    cq.Vector(normal.x, normal.y, normal.z)
                )
                shape = shape.fuse(sock_outer).cut(sock_inner)

    return shape


# =============================================================================
# DATA STRUCTURES
# =============================================================================


@dataclass
class SocketWeldElbowDims:
    """ASME B16.11 Socket Weld 90° Elbow dimensions (all in mm)."""

    center_to_end: float  # A - center to bottom of socket
    socket_bore_max: float  # B max - socket bore diameter (fits pipe OD)
    socket_bore_min: float  # B min
    socket_depth: float  # J - depth of socket
    min_bore: float  # D min - minimum flow bore diameter
    socket_wall_thickness: float  # C - socket wall thickness (avg)
    body_wall_thickness: float  # G - body wall thickness (min)


# ASME B16.11 Class 3000 Socket Weld 90° Elbow dimension table
ASME_B1611_ELBOW90_CLASS3000: dict[str, SocketWeldElbowDims] = {
    "1/8": SocketWeldElbowDims(
        center_to_end=9.5,
        socket_bore_max=11.2,
        socket_bore_min=10.8,
        socket_depth=9.5,
        min_bore=6.1,
        socket_wall_thickness=3.18,
        body_wall_thickness=3.18,
    ),
    "1/4": SocketWeldElbowDims(
        center_to_end=11.0,
        socket_bore_max=14.6,
        socket_bore_min=14.2,
        socket_depth=9.5,
        min_bore=8.5,
        socket_wall_thickness=3.78,
        body_wall_thickness=3.78,
    ),
    "3/8": SocketWeldElbowDims(
        center_to_end=13.5,
        socket_bore_max=18.0,
        socket_bore_min=17.6,
        socket_depth=9.5,
        min_bore=11.8,
        socket_wall_thickness=4.01,
        body_wall_thickness=4.01,
    ),
    "1/2": SocketWeldElbowDims(
        center_to_end=15.5,
        socket_bore_max=22.2,
        socket_bore_min=21.8,
        socket_depth=10.0,
        min_bore=15.0,
        socket_wall_thickness=4.67,
        body_wall_thickness=3.75,
    ),
    "3/4": SocketWeldElbowDims(
        center_to_end=19.0,
        socket_bore_max=27.6,
        socket_bore_min=27.2,
        socket_depth=13.0,
        min_bore=20.2,
        socket_wall_thickness=4.90,
        body_wall_thickness=3.90,
    ),
    "1": SocketWeldElbowDims(
        center_to_end=22.5,
        socket_bore_max=34.3,
        socket_bore_min=33.9,
        socket_depth=13.0,
        min_bore=25.9,
        socket_wall_thickness=5.69,
        body_wall_thickness=4.55,
    ),
    "1-1/4": SocketWeldElbowDims(
        center_to_end=27.0,
        socket_bore_max=43.1,
        socket_bore_min=42.7,
        socket_depth=13.0,
        min_bore=34.3,
        socket_wall_thickness=6.07,
        body_wall_thickness=4.85,
    ),
    "1-1/2": SocketWeldElbowDims(
        center_to_end=32.0,
        socket_bore_max=49.2,
        socket_bore_min=48.8,
        socket_depth=13.0,
        min_bore=40.1,
        socket_wall_thickness=6.35,
        body_wall_thickness=5.10,
    ),
    "2": SocketWeldElbowDims(
        center_to_end=38.0,
        socket_bore_max=61.7,
        socket_bore_min=61.2,
        socket_depth=16.0,
        min_bore=51.7,
        socket_wall_thickness=6.93,
        body_wall_thickness=5.55,
    ),
    "2-1/2": SocketWeldElbowDims(
        center_to_end=41.0,
        socket_bore_max=74.4,
        socket_bore_min=73.9,
        socket_depth=16.0,
        min_bore=61.2,
        socket_wall_thickness=8.76,
        body_wall_thickness=7.00,
    ),
    "3": SocketWeldElbowDims(
        center_to_end=57.0,
        socket_bore_max=90.3,
        socket_bore_min=89.8,
        socket_depth=16.0,
        min_bore=76.4,
        socket_wall_thickness=9.52,
        body_wall_thickness=7.60,
    ),
    "4": SocketWeldElbowDims(
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
# ELBOW GEOMETRY
# =============================================================================


def make_socket_weld_elbow_90(
    nps: str,
    pressure_class: int = 3000,
) -> cq.Shape:
    """
    Create an ASME B16.11 90° socket weld elbow.

    The elbow is oriented with:
    - Inlet from +X direction toward origin
    - Outlet from origin toward +Y direction
    - A sphere at origin connects the two legs

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        pressure_class: 3000 or 6000 (only 3000 currently implemented)

    Returns:
        CadQuery Shape of the elbow
    """
    if pressure_class != 3000:
        raise ValueError(f"Pressure class {pressure_class} not implemented")

    dims = ASME_B1611_ELBOW90_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.11 Class {pressure_class} dimensions for NPS {nps}")

    # Get dimensions from ASME B16.11
    A = dims.center_to_end  # Center to end of socket
    B = (dims.socket_bore_max + dims.socket_bore_min) / 2.0  # Socket bore diameter
    C = dims.socket_wall_thickness  # Socket wall thickness
    D = dims.min_bore  # Flow bore diameter
    G = dims.body_wall_thickness  # Body wall thickness
    J = dims.socket_depth  # Socket depth

    # Radii
    r_bore = D / 2.0  # Flow bore radius
    r_outer = r_bore + G  # Outer radius of elbow body
    r_socket_inner = B / 2.0  # Socket bore radius (accepts pipe OD)
    r_socket_outer = r_socket_inner + C  # Outer radius of socket

    # Outer body: two cylinders + full sphere at origin
    # Leg 1: from (A, 0, 0) to origin along -X
    leg1_outer = cq.Solid.makeCylinder(
        r_outer, A,
        cq.Vector(A, 0, 0),
        cq.Vector(-1, 0, 0)
    )
    # Leg 2: from origin to (0, A, 0) along +Y
    leg2_outer = cq.Solid.makeCylinder(
        r_outer, A,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 1, 0)
    )
    # Full sphere at origin (angleDegrees1=-90, angleDegrees2=90 for full sphere)
    sphere_outer = cq.Solid.makeSphere(
        r_outer,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=360
    )

    outer = leg1_outer.fuse(leg2_outer).fuse(sphere_outer)

    # Inner bore: same shape with r_bore
    leg1_inner = cq.Solid.makeCylinder(
        r_bore, A,
        cq.Vector(A, 0, 0),
        cq.Vector(-1, 0, 0)
    )
    leg2_inner = cq.Solid.makeCylinder(
        r_bore, A,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 1, 0)
    )
    sphere_inner = cq.Solid.makeSphere(
        r_bore,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=360
    )

    inner = leg1_inner.fuse(leg2_inner).fuse(sphere_inner)

    # Cut inner from outer to create hollow elbow
    elbow = outer.cut(inner)

    # Add socket counterbores to end faces
    elbow = add_socket_counterbores(elbow, r_socket_outer, r_socket_inner, J)

    return elbow


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
    """Create a single 2" Class 3000 socket weld elbow."""
    elbow = make_socket_weld_elbow_90("2", pressure_class=3000)
    export_to_step(elbow, "socket_weld_elbow_2in_cl3000.step")

    dims = ASME_B1611_ELBOW90_CLASS3000["2"]
    print("Exported socket_weld_elbow_2in_cl3000.step")
    print(f"  Center-to-end: {dims.center_to_end} mm")
    print(f"  Socket bore: {(dims.socket_bore_max + dims.socket_bore_min) / 2:.1f} mm")
    print(f"  Socket depth: {dims.socket_depth} mm")
    print(f"  Min bore: {dims.min_bore} mm")


def example_all_sizes():
    """Create socket weld elbows for all available sizes."""
    solids = []
    offset = 0

    for nps, dims in ASME_B1611_ELBOW90_CLASS3000.items():
        try:
            elbow = make_socket_weld_elbow_90(nps, pressure_class=3000)
            loc = cq.Location(cq.Vector(offset, 0, 0))
            solids.append(elbow.moved(loc))
            print(f"Created {nps}\" elbow: A={dims.center_to_end}mm, bore={dims.min_bore}mm")
            offset += dims.center_to_end * 3
        except Exception as e:
            print(f"Error creating {nps}\" elbow: {e}")

    if solids:
        compound = cq.Compound.makeCompound(solids)
        cq.exporters.export(compound, "socket_weld_elbows_all_sizes.step")
        print(f"\nExported socket_weld_elbows_all_sizes.step with {len(solids)} elbows")


if __name__ == "__main__":
    print("ASME B16.11 Socket Weld Elbow Generator")
    print("=" * 40)
    example_single_elbow()
    print()
    example_all_sizes()
    print("\nDone!")
