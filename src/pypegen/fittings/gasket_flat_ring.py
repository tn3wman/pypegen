#!/usr/bin/env python3
"""
ASME B16.21 Flat Ring Gasket Generator

Generates non-metallic flat ring gaskets for raised face flanges per ASME B16.21.

These gaskets are used with ASME B16.5 Class 150, 300, 400, 600 raised face flanges.
The gasket sits between two flange raised faces and provides the seal when bolts
are tightened.

References:
- ASME B16.21-2021: Nonmetallic Flat Gaskets for Pipe Flanges
- https://www.wermac.org/gaskets/dim_gaskets_fl.html
"""

from dataclasses import dataclass

import cadquery as cq


# =============================================================================
# ASME B16.21 FLAT RING GASKET DIMENSIONS
# =============================================================================


@dataclass
class FlatRingGasketDims:
    """
    Dimensions for an ASME B16.21 flat ring gasket (mm).

    Flat ring gaskets are simple annular rings that sit between
    the raised faces of two mating flanges.

    Attributes:
        nps: Nominal pipe size
        inner_diameter: Inside diameter (mm) - matches or slightly larger than pipe bore
        outer_diameter: Outside diameter (mm) - fits within raised face diameter
        thickness: Standard compressed thickness (mm) - typically 1.6mm (1/16") or 3.2mm (1/8")
    """

    nps: str
    inner_diameter: float  # ID in mm
    outer_diameter: float  # OD in mm
    thickness: float = 1.6  # Default 1/16" (1.6mm)


# ASME B16.21 Class 300 Flat Ring Gasket Dimensions (mm)
# For use with ASME B16.5 Class 300 raised face flanges
# ID = approximately matches pipe bore
# OD = fits within raised face diameter
ASME_B1621_CLASS300_FLAT: dict[str, FlatRingGasketDims] = {
    "1/2": FlatRingGasketDims(nps="1/2", inner_diameter=21, outer_diameter=54, thickness=1.6),
    "3/4": FlatRingGasketDims(nps="3/4", inner_diameter=27, outer_diameter=67, thickness=1.6),
    "1": FlatRingGasketDims(nps="1", inner_diameter=33, outer_diameter=73, thickness=1.6),
    "1-1/4": FlatRingGasketDims(nps="1-1/4", inner_diameter=42, outer_diameter=83, thickness=1.6),
    "1-1/2": FlatRingGasketDims(nps="1-1/2", inner_diameter=48, outer_diameter=95, thickness=1.6),
    "2": FlatRingGasketDims(nps="2", inner_diameter=60, outer_diameter=111, thickness=1.6),
    "2-1/2": FlatRingGasketDims(nps="2-1/2", inner_diameter=73, outer_diameter=130, thickness=1.6),
    "3": FlatRingGasketDims(nps="3", inner_diameter=89, outer_diameter=149, thickness=1.6),
    "3-1/2": FlatRingGasketDims(nps="3-1/2", inner_diameter=102, outer_diameter=165, thickness=1.6),
    "4": FlatRingGasketDims(nps="4", inner_diameter=114, outer_diameter=181, thickness=1.6),
    "5": FlatRingGasketDims(nps="5", inner_diameter=141, outer_diameter=216, thickness=1.6),
    "6": FlatRingGasketDims(nps="6", inner_diameter=168, outer_diameter=251, thickness=1.6),
    "8": FlatRingGasketDims(nps="8", inner_diameter=219, outer_diameter=308, thickness=1.6),
    "10": FlatRingGasketDims(nps="10", inner_diameter=273, outer_diameter=362, thickness=1.6),
    "12": FlatRingGasketDims(nps="12", inner_diameter=324, outer_diameter=422, thickness=1.6),
    "14": FlatRingGasketDims(nps="14", inner_diameter=356, outer_diameter=486, thickness=1.6),
    "16": FlatRingGasketDims(nps="16", inner_diameter=406, outer_diameter=540, thickness=1.6),
    "18": FlatRingGasketDims(nps="18", inner_diameter=457, outer_diameter=597, thickness=1.6),
    "20": FlatRingGasketDims(nps="20", inner_diameter=508, outer_diameter=654, thickness=1.6),
    "24": FlatRingGasketDims(nps="24", inner_diameter=610, outer_diameter=775, thickness=1.6),
}

# ASME B16.21 Class 150 Flat Ring Gasket Dimensions (mm)
# Class 150 has slightly smaller OD than Class 300
ASME_B1621_CLASS150_FLAT: dict[str, FlatRingGasketDims] = {
    "1/2": FlatRingGasketDims(nps="1/2", inner_diameter=21, outer_diameter=48, thickness=1.6),
    "3/4": FlatRingGasketDims(nps="3/4", inner_diameter=27, outer_diameter=57, thickness=1.6),
    "1": FlatRingGasketDims(nps="1", inner_diameter=33, outer_diameter=67, thickness=1.6),
    "1-1/4": FlatRingGasketDims(nps="1-1/4", inner_diameter=42, outer_diameter=76, thickness=1.6),
    "1-1/2": FlatRingGasketDims(nps="1-1/2", inner_diameter=48, outer_diameter=86, thickness=1.6),
    "2": FlatRingGasketDims(nps="2", inner_diameter=60, outer_diameter=105, thickness=1.6),
    "2-1/2": FlatRingGasketDims(nps="2-1/2", inner_diameter=73, outer_diameter=124, thickness=1.6),
    "3": FlatRingGasketDims(nps="3", inner_diameter=89, outer_diameter=137, thickness=1.6),
    "3-1/2": FlatRingGasketDims(nps="3-1/2", inner_diameter=102, outer_diameter=162, thickness=1.6),
    "4": FlatRingGasketDims(nps="4", inner_diameter=114, outer_diameter=175, thickness=1.6),
    "5": FlatRingGasketDims(nps="5", inner_diameter=141, outer_diameter=197, thickness=1.6),
    "6": FlatRingGasketDims(nps="6", inner_diameter=168, outer_diameter=222, thickness=1.6),
    "8": FlatRingGasketDims(nps="8", inner_diameter=219, outer_diameter=279, thickness=1.6),
    "10": FlatRingGasketDims(nps="10", inner_diameter=273, outer_diameter=340, thickness=1.6),
    "12": FlatRingGasketDims(nps="12", inner_diameter=324, outer_diameter=410, thickness=1.6),
    "14": FlatRingGasketDims(nps="14", inner_diameter=356, outer_diameter=451, thickness=1.6),
    "16": FlatRingGasketDims(nps="16", inner_diameter=406, outer_diameter=514, thickness=1.6),
    "18": FlatRingGasketDims(nps="18", inner_diameter=457, outer_diameter=549, thickness=1.6),
    "20": FlatRingGasketDims(nps="20", inner_diameter=508, outer_diameter=606, thickness=1.6),
    "24": FlatRingGasketDims(nps="24", inner_diameter=610, outer_diameter=718, thickness=1.6),
}


# =============================================================================
# GASKET GEOMETRY
# =============================================================================


def make_flat_ring_gasket(
    nps: str,
    thickness_mm: float | None = None,
    pressure_class: int = 300,
) -> cq.Shape:
    """
    Create an ASME B16.21 flat ring gasket.

    The gasket is oriented with:
    - Center at origin (0, 0, 0)
    - Gasket lies in XY plane
    - Thickness centered around Z=0 (from Z=-t/2 to Z=+t/2)
    - "face_a" (top face) at Z=+t/2
    - "face_b" (bottom face) at Z=-t/2

    Args:
        nps: Nominal pipe size (e.g., "2", "4", "6")
        thickness_mm: Override thickness in mm (default uses standard 1.6mm)
        pressure_class: 150 or 300 (determines gasket OD)

    Returns:
        CadQuery Shape of the gasket
    """
    # Select dimension table based on pressure class
    if pressure_class == 150:
        dims_table = ASME_B1621_CLASS150_FLAT
    elif pressure_class == 300:
        dims_table = ASME_B1621_CLASS300_FLAT
    else:
        raise ValueError(f"Unsupported pressure class: {pressure_class}. Use 150 or 300.")

    dims = dims_table.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.21 Class {pressure_class} flat ring gasket dimensions for NPS {nps}")

    # Use provided thickness or default from dimensions
    thickness = thickness_mm if thickness_mm is not None else dims.thickness

    # Radii
    r_outer = dims.outer_diameter / 2
    r_inner = dims.inner_diameter / 2

    # Create annular ring centered at origin
    # Gasket extends from Z=-thickness/2 to Z=+thickness/2
    z_start = -thickness / 2

    outer = cq.Solid.makeCylinder(
        r_outer,
        thickness,
        cq.Vector(0, 0, z_start),
        cq.Vector(0, 0, 1),
    )
    inner = cq.Solid.makeCylinder(
        r_inner,
        thickness,
        cq.Vector(0, 0, z_start),
        cq.Vector(0, 0, 1),
    )

    gasket = outer.cut(inner)

    return gasket


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================


def get_gasket_thickness(nps: str, pressure_class: int = 300) -> float:
    """
    Get the standard thickness for a flat ring gasket.

    Args:
        nps: Nominal pipe size
        pressure_class: 150 or 300

    Returns:
        Thickness in mm
    """
    if pressure_class == 150:
        dims = ASME_B1621_CLASS150_FLAT.get(nps)
    else:
        dims = ASME_B1621_CLASS300_FLAT.get(nps)

    if dims is None:
        raise ValueError(f"No gasket dimensions for NPS {nps} Class {pressure_class}")

    return dims.thickness


def get_available_sizes(pressure_class: int = 300) -> list[str]:
    """
    Get list of available NPS sizes for flat ring gaskets.

    Args:
        pressure_class: 150 or 300

    Returns:
        List of NPS size strings
    """
    if pressure_class == 150:
        return list(ASME_B1621_CLASS150_FLAT.keys())
    else:
        return list(ASME_B1621_CLASS300_FLAT.keys())


# =============================================================================
# EXPORT UTILITIES
# =============================================================================


def export_to_step(shape: cq.Shape, filename: str):
    """Export a shape to STEP file."""
    cq.exporters.export(shape, filename)


# =============================================================================
# EXAMPLES
# =============================================================================


def example_single_gasket():
    """Create a single 2" flat ring gasket."""
    print("ASME B16.21 Flat Ring Gasket Generator")
    print("=" * 50)

    gasket = make_flat_ring_gasket("2", pressure_class=300)
    export_to_step(gasket, "step/flat_ring_gasket_2in.step")

    dims = ASME_B1621_CLASS300_FLAT["2"]
    print("Exported step/flat_ring_gasket_2in.step")
    print(f"  NPS: 2\"")
    print(f"  Inner diameter: {dims.inner_diameter:.1f} mm")
    print(f"  Outer diameter: {dims.outer_diameter:.1f} mm")
    print(f"  Thickness: {dims.thickness:.1f} mm")


def example_all_sizes():
    """Create gaskets for several common sizes."""
    print("\nCreating gaskets for common sizes...")

    gaskets = []
    spacing = 0

    for nps in ["1", "2", "4", "6"]:
        dims = ASME_B1621_CLASS300_FLAT[nps]
        gasket = make_flat_ring_gasket(nps)

        # Offset for display
        gasket_moved = gasket.moved(cq.Location(cq.Vector(spacing, 0, 0)))
        gaskets.append(gasket_moved)

        print(f"Created {nps}\" gasket: ID={dims.inner_diameter:.0f}mm, OD={dims.outer_diameter:.0f}mm")

        spacing += dims.outer_diameter + 20

    if gaskets:
        compound = cq.Compound.makeCompound(gaskets)
        export_to_step(compound, "step/flat_ring_gaskets_sample.step")
        print(f"\nExported step/flat_ring_gaskets_sample.step with {len(gaskets)} gaskets")


if __name__ == "__main__":
    example_single_gasket()
    example_all_sizes()
    print("\nDone!")
