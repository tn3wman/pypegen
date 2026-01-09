#!/usr/bin/env python3
"""
ASME B16.20 Spiral Wound Gasket Generator

Generates metallic spiral wound gaskets for raised face flanges per ASME B16.20.

Spiral wound gaskets consist of:
- Inner centering ring (optional, for vacuum/pressure surge protection)
- Sealing element (V-shaped metal strip wound with soft filler)
- Outer centering ring (positions gasket within bolt circle)

These gaskets are used for higher pressure and temperature applications
compared to flat ring gaskets.

References:
- ASME B16.20-2017: Metallic Gaskets for Pipe Flanges
- https://www.wermac.org/gaskets/dim_gaskets_sw300.html
"""

from dataclasses import dataclass

import cadquery as cq


# =============================================================================
# ASME B16.20 SPIRAL WOUND GASKET DIMENSIONS
# =============================================================================


@dataclass
class SpiralWoundGasketDims:
    """
    Dimensions for an ASME B16.20 spiral wound gasket (mm).

    Spiral wound gaskets have three concentric sections:
    - Inner ring: Solid metal ring protecting sealing element from process media
    - Sealing element: Spiral wound metal/filler that provides the seal
    - Outer ring: Centering ring that positions gasket within bolt circle

    Attributes:
        nps: Nominal pipe size
        inner_ring_id: Inside diameter of inner centering ring (d1)
        sealing_element_id: Inside diameter of sealing element (d2)
        sealing_element_od: Outside diameter of sealing element (d3)
        outer_ring_od: Outside diameter of outer centering ring (d4)
        compressed_thickness: Typical compressed thickness under bolt load (mm)
    """

    nps: str
    inner_ring_id: float  # d1 - inner ring ID (mm)
    sealing_element_id: float  # d2 - sealing element ID (mm)
    sealing_element_od: float  # d3 - sealing element OD (mm)
    outer_ring_od: float  # d4 - outer ring OD (mm)
    compressed_thickness: float = 4.5  # Typical compressed thickness (mm)


# ASME B16.20 Class 300 Spiral Wound Gasket Dimensions (mm)
# For use with ASME B16.5 Class 300 raised face flanges
ASME_B1620_CLASS300_SPIRAL: dict[str, SpiralWoundGasketDims] = {
    "1/2": SpiralWoundGasketDims(
        nps="1/2",
        inner_ring_id=14.2,
        sealing_element_id=19.1,
        sealing_element_od=31.8,
        outer_ring_od=54.1,
    ),
    "3/4": SpiralWoundGasketDims(
        nps="3/4",
        inner_ring_id=20.6,
        sealing_element_id=25.4,
        sealing_element_od=39.6,
        outer_ring_od=66.8,
    ),
    "1": SpiralWoundGasketDims(
        nps="1",
        inner_ring_id=26.9,
        sealing_element_id=31.8,
        sealing_element_od=47.8,
        outer_ring_od=73.2,
    ),
    "1-1/4": SpiralWoundGasketDims(
        nps="1-1/4",
        inner_ring_id=38.1,
        sealing_element_id=47.8,
        sealing_element_od=60.5,
        outer_ring_od=82.6,
    ),
    "1-1/2": SpiralWoundGasketDims(
        nps="1-1/2",
        inner_ring_id=44.5,
        sealing_element_id=54.1,
        sealing_element_od=69.9,
        outer_ring_od=95.3,
    ),
    "2": SpiralWoundGasketDims(
        nps="2",
        inner_ring_id=55.6,
        sealing_element_id=69.9,
        sealing_element_od=85.9,
        outer_ring_od=111.3,
    ),
    "2-1/2": SpiralWoundGasketDims(
        nps="2-1/2",
        inner_ring_id=66.5,
        sealing_element_id=82.6,
        sealing_element_od=98.6,
        outer_ring_od=130.3,
    ),
    "3": SpiralWoundGasketDims(
        nps="3",
        inner_ring_id=81.0,
        sealing_element_id=101.6,
        sealing_element_od=120.7,
        outer_ring_od=149.4,
    ),
    "4": SpiralWoundGasketDims(
        nps="4",
        inner_ring_id=106.4,
        sealing_element_id=127.0,
        sealing_element_od=149.4,
        outer_ring_od=181.1,
    ),
    "5": SpiralWoundGasketDims(
        nps="5",
        inner_ring_id=131.8,
        sealing_element_id=155.7,
        sealing_element_od=177.8,
        outer_ring_od=215.9,
    ),
    "6": SpiralWoundGasketDims(
        nps="6",
        inner_ring_id=157.2,
        sealing_element_id=182.6,
        sealing_element_od=209.6,
        outer_ring_od=251.0,
    ),
    "8": SpiralWoundGasketDims(
        nps="8",
        inner_ring_id=215.9,
        sealing_element_id=233.4,
        sealing_element_od=263.7,
        outer_ring_od=308.1,
    ),
    "10": SpiralWoundGasketDims(
        nps="10",
        inner_ring_id=268.2,
        sealing_element_id=287.3,
        sealing_element_od=317.5,
        outer_ring_od=362.0,
    ),
    "12": SpiralWoundGasketDims(
        nps="12",
        inner_ring_id=317.5,
        sealing_element_id=339.9,
        sealing_element_od=374.7,
        outer_ring_od=422.4,
    ),
    "14": SpiralWoundGasketDims(
        nps="14",
        inner_ring_id=349.3,
        sealing_element_id=371.6,
        sealing_element_od=406.4,
        outer_ring_od=485.9,
    ),
    "16": SpiralWoundGasketDims(
        nps="16",
        inner_ring_id=400.1,
        sealing_element_id=422.4,
        sealing_element_od=463.6,
        outer_ring_od=539.8,
    ),
    "18": SpiralWoundGasketDims(
        nps="18",
        inner_ring_id=449.3,
        sealing_element_id=474.7,
        sealing_element_od=527.1,
        outer_ring_od=596.9,
    ),
    "20": SpiralWoundGasketDims(
        nps="20",
        inner_ring_id=500.1,
        sealing_element_id=525.6,
        sealing_element_od=577.9,
        outer_ring_od=654.1,
    ),
    "24": SpiralWoundGasketDims(
        nps="24",
        inner_ring_id=603.3,
        sealing_element_id=628.7,
        sealing_element_od=685.8,
        outer_ring_od=774.7,
    ),
}

# Ring thickness constants (typical values in mm)
INNER_RING_THICKNESS = 3.2  # Inner centering ring radial thickness
OUTER_RING_THICKNESS = 3.2  # Outer centering ring radial thickness


# =============================================================================
# GASKET GEOMETRY
# =============================================================================


def make_spiral_wound_gasket(
    nps: str,
    thickness_mm: float | None = None,
    include_inner_ring: bool = True,
) -> cq.Shape:
    """
    Create an ASME B16.20 spiral wound gasket.

    The gasket is oriented with:
    - Center at origin (0, 0, 0)
    - Gasket lies in XY plane
    - Thickness centered around Z=0 (from Z=-t/2 to Z=+t/2)
    - "face_a" (top face) at Z=+t/2
    - "face_b" (bottom face) at Z=-t/2

    The geometry is simplified to show the three concentric sections:
    - Inner ring (if include_inner_ring=True)
    - Sealing element
    - Outer ring

    Args:
        nps: Nominal pipe size (e.g., "2", "4", "6")
        thickness_mm: Override thickness in mm (default uses standard 4.5mm)
        include_inner_ring: Whether to include the inner centering ring

    Returns:
        CadQuery Shape of the gasket
    """
    dims = ASME_B1620_CLASS300_SPIRAL.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.20 Class 300 spiral wound gasket dimensions for NPS {nps}")

    # Use provided thickness or default from dimensions
    thickness = thickness_mm if thickness_mm is not None else dims.compressed_thickness

    z_start = -thickness / 2

    # Create the gasket as a single solid with bore
    # (Simplified representation - actual spiral wound has distinct rings)
    r_outer = dims.outer_ring_od / 2
    r_inner = dims.inner_ring_id / 2

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


def make_spiral_wound_gasket_detailed(
    nps: str,
    thickness_mm: float | None = None,
) -> cq.Shape:
    """
    Create a detailed ASME B16.20 spiral wound gasket showing all three rings.

    This creates a more visually accurate representation with distinct:
    - Inner centering ring
    - Sealing element (spiral wound section)
    - Outer centering ring

    Each section is created as a separate annular ring, then combined.

    Args:
        nps: Nominal pipe size (e.g., "2", "4", "6")
        thickness_mm: Override thickness in mm (default uses standard 4.5mm)

    Returns:
        CadQuery Compound shape of the gasket
    """
    dims = ASME_B1620_CLASS300_SPIRAL.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.20 Class 300 spiral wound gasket dimensions for NPS {nps}")

    thickness = thickness_mm if thickness_mm is not None else dims.compressed_thickness
    z_start = -thickness / 2

    parts = []

    # Inner ring: from inner_ring_id to sealing_element_id
    r_inner_ring_in = dims.inner_ring_id / 2
    r_inner_ring_out = dims.sealing_element_id / 2

    inner_ring_outer = cq.Solid.makeCylinder(
        r_inner_ring_out,
        thickness,
        cq.Vector(0, 0, z_start),
        cq.Vector(0, 0, 1),
    )
    inner_ring_inner = cq.Solid.makeCylinder(
        r_inner_ring_in,
        thickness,
        cq.Vector(0, 0, z_start),
        cq.Vector(0, 0, 1),
    )
    inner_ring = inner_ring_outer.cut(inner_ring_inner)
    parts.append(inner_ring)

    # Sealing element: from sealing_element_id to sealing_element_od
    r_seal_in = dims.sealing_element_id / 2
    r_seal_out = dims.sealing_element_od / 2

    seal_outer = cq.Solid.makeCylinder(
        r_seal_out,
        thickness,
        cq.Vector(0, 0, z_start),
        cq.Vector(0, 0, 1),
    )
    seal_inner = cq.Solid.makeCylinder(
        r_seal_in,
        thickness,
        cq.Vector(0, 0, z_start),
        cq.Vector(0, 0, 1),
    )
    sealing_element = seal_outer.cut(seal_inner)
    parts.append(sealing_element)

    # Outer ring: from sealing_element_od to outer_ring_od
    r_outer_ring_in = dims.sealing_element_od / 2
    r_outer_ring_out = dims.outer_ring_od / 2

    outer_ring_outer = cq.Solid.makeCylinder(
        r_outer_ring_out,
        thickness,
        cq.Vector(0, 0, z_start),
        cq.Vector(0, 0, 1),
    )
    outer_ring_inner = cq.Solid.makeCylinder(
        r_outer_ring_in,
        thickness,
        cq.Vector(0, 0, z_start),
        cq.Vector(0, 0, 1),
    )
    outer_ring = outer_ring_outer.cut(outer_ring_inner)
    parts.append(outer_ring)

    # Combine all parts
    return cq.Compound.makeCompound(parts)


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================


def get_gasket_thickness(nps: str) -> float:
    """
    Get the standard compressed thickness for a spiral wound gasket.

    Args:
        nps: Nominal pipe size

    Returns:
        Thickness in mm
    """
    dims = ASME_B1620_CLASS300_SPIRAL.get(nps)
    if dims is None:
        raise ValueError(f"No gasket dimensions for NPS {nps}")

    return dims.compressed_thickness


def get_available_sizes() -> list[str]:
    """
    Get list of available NPS sizes for spiral wound gaskets.

    Returns:
        List of NPS size strings
    """
    return list(ASME_B1620_CLASS300_SPIRAL.keys())


# =============================================================================
# EXPORT UTILITIES
# =============================================================================


def export_to_step(shape: cq.Shape | cq.Compound, filename: str):
    """Export a shape to STEP file."""
    cq.exporters.export(shape, filename)


# =============================================================================
# EXAMPLES
# =============================================================================


def example_single_gasket():
    """Create a single 2" spiral wound gasket."""
    print("ASME B16.20 Spiral Wound Gasket Generator")
    print("=" * 50)

    gasket = make_spiral_wound_gasket("2")
    export_to_step(gasket, "step/spiral_wound_gasket_2in.step")

    dims = ASME_B1620_CLASS300_SPIRAL["2"]
    print("Exported step/spiral_wound_gasket_2in.step")
    print(f"  NPS: 2\"")
    print(f"  Inner ring ID: {dims.inner_ring_id:.1f} mm")
    print(f"  Sealing element: {dims.sealing_element_id:.1f} - {dims.sealing_element_od:.1f} mm")
    print(f"  Outer ring OD: {dims.outer_ring_od:.1f} mm")
    print(f"  Thickness: {dims.compressed_thickness:.1f} mm")


def example_detailed_gasket():
    """Create a detailed spiral wound gasket showing all rings."""
    print("\nCreating detailed spiral wound gasket...")

    gasket = make_spiral_wound_gasket_detailed("4")
    export_to_step(gasket, "step/spiral_wound_gasket_4in_detailed.step")

    dims = ASME_B1620_CLASS300_SPIRAL["4"]
    print("Exported step/spiral_wound_gasket_4in_detailed.step")
    print("  Shows inner ring, sealing element, and outer ring as separate bodies")
    print(f"  Outer ring OD: {dims.outer_ring_od:.1f} mm")


def example_all_sizes():
    """Create gaskets for several common sizes."""
    print("\nCreating gaskets for common sizes...")

    gaskets = []
    spacing = 0

    for nps in ["1", "2", "4", "6"]:
        dims = ASME_B1620_CLASS300_SPIRAL[nps]
        gasket = make_spiral_wound_gasket(nps)

        # Offset for display
        gasket_moved = gasket.moved(cq.Location(cq.Vector(spacing, 0, 0)))
        gaskets.append(gasket_moved)

        print(f"Created {nps}\" gasket: ID={dims.inner_ring_id:.0f}mm, OD={dims.outer_ring_od:.0f}mm")

        spacing += dims.outer_ring_od + 20

    if gaskets:
        compound = cq.Compound.makeCompound(gaskets)
        export_to_step(compound, "step/spiral_wound_gaskets_sample.step")
        print(f"\nExported step/spiral_wound_gaskets_sample.step with {len(gaskets)} gaskets")


if __name__ == "__main__":
    example_single_gasket()
    example_detailed_gasket()
    example_all_sizes()
    print("\nDone!")
