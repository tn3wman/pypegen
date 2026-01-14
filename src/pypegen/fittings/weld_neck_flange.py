#!/usr/bin/env python3
"""
ASME B16.5 Class 300 Weld Neck Flange Generator

Generates raised face weld neck flanges per ASME B16.5 specifications.

References:
- https://www.wermac.org/flanges/dimensions_weldneck_in_asme-b16-5_300.html
- https://www.ferrobend.com/dimensions/ansi-asme/flange/b16.5-class-300-weld-neck/
"""

from dataclasses import dataclass
from typing import Literal

import cadquery as cq

# =============================================================================
# BOLT HOLE ORIENTATION TYPE
# =============================================================================

BoltHoleOrientation = Literal["single_hole", "two_hole"]
"""
Bolt hole orientation relative to a reference plane (the XZ plane in local coordinates).

- "single_hole": One bolt hole lies exactly on the reference plane (at angle 0°)
- "two_hole": Two bolt holes are symmetric about the reference plane
             (first hole at angle = 360° / num_bolts / 2)
"""

# =============================================================================
# ASME B16.5 CLASS 300 WELD NECK FLANGE DIMENSIONS
# =============================================================================

@dataclass
class WeldNeckFlangeDims:
    """Dimensions for an ASME B16.5 weld neck flange (inches)."""
    nps: str                    # Nominal pipe size
    flange_od: float            # O - Outside diameter of flange
    flange_thickness: float     # T - Minimum flange thickness
    hub_length: float           # Y - Length through hub
    raised_face_od: float       # R - Raised face diameter
    raised_face_height: float   # Raised face height (0.06" for Class 300)
    bolt_circle: float          # K - Bolt circle diameter
    hub_od_at_base: float       # X - Hub diameter at base of flange
    hub_od_at_weld: float       # A - Hub diameter at weld end (pipe OD)
    bore: float                 # B - Bore diameter
    num_bolts: int              # Number of bolt holes
    bolt_hole_dia: float        # Bolt hole diameter


# ASME B16.5 Class 300 Weld Neck Flange Dimensions (inches)
# Raised face height is 1/16" (0.0625") for Class 300
ASME_B165_CLASS300_WN: dict[str, WeldNeckFlangeDims] = {
    "1/2": WeldNeckFlangeDims(
        nps="1/2", flange_od=3.75, flange_thickness=0.56, hub_length=2.06,
        raised_face_od=1.38, raised_face_height=0.0625, bolt_circle=2.62,
        hub_od_at_base=1.50, hub_od_at_weld=0.84, bore=0.62, num_bolts=4, bolt_hole_dia=0.625
    ),
    "3/4": WeldNeckFlangeDims(
        nps="3/4", flange_od=4.62, flange_thickness=0.62, hub_length=2.25,
        raised_face_od=1.69, raised_face_height=0.0625, bolt_circle=3.25,
        hub_od_at_base=1.88, hub_od_at_weld=1.05, bore=0.82, num_bolts=4, bolt_hole_dia=0.75
    ),
    "1": WeldNeckFlangeDims(
        nps="1", flange_od=4.88, flange_thickness=0.69, hub_length=2.44,
        raised_face_od=2.00, raised_face_height=0.0625, bolt_circle=3.50,
        hub_od_at_base=2.12, hub_od_at_weld=1.32, bore=1.05, num_bolts=4, bolt_hole_dia=0.75
    ),
    "1-1/4": WeldNeckFlangeDims(
        nps="1-1/4", flange_od=5.25, flange_thickness=0.75, hub_length=2.56,
        raised_face_od=2.50, raised_face_height=0.0625, bolt_circle=3.88,
        hub_od_at_base=2.50, hub_od_at_weld=1.66, bore=1.38, num_bolts=4, bolt_hole_dia=0.75
    ),
    "1-1/2": WeldNeckFlangeDims(
        nps="1-1/2", flange_od=6.12, flange_thickness=0.75, hub_length=2.63,
        raised_face_od=2.88, raised_face_height=0.0625, bolt_circle=4.50,
        hub_od_at_base=2.75, hub_od_at_weld=1.90, bore=1.61, num_bolts=4, bolt_hole_dia=0.875
    ),
    "2": WeldNeckFlangeDims(
        nps="2", flange_od=6.50, flange_thickness=0.81, hub_length=2.69,
        raised_face_od=3.62, raised_face_height=0.0625, bolt_circle=5.00,
        hub_od_at_base=3.31, hub_od_at_weld=2.375, bore=2.07, num_bolts=8, bolt_hole_dia=0.75
    ),
    "2-1/2": WeldNeckFlangeDims(
        nps="2-1/2", flange_od=7.50, flange_thickness=1.00, hub_length=3.25,
        raised_face_od=4.12, raised_face_height=0.0625, bolt_circle=5.88,
        hub_od_at_base=3.94, hub_od_at_weld=2.88, bore=2.47, num_bolts=8, bolt_hole_dia=0.875
    ),
    "3": WeldNeckFlangeDims(
        nps="3", flange_od=8.25, flange_thickness=1.12, hub_length=3.50,
        raised_face_od=5.00, raised_face_height=0.0625, bolt_circle=6.62,
        hub_od_at_base=4.62, hub_od_at_weld=3.50, bore=3.07, num_bolts=8, bolt_hole_dia=0.875
    ),
    "3-1/2": WeldNeckFlangeDims(
        nps="3-1/2", flange_od=9.00, flange_thickness=1.19, hub_length=3.62,
        raised_face_od=5.50, raised_face_height=0.0625, bolt_circle=7.25,
        hub_od_at_base=5.25, hub_od_at_weld=4.00, bore=3.55, num_bolts=8, bolt_hole_dia=0.875
    ),
    "4": WeldNeckFlangeDims(
        nps="4", flange_od=10.00, flange_thickness=1.25, hub_length=3.75,
        raised_face_od=6.19, raised_face_height=0.0625, bolt_circle=7.88,
        hub_od_at_base=5.75, hub_od_at_weld=4.50, bore=4.03, num_bolts=8, bolt_hole_dia=0.875
    ),
    "5": WeldNeckFlangeDims(
        nps="5", flange_od=11.00, flange_thickness=1.38, hub_length=4.25,
        raised_face_od=7.31, raised_face_height=0.0625, bolt_circle=9.25,
        hub_od_at_base=7.00, hub_od_at_weld=5.56, bore=5.05, num_bolts=8, bolt_hole_dia=0.875
    ),
    "6": WeldNeckFlangeDims(
        nps="6", flange_od=12.50, flange_thickness=1.44, hub_length=4.50,
        raised_face_od=8.50, raised_face_height=0.0625, bolt_circle=10.62,
        hub_od_at_base=8.12, hub_od_at_weld=6.63, bore=6.07, num_bolts=12, bolt_hole_dia=0.875
    ),
    "8": WeldNeckFlangeDims(
        nps="8", flange_od=15.00, flange_thickness=1.62, hub_length=5.00,
        raised_face_od=10.62, raised_face_height=0.0625, bolt_circle=13.00,
        hub_od_at_base=10.25, hub_od_at_weld=8.63, bore=7.98, num_bolts=12, bolt_hole_dia=1.00
    ),
    "10": WeldNeckFlangeDims(
        nps="10", flange_od=17.50, flange_thickness=1.88, hub_length=5.50,
        raised_face_od=12.75, raised_face_height=0.0625, bolt_circle=15.25,
        hub_od_at_base=12.62, hub_od_at_weld=10.75, bore=10.02, num_bolts=16, bolt_hole_dia=1.125
    ),
    "12": WeldNeckFlangeDims(
        nps="12", flange_od=20.50, flange_thickness=2.00, hub_length=5.75,
        raised_face_od=15.00, raised_face_height=0.0625, bolt_circle=17.75,
        hub_od_at_base=14.75, hub_od_at_weld=12.75, bore=11.94, num_bolts=16, bolt_hole_dia=1.25
    ),
    "14": WeldNeckFlangeDims(
        nps="14", flange_od=23.00, flange_thickness=2.12, hub_length=6.00,
        raised_face_od=16.25, raised_face_height=0.0625, bolt_circle=20.25,
        hub_od_at_base=16.75, hub_od_at_weld=14.00, bore=13.13, num_bolts=20, bolt_hole_dia=1.25
    ),
    "16": WeldNeckFlangeDims(
        nps="16", flange_od=25.50, flange_thickness=2.25, hub_length=6.25,
        raised_face_od=18.50, raised_face_height=0.0625, bolt_circle=22.50,
        hub_od_at_base=19.00, hub_od_at_weld=16.00, bore=15.00, num_bolts=20, bolt_hole_dia=1.375
    ),
    "18": WeldNeckFlangeDims(
        nps="18", flange_od=28.00, flange_thickness=2.38, hub_length=6.62,
        raised_face_od=21.00, raised_face_height=0.0625, bolt_circle=24.75,
        hub_od_at_base=21.00, hub_od_at_weld=18.00, bore=16.94, num_bolts=24, bolt_hole_dia=1.375
    ),
    "20": WeldNeckFlangeDims(
        nps="20", flange_od=30.50, flange_thickness=2.50, hub_length=6.88,
        raised_face_od=23.00, raised_face_height=0.0625, bolt_circle=27.00,
        hub_od_at_base=23.12, hub_od_at_weld=20.00, bore=18.88, num_bolts=24, bolt_hole_dia=1.375
    ),
    "24": WeldNeckFlangeDims(
        nps="24", flange_od=36.00, flange_thickness=2.75, hub_length=7.25,
        raised_face_od=27.25, raised_face_height=0.0625, bolt_circle=32.00,
        hub_od_at_base=27.62, hub_od_at_weld=24.00, bore=22.88, num_bolts=24, bolt_hole_dia=1.625
    ),
}


# =============================================================================
# FLANGE GEOMETRY
# =============================================================================

def make_weld_neck_flange_class300(
    nps: str,
    units: str = "inch",
    bolt_hole_orientation: BoltHoleOrientation = "single_hole",
) -> cq.Shape:
    """
    Create an ASME B16.5 Class 300 raised face weld neck flange.

    The flange is oriented with:
    - Flange face at Z=0, facing +Z direction
    - Hub/neck extending in -Z direction
    - Centerline along Z axis

    Args:
        nps: Nominal pipe size (e.g., "2", "4", "6")
        units: "inch" or "mm" - output units
        bolt_hole_orientation: Bolt hole orientation relative to the XZ reference plane.
            - "single_hole": One bolt hole on the reference plane (at angle 0°)
            - "two_hole": Two bolt holes symmetric about the reference plane
            Default is "two_hole" which is the most common orientation.

    Returns:
        CadQuery Shape of the flange
    """
    dims = ASME_B165_CLASS300_WN.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.5 Class 300 dimensions for NPS {nps}")

    # Convert to mm if needed (all dims stored in inches)
    scale = 25.4 if units == "mm" else 1.0

    # Get dimensions and scale
    flange_od = dims.flange_od * scale
    flange_t = dims.flange_thickness * scale
    hub_length = dims.hub_length * scale
    rf_od = dims.raised_face_od * scale
    rf_height = dims.raised_face_height * scale
    bolt_circle = dims.bolt_circle * scale
    hub_base_od = dims.hub_od_at_base * scale
    hub_weld_od = dims.hub_od_at_weld * scale
    bore = dims.bore * scale
    num_bolts = dims.num_bolts
    bolt_hole_dia = dims.bolt_hole_dia * scale

    # Radii
    r_flange = flange_od / 2
    r_rf = rf_od / 2
    r_bolt_circle = bolt_circle / 2
    r_hub_base = hub_base_od / 2
    r_hub_weld = hub_weld_od / 2
    r_bore = bore / 2
    r_bolt_hole = bolt_hole_dia / 2

    # Build the flange using revolution of a profile
    # Profile in XZ plane, revolve around Z axis

    # The flange consists of:
    # 1. Main flange body (disk) at top
    # 2. Raised face protruding from flange face
    # 3. Tapered hub below the flange

    # Create profile points (starting from bore at top, going clockwise)
    # Top of raised face
    p1 = (r_bore, rf_height)
    p2 = (r_rf, rf_height)
    # Flange face (step down from raised face)
    p3 = (r_rf, 0)
    p4 = (r_flange, 0)
    # Bottom of flange body
    p5 = (r_flange, -flange_t)
    # Hub at base of flange
    p6 = (r_hub_base, -flange_t)
    # Hub at weld end (bottom)
    _neck_length = hub_length - flange_t  # Calculated but not directly used
    p7 = (r_hub_weld, -hub_length)
    # Bore at bottom
    p8 = (r_bore, -hub_length)
    # Back to start (bore runs all the way through)

    # Create the profile
    profile = (
        cq.Workplane("XZ")
        .moveTo(*p1)
        .lineTo(*p2)
        .lineTo(*p3)
        .lineTo(*p4)
        .lineTo(*p5)
        .lineTo(*p6)
        .lineTo(*p7)
        .lineTo(*p8)
        .close()
    )

    # Revolve to create solid
    flange = profile.revolve(360, (0, 0), (0, 1))

    # Calculate starting angle for bolt holes based on orientation
    # Angular spacing between bolt holes
    angular_spacing = 360.0 / num_bolts
    if bolt_hole_orientation == "single_hole":
        # One bolt hole lies on the reference plane (XZ plane, at angle 0°)
        start_angle = 0.0
    else:
        # "two_hole": Two bolt holes symmetric about the reference plane
        # First hole is at half the angular spacing from the X-axis
        start_angle = angular_spacing / 2.0

    # Add bolt holes
    flange = (
        flange
        .faces(">Z")
        .workplane()
        .polarArray(r_bolt_circle, start_angle, 360, num_bolts)
        .circle(r_bolt_hole)
        .cutThruAll()
    )

    result = flange.solids().val()
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

def example_single_flange():
    """Create a single 4" Class 300 weld neck flange."""
    print("ASME B16.5 Class 300 Weld Neck Flange Generator")
    print("=" * 50)

    flange = make_weld_neck_flange_class300("4", units="mm")
    export_to_step(flange, "step/weld_neck_flange_4in_cl300.step")

    dims = ASME_B165_CLASS300_WN["4"]
    print("Exported step/weld_neck_flange_4in_cl300.step")
    print("  NPS: 4\"")
    print(f"  Flange OD: {dims.flange_od * 25.4:.1f} mm")
    print(f"  Bolt circle: {dims.bolt_circle * 25.4:.1f} mm")
    print(f"  Number of bolts: {dims.num_bolts}")
    print(f"  Bore: {dims.bore * 25.4:.1f} mm")


def example_all_sizes():
    """Create all available flange sizes."""
    print("\nCreating all Class 300 weld neck flange sizes...")

    flanges = []
    spacing = 0

    for nps, dims in ASME_B165_CLASS300_WN.items():
        try:
            flange = make_weld_neck_flange_class300(nps, units="mm")

            # Offset for display
            flange_moved = flange.moved(cq.Location(cq.Vector(spacing, 0, 0)))
            flanges.append(flange_moved)

            print(f"Created {nps}\" flange: OD={dims.flange_od * 25.4:.1f}mm")

            spacing += dims.flange_od * 25.4 + 20

        except Exception as e:
            print(f"Error creating {nps}\" flange: {e}")

    if flanges:
        # Combine all flanges into one compound
        compound = cq.Compound.makeCompound(flanges)
        export_to_step(compound, "step/weld_neck_flanges_all_sizes.step")
        print(f"\nExported step/weld_neck_flanges_all_sizes.step with {len(flanges)} flanges")


if __name__ == "__main__":
    example_single_flange()
    example_all_sizes()
    print("\nDone!")
