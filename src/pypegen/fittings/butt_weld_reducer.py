"""
ASME B16.9 Butt Weld Concentric Reducer Generator using CadQuery.

This module creates butt weld concentric reducers per ASME B16.9 dimensions.
The reducer connects two different pipe sizes with a tapered transition.

Sources:
- https://www.wermac.org/fittings/dim_reducers.html
- https://www.ferrobend.com/dimensions/ansi-asme/pipe-fitting/b16.9-concentric-reducer/
- https://www.skylandmetal.in/dimensions/asme-b16-9-concentric-reducer.html
"""

from __future__ import annotations

import re
from dataclasses import dataclass

import cadquery as cq

# =============================================================================
# NPS PARSING UTILITIES
# =============================================================================


def nps_to_float(nps: str) -> float:
    """
    Convert NPS string to a float value for comparison.

    Args:
        nps: Nominal pipe size string (e.g., "4", "1-1/2", "3/4")

    Returns:
        Float value of the NPS

    Examples:
        >>> nps_to_float("4")
        4.0
        >>> nps_to_float("1-1/2")
        1.5
        >>> nps_to_float("3/4")
        0.75
        >>> nps_to_float("2-1/2")
        2.5
    """
    nps = nps.strip()

    # Handle whole numbers: "4", "2", "10", etc.
    if re.match(r"^\d+$", nps):
        return float(nps)

    # Handle fractional: "1/2", "3/4"
    if "/" in nps and "-" not in nps:
        num, denom = nps.split("/")
        return float(num) / float(denom)

    # Handle mixed: "1-1/2", "2-1/2", "3-1/2"
    if "-" in nps and "/" in nps:
        whole, frac = nps.split("-")
        num, denom = frac.split("/")
        return float(whole) + float(num) / float(denom)

    raise ValueError(f"Cannot parse NPS: {nps}")


def compare_nps(nps1: str, nps2: str) -> int:
    """
    Compare two NPS values numerically.

    Args:
        nps1: First NPS string
        nps2: Second NPS string

    Returns:
        -1 if nps1 < nps2
         0 if nps1 == nps2
        +1 if nps1 > nps2

    Examples:
        >>> compare_nps("4", "2")
        1
        >>> compare_nps("2", "4")
        -1
        >>> compare_nps("1-1/2", "1-1/2")
        0
    """
    v1 = nps_to_float(nps1)
    v2 = nps_to_float(nps2)
    if v1 < v2:
        return -1
    elif v1 > v2:
        return 1
    return 0


# =============================================================================
# DATA STRUCTURES
# =============================================================================


@dataclass
class ButtWeldReducerDims:
    """ASME B16.9 Butt Weld Concentric Reducer dimensions (all in mm).

    The reducer has a large end and a small end. Dimensions are per ASME B16.9.

    Attributes:
        large_od: Outside diameter at large end
        small_od: Outside diameter at small end
        large_wall: Wall thickness at large end (STD schedule)
        small_wall: Wall thickness at small end (STD schedule)
        length_h: Overall length (H dimension)
    """

    large_od: float
    small_od: float
    large_wall: float
    small_wall: float
    length_h: float


# =============================================================================
# ASME B16.9 DIMENSION TABLE
# =============================================================================

# ASME B16.9 Butt Weld Concentric Reducer dimension table
# Key is (large_nps, small_nps) tuple
# Wall thicknesses from STD schedule pipe
ASME_B169_REDUCER: dict[tuple[str, str], ButtWeldReducerDims] = {
    # 3/4" reducers
    ("3/4", "1/2"): ButtWeldReducerDims(
        large_od=26.7, small_od=21.3,
        large_wall=2.87, small_wall=2.78, length_h=38.0
    ),
    # 1" reducers
    ("1", "3/4"): ButtWeldReducerDims(
        large_od=33.4, small_od=26.7,
        large_wall=3.38, small_wall=2.87, length_h=51.0
    ),
    ("1", "1/2"): ButtWeldReducerDims(
        large_od=33.4, small_od=21.3,
        large_wall=3.38, small_wall=2.78, length_h=51.0
    ),
    # 1-1/4" reducers
    ("1-1/4", "1"): ButtWeldReducerDims(
        large_od=42.2, small_od=33.4,
        large_wall=3.56, small_wall=3.38, length_h=51.0
    ),
    ("1-1/4", "3/4"): ButtWeldReducerDims(
        large_od=42.2, small_od=26.7,
        large_wall=3.56, small_wall=2.87, length_h=51.0
    ),
    ("1-1/4", "1/2"): ButtWeldReducerDims(
        large_od=42.2, small_od=21.3,
        large_wall=3.56, small_wall=2.78, length_h=51.0
    ),
    # 1-1/2" reducers
    ("1-1/2", "1-1/4"): ButtWeldReducerDims(
        large_od=48.3, small_od=42.2,
        large_wall=3.68, small_wall=3.56, length_h=64.0
    ),
    ("1-1/2", "1"): ButtWeldReducerDims(
        large_od=48.3, small_od=33.4,
        large_wall=3.68, small_wall=3.38, length_h=64.0
    ),
    ("1-1/2", "3/4"): ButtWeldReducerDims(
        large_od=48.3, small_od=26.7,
        large_wall=3.68, small_wall=2.87, length_h=64.0
    ),
    ("1-1/2", "1/2"): ButtWeldReducerDims(
        large_od=48.3, small_od=21.3,
        large_wall=3.68, small_wall=2.78, length_h=64.0
    ),
    # 2" reducers
    ("2", "1-1/2"): ButtWeldReducerDims(
        large_od=60.3, small_od=48.3,
        large_wall=3.91, small_wall=3.68, length_h=76.0
    ),
    ("2", "1-1/4"): ButtWeldReducerDims(
        large_od=60.3, small_od=42.2,
        large_wall=3.91, small_wall=3.56, length_h=76.0
    ),
    ("2", "1"): ButtWeldReducerDims(
        large_od=60.3, small_od=33.4,
        large_wall=3.91, small_wall=3.38, length_h=76.0
    ),
    ("2", "3/4"): ButtWeldReducerDims(
        large_od=60.3, small_od=26.7,
        large_wall=3.91, small_wall=2.87, length_h=76.0
    ),
    # 2-1/2" reducers
    ("2-1/2", "2"): ButtWeldReducerDims(
        large_od=73.0, small_od=60.3,
        large_wall=5.16, small_wall=3.91, length_h=88.9
    ),
    ("2-1/2", "1-1/2"): ButtWeldReducerDims(
        large_od=73.0, small_od=48.3,
        large_wall=5.16, small_wall=3.68, length_h=89.0
    ),
    ("2-1/2", "1-1/4"): ButtWeldReducerDims(
        large_od=73.0, small_od=42.2,
        large_wall=5.16, small_wall=3.56, length_h=89.0
    ),
    ("2-1/2", "1"): ButtWeldReducerDims(
        large_od=73.0, small_od=33.4,
        large_wall=5.16, small_wall=3.38, length_h=89.0
    ),
    # 3" reducers
    ("3", "2-1/2"): ButtWeldReducerDims(
        large_od=88.9, small_od=73.0,
        large_wall=5.49, small_wall=5.16, length_h=89.0
    ),
    ("3", "2"): ButtWeldReducerDims(
        large_od=88.9, small_od=60.3,
        large_wall=5.49, small_wall=3.91, length_h=89.0
    ),
    ("3", "1-1/2"): ButtWeldReducerDims(
        large_od=88.9, small_od=48.3,
        large_wall=5.49, small_wall=3.68, length_h=89.0
    ),
    ("3", "1-1/4"): ButtWeldReducerDims(
        large_od=88.9, small_od=42.2,
        large_wall=5.49, small_wall=3.56, length_h=89.0
    ),
    # 3-1/2" reducers
    ("3-1/2", "3"): ButtWeldReducerDims(
        large_od=101.6, small_od=88.9,
        large_wall=5.74, small_wall=5.49, length_h=102.0
    ),
    ("3-1/2", "2-1/2"): ButtWeldReducerDims(
        large_od=101.6, small_od=73.0,
        large_wall=5.74, small_wall=5.16, length_h=102.0
    ),
    ("3-1/2", "2"): ButtWeldReducerDims(
        large_od=101.6, small_od=60.3,
        large_wall=5.74, small_wall=3.91, length_h=102.0
    ),
    ("3-1/2", "1-1/2"): ButtWeldReducerDims(
        large_od=101.6, small_od=48.3,
        large_wall=5.74, small_wall=3.68, length_h=102.0
    ),
    ("3-1/2", "1-1/4"): ButtWeldReducerDims(
        large_od=101.6, small_od=42.2,
        large_wall=5.74, small_wall=3.56, length_h=102.0
    ),
    # 4" reducers
    ("4", "3-1/2"): ButtWeldReducerDims(
        large_od=114.3, small_od=101.6,
        large_wall=6.02, small_wall=5.74, length_h=102.0
    ),
    ("4", "3"): ButtWeldReducerDims(
        large_od=114.3, small_od=88.9,
        large_wall=6.02, small_wall=5.49, length_h=102.0
    ),
    ("4", "2-1/2"): ButtWeldReducerDims(
        large_od=114.3, small_od=73.0,
        large_wall=6.02, small_wall=5.16, length_h=102.0
    ),
    ("4", "2"): ButtWeldReducerDims(
        large_od=114.3, small_od=60.3,
        large_wall=6.02, small_wall=3.91, length_h=102.0
    ),
    ("4", "1-1/2"): ButtWeldReducerDims(
        large_od=114.3, small_od=48.3,
        large_wall=6.02, small_wall=3.68, length_h=102.0
    ),
    # 5" reducers
    ("5", "4"): ButtWeldReducerDims(
        large_od=141.3, small_od=114.3,
        large_wall=6.55, small_wall=6.02, length_h=127.0
    ),
    ("5", "3-1/2"): ButtWeldReducerDims(
        large_od=141.3, small_od=101.6,
        large_wall=6.55, small_wall=5.74, length_h=127.0
    ),
    ("5", "3"): ButtWeldReducerDims(
        large_od=141.3, small_od=88.9,
        large_wall=6.55, small_wall=5.49, length_h=127.0
    ),
    ("5", "2-1/2"): ButtWeldReducerDims(
        large_od=141.3, small_od=73.0,
        large_wall=6.55, small_wall=5.16, length_h=127.0
    ),
    ("5", "2"): ButtWeldReducerDims(
        large_od=141.3, small_od=60.3,
        large_wall=6.55, small_wall=3.91, length_h=127.0
    ),
    # 6" reducers
    ("6", "5"): ButtWeldReducerDims(
        large_od=168.3, small_od=141.3,
        large_wall=7.11, small_wall=6.55, length_h=140.0
    ),
    ("6", "4"): ButtWeldReducerDims(
        large_od=168.3, small_od=114.3,
        large_wall=7.11, small_wall=6.02, length_h=140.0
    ),
    ("6", "3-1/2"): ButtWeldReducerDims(
        large_od=168.3, small_od=101.6,
        large_wall=7.11, small_wall=5.74, length_h=140.0
    ),
    ("6", "3"): ButtWeldReducerDims(
        large_od=168.3, small_od=88.9,
        large_wall=7.11, small_wall=5.49, length_h=140.0
    ),
    ("6", "2-1/2"): ButtWeldReducerDims(
        large_od=168.3, small_od=73.0,
        large_wall=7.11, small_wall=5.16, length_h=140.0
    ),
    # 8" reducers
    ("8", "6"): ButtWeldReducerDims(
        large_od=219.1, small_od=168.3,
        large_wall=8.18, small_wall=7.11, length_h=152.0
    ),
    ("8", "5"): ButtWeldReducerDims(
        large_od=219.1, small_od=141.3,
        large_wall=8.18, small_wall=6.55, length_h=152.0
    ),
    ("8", "4"): ButtWeldReducerDims(
        large_od=219.1, small_od=114.3,
        large_wall=8.18, small_wall=6.02, length_h=152.0
    ),
    ("8", "3-1/2"): ButtWeldReducerDims(
        large_od=219.1, small_od=101.6,
        large_wall=8.18, small_wall=5.74, length_h=152.0
    ),
    # 10" reducers
    ("10", "8"): ButtWeldReducerDims(
        large_od=273.0, small_od=219.1,
        large_wall=9.27, small_wall=8.18, length_h=178.0
    ),
    ("10", "6"): ButtWeldReducerDims(
        large_od=273.0, small_od=168.3,
        large_wall=9.27, small_wall=7.11, length_h=178.0
    ),
    ("10", "5"): ButtWeldReducerDims(
        large_od=273.0, small_od=141.3,
        large_wall=9.27, small_wall=6.55, length_h=178.0
    ),
    ("10", "4"): ButtWeldReducerDims(
        large_od=273.0, small_od=114.3,
        large_wall=9.27, small_wall=6.02, length_h=178.0
    ),
    # 12" reducers
    ("12", "10"): ButtWeldReducerDims(
        large_od=323.8, small_od=273.0,
        large_wall=9.52, small_wall=9.27, length_h=203.0
    ),
    ("12", "8"): ButtWeldReducerDims(
        large_od=323.8, small_od=219.1,
        large_wall=9.52, small_wall=8.18, length_h=203.0
    ),
    ("12", "6"): ButtWeldReducerDims(
        large_od=323.8, small_od=168.3,
        large_wall=9.52, small_wall=7.11, length_h=203.0
    ),
    ("12", "5"): ButtWeldReducerDims(
        large_od=323.8, small_od=141.3,
        large_wall=9.52, small_wall=6.55, length_h=203.0
    ),
}


# =============================================================================
# DIMENSION LOOKUP
# =============================================================================


def get_butt_weld_reducer_dims(large_nps: str, small_nps: str) -> ButtWeldReducerDims:
    """
    Get the dimensions for a butt weld concentric reducer.

    Args:
        large_nps: Nominal pipe size of large end (e.g., "4", "6")
        small_nps: Nominal pipe size of small end (e.g., "2", "4")

    Returns:
        ButtWeldReducerDims with all dimensions

    Raises:
        ValueError: If size combination is not supported
    """
    key = (large_nps, small_nps)
    dims = ASME_B169_REDUCER.get(key)
    if dims is None:
        raise ValueError(f"No ASME B16.9 dimensions for reducer {large_nps} x {small_nps}")

    return dims


def get_available_reducers_from(nps: str) -> tuple[list[str], list[str]]:
    """
    Get all available target NPS values for reducers from given NPS.

    Returns both contraction (going smaller) and expansion (going larger) options.

    Args:
        nps: Current pipe NPS

    Returns:
        Tuple of (contractions, expansions) where each is a list of available
        target NPS values, sorted by size (contractions largest first,
        expansions smallest first)
    """
    contractions = []  # Current NPS as large end
    expansions = []  # Current NPS as small end

    for large, small in ASME_B169_REDUCER.keys():
        if large == nps:
            contractions.append(small)
        if small == nps:
            expansions.append(large)

    # Sort by size
    contractions.sort(key=nps_to_float, reverse=True)  # Largest first
    expansions.sort(key=nps_to_float)  # Smallest first

    return contractions, expansions


def validate_reducer(current_nps: str, target_nps: str) -> tuple[str, str, bool]:
    """
    Validate a reducer transition and determine orientation.

    Args:
        current_nps: Current pipe NPS (where flow is coming from)
        target_nps: Target pipe NPS (where flow is going to)

    Returns:
        Tuple of (large_nps, small_nps, is_expansion) where is_expansion
        indicates if the pipe is getting larger (True) or smaller (False)

    Raises:
        ValueError: If no reducer exists for this combination, with helpful
        message showing available options
    """
    cmp = compare_nps(current_nps, target_nps)

    if cmp == 0:
        raise ValueError(
            f"Current NPS ({current_nps}) equals target NPS ({target_nps}). "
            f"No reducer needed."
        )

    if cmp > 0:
        # Contraction: current is larger
        large_nps, small_nps = current_nps, target_nps
        is_expansion = False
    else:
        # Expansion: target is larger
        large_nps, small_nps = target_nps, current_nps
        is_expansion = True

    # Check if this combination exists
    if (large_nps, small_nps) not in ASME_B169_REDUCER:
        contractions, expansions = get_available_reducers_from(current_nps)

        error_lines = [
            f"No ASME B16.9 reducer for {current_nps}\" to {target_nps}\"."
        ]

        if contractions:
            sizes = ", ".join(f'{c}"' for c in contractions)
            error_lines.append(f"  Available contractions from {current_nps}\": {sizes}")
        if expansions:
            sizes = ", ".join(f'{e}"' for e in expansions)
            error_lines.append(f"  Available expansions from {current_nps}\": {sizes}")
        if not contractions and not expansions:
            error_lines.append(f"  No reducers available from {current_nps}\".")

        raise ValueError("\n".join(error_lines))

    return large_nps, small_nps, is_expansion


# =============================================================================
# REDUCER GEOMETRY
# =============================================================================


def make_butt_weld_reducer(
    large_nps: str,
    small_nps: str,
    _schedule: str = "STD",
) -> cq.Shape:
    """
    Create an ASME B16.9 butt weld concentric reducer.

    The reducer is oriented with:
    - Large end at origin (X=0), weld face perpendicular to X-axis
    - Small end at X=H, weld face perpendicular to X-axis
    - Centerline along the +X axis

    The geometry is a truncated cone (frustum) with the bore hollowed out.

    Args:
        large_nps: Nominal pipe size of large end (e.g., "4", "6")
        small_nps: Nominal pipe size of small end (e.g., "2", "4")
        _schedule: Pipe schedule (currently only "STD" implemented)

    Returns:
        CadQuery Shape of the reducer
    """
    dims = get_butt_weld_reducer_dims(large_nps, small_nps)

    # Get dimensions
    large_od = dims.large_od
    small_od = dims.small_od
    large_wall = dims.large_wall
    small_wall = dims.small_wall
    length_h = dims.length_h

    # Calculate inner diameters
    large_id = large_od - 2 * large_wall
    small_id = small_od - 2 * small_wall

    # Radii
    r_large_outer = large_od / 2.0
    r_small_outer = small_od / 2.0
    r_large_inner = large_id / 2.0
    r_small_inner = small_id / 2.0

    # Create outer frustum (truncated cone)
    # makeCone creates a cone from origin along specified direction
    # We use r1 at origin (large end) and r2 at height (small end)
    outer_cone = cq.Solid.makeCone(
        r_large_outer,  # radius at origin (large end)
        r_small_outer,  # radius at height (small end)
        length_h,
        cq.Vector(0, 0, 0),  # Start position
        cq.Vector(1, 0, 0),  # Direction along +X
    )

    # Create inner frustum for the bore
    inner_cone = cq.Solid.makeCone(
        r_large_inner,  # inner radius at origin (large end)
        r_small_inner,  # inner radius at height (small end)
        length_h,
        cq.Vector(0, 0, 0),  # Start position
        cq.Vector(1, 0, 0),  # Direction along +X
    )

    # Subtract inner from outer to create hollow reducer
    reducer_shape = outer_cone.cut(inner_cone)

    return reducer_shape


# =============================================================================
# EXPORT UTILITIES
# =============================================================================


def export_to_step(shape: cq.Shape, filename: str):
    """Export a shape to STEP file."""
    cq.exporters.export(shape, filename)


# =============================================================================
# EXAMPLES
# =============================================================================


def example_single_reducer():
    """Create a single 4" x 2" reducer."""
    reducer = make_butt_weld_reducer("4", "2")
    export_to_step(reducer, "butt_weld_reducer_4x2.step")
    print("Exported butt_weld_reducer_4x2.step")

    dims = get_butt_weld_reducer_dims("4", "2")
    print(f"  Large OD: {dims.large_od} mm")
    print(f"  Small OD: {dims.small_od} mm")
    print(f"  Length H: {dims.length_h} mm")


def example_all_sizes():
    """Create reducers for various sizes."""
    sizes = [
        ("2", "1"),
        ("3", "2"),
        ("4", "3"),
        ("4", "2"),
        ("6", "4"),
        ("8", "6"),
    ]

    solids = []
    offset = 0

    for large_nps, small_nps in sizes:
        try:
            dims = get_butt_weld_reducer_dims(large_nps, small_nps)
            reducer = make_butt_weld_reducer(large_nps, small_nps)
            loc = cq.Location(cq.Vector(offset, 0, 0))
            solids.append(reducer.moved(loc))
            print(f"Created {large_nps}\" x {small_nps}\" reducer: H={dims.length_h}mm")
            offset += dims.length_h + 50
        except Exception as e:
            print(f"Error creating {large_nps}\" x {small_nps}\" reducer: {e}")

    if solids:
        compound = cq.Compound.makeCompound(solids)
        cq.exporters.export(compound, "butt_weld_reducers_various.step")
        print(f"\nExported butt_weld_reducers_various.step with {len(solids)} reducers")


if __name__ == "__main__":
    print("ASME B16.9 Butt Weld Concentric Reducer Generator")
    print("=" * 50)
    example_single_reducer()
    print()
    example_all_sizes()
    print("\nDone!")
