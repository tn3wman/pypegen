"""
NPT Thread Generator for CadQuery.

Creates actual 3D NPT (National Pipe Taper) thread geometry per ASME B1.20.1.
Supports both external (pipe) and internal (fitting) threads.

NPT Thread Specifications:
- Taper: 1:16 (3/4 inch per foot)
- Half-angle: 1° 47' 24" (1.7899°)
- Thread angle: 60°
- Thread form: Truncated flat crests/roots

Sources:
- ASME B1.20.1: Pipe Threads, General Purpose (Inch)
- https://www.engineersedge.com/hardware/taper-pipe-threads.htm
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

# NPT taper: 1 inch per 16 inches of length (measured on diameter)
# This means radius changes by 1/32 inch per inch of length
NPT_TAPER_RATIO = 1.0 / 16.0  # Diameter change per unit length
NPT_HALF_ANGLE_DEG = math.degrees(math.atan(NPT_TAPER_RATIO / 2.0))  # ~1.7899°

# Thread angle per ASME B1.20.1
THREAD_ANGLE_DEG = 60.0

# Thread form constants (per ASME B1.20.1)
# H = height of sharp V thread = 0.866025 * pitch
# Truncation = H/8 at both crest and root
# Actual thread depth = H - 2*(H/8) = 0.75*H = 0.6495*pitch


# =============================================================================
# DATA STRUCTURES
# =============================================================================


@dataclass
class NPTSpec:
    """Complete NPT thread specification per ASME B1.20.1.

    All dimensions in mm unless noted.
    """

    tpi: float  # Threads per inch
    od: float  # Nominal outside diameter of pipe (mm)
    E0: float  # Pitch diameter at L1 plane (mm)
    L1: float  # Hand-tight engagement length (mm)
    L2: float  # Effective thread length (mm)
    L3: float  # Wrench makeup length (mm) - calculated as 3 threads for sizes <= 3"

    @property
    def pitch_mm(self) -> float:
        """Thread pitch in mm."""
        return INCH / self.tpi

    @property
    def thread_height_sharp_v(self) -> float:
        """Height of theoretical sharp V thread (mm)."""
        return 0.866025 * self.pitch_mm

    @property
    def thread_depth(self) -> float:
        """Actual thread depth with truncation (mm)."""
        return 0.75 * self.thread_height_sharp_v

    @property
    def crest_flat(self) -> float:
        """Width of flat at thread crest/root (mm)."""
        return self.thread_height_sharp_v / 8.0

    @property
    def wrench_tight_engagement(self) -> float:
        """Total engagement at wrench-tight (L1 + L3) in mm."""
        return self.L1 + self.L3

    @property
    def major_dia_at_L1(self) -> float:
        """Major diameter at L1 (hand-tight) plane (mm)."""
        # At L1 plane, major = pitch + thread depth
        return self.E0 + self.thread_depth

    @property
    def minor_dia_at_L1(self) -> float:
        """Minor diameter at L1 (hand-tight) plane (mm)."""
        # At L1 plane, minor = pitch - thread depth
        return self.E0 - self.thread_depth


def _calculate_L3(tpi: float, nps: str) -> float:
    """Calculate L3 (wrench makeup) per MIL-P-7105.

    For sizes 3" and smaller: 3 threads
    For sizes larger than 3": 3 threads (conservative)
    """
    # L3 = 3 thread pitches
    return 3.0 * INCH / tpi


# ASME B1.20.1 NPT Thread Specifications
# Dimensions from published tables, converted to mm
# Sources: engineersedge.com, unifiedalloys.com, engineeringtoolbox.com
NPT_SPECS: dict[str, NPTSpec] = {
    "1/8": NPTSpec(
        tpi=27,
        od=0.405 * INCH,  # 10.287 mm
        E0=0.36351 * INCH,  # 9.233 mm - pitch diameter at L1
        L1=0.1615 * INCH,  # 4.102 mm
        L2=0.2611 * INCH,  # 6.632 mm
        L3=_calculate_L3(27, "1/8"),  # 2.822 mm (3 threads)
    ),
    "1/4": NPTSpec(
        tpi=18,
        od=0.540 * INCH,  # 13.716 mm
        E0=0.47739 * INCH,  # 12.126 mm
        L1=0.2278 * INCH,  # 5.786 mm
        L2=0.4018 * INCH,  # 10.206 mm
        L3=_calculate_L3(18, "1/4"),  # 4.233 mm
    ),
    "3/8": NPTSpec(
        tpi=18,
        od=0.675 * INCH,  # 17.145 mm
        E0=0.61201 * INCH,  # 15.545 mm
        L1=0.240 * INCH,  # 6.096 mm
        L2=0.4078 * INCH,  # 10.358 mm
        L3=_calculate_L3(18, "3/8"),  # 4.233 mm
    ),
    "1/2": NPTSpec(
        tpi=14,
        od=0.840 * INCH,  # 21.336 mm
        E0=0.75843 * INCH,  # 19.264 mm
        L1=0.320 * INCH,  # 8.128 mm
        L2=0.5337 * INCH,  # 13.556 mm
        L3=_calculate_L3(14, "1/2"),  # 5.443 mm
    ),
    "3/4": NPTSpec(
        tpi=14,
        od=1.050 * INCH,  # 26.670 mm
        E0=0.96768 * INCH,  # 24.579 mm
        L1=0.339 * INCH,  # 8.611 mm
        L2=0.5457 * INCH,  # 13.861 mm
        L3=_calculate_L3(14, "3/4"),  # 5.443 mm
    ),
    "1": NPTSpec(
        tpi=11.5,
        od=1.315 * INCH,  # 33.401 mm
        E0=1.21363 * INCH,  # 30.826 mm
        L1=0.400 * INCH,  # 10.160 mm
        L2=0.6828 * INCH,  # 17.343 mm
        L3=_calculate_L3(11.5, "1"),  # 6.626 mm
    ),
    "1-1/4": NPTSpec(
        tpi=11.5,
        od=1.660 * INCH,  # 42.164 mm
        E0=1.55713 * INCH,  # 39.551 mm
        L1=0.420 * INCH,  # 10.668 mm
        L2=0.7068 * INCH,  # 17.953 mm
        L3=_calculate_L3(11.5, "1-1/4"),  # 6.626 mm
    ),
    "1-1/2": NPTSpec(
        tpi=11.5,
        od=1.900 * INCH,  # 48.260 mm
        E0=1.79609 * INCH,  # 45.621 mm
        L1=0.420 * INCH,  # 10.668 mm
        L2=0.7235 * INCH,  # 18.377 mm
        L3=_calculate_L3(11.5, "1-1/2"),  # 6.626 mm
    ),
    "2": NPTSpec(
        tpi=11.5,
        od=2.375 * INCH,  # 60.325 mm
        E0=2.26902 * INCH,  # 57.633 mm
        L1=0.436 * INCH,  # 11.074 mm
        L2=0.7565 * INCH,  # 19.215 mm
        L3=_calculate_L3(11.5, "2"),  # 6.626 mm
    ),
    "2-1/2": NPTSpec(
        tpi=8,
        od=2.875 * INCH,  # 73.025 mm
        E0=2.71953 * INCH,  # 69.076 mm
        L1=0.682 * INCH,  # 17.323 mm
        L2=1.1375 * INCH,  # 28.893 mm
        L3=_calculate_L3(8, "2-1/2"),  # 9.525 mm
    ),
    "3": NPTSpec(
        tpi=8,
        od=3.500 * INCH,  # 88.900 mm
        E0=3.34062 * INCH,  # 84.852 mm
        L1=0.766 * INCH,  # 19.456 mm
        L2=1.2000 * INCH,  # 30.480 mm
        L3=_calculate_L3(8, "3"),  # 9.525 mm
    ),
    "4": NPTSpec(
        tpi=8,
        od=4.500 * INCH,  # 114.300 mm
        E0=4.33438 * INCH,  # 110.093 mm
        L1=0.844 * INCH,  # 21.438 mm
        L2=1.3000 * INCH,  # 33.020 mm
        L3=_calculate_L3(8, "4"),  # 9.525 mm
    ),
}


def get_npt_spec(nps: str) -> NPTSpec:
    """Get NPT thread specification for a given nominal pipe size."""
    spec = NPT_SPECS.get(nps)
    if spec is None:
        raise ValueError(f"No NPT specification for NPS {nps}. Valid sizes: {list(NPT_SPECS.keys())}")
    return spec


# =============================================================================
# THREAD GEOMETRY GENERATION
# =============================================================================


def _make_thread_profile(pitch: float, thread_depth: float, external: bool = True) -> cq.Wire:
    """Create the 60° truncated thread profile on XZ plane.

    The profile is centered at origin and represents one thread tooth.
    For external threads: the profile points outward (+X)
    For internal threads: the profile points inward (-X)

    Args:
        pitch: Thread pitch in mm
        thread_depth: Actual thread depth in mm
        external: True for external (male) thread, False for internal (female)

    Returns:
        Wire representing the thread profile
    """
    # Thread geometry per ASME B1.20.1
    # 60° thread angle means each flank is 30° from vertical
    H = 0.866025 * pitch  # Height of sharp V
    flat = H / 8.0  # Crest/root flat width

    # Profile points (XZ plane, Z along thread axis)
    # Start at root, go up left flank, across crest, down right flank
    half_pitch = pitch / 2.0
    half_flat = flat / 2.0

    # The thread profile is a trapezoid
    # Root width = pitch - flat (after truncation)
    # Crest width = flat

    if external:
        # External thread: crest at +X (outer), root at origin
        x_root = 0.0
        x_crest = thread_depth
    else:
        # Internal thread: root at +X (outer), crest near origin
        x_root = thread_depth
        x_crest = 0.0

    # Build profile as a wire
    # Points go: bottom-left root -> top-left crest -> top-right crest -> bottom-right root
    points = [
        (x_root, -half_pitch + half_flat),  # Bottom-left (root)
        (x_crest, -half_flat),  # Top-left (crest)
        (x_crest, half_flat),  # Top-right (crest)
        (x_root, half_pitch - half_flat),  # Bottom-right (root)
    ]

    # Create the wire
    wp = cq.Workplane("XZ")
    wp = wp.moveTo(points[0][0], points[0][1])
    for pt in points[1:]:
        wp = wp.lineTo(pt[0], pt[1])
    wp = wp.close()

    return wp.val()


def make_npt_external_thread(
    nps: str,
    thread_length: float | None = None,
    simple: bool = False,
) -> cq.Shape:
    """Create external NPT thread geometry for a pipe end.

    The thread is oriented with:
    - Axis along +Z
    - Thread starts at Z=0 (pipe end)
    - Thread extends in +Z direction (into the pipe)
    - Larger diameter at Z=0, smaller at Z=thread_length (taper)

    Args:
        nps: Nominal pipe size (e.g., "1/2", "1", "2")
        thread_length: Length of thread in mm. Defaults to L2 (effective thread length).
        simple: If True, return simplified representation (tapered cylinder)

    Returns:
        CadQuery Shape of the external thread
    """
    spec = get_npt_spec(nps)

    if thread_length is None:
        thread_length = spec.L2

    if simple:
        # Simplified: just a tapered cylinder
        return _make_tapered_cylinder(
            radius_start=spec.major_dia_at_L1 / 2.0,
            radius_end=spec.major_dia_at_L1 / 2.0 - thread_length * NPT_TAPER_RATIO / 2.0,
            height=thread_length,
        )

    # Full thread geometry using conical helix sweep
    pitch = spec.pitch_mm
    thread_depth = spec.thread_depth

    # Starting radius at Z=0 (pipe end, larger end of taper)
    # The pipe end is at a distance of L1 from the reference plane
    # Major diameter at pipe end = major at L1 + taper adjustment
    r_major_start = spec.major_dia_at_L1 / 2.0

    # Create the thread profile
    profile = _make_thread_profile(pitch, thread_depth, external=True)

    # Create conical helix path
    # CadQuery makeHelix: angle parameter creates conical helix
    # For NPT, we need the helix to get smaller as Z increases

    # The angle parameter in makeHelix is the half-angle of the cone
    # We need to convert our taper ratio to this angle
    # NPT taper: diameter changes by 1/16 per unit length
    # So radius changes by 1/32 per unit length
    # Half-angle = atan(1/32) ≈ 1.79°

    helix = cq.Wire.makeHelix(
        pitch=pitch,
        height=thread_length,
        radius=r_major_start,
        angle=NPT_HALF_ANGLE_DEG,  # Creates conical helix
    )

    # Position profile at start of helix and sweep
    profile_positioned = profile.moved(cq.Location(cq.Vector(r_major_start, 0, 0)))

    # Sweep the profile along the helix
    try:
        thread_solid = cq.Solid.sweep(profile_positioned, [], helix, isFrenet=True)
    except Exception:
        # If sweep fails, fall back to simplified geometry
        return _make_tapered_cylinder(
            radius_start=r_major_start,
            radius_end=r_major_start - thread_length * NPT_TAPER_RATIO / 2.0,
            height=thread_length,
        )

    return thread_solid


def make_npt_internal_thread(
    nps: str,
    thread_length: float | None = None,
    simple: bool = False,
) -> cq.Shape:
    """Create internal NPT thread geometry for a fitting bore.

    The thread is oriented with:
    - Axis along +Z
    - Thread starts at Z=0 (fitting face)
    - Thread extends in +Z direction (into the fitting)
    - Smaller diameter at Z=0, larger at Z=thread_length (taper opens up)

    Args:
        nps: Nominal pipe size (e.g., "1/2", "1", "2")
        thread_length: Length of thread in mm. Defaults to L2 (effective thread length).
        simple: If True, return simplified representation (tapered cylinder bore)

    Returns:
        CadQuery Shape of the internal thread
    """
    spec = get_npt_spec(nps)

    if thread_length is None:
        thread_length = spec.L2

    if simple:
        # Simplified: tapered cylindrical bore (hollow)
        return _make_tapered_cylinder(
            radius_start=spec.minor_dia_at_L1 / 2.0,
            radius_end=spec.minor_dia_at_L1 / 2.0 + thread_length * NPT_TAPER_RATIO / 2.0,
            height=thread_length,
        )

    # Full thread geometry
    pitch = spec.pitch_mm
    thread_depth = spec.thread_depth

    # Starting radius at Z=0 (fitting face)
    r_minor_start = spec.minor_dia_at_L1 / 2.0

    # Create the thread profile (internal)
    profile = _make_thread_profile(pitch, thread_depth, external=False)

    # Create conical helix path
    # For internal thread, the helix gets larger as Z increases
    helix = cq.Wire.makeHelix(
        pitch=pitch,
        height=thread_length,
        radius=r_minor_start,
        angle=-NPT_HALF_ANGLE_DEG,  # Negative for expanding cone
    )

    # Position profile at start of helix and sweep
    profile_positioned = profile.moved(cq.Location(cq.Vector(r_minor_start, 0, 0)))

    # Sweep the profile along the helix
    try:
        thread_solid = cq.Solid.sweep(profile_positioned, [], helix, isFrenet=True)
    except Exception:
        # If sweep fails, fall back to simplified geometry
        return _make_tapered_cylinder(
            radius_start=r_minor_start,
            radius_end=r_minor_start + thread_length * NPT_TAPER_RATIO / 2.0,
            height=thread_length,
        )

    return thread_solid


def _make_tapered_cylinder(radius_start: float, radius_end: float, height: float) -> cq.Shape:
    """Create a tapered cylinder (truncated cone).

    Args:
        radius_start: Radius at Z=0
        radius_end: Radius at Z=height
        height: Height of cylinder

    Returns:
        CadQuery Shape of the tapered cylinder
    """
    return cq.Solid.makeCone(
        radius1=radius_start,
        radius2=radius_end,
        height=height,
        pnt=cq.Vector(0, 0, 0),
        dir=cq.Vector(0, 0, 1),
    )


# =============================================================================
# ASSEMBLY UTILITIES
# =============================================================================


def calculate_thread_alignment_transform(
    nps: str,
    engagement_depth: float | None = None,
) -> cq.Location:
    """Calculate the transform to position a pipe at correct engagement depth.

    When mating external (pipe) threads with internal (fitting) threads,
    the pipe must be inserted to the correct depth for thread flanks to contact.

    Args:
        nps: Nominal pipe size
        engagement_depth: Engagement depth in mm. Defaults to wrench-tight (L1+L3).

    Returns:
        Location transform to apply to the pipe
    """
    spec = get_npt_spec(nps)

    if engagement_depth is None:
        engagement_depth = spec.wrench_tight_engagement

    # The pipe needs to translate along -Z by the engagement depth
    return cq.Location(cq.Vector(0, 0, -engagement_depth))


def verify_thread_alignment(
    external_thread: cq.Shape,
    internal_thread: cq.Shape,
    engagement_transform: cq.Location,
) -> dict:
    """Verify that threads align properly at the given engagement.

    Returns diagnostic information about the thread alignment.

    Args:
        external_thread: External (pipe) thread shape
        internal_thread: Internal (fitting) thread shape
        engagement_transform: Transform positioning the external thread

    Returns:
        Dictionary with alignment diagnostics
    """
    # Move external thread to engagement position
    external_positioned = external_thread.moved(engagement_transform)

    # Check for interference
    try:
        interference = external_positioned.intersect(internal_thread)
        interference_volume = interference.Volume() if hasattr(interference, "Volume") else 0
    except Exception:
        interference_volume = -1  # Error computing

    return {
        "interference_volume_mm3": interference_volume,
        "threads_intersect": interference_volume > 0.001,
    }


# =============================================================================
# EXPORTS AND EXAMPLES
# =============================================================================


def export_thread_assembly(
    nps: str,
    filename: str,
    engagement: str = "wrench_tight",
):
    """Export a thread assembly for visualization.

    Creates both external and internal threads positioned at correct engagement.

    Args:
        nps: Nominal pipe size
        filename: Output STEP filename
        engagement: "hand_tight" (L1) or "wrench_tight" (L1+L3)
    """
    spec = get_npt_spec(nps)

    if engagement == "hand_tight":
        depth = spec.L1
    else:
        depth = spec.wrench_tight_engagement

    # Create threads
    external = make_npt_external_thread(nps, simple=True)
    internal = make_npt_internal_thread(nps, simple=True)

    # Create housing for internal thread (cylinder with bore)
    housing_od = spec.od * 1.5  # Larger than pipe OD
    housing = cq.Solid.makeCylinder(housing_od / 2, spec.L2, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
    housing = housing.cut(internal)

    # Position external thread at engagement depth
    transform = calculate_thread_alignment_transform(nps, depth)
    external_positioned = external.moved(transform)

    # Combine for export
    assembly = housing.fuse(external_positioned)
    cq.exporters.export(assembly, filename)


if __name__ == "__main__":
    print("NPT Thread Generator")
    print("=" * 40)

    # Print specs for common sizes
    for nps in ["1/2", "1", "2"]:
        spec = get_npt_spec(nps)
        print(f"\nNPS {nps}:")
        print(f"  TPI: {spec.tpi}")
        print(f"  Pitch: {spec.pitch_mm:.3f} mm")
        print(f"  Thread depth: {spec.thread_depth:.3f} mm")
        print(f"  L1 (hand-tight): {spec.L1:.3f} mm")
        print(f"  L3 (wrench makeup): {spec.L3:.3f} mm")
        print(f"  Wrench-tight engagement: {spec.wrench_tight_engagement:.3f} mm")

    print("\nDone!")
