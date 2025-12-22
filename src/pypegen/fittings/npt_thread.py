"""
NPT Thread Generator for CadQuery - Die-Cut Approach.

Creates actual 3D NPT (National Pipe Taper) thread geometry per ASME B1.20.1
using a die-cut (subtractive) approach that mimics real-world thread cutting.

Approach:
1. Create tapered pipe end at major thread OD
2. Create helical V-groove cutting tool by sweeping profile along helix
3. Boolean subtract cutter from pipe to form threads
4. Add gradual runout fade at pipe junction

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
# h = thread height = 0.8 * pitch (truncated thread height)
# f = truncation = 0.033 * pitch (minimum, radial from sharp V point to flat)
# F = flat width = ~0.038 * pitch (at both crest and root, F = 2 * f * tan(30°))


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
        """Actual thread depth per ASME B1.20.1: h = 0.8P."""
        return 0.8 * self.pitch_mm

    @property
    def truncation(self) -> float:
        """Radial truncation per ASME B1.20.1: f = 0.033P (minimum)."""
        return 0.033 * self.pitch_mm

    @property
    def flat_width(self) -> float:
        """Flat width at crest/root per ASME B1.20.1: F = 2 × f × tan(30°) ≈ 0.038P."""
        return 2 * self.truncation * math.tan(math.radians(30))

    @property
    def crest_flat(self) -> float:
        """Width of flat at thread crest/root (mm). Alias for flat_width."""
        return self.flat_width

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


def _calculate_L3(tpi: float) -> float:
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
        L3=_calculate_L3(27),  # 2.822 mm (3 threads)
    ),
    "1/4": NPTSpec(
        tpi=18,
        od=0.540 * INCH,  # 13.716 mm
        E0=0.47739 * INCH,  # 12.126 mm
        L1=0.2278 * INCH,  # 5.786 mm
        L2=0.4018 * INCH,  # 10.206 mm
        L3=_calculate_L3(18),  # 4.233 mm
    ),
    "3/8": NPTSpec(
        tpi=18,
        od=0.675 * INCH,  # 17.145 mm
        E0=0.61201 * INCH,  # 15.545 mm
        L1=0.240 * INCH,  # 6.096 mm
        L2=0.4078 * INCH,  # 10.358 mm
        L3=_calculate_L3(18),  # 4.233 mm
    ),
    "1/2": NPTSpec(
        tpi=14,
        od=0.840 * INCH,  # 21.336 mm
        E0=0.75843 * INCH,  # 19.264 mm
        L1=0.320 * INCH,  # 8.128 mm
        L2=0.5337 * INCH,  # 13.556 mm
        L3=_calculate_L3(14),  # 5.443 mm
    ),
    "3/4": NPTSpec(
        tpi=14,
        od=1.050 * INCH,  # 26.670 mm
        E0=0.96768 * INCH,  # 24.579 mm
        L1=0.339 * INCH,  # 8.611 mm
        L2=0.5457 * INCH,  # 13.861 mm
        L3=_calculate_L3(14),  # 5.443 mm
    ),
    "1": NPTSpec(
        tpi=11.5,
        od=1.315 * INCH,  # 33.401 mm
        E0=1.21363 * INCH,  # 30.826 mm
        L1=0.400 * INCH,  # 10.160 mm
        L2=0.6828 * INCH,  # 17.343 mm
        L3=_calculate_L3(11.5),  # 6.626 mm
    ),
    "1-1/4": NPTSpec(
        tpi=11.5,
        od=1.660 * INCH,  # 42.164 mm
        E0=1.55713 * INCH,  # 39.551 mm
        L1=0.420 * INCH,  # 10.668 mm
        L2=0.7068 * INCH,  # 17.953 mm
        L3=_calculate_L3(11.5),  # 6.626 mm
    ),
    "1-1/2": NPTSpec(
        tpi=11.5,
        od=1.900 * INCH,  # 48.260 mm
        E0=1.79609 * INCH,  # 45.621 mm
        L1=0.420 * INCH,  # 10.668 mm
        L2=0.7235 * INCH,  # 18.377 mm
        L3=_calculate_L3(11.5),  # 6.626 mm
    ),
    "2": NPTSpec(
        tpi=11.5,
        od=2.375 * INCH,  # 60.325 mm
        E0=2.26902 * INCH,  # 57.633 mm
        L1=0.436 * INCH,  # 11.074 mm
        L2=0.7565 * INCH,  # 19.215 mm
        L3=_calculate_L3(11.5),  # 6.626 mm
    ),
    "2-1/2": NPTSpec(
        tpi=8,
        od=2.875 * INCH,  # 73.025 mm
        E0=2.71953 * INCH,  # 69.076 mm
        L1=0.682 * INCH,  # 17.323 mm
        L2=1.1375 * INCH,  # 28.893 mm
        L3=_calculate_L3(8),  # 9.525 mm
    ),
    "3": NPTSpec(
        tpi=8,
        od=3.500 * INCH,  # 88.900 mm
        E0=3.34062 * INCH,  # 84.852 mm
        L1=0.766 * INCH,  # 19.456 mm
        L2=1.2000 * INCH,  # 30.480 mm
        L3=_calculate_L3(8),  # 9.525 mm
    ),
    "4": NPTSpec(
        tpi=8,
        od=4.500 * INCH,  # 114.300 mm
        E0=4.33438 * INCH,  # 110.093 mm
        L1=0.844 * INCH,  # 21.438 mm
        L2=1.3000 * INCH,  # 33.020 mm
        L3=_calculate_L3(8),  # 9.525 mm
    ),
}


def get_npt_spec(nps: str) -> NPTSpec:
    """Get NPT thread specification for a given nominal pipe size."""
    spec = NPT_SPECS.get(nps)
    if spec is None:
        raise ValueError(f"No NPT specification for NPS {nps}. Valid sizes: {list(NPT_SPECS.keys())}")
    return spec


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================


def _make_tapered_cylinder(radius_start: float, radius_end: float, height: float) -> cq.Solid:
    """Create a tapered cylinder (truncated cone).

    Args:
        radius_start: Radius at Z=0
        radius_end: Radius at Z=height
        height: Height of cylinder

    Returns:
        CadQuery Solid of the tapered cylinder
    """
    return cq.Solid.makeCone(
        radius1=radius_start,
        radius2=radius_end,
        height=height,
        pnt=cq.Vector(0, 0, 0),
        dir=cq.Vector(0, 0, 1),
    )


# =============================================================================
# DIE-CUT THREAD GENERATION
# =============================================================================


def make_npt_thread_profile(
    pitch: float,
    thread_depth: float,
    at_radius: float,
) -> cq.Workplane:
    """Create truncated 60° V-profile for NPT thread groove.

    The profile is a trapezoid representing the cross-section of the thread
    groove to be cut from the pipe. For external threads, this is the material
    removed by a threading die.

    Profile geometry (per ASME B1.20.1):
    - Thread depth: h = 0.8P
    - Flank angle: 30° (60° included)
    - Flat width at crest/root: F = 0.038P

    The profile is created in a plane perpendicular to the helix at the start
    point, with the groove opening facing inward (toward the pipe axis).

    Args:
        pitch: Thread pitch in mm
        thread_depth: Thread depth (h = 0.8P) in mm
        at_radius: Radius at which the profile apex (crest) is located

    Returns:
        CadQuery Workplane with closed wire profile
    """
    # Calculate profile dimensions
    flat_width = 0.038 * pitch
    half_flat = flat_width / 2

    # Flank angle is 30° from vertical
    # The axial span of each flank = thread_depth * tan(30°)
    flank_axial_span = thread_depth * math.tan(math.radians(30))

    # Profile points (in local YZ plane, will be positioned at radius)
    # Y = axial direction (along helix), Z = radial direction (inward is negative)
    # The groove cuts inward from the surface
    points = [
        (-half_flat, 0),  # Crest left (at surface)
        (-half_flat - flank_axial_span, -thread_depth),  # Root left
        (half_flat + flank_axial_span, -thread_depth),  # Root right
        (half_flat, 0),  # Crest right (at surface)
    ]

    # Create the profile in YZ plane, positioned at the radius
    profile = cq.Workplane("YZ").transformed(offset=(at_radius, 0, 0)).polyline(points).close()

    return profile


def make_tapered_pipe_end(
    nps: str,
    thread_length: float,
    pipe_id: float | None = None,
) -> cq.Shape:
    """Create pipe end with NPT taper at major thread diameter.

    The taper follows NPT specifications:
    - Z=0: Pipe free end (smallest diameter)
    - Z=thread_length: Pipe junction (largest diameter = pipe OD)
    - Taper ratio: 1:16 (diameter change per unit length)

    Args:
        nps: Nominal pipe size
        thread_length: Length of tapered section in mm
        pipe_id: Inner diameter for hollow pipe (optional)

    Returns:
        Tapered cone or hollow frustum
    """
    spec = get_npt_spec(nps)

    # At Z=0 (small end / free end)
    # The major diameter is smaller by the taper amount
    taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
    major_radius_small = spec.od / 2.0 - taper_delta

    # At Z=thread_length (large end / into pipe body)
    major_radius_large = spec.od / 2.0

    # Create tapered outer surface
    outer = _make_tapered_cylinder(major_radius_small, major_radius_large, thread_length)

    # If hollow, subtract bore
    if pipe_id is not None and pipe_id > 0:
        bore_radius = pipe_id / 2.0
        bore = cq.Solid.makeCylinder(bore_radius, thread_length)
        return outer.cut(bore)

    return outer


def make_npt_thread_cutter(
    nps: str,
    thread_length: float,
) -> cq.Shape:
    """Create helical thread ridges for external NPT threads.

    EXACT mirror of internal thread cutter (make_npt_internal_thread_cutter).

    Internal cutter geometry:
    - inner_r = minor_r - 0.3 (extends into bore for overlap)
    - major_r = minor_r + thread_depth (extends into wall)
    - WIDE span at inner_r, NARROW span at major_r
    - Points: inner_r first, then major_r

    External cutter geometry (exact mirror):
    - inner_r = root_r - 0.3 (extends into pipe for overlap)
    - outer_r = root_r + thread_depth = major_r (extends to pipe surface)
    - WIDE span at inner_r, NARROW span at outer_r
    - Points: inner_r first, then outer_r

    Orientation:
    - Axis along +Z
    - Z=0 is the small end (pipe free end)
    - Z=thread_length is the large end (into pipe body)
    - NPT taper: radius increases with Z

    Args:
        nps: Nominal pipe size
        thread_length: Length of threaded section in mm

    Returns:
        Solid representing the thread ridges to FUSE with base cylinder
    """
    spec = get_npt_spec(nps)
    pitch = spec.pitch_mm
    thread_depth = spec.thread_depth

    # Calculate root radius at Z=0 (small end)
    # This mirrors internal where we start at minor_r (bore surface)
    taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
    major_radius_at_small = spec.od / 2.0 - taper_delta
    root_radius_at_small = major_radius_at_small - thread_depth

    # Thread profile dimensions (per ASME B1.20.1)
    flat_width = 0.038 * pitch
    half_flat = flat_width / 2
    flank_axial_span = thread_depth * math.tan(math.radians(30))

    # Calculate number of thread turns
    num_turns = thread_length / pitch
    num_sections = max(int(num_turns * 36), 36)

    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.gp import gp_Pnt

    loft = BRepOffsetAPI_ThruSections(True, False)

    for i in range(num_sections + 1):
        t = i / num_sections
        z = t * thread_length
        angle = t * num_turns * 2 * math.pi

        # Radius at this Z position (NPT taper - INCREASES with Z)
        taper_offset = z * NPT_TAPER_RATIO / 2.0
        root_r = root_radius_at_small + taper_offset  # Thread root (base)
        major_r = root_r + thread_depth  # Pipe surface (crest)

        # Extend into pipe for overlap with base cylinder (mirrors internal)
        inner_r = root_r - 0.3

        cx = math.cos(angle)
        cy = math.sin(angle)

        # Profile mirrors internal exactly:
        # - WIDE span at inner_r (toward axis)
        # - NARROW span at major_r (at pipe surface)
        z_inner_top = z - half_flat - flank_axial_span
        z_inner_bot = z + half_flat + flank_axial_span
        z_outer_top = z - half_flat
        z_outer_bot = z + half_flat

        # Points ordered same as internal: inner first, then outer
        pts = [
            gp_Pnt(cx * inner_r, cy * inner_r, z_inner_top),  # Inner top (wide)
            gp_Pnt(cx * inner_r, cy * inner_r, z_inner_bot),  # Inner bot (wide)
            gp_Pnt(cx * major_r, cy * major_r, z_outer_bot),  # Outer bot (narrow)
            gp_Pnt(cx * major_r, cy * major_r, z_outer_top),  # Outer top (narrow)
        ]

        wire_builder = BRepBuilderAPI_MakeWire()
        for j in range(4):
            edge = BRepBuilderAPI_MakeEdge(pts[j], pts[(j + 1) % 4]).Edge()
            wire_builder.Add(edge)

        loft.AddWire(wire_builder.Wire())

    loft.Build()
    if not loft.IsDone():
        raise ValueError("Failed to create thread cutter loft")

    return cq.Shape(loft.Shape())


def make_npt_runout_cutter(
    nps: str,
    runout_turns: float = 0.25,
    z_offset: float = 0.0,
) -> cq.Shape | None:
    """Create fading runout section where thread depth decreases to zero.

    The runout creates a smooth transition from full thread to pipe surface
    using the same lofting approach as the main cutter but with decreasing depth.

    Args:
        nps: Nominal pipe size
        runout_turns: Runout length in thread turns (default 0.25 = 90°)
        z_offset: Z position where runout starts

    Returns:
        Solid representing the runout cutter, or None if runout too short
    """
    spec = get_npt_spec(nps)
    pitch = spec.pitch_mm
    thread_depth = spec.thread_depth
    runout_length = pitch * runout_turns

    if runout_length < 0.3:  # Too short for meaningful runout
        return None

    # Thread profile dimensions
    flat_width = 0.038 * pitch
    half_flat = flat_width / 2

    # Calculate starting angle (continuing from main thread)
    main_thread_turns = z_offset / pitch
    start_angle = main_thread_turns * 2 * math.pi

    try:
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
        from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
        from OCP.gp import gp_Pnt

        num_sections = max(int(runout_turns * 36), 12)
        loft = BRepOffsetAPI_ThruSections(True, False)

        for i in range(num_sections + 1):
            t = i / num_sections
            local_z = t * runout_length
            z = z_offset + local_z
            angle = start_angle + t * runout_turns * 2 * math.pi

            # Fade factor: 1.0 at start, 0.0 at end
            fade = 1.0 - t
            current_depth = thread_depth * fade
            if current_depth < 0.1:
                current_depth = 0.1  # Minimum depth

            flank_span = current_depth * math.tan(math.radians(30))

            # Radius at this Z position
            taper_per_z = NPT_TAPER_RATIO / 2.0
            major_r = spec.od / 2.0 - ((spec.L2 - z) * taper_per_z)
            root_r = major_r - current_depth
            outer_r = major_r + 0.3  # Extend beyond surface for clean cuts

            cx = math.cos(angle)
            cy = math.sin(angle)

            # V-groove: WIDE at surface, NARROW at root
            z_outer_top = z - half_flat - flank_span
            z_outer_bot = z + half_flat + flank_span
            z_root_top = z - half_flat
            z_root_bot = z + half_flat

            pts = [
                gp_Pnt(cx * outer_r, cy * outer_r, z_outer_top),
                gp_Pnt(cx * outer_r, cy * outer_r, z_outer_bot),
                gp_Pnt(cx * root_r, cy * root_r, z_root_bot),
                gp_Pnt(cx * root_r, cy * root_r, z_root_top),
            ]

            wire_builder = BRepBuilderAPI_MakeWire()
            for j in range(4):
                edge = BRepBuilderAPI_MakeEdge(pts[j], pts[(j + 1) % 4]).Edge()
                wire_builder.Add(edge)

            loft.AddWire(wire_builder.Wire())

        loft.Build()
        if loft.IsDone():
            return cq.Shape(loft.Shape())

    except Exception:
        pass

    return None


def make_npt_internal_runout_cutter(
    nps: str,
    runout_turns: float = 0.25,
    z_offset: float = 0.0,
) -> cq.Shape | None:
    """Create fading runout section for internal threads.

    The runout creates a smooth transition from full thread to bore surface
    as the thread goes deeper into the fitting.

    For internal threads:
    - Cutter is INSIDE the bore cutting OUTWARD into wall
    - Runout fades thread depth to zero going INTO fitting

    Args:
        nps: Nominal pipe size
        runout_turns: Runout length in thread turns (default 0.25 = 90°)
        z_offset: Z position where runout starts

    Returns:
        Solid representing the runout cutter, or None if runout too short
    """
    spec = get_npt_spec(nps)
    pitch = spec.pitch_mm
    thread_depth = spec.thread_depth
    runout_length = pitch * runout_turns

    if runout_length < 0.3:  # Too short for meaningful runout
        return None

    # Thread profile dimensions
    flat_width = 0.038 * pitch
    half_flat = flat_width / 2

    # Starting radius at z_offset (bore surface)
    minor_radius_at_start = spec.od / 2.0

    # Calculate starting angle (continuing from main thread)
    main_thread_turns = z_offset / pitch
    start_angle = main_thread_turns * 2 * math.pi

    try:
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
        from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
        from OCP.gp import gp_Pnt

        num_sections = max(int(runout_turns * 36), 12)
        loft = BRepOffsetAPI_ThruSections(True, False)

        for i in range(num_sections + 1):
            t = i / num_sections
            local_z = t * runout_length
            z = z_offset + local_z
            angle = start_angle + t * runout_turns * 2 * math.pi

            # Fade factor: 1.0 at start, 0.0 at end
            fade = 1.0 - t
            current_depth = thread_depth * fade
            if current_depth < 0.1:
                current_depth = 0.1  # Minimum depth

            flank_span = current_depth * math.tan(math.radians(30))

            # Radius at this Z position (NPT taper - DECREASES with Z for internal)
            taper_offset = z * NPT_TAPER_RATIO / 2.0
            minor_r = minor_radius_at_start - taper_offset  # Bore surface (decreases going in)
            major_r = minor_r + current_depth  # Thread root (into wall, fading)

            # Extend cutter slightly inside bore for clean cuts
            inner_r = minor_r - 0.3

            cx = math.cos(angle)
            cy = math.sin(angle)

            # V-groove for INTERNAL threads cutting OUTWARD:
            # WIDE at inner_r (bore surface), NARROW at major_r (into wall)
            z_inner_top = z - half_flat - flank_span
            z_inner_bot = z + half_flat + flank_span
            z_outer_top = z - half_flat
            z_outer_bot = z + half_flat

            pts = [
                gp_Pnt(cx * inner_r, cy * inner_r, z_inner_top),
                gp_Pnt(cx * inner_r, cy * inner_r, z_inner_bot),
                gp_Pnt(cx * major_r, cy * major_r, z_outer_bot),
                gp_Pnt(cx * major_r, cy * major_r, z_outer_top),
            ]

            wire_builder = BRepBuilderAPI_MakeWire()
            for j in range(4):
                edge = BRepBuilderAPI_MakeEdge(pts[j], pts[(j + 1) % 4]).Edge()
                wire_builder.Add(edge)

            loft.AddWire(wire_builder.Wire())

        loft.Build()
        if loft.IsDone():
            return cq.Shape(loft.Shape())

    except Exception:
        pass

    return None


def make_npt_internal_thread_cutter(
    nps: str,
    thread_length: float,
) -> cq.Shape:
    """Create helical V-groove cutting tool for internal NPT threads.

    The cutter is positioned INSIDE the bore and cuts OUTWARD into the
    fitting wall. This mirrors the external thread cutter approach.

    When subtracted from a fitting with a tapered bore, this cutter
    creates proper internal NPT threads CUT into the wall.

    V-groove geometry:
    - WIDE at bore surface (minor_r) - where groove enters wall
    - NARROW at thread root (major_r) - deepest into wall

    Orientation:
    - Axis along +Z
    - Z=0 is the fitting face (SMALLEST diameter)
    - Z=thread_length is into the fitting body (LARGEST diameter)
    - NPT taper: radius INCREASES with Z

    Args:
        nps: Nominal pipe size
        thread_length: Length of threaded section in mm

    Returns:
        Solid representing the cutting tool to SUBTRACT from fitting
    """
    spec = get_npt_spec(nps)
    pitch = spec.pitch_mm
    thread_depth = spec.thread_depth

    # For internal threads at Z=0 (fitting face - SMALLEST diameter)
    # minor_r = bore surface (where external pipe crests contact)
    # major_r = minor_r + thread_depth (thread roots - deepest into wall)
    minor_radius_at_start = spec.od / 2.0

    # Thread profile dimensions (per ASME B1.20.1)
    flat_width = 0.038 * pitch  # Flat at crest/root
    half_flat = flat_width / 2
    flank_axial_span = thread_depth * math.tan(math.radians(30))  # 60° included angle

    # Calculate number of thread turns
    num_turns = thread_length / pitch

    # Create thread groove solid by lofting cross-sections
    num_sections = max(int(num_turns * 36), 36)  # At least 36 sections

    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.gp import gp_Pnt

    loft = BRepOffsetAPI_ThruSections(True, False)  # solid=True, ruled=False

    for i in range(num_sections + 1):
        t = i / num_sections
        z = t * thread_length
        angle = t * num_turns * 2 * math.pi  # Angle around helix

        # Radius at this Z position (NPT taper - DECREASES with Z for internal)
        # Z=0 is opening (LARGEST), Z=thread_length is into fitting (SMALLEST)
        taper_offset = z * NPT_TAPER_RATIO / 2.0
        minor_r = minor_radius_at_start - taper_offset  # Bore surface (decreases going in)
        major_r = minor_r + thread_depth  # Thread root (into wall)

        # Extend cutter slightly inside bore for clean cuts
        inner_r = minor_r - 0.3

        # Profile center position (in XY plane at angle)
        cx = math.cos(angle)
        cy = math.sin(angle)

        # Create V-groove profile points (4 corners of trapezoid)
        # For INTERNAL threads cutting OUTWARD:
        # - WIDE span at inner_r (bore surface, where groove enters wall)
        # - NARROW span at major_r (thread root, deepest into wall)

        # Inner edge (at bore surface) - WIDE span
        z_inner_top = z - half_flat - flank_axial_span
        z_inner_bot = z + half_flat + flank_axial_span

        # Outer edge (at thread root) - NARROW span (just the flat)
        z_outer_top = z - half_flat
        z_outer_bot = z + half_flat

        pts = [
            gp_Pnt(cx * inner_r, cy * inner_r, z_inner_top),  # Inner top (wide)
            gp_Pnt(cx * inner_r, cy * inner_r, z_inner_bot),  # Inner bot (wide)
            gp_Pnt(cx * major_r, cy * major_r, z_outer_bot),  # Outer bot (narrow)
            gp_Pnt(cx * major_r, cy * major_r, z_outer_top),  # Outer top (narrow)
        ]

        # Build wire from points
        wire_builder = BRepBuilderAPI_MakeWire()
        for j in range(4):
            edge = BRepBuilderAPI_MakeEdge(pts[j], pts[(j + 1) % 4]).Edge()
            wire_builder.Add(edge)

        wire = wire_builder.Wire()
        loft.AddWire(wire)

    loft.Build()
    if not loft.IsDone():
        raise ValueError("Failed to create internal thread cutter loft")

    return cq.Shape(loft.Shape())


def make_pipe_end_chamfer_cutter(
    major_radius: float,
    chamfer_size: float,
    chamfer_angle: float = 30.0,
) -> cq.Solid:
    """Create a cutter for external chamfer at pipe free end.

    Creates a conical cutter that bevels the outer corner of the pipe end.
    The chamfer faces OUTWARD (visible when looking at pipe end from outside).

    For NPT threads, the chamfer angle should be 30° (matching thread flank angle).

    Args:
        major_radius: Major thread radius at Z=0 (pipe free end)
        chamfer_size: Size of chamfer in mm (radial depth into pipe)
        chamfer_angle: Angle from pipe axis (default 30° for NPT)

    Returns:
        Solid to subtract from pipe to create external chamfer
    """
    # Calculate chamfer axial depth based on angle
    # For 30° chamfer: axial_depth = chamfer_size / tan(30°) ≈ 1.73 * chamfer_size
    chamfer_axial = chamfer_size / math.tan(math.radians(chamfer_angle))

    # The chamfer cutter is a cone that:
    # - At Z=0: has radius = major_radius + margin (beyond pipe surface)
    # - At Z=chamfer_axial: has radius = major_radius - chamfer_size (cuts into pipe)
    # This creates an outward-facing bevel at the pipe end

    outer_radius = major_radius + 2.0  # Well beyond pipe surface
    inner_radius = major_radius - chamfer_size

    # Create the chamfer cone starting at Z=0
    # Large radius at Z=0, small radius at Z=chamfer_axial
    chamfer_cone = cq.Solid.makeCone(
        radius1=outer_radius,
        radius2=inner_radius,
        height=chamfer_axial,
        pnt=cq.Vector(0, 0, 0),
        dir=cq.Vector(0, 0, 1),
    )

    return chamfer_cone


def apply_bore_chamfer(
    bore_blank: cq.Shape,
    radial_depth: float,
    axial_depth: float,
) -> cq.Shape:
    """Apply internal chamfer to bore opening by fusing a flared cone.

    Creates a cone that flares OUTWARD at Z=0 and fuses it with the bore,
    effectively enlarging the bore entry to create a funnel-like guide for
    pipe insertion.

    This is different from the external pipe chamfer (which uses OCCT's
    BRepFilletAPI_MakeChamfer to cut into the pipe). For internal bore
    chamfer, we need to ADD material to the bore volume (enlarge it),
    so we fuse a cone instead.

    This should be called BEFORE adding thread ridges, as the simple cone
    geometry fuses cleanly with the tapered bore.

    Args:
        bore_blank: The tapered bore cylinder (before thread ridges are added)
        radial_depth: Chamfer depth radially outward from bore (mm)
        axial_depth: Chamfer depth axially into bore (mm)

    Returns:
        Bore with flared entry at Z=0 (larger radius at opening)
    """
    try:
        # Find the radius at Z=0 by examining the geometry vertices
        from OCP.TopAbs import TopAbs_VERTEX
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopoDS import TopoDS

        solid = bore_blank.wrapped if hasattr(bore_blank, "wrapped") else bore_blank
        min_radius_at_z0 = float("inf")

        explorer = TopExp_Explorer(solid, TopAbs_VERTEX)
        while explorer.More():
            vertex = TopoDS.Vertex_s(explorer.Current())
            v = cq.Vertex(vertex)
            pnt = v.toTuple()
            if abs(pnt[2]) < 0.01:  # At Z=0
                r = math.sqrt(pnt[0] ** 2 + pnt[1] ** 2)
                min_radius_at_z0 = min(min_radius_at_z0, r)
            explorer.Next()

        if min_radius_at_z0 == float("inf"):
            # Fallback: estimate from bounding box
            bb = bore_blank.BoundingBox()
            min_radius_at_z0 = min(abs(bb.xmin), abs(bb.xmax))

        # Create chamfer cone - LARGER radius at Z=0 (flared), normal at Z=axial_depth
        chamfer_radius_at_z0 = min_radius_at_z0 + radial_depth
        chamfer_radius_at_end = min_radius_at_z0

        chamfer_cone = cq.Solid.makeCone(
            radius1=chamfer_radius_at_z0,  # Large at Z=0 (flared opening)
            radius2=chamfer_radius_at_end,  # Normal bore radius at Z=axial_depth
            height=axial_depth,
            pnt=cq.Vector(0, 0, 0),
            dir=cq.Vector(0, 0, 1),
        )

        # Fuse the chamfer cone with the bore
        return bore_blank.fuse(chamfer_cone)

    except Exception:
        return bore_blank


def apply_pipe_end_chamfer(
    pipe_blank: cq.Shape,
    radial_depth: float,
    axial_depth: float,
) -> cq.Shape:
    """Apply external chamfer to pipe free end on OD edge only.

    Uses direct OCCT API to apply asymmetric chamfer to only the outer edge
    at Z=0, leaving the inner edge (bore) unchanged.

    This should be called BEFORE cutting threads, as chamfer works
    reliably on simple geometry but can fail on complex threaded surfaces.

    Args:
        pipe_blank: The tapered pipe section (before threads are cut)
        radial_depth: Chamfer depth radially into pipe wall (mm)
        axial_depth: Chamfer depth axially into pipe (mm)

    Returns:
        Pipe with chamfered entry edge at Z=0 (OD only)
    """
    from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.TopExp import TopExp, TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape

    try:
        solid = pipe_blank.wrapped if hasattr(pipe_blank, "wrapped") else pipe_blank

        # Build edge-to-face map
        edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
        TopExp.MapShapesAndAncestors_s(solid, TopAbs_EDGE, TopAbs_FACE, edge_face_map)

        # Find the OD edge at Z=0 (the largest radius circular edge)
        od_edge = None
        outer_face = None
        max_radius = 0.0

        # First pass: find all edges at Z=0 and identify the largest radius
        explorer = TopExp_Explorer(solid, TopAbs_EDGE)
        while explorer.More():
            edge = TopoDS.Edge_s(explorer.Current())
            edge_shape = cq.Edge(edge)
            start = edge_shape.startPoint()
            end = edge_shape.endPoint()

            # Check if edge is at Z=0 (circular edge on bottom face)
            if abs(start.z) < 0.01 and abs(end.z) < 0.01:
                edge_radius = math.sqrt(start.x**2 + start.y**2)
                # Track largest radius (OD edge)
                if edge_radius > max_radius:
                    # Get adjacent faces
                    adjacent_faces = edge_face_map.FindFromKey(edge)
                    if not adjacent_faces.IsEmpty():
                        f1 = TopoDS.Face_s(adjacent_faces.First())
                        fc1 = cq.Face(f1).Center()
                        f2 = TopoDS.Face_s(adjacent_faces.Last())
                        fc2 = cq.Face(f2).Center()

                        # Outer face is the conical surface (center not at Z=0)
                        candidate_face = None
                        if abs(fc1.z) > 0.1:
                            candidate_face = f1
                        elif abs(fc2.z) > 0.1:
                            candidate_face = f2

                        if candidate_face is not None:
                            od_edge = edge
                            outer_face = candidate_face
                            max_radius = edge_radius

            explorer.Next()

        if od_edge is None or outer_face is None:
            return pipe_blank

        # Apply asymmetric chamfer to OD edge only
        chamfer_maker = BRepFilletAPI_MakeChamfer(solid)
        chamfer_maker.Add(axial_depth, radial_depth, od_edge, outer_face)
        chamfer_maker.Build()

        if chamfer_maker.IsDone():
            return cq.Shape(chamfer_maker.Shape())
        else:
            return pipe_blank

    except Exception:
        return pipe_blank


def apply_bore_id_chamfer(
    fitting: cq.Shape,
    bore_axis: tuple[float, float, float],
    bore_opening: tuple[float, float, float],
    radial_depth: float,
    axial_depth: float,
) -> cq.Shape:
    """Apply chamfer to ID edge of bore using BRepFilletAPI_MakeChamfer.

    Creates a flared entry at the bore opening (chamfer turns outward).
    Uses the same OCCT chamfer API as apply_pipe_end_chamfer.

    Args:
        fitting: The fitting solid with a bored hole
        bore_axis: Unit vector along bore axis pointing INTO the fitting
        bore_opening: 3D point at the center of the bore opening face
        radial_depth: Chamfer depth radially into wall (mm)
        axial_depth: Chamfer depth axially into fitting (mm)

    Returns:
        Fitting with chamfered bore entry edge
    """
    from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.TopExp import TopExp, TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape

    try:
        solid = fitting.wrapped if hasattr(fitting, "wrapped") else fitting

        # Build edge-to-face map
        edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
        TopExp.MapShapesAndAncestors_s(solid, TopAbs_EDGE, TopAbs_FACE, edge_face_map)

        opening_x, opening_y, opening_z = bore_opening
        ax, ay, az = bore_axis

        # Find all circular edges at the bore opening plane
        # We want the ID edge (smaller radius), not the OD edge
        best_edge = None
        best_face = None
        best_radius = float("inf")

        explorer = TopExp_Explorer(solid, TopAbs_EDGE)
        while explorer.More():
            edge = TopoDS.Edge_s(explorer.Current())
            edge_shape = cq.Edge(edge)
            start = edge_shape.startPoint()
            end = edge_shape.endPoint()

            # Vector from opening center to start point
            dx = start.x - opening_x
            dy = start.y - opening_y
            dz = start.z - opening_z

            # Distance along bore axis
            dist_along = dx * ax + dy * ay + dz * az

            # Check if edge is at opening plane (dist_along ~ 0)
            if abs(dist_along) < 0.5:
                # Calculate perpendicular distance (radius from bore axis)
                # Remove the component along axis to get perpendicular vector
                perp_x = dx - dist_along * ax
                perp_y = dy - dist_along * ay
                perp_z = dz - dist_along * az
                radius = math.sqrt(perp_x**2 + perp_y**2 + perp_z**2)

                # Check end point too - both should have same radius for circular edge
                dx_e = end.x - opening_x
                dy_e = end.y - opening_y
                dz_e = end.z - opening_z
                dist_along_e = dx_e * ax + dy_e * ay + dz_e * az
                perp_x_e = dx_e - dist_along_e * ax
                perp_y_e = dy_e - dist_along_e * ay
                perp_z_e = dz_e - dist_along_e * az
                radius_e = math.sqrt(perp_x_e**2 + perp_y_e**2 + perp_z_e**2)

                # Circular edge: both endpoints at same radius
                if abs(radius - radius_e) < 0.5 and radius > 5.0:
                    # We want the smallest radius (ID edge, not OD edge)
                    if radius < best_radius:
                        # Get adjacent faces
                        adjacent_faces = edge_face_map.FindFromKey(edge)
                        if not adjacent_faces.IsEmpty():
                            # Find the bore surface (tapered face going INTO fitting)
                            for i in range(adjacent_faces.Size()):
                                if i == 0:
                                    f = TopoDS.Face_s(adjacent_faces.First())
                                else:
                                    f = TopoDS.Face_s(adjacent_faces.Last())
                                fc = cq.Face(f).Center()

                                # Bore face center is INSIDE the fitting (along axis)
                                fc_dx = fc.x - opening_x
                                fc_dy = fc.y - opening_y
                                fc_dz = fc.z - opening_z
                                fc_along = fc_dx * ax + fc_dy * ay + fc_dz * az

                                # Bore face extends into fitting (positive along axis)
                                if fc_along > 0.5:
                                    best_edge = edge
                                    best_face = f
                                    best_radius = radius
                                    break

            explorer.Next()

        if best_edge is None or best_face is None:
            return fitting

        # Apply chamfer to bore edge
        chamfer_maker = BRepFilletAPI_MakeChamfer(solid)
        chamfer_maker.Add(axial_depth, radial_depth, best_edge, best_face)
        chamfer_maker.Build()

        if chamfer_maker.IsDone():
            return cq.Shape(chamfer_maker.Shape())
        else:
            return fitting

    except Exception:
        return fitting


# =============================================================================
# MAIN API FUNCTIONS
# =============================================================================


def make_npt_external_thread(
    nps: str,
    thread_length: float | None = None,
    simple: bool = False,
    pipe_id: float | None = None,
    end_finishes: tuple[str, str] = ("chamfer", "fade"),
    runout_turns: float = 0.25,
) -> cq.Shape:
    """Create external NPT thread geometry.

    Uses the SAME approach as internal threads (make_npt_internal_thread):
    1. Create base cylinder at root radius
    2. Create thread ridges that extend outward to major radius
    3. FUSE ridges with base cylinder to create threaded section
    4. Cut bore if hollow pipe

    This mirrors internal threads which:
    1. Create base bore at minor radius
    2. Create thread ridges that extend outward to major radius
    3. FUSE ridges with base bore

    Orientation:
    - Axis along +Z
    - Z=0 is the pipe free end (smallest diameter)
    - Z=thread_length is the pipe junction (largest diameter)
    - Diameter INCREASES as Z increases (NPT taper)

    Args:
        nps: Nominal pipe size (e.g., "1/2", "1", "2")
        thread_length: Thread length in mm (defaults to L2)
        simple: If True, return simplified envelope geometry (no threads)
        pipe_id: Inner diameter for hollow pipe
        end_finishes: Tuple of (free_end, pipe_junction_end):
            - "chamfer": Add entry chamfer (default for free end)
            - "raw": No special treatment
        runout_turns: Fade length in thread turns (unused currently)

    Returns:
        CadQuery Shape of the threaded pipe section
    """
    spec = get_npt_spec(nps)

    if thread_length is None:
        thread_length = spec.L2

    thread_depth = spec.thread_depth
    free_end_finish = end_finishes[0] if len(end_finishes) > 0 else "chamfer"

    # Calculate radii (mirrors internal thread calculation)
    # For external: root_r is the base, major_r is the crest (pipe OD)
    taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
    major_radius_start = spec.od / 2.0 - taper_delta  # At Z=0
    major_radius_end = spec.od / 2.0  # At Z=thread_length
    root_radius_start = major_radius_start - thread_depth
    root_radius_end = major_radius_end - thread_depth

    if simple:
        # Simplified: tapered cylinder at major (crest) diameter
        result = _make_tapered_cylinder(major_radius_start, major_radius_end, thread_length)
        if pipe_id is not None and pipe_id > 0:
            bore = cq.Solid.makeCylinder(pipe_id / 2.0, thread_length)
            result = result.cut(bore)
        return result

    try:
        # Step 1: Create base cylinder at root radius (mirrors internal base bore)
        base_cylinder = _make_tapered_cylinder(root_radius_start, root_radius_end, thread_length)

        # Step 2: Apply chamfer to base cylinder if requested
        # (This adds a chamfer at the thread root level)
        if free_end_finish == "chamfer":
            radial_depth = thread_depth
            axial_depth = radial_depth * math.tan(math.radians(30))
            # For external, we need to chamfer the OD edge, but base is at root
            # Skip chamfer on base, apply to final shape instead

        # Step 3: Create thread ridges (mirrors internal thread cutter)
        thread_ridges = make_npt_thread_cutter(nps, thread_length)

        # Step 4: Fuse ridges with base cylinder (same as internal)
        threaded_section = base_cylinder.fuse(thread_ridges)

        # Step 5: Apply chamfer to the threaded section OD if requested
        if free_end_finish == "chamfer":
            radial_depth = thread_depth
            axial_depth = radial_depth * math.tan(math.radians(30))
            threaded_section = apply_pipe_end_chamfer(threaded_section, radial_depth, axial_depth)

        # Step 6: Cut bore if hollow pipe
        if pipe_id is not None and pipe_id > 0:
            bore = cq.Solid.makeCylinder(pipe_id / 2.0, thread_length)
            threaded_section = threaded_section.cut(bore)

        if threaded_section.isValid():
            return threaded_section

    except Exception as e:
        print(f"External thread creation failed: {e}, using simplified geometry")

    # Fallback to simplified geometry
    result = _make_tapered_cylinder(major_radius_start, major_radius_end, thread_length)
    if pipe_id is not None and pipe_id > 0:
        bore = cq.Solid.makeCylinder(pipe_id / 2.0, thread_length)
        result = result.cut(bore)
    return result


def make_npt_internal_thread(
    nps: str,
    thread_length: float | None = None,
    simple: bool = False,
    end_finishes: tuple[str, str] = ("chamfer", "fade"),
) -> cq.Shape:
    """Create internal NPT thread geometry (bore volume to cut from fitting).

    Uses the die-cut approach: creates thread ridges using lofting and fuses
    with a base bore cylinder. Supports chamfering at the bore opening.

    The thread is oriented with:
    - Axis along +Z
    - Z=0 is the fitting face (opening) - SMALLEST diameter
    - Thread extends in +Z direction (into the fitting body)
    - Diameter INCREASES as Z increases (NPT taper opens up into fitting)

    For internal threads:
    - Thread crests (apex) project INTO the bore (at smaller radius)
    - Thread roots (valleys) are at larger radius (deeper into fitting wall)
    - The NPT taper means diameter INCREASES going into the fitting

    This function returns the BORE VOLUME to be CUT from the fitting.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "1", "2")
        thread_length: Length of thread in mm. Defaults to L2 (effective thread length).
        simple: If True, return simplified representation (tapered cylinder bore)
        end_finishes: Tuple of (start_finish, end_finish).
            start_finish applies to Z=0 (fitting face): "chamfer" or "raw"
            end_finish applies to Z=thread_length (into fitting): "fade" or "raw"

    Returns:
        CadQuery Shape representing the material to CUT from the fitting
    """
    spec = get_npt_spec(nps)

    if thread_length is None:
        thread_length = spec.L2

    thread_depth = spec.thread_depth

    # For internal threads:
    # - Apex (crest) projects INTO bore at pipe OD (matches external pipe OD)
    # - Root (valley) is at larger radius (thread cuts into fitting wall)
    # - NPT taper: diameter INCREASES going into the fitting (+Z direction)

    # At Z=0 (fitting face - SMALLEST diameter)
    apex_radius_start = spec.od / 2.0  # Internal crests match external pipe OD
    root_radius_start = apex_radius_start + thread_depth  # Roots are larger

    # At Z=thread_length (diameter INCREASES - larger)
    taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
    apex_radius_end = apex_radius_start + taper_delta
    root_radius_end = root_radius_start + taper_delta

    if simple:
        # Simplified: tapered cylinder at root (major) diameter
        return _make_tapered_cylinder(root_radius_start, root_radius_end, thread_length)

    # Parse end finishes
    start_finish = end_finishes[0] if len(end_finishes) > 0 else "chamfer"

    try:
        # Step 1: Create base bore at apex (minor) radius
        base_bore = _make_tapered_cylinder(apex_radius_start, apex_radius_end, thread_length)

        # Step 2: Apply ID chamfer BEFORE creating thread ridges (if requested)
        # The chamfer creates a flared entry at Z=0 (fitting face)
        # Chamfer is 60° from vertical (30° from horizontal) - steeper angle
        if start_finish == "chamfer":
            radial_depth = thread_depth
            # 60° from vertical means axial = radial × tan(30°)
            axial_depth = radial_depth * math.tan(math.radians(30))
            base_bore = apply_bore_chamfer(base_bore, radial_depth, axial_depth)

        # Step 3: Create thread ridges using lofted cutter
        thread_ridges = make_npt_internal_thread_cutter(nps, thread_length)

        # Step 4: Fuse ridges with base bore to get complete bore volume
        threaded_bore = base_bore.fuse(thread_ridges)

        if threaded_bore.isValid():
            return threaded_bore

    except Exception as e:
        print(f"Internal thread creation failed: {e}, using simplified geometry")

    # Fallback to simplified geometry (tapered cylinder at root diameter)
    return _make_tapered_cylinder(root_radius_start, root_radius_end, thread_length)


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
    external = make_npt_external_thread(nps, simple=False)
    internal = make_npt_internal_thread(nps, simple=False)

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
    print("NPT Thread Generator (Die-Cut Approach)")
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
