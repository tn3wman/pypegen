"""
NPT Thread Generator for CadQuery.

Creates actual 3D NPT (National Pipe Taper) thread geometry per ASME B1.20.1.
Supports both external (pipe) and internal (fitting) threads.

NPT Thread Specifications:
- Taper: 1:16 (3/4 inch per foot)
- Half-angle: 1° 47' 24" (1.7899°)
- Thread angle: 60°
- Thread form: Truncated flat crests/roots

Implementation uses ruled surfaces between helical wires (like cq_warehouse)
for reliable thread generation.

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
# THREAD GEOMETRY GENERATION - RULED SURFACE APPROACH (based on cq_warehouse)
# =============================================================================


def _make_thread_solid(
    apex_radius: float,
    root_radius: float,
    pitch: float,
    length: float,
    taper_angle: float = 360.0,
    external: bool = True,
) -> cq.Shape:
    """Create thread solid using ruled surfaces between helical wires.

    This follows cq_warehouse's proven approach:
    1. Create 4 helical wires (2 at apex, 2 at root) with Z-offsets for width
    2. Connect wires with ruled surfaces to form thread flanks
    3. Add end caps
    4. Create shell and solid

    For external threads: apex is outer (crest), root is inner (valley)
    For internal threads: apex is inner (crest projects in), root is outer

    Args:
        apex_radius: Radius at thread crest (tips)
        root_radius: Radius at thread root (valleys)
        pitch: Thread pitch (mm)
        length: Thread length (mm)
        taper_angle: Helix taper angle in degrees (360 = no taper, else half-angle)
        external: True for external threads (apex > root)

    Returns:
        Thread solid as CadQuery Shape
    """
    # For 60° ISO/NPT threads, calculate widths per cq_warehouse formulas
    # h_parameter = (pitch/2) / tan(30°) = pitch * 0.866
    # For external: apex_width = pitch/8, root_width = 3*pitch/4
    # For internal: apex_width = pitch/4, root_width = 7*pitch/8

    if external:
        apex_width = pitch / 8.0
        root_width = 3.0 * pitch / 4.0
    else:
        apex_width = pitch / 4.0
        root_width = 7.0 * pitch / 8.0

    # Small adjustment to root radius to avoid coincident surfaces (per cq_warehouse)
    adjusted_root = root_radius - 0.001 if external else root_radius + 0.001

    try:
        # Create helical wires at apex radius (thread crests)
        # Z-offset: i * apex_width where i = [-0.5, 0.5] centers the width
        apex_helix_wires = []
        for i in [-0.5, 0.5]:
            helix = cq.Wire.makeHelix(
                pitch=pitch,
                height=length,
                radius=apex_radius,
                angle=taper_angle,
                lefthand=False,
            )
            helix = helix.translate((0, 0, i * apex_width))
            apex_helix_wires.append(helix)

        # Create helical wires at root radius (thread valleys)
        # Z-offset: i * root_width where i = [-0.5, 0.5]
        root_helix_wires = []
        for i in [-0.5, 0.5]:
            helix = cq.Wire.makeHelix(
                pitch=pitch,
                height=length,
                radius=adjusted_root,
                angle=taper_angle,
                lefthand=False,
            )
            helix = helix.translate((0, 0, i * root_width))
            root_helix_wires.append(helix)

        # Create ruled surfaces between the 4 wires (forming the thread profile)
        # Order: apex[0] -> apex[1] -> root[1] -> root[0] -> apex[0]
        thread_faces = [
            cq.Face.makeRuledSurface(apex_helix_wires[0], apex_helix_wires[1]),  # Apex surface
            cq.Face.makeRuledSurface(apex_helix_wires[1], root_helix_wires[1]),  # Right flank
            cq.Face.makeRuledSurface(root_helix_wires[1], root_helix_wires[0]),  # Root surface
            cq.Face.makeRuledSurface(root_helix_wires[0], apex_helix_wires[0]),  # Left flank
        ]

        # Create end caps using positionAt (per cq_warehouse)
        end_faces = []
        for pos in [0, 1]:  # 0 = start, 1 = end
            cap_points = [
                apex_helix_wires[0].positionAt(pos),
                apex_helix_wires[1].positionAt(pos),
                root_helix_wires[1].positionAt(pos),
                root_helix_wires[0].positionAt(pos),
                apex_helix_wires[0].positionAt(pos),  # Close the polygon
            ]
            cap_wire = cq.Wire.makePolygon(cap_points)
            cap_face = cq.Face.makeFromWires(cap_wire)
            end_faces.append(cap_face)

        # Create shell from all faces and convert to solid
        all_faces = thread_faces + end_faces
        thread_shell = cq.Shell.makeShell(all_faces)
        thread_solid = cq.Solid.makeSolid(thread_shell)

        if not thread_solid.isValid():
            raise ValueError("Thread solid is not valid")

        return thread_solid

    except Exception:
        # Fallback: simplified thread as tapered annulus
        if external:
            r_apex_end = apex_radius + length * math.tan(math.radians(taper_angle)) if taper_angle != 360 else apex_radius
            r_root_end = root_radius + length * math.tan(math.radians(taper_angle)) if taper_angle != 360 else root_radius
        else:
            r_apex_end = apex_radius + length * math.tan(math.radians(taper_angle)) if taper_angle != 360 else apex_radius
            r_root_end = root_radius + length * math.tan(math.radians(taper_angle)) if taper_angle != 360 else root_radius

        outer = _make_tapered_cylinder(max(apex_radius, root_radius), max(r_apex_end, r_root_end), length)
        inner = _make_tapered_cylinder(min(apex_radius, root_radius), min(r_apex_end, r_root_end), length)
        return outer.cut(inner)


def make_npt_external_thread_grooves(
    nps: str,
    thread_length: float | None = None,
) -> cq.Shape:
    """Create external NPT thread geometry for CUTTING from a pipe.

    IMPORTANT: This is a WORKAROUND function. The proper way to add external
    threads to a pipe is to create the thread solid and FUSE it, not cut.

    For external threads, the thread ridges project OUTWARD from the pipe body.
    The pipe OD becomes the thread root, and the crests are at a larger radius.

    For now, this returns a tapered annular volume that represents the thread
    envelope (minor to major diameter). This should be CUT from a plain pipe
    to create a stepped appearance, but proper helical threads require FUSING
    the thread solid onto the pipe.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "1", "2")
        thread_length: Length of thread in mm. Defaults to L2 (effective thread length).

    Returns:
        CadQuery Shape representing thread envelope for cutting
    """
    spec = get_npt_spec(nps)

    if thread_length is None:
        thread_length = spec.L2

    thread_depth = spec.thread_depth

    # For external threads going INTO the pipe body:
    # Z=0 is pipe end (LARGEST diameter), Z=thread_length is smaller
    # At Z=0: major = OD, minor = OD - thread_depth
    # Taper decreases as we go into the pipe

    r_major_start = spec.od / 2.0
    r_minor_start = r_major_start - thread_depth

    taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
    r_major_end = r_major_start - taper_delta
    r_minor_end = r_minor_start - taper_delta

    # Return envelope (annular taper) - this is what gets removed in threading
    # Actually for external threads we don't cut - we add material
    # But for hollow pipes, we can represent the threaded section by having
    # the OD at major diameter with grooves cut in

    outer = _make_tapered_cylinder(r_major_start, r_major_end, thread_length)
    inner = _make_tapered_cylinder(r_minor_start, r_minor_end, thread_length)
    return outer.cut(inner)


def make_npt_external_thread(
    nps: str,
    thread_length: float | None = None,
    simple: bool = False,
    pipe_id: float | None = None,
) -> cq.Shape:
    """Create external NPT thread geometry for a pipe end.

    The thread is oriented with:
    - Axis along +Z
    - Z=0 is the pipe end (free end) - LARGEST diameter
    - Thread extends in +Z direction (into the pipe body)
    - Diameter DECREASES as Z increases (NPT taper going into pipe)

    For external threads on a pipe:
    - Thread crests (apex) are at the pipe OD
    - Thread roots (valleys) are cut into the pipe wall
    - The result is a HOLLOW threaded section (if pipe_id is provided)

    Args:
        nps: Nominal pipe size (e.g., "1/2", "1", "2")
        thread_length: Length of thread in mm. Defaults to L2 (effective thread length).
        simple: If True, return simplified representation (tapered cylinder envelope)
        pipe_id: Inner diameter of pipe (for hollow threaded section). If None, solid.

    Returns:
        CadQuery Shape of the threaded section
    """
    spec = get_npt_spec(nps)

    if thread_length is None:
        thread_length = spec.L2

    thread_depth = spec.thread_depth
    pitch = spec.pitch_mm

    # For external threads:
    # - Apex (crest) radius = pipe OD / 2 (crests at pipe surface)
    # - Root radius = apex - thread_depth (valleys cut into pipe wall)
    # - NPT taper: diameter DECREASES as we go INTO the pipe (+Z direction)

    # At Z=0 (pipe free end - LARGEST diameter)
    apex_radius_start = spec.od / 2.0
    root_radius_start = apex_radius_start - thread_depth

    # At Z=thread_length (smaller due to NPT taper)
    taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
    apex_radius_end = apex_radius_start - taper_delta
    root_radius_end = root_radius_start - taper_delta

    # Bore radius (default to solid if not specified)
    bore_radius = pipe_id / 2.0 if pipe_id is not None else 0.0

    if simple:
        # Simplified: tapered annular section (thread envelope)
        outer_cone = _make_tapered_cylinder(apex_radius_start, apex_radius_end, thread_length)
        if bore_radius > 0:
            bore = cq.Solid.makeCylinder(bore_radius, thread_length)
            return outer_cone.cut(bore)
        return outer_cone

    # Full thread geometry using ruled surface approach
    # The thread solid from _make_thread_solid is just the teeth (root to apex)
    # We also need a "core" cylinder from bore to root level
    #
    # CANONICAL ORIENTATION (caller must flip for specific end):
    # - Z=0: SMALL end (thread diameter is smallest here)
    # - Z=thread_length: LARGE end (thread diameter is largest here)
    # This matches the natural direction of positive taper (expanding with +Z)

    try:
        # Create thread teeth starting at SMALL end with POSITIVE taper
        thread_teeth = _make_thread_solid(
            apex_radius=apex_radius_end,   # Start at small end
            root_radius=root_radius_end,
            pitch=pitch,
            length=thread_length,
            taper_angle=NPT_HALF_ANGLE_DEG,  # POSITIVE = expanding as Z increases
            external=True,
        )

        # Create core cylinder from bore to root (tapered, small to large)
        if bore_radius > 0:
            core = _make_tapered_cylinder(root_radius_end, root_radius_start, thread_length)
            bore_cyl = cq.Solid.makeCylinder(bore_radius, thread_length)
            core = core.cut(bore_cyl)
            # Fuse thread teeth onto hollow core
            threaded_section = core.fuse(thread_teeth)
        else:
            # Solid thread (no bore) - just the teeth on a solid core
            core = _make_tapered_cylinder(root_radius_end, root_radius_start, thread_length)
            threaded_section = core.fuse(thread_teeth)

        # NO FLIP HERE - return in canonical orientation
        # Caller (make_threaded_pipe) will flip as needed for each end:
        # - Inlet: flip so LARGE end is at Z=0 (free end)
        # - Outlet: no flip needed, position so LARGE end is at Z=length

        if threaded_section.isValid():
            return threaded_section
        else:
            raise ValueError("Thread creation failed")

    except Exception as e:
        # Fallback to simplified geometry
        print(f"Thread creation failed: {e}, using simplified geometry")
        outer_cone = _make_tapered_cylinder(apex_radius_start, apex_radius_end, thread_length)
        if bore_radius > 0:
            bore = cq.Solid.makeCylinder(bore_radius, thread_length)
            return outer_cone.cut(bore)
        return outer_cone


def make_npt_internal_thread(
    nps: str,
    thread_length: float | None = None,
    simple: bool = False,
) -> cq.Shape:
    """Create internal NPT thread geometry for a fitting bore.

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
    The bore = base bore at apex level + thread grooves to root level.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "1", "2")
        thread_length: Length of thread in mm. Defaults to L2 (effective thread length).
        simple: If True, return simplified representation (tapered cylinder bore)

    Returns:
        CadQuery Shape representing the material to CUT from the fitting
    """
    spec = get_npt_spec(nps)

    if thread_length is None:
        thread_length = spec.L2

    pitch = spec.pitch_mm
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

    # Full thread geometry:
    # Internal threads form grooves that extend from apex (crest) to root (valley)
    # The bore volume = base bore (at apex level) + thread grooves (apex to root)

    try:
        # Create thread teeth (the grooves from apex to root)
        # For internal threads, the taper naturally expands with +Z (positive angle)
        thread_teeth = _make_thread_solid(
            apex_radius=apex_radius_start,
            root_radius=root_radius_start,
            pitch=pitch,
            length=thread_length,
            taper_angle=NPT_HALF_ANGLE_DEG,  # Positive = expanding (correct for internal)
            external=False,  # Internal thread
        )

        # Create base bore cylinder at apex level (the main bore opening)
        base_bore = _make_tapered_cylinder(apex_radius_start, apex_radius_end, thread_length)

        # Fuse base bore with thread teeth to get complete bore volume
        threaded_bore = base_bore.fuse(thread_teeth)

        if threaded_bore.isValid():
            return threaded_bore
        else:
            raise ValueError("Internal thread creation failed")

    except Exception as e:
        # Fallback to simplified geometry
        print(f"Internal thread creation failed: {e}, using simplified geometry")
        return _make_tapered_cylinder(root_radius_start, root_radius_end, thread_length)


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
