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
# THREAD GEOMETRY GENERATION - RULED SURFACE APPROACH (based on cq_warehouse)
# =============================================================================


class ThreadBuilder:
    """Builder class for creating NPT thread geometry following cq_warehouse approach exactly.

    This class encapsulates all the thread generation logic including:
    - Main helix sections using cq.Wire.makeHelix()
    - Fade sections using parametric curves
    - End caps and connecting faces
    - All end_finishes: raw, fade, square, chamfer
    """

    def __init__(
        self,
        apex_radius: float,
        root_radius: float,
        pitch: float,
        length: float,
        taper_angle: float = 0.0,
        external: bool = True,
        end_finishes: tuple[str, str] = ("fade", "fade"),
        right_hand: bool = True,
        blend_radius: float | None = None,
    ):
        """Initialize thread builder.

        Args:
            apex_radius: Radius at thread crest (tips)
            root_radius: Radius at thread root (valleys)
            pitch: Thread pitch (mm)
            length: Thread length (mm)
            taper_angle: Helix taper half-angle in degrees
            external: True for external threads
            end_finishes: Tuple of (start_finish, end_finish) - "fade", "pipe_junction", "raw", "square"
            right_hand: True for right-hand threads
            blend_radius: Target radius for "pipe_junction" fade (typically pipe OD)
        """
        self.apex_radius = apex_radius
        self.root_radius = root_radius
        self.pitch = pitch
        self.length = length
        self.taper = taper_angle  # cq_warehouse calls it "taper"
        self.external = external
        self.end_finishes = end_finishes
        self.right_hand = right_hand
        self.blend_radius = blend_radius

        # Thread profile widths (helical wire offsets for ruled surface geometry)
        # These define the Z-offset between paired helical wires that form
        # the crest and root surfaces.
        #
        # For NPT threads per ASME B1.20.1:
        # - Thread depth h = 0.8P
        # - Flank angle = 30° (60° included angle)
        # - Flank axial span = h × tan(30°) = 0.8P × 0.577 = 0.462P
        # - Remaining for flats = P - 2×0.462P = 0.076P
        # - Symmetric truncation: apex_flat = root_flat = 0.038P
        #
        # The apex_width is the crest flat width.
        # The root_width spans from one flank edge across the root to the other flank.
        # root_width = apex_width + 2×flank_span = 0.038P + 0.924P = 0.962P
        thread_depth = 0.8 * pitch
        flank_axial_span = thread_depth * math.tan(math.radians(30))  # 0.462P
        flat_width = pitch - 2 * flank_axial_span  # 0.076P total for both flats

        if external:
            self.apex_width = flat_width / 2  # ~0.038P (crest flat)
            self.root_width = self.apex_width + 2 * flank_axial_span  # ~0.962P
        else:
            # Internal threads: apex is at smaller radius, root at larger
            self.apex_width = flat_width / 2  # ~0.038P
            self.root_width = self.apex_width + 2 * flank_axial_span  # ~0.962P

        # Tooth height
        self.tooth_height = abs(apex_radius - root_radius)

        # Small adjustment to root radius to avoid coincident surfaces
        self.root_radius_adjusted = root_radius - 0.001 if external else root_radius + 0.001

    def fade_helix(
        self,
        t: float,
        apex: bool,
        vertical_displacement: float,
        z_offset: float = 0.0,
    ) -> tuple[float, float, float]:
        """Parametric fade helix function with NPT taper correction.

        Creates a 90-degree arc where the thread height fades from full to zero
        over pitch/4 distance. For NPT threads, the radius also changes during
        this arc according to the taper angle.

        Args:
            t: Parameter from 0 to 1 (0 = full thread height, 1 = faded to nothing)
            apex: True for apex wire, False for root wire
            vertical_displacement: Additional Z displacement for root wires
            z_offset: Starting Z position for radius taper calculation

        Returns:
            (x, y, z) coordinates
        """
        # Z position within this fade section
        z_local = t * self.pitch / 4 + t * vertical_displacement
        z_total = z_offset + z_local

        # Calculate taper-adjusted base radii at this Z position
        # For positive taper angle, radius increases with Z
        taper_delta = z_total * math.tan(math.radians(self.taper)) if self.taper != 0 else 0.0

        apex_radius_at_z = self.apex_radius + taper_delta
        root_radius_at_z = self.root_radius_adjusted + taper_delta
        tooth_height_at_z = abs(apex_radius_at_z - root_radius_at_z)

        # Apply fade interpolation (sine curve from full height to zero)
        fade_factor = math.sin(t * math.pi / 2)

        if self.external:
            radius = apex_radius_at_z - fade_factor * tooth_height_at_z if apex else root_radius_at_z
        else:
            radius = apex_radius_at_z + fade_factor * tooth_height_at_z if apex else root_radius_at_z

        x_pos = radius * math.cos(t * math.pi / 2)
        y_pos = radius * math.sin(t * math.pi / 2)

        return (x_pos, y_pos, z_local)

    def pipe_junction_fade_helix(
        self,
        t: float,
        apex: bool,
        vertical_displacement: float,
        z_offset: float = 0.0,
    ) -> tuple[float, float, float]:
        """Parametric fade for pipe junction where both apex and root converge to blend_radius.

        Unlike regular fade_helix (where apex fades to root level), this fade:
        - Apex fades DOWN from apex_radius to blend_radius (pipe OD)
        - Root EXPANDS UP from root_radius to blend_radius (pipe OD)
        - Both converge at blend_radius, eliminating the step at pipe junction

        Args:
            t: Parameter from 0 to 1 (0 = full thread, 1 = blended to pipe OD)
            apex: True for apex wire, False for root wire
            vertical_displacement: Additional Z displacement for root wires
            z_offset: Starting Z position for radius taper calculation

        Returns:
            (x, y, z) coordinates
        """
        if self.blend_radius is None:
            raise ValueError("blend_radius must be set for pipe_junction fade")

        # Z position within this fade section
        z_local = t * self.pitch / 4 + t * vertical_displacement
        z_total = z_offset + z_local

        # Calculate taper-adjusted radii at this Z position
        taper_delta = z_total * math.tan(math.radians(self.taper)) if self.taper != 0 else 0.0

        apex_radius_at_z = self.apex_radius + taper_delta
        root_radius_at_z = self.root_radius_adjusted + taper_delta
        blend_radius_at_z = self.blend_radius + taper_delta

        # Apply fade interpolation (sine curve)
        fade_factor = math.sin(t * math.pi / 2)

        if apex:
            # Apex fades from apex_radius toward blend_radius
            if self.external:
                # External: apex is larger, blend is even larger (pipe OD)
                # apex → blend (upward for external threads)
                radius = apex_radius_at_z + fade_factor * (blend_radius_at_z - apex_radius_at_z)
            else:
                # Internal: apex is smaller, blend is smaller (pipe ID)
                radius = apex_radius_at_z - fade_factor * (apex_radius_at_z - blend_radius_at_z)
        else:
            # Root expands from root_radius toward blend_radius
            if self.external:
                # External: root is smaller than blend (pipe OD)
                radius = root_radius_at_z + fade_factor * (blend_radius_at_z - root_radius_at_z)
            else:
                # Internal: root is larger, blend is smaller
                radius = root_radius_at_z - fade_factor * (root_radius_at_z - blend_radius_at_z)

        x_pos = radius * math.cos(t * math.pi / 2)
        y_pos = radius * math.sin(t * math.pi / 2)

        return (x_pos, y_pos, z_local)

    def make_thread_faces(
        self,
        length: float,
        fade_helix: bool = False,
    ) -> tuple[list[cq.Face], list[cq.Face]]:
        """Create thread faces - following cq_warehouse approach exactly.

        Args:
            length: Length of thread
            fade_helix: If True, use parametric fade curves instead of helixes

        Returns:
            Tuple of (thread_faces, end_faces)
        """
        local_apex_offset = 0  # Can be used for alignment

        if fade_helix:
            # Use parametric curves for fade
            apex_helix_wires = []
            root_helix_wires = []

            for i in [-0.5, 0.5]:
                # Apex wire - no vertical displacement
                i_val = i  # Capture for closure

                def make_apex(t: float, i_=i_val) -> tuple[float, float, float]:
                    return self.fade_helix(t, apex=True, vertical_displacement=0)

                apex_result = cq.Workplane("XY").parametricCurve(make_apex, N=20).val()
                if isinstance(apex_result, cq.Edge):
                    apex_result = cq.Wire.assembleEdges([apex_result])
                apex_wire = apex_result.translate(cq.Vector(0, 0, i_val * self.apex_width + local_apex_offset))
                apex_helix_wires.append(apex_wire)

                # Root wire - has vertical displacement per cq_warehouse
                vert_disp = -i_val * (self.root_width - self.apex_width)

                def make_root(t: float, vd=vert_disp) -> tuple[float, float, float]:
                    return self.fade_helix(t, apex=False, vertical_displacement=vd)

                root_result = cq.Workplane("XY").parametricCurve(make_root, N=20).val()
                if isinstance(root_result, cq.Edge):
                    root_result = cq.Wire.assembleEdges([root_result])
                root_wire = root_result.translate(cq.Vector(0, 0, i_val * self.root_width))
                root_helix_wires.append(root_wire)
        else:
            # Use standard makeHelix
            apex_helix_wires = [
                cq.Wire.makeHelix(
                    pitch=self.pitch,
                    height=length,
                    radius=self.apex_radius,
                    angle=self.taper,
                    lefthand=not self.right_hand,
                ).translate(cq.Vector(0, 0, i * self.apex_width + local_apex_offset))
                for i in [-0.5, 0.5]
            ]

            root_helix_wires = [
                cq.Wire.makeHelix(
                    pitch=self.pitch,
                    height=length,
                    radius=self.root_radius_adjusted,
                    angle=self.taper,
                    lefthand=not self.right_hand,
                ).translate(cq.Vector(0, 0, i * self.root_width))
                for i in [-0.5, 0.5]
            ]

        # Create 4 ruled surfaces forming thread profile
        thread_faces = [
            cq.Face.makeRuledSurface(apex_helix_wires[0], apex_helix_wires[1]),
            cq.Face.makeRuledSurface(apex_helix_wires[1], root_helix_wires[1]),
            cq.Face.makeRuledSurface(root_helix_wires[1], root_helix_wires[0]),
            cq.Face.makeRuledSurface(root_helix_wires[0], apex_helix_wires[0]),
        ]

        # End caps - only create at positions where not faded
        # For fade_helix, only cap at position 0 (not 1, since it fades to nothing)
        end_caps = [0] if fade_helix else [0, 1]
        end_cap_wires = [
            cq.Wire.makePolygon(
                [
                    apex_helix_wires[0].positionAt(i),
                    apex_helix_wires[1].positionAt(i),
                    root_helix_wires[1].positionAt(i),
                    root_helix_wires[0].positionAt(i),
                    apex_helix_wires[0].positionAt(i),
                ]
            )
            for i in end_caps
        ]
        end_faces = [cq.Face.makeFromWires(w) for w in end_cap_wires]

        return thread_faces, end_faces

    def make_thread_faces_tapered_fade(
        self,
        z_offset: float,
    ) -> tuple[list[cq.Face], list[cq.Face]]:
        """Create fade section faces with NPT taper correction.

        The fade section is always pitch/4 in length (90-degree arc).

        Args:
            z_offset: Z position where this fade section starts (for taper calculation)

        Returns:
            Tuple of (thread_faces, end_faces)
        """
        apex_helix_wires = []
        root_helix_wires = []

        for i in [-0.5, 0.5]:
            # Capture values for closure
            i_val = i
            z_off = z_offset

            # Apex wire with taper-corrected fade
            def make_apex(t: float, z_=z_off) -> tuple[float, float, float]:
                return self.fade_helix(t, apex=True, vertical_displacement=0, z_offset=z_)

            apex_result = cq.Workplane("XY").parametricCurve(make_apex, N=30).val()
            if isinstance(apex_result, cq.Edge):
                apex_result = cq.Wire.assembleEdges([apex_result])
            apex_wire = apex_result.translate(cq.Vector(0, 0, i_val * self.apex_width))
            apex_helix_wires.append(apex_wire)

            # Root wire with taper-corrected fade
            vert_disp = -i_val * (self.root_width - self.apex_width)

            def make_root(t: float, vd=vert_disp, z_=z_off) -> tuple[float, float, float]:
                return self.fade_helix(t, apex=False, vertical_displacement=vd, z_offset=z_)

            root_result = cq.Workplane("XY").parametricCurve(make_root, N=30).val()
            if isinstance(root_result, cq.Edge):
                root_result = cq.Wire.assembleEdges([root_result])
            root_wire = root_result.translate(cq.Vector(0, 0, i_val * self.root_width))
            root_helix_wires.append(root_wire)

        # Create 4 ruled surfaces forming thread profile
        thread_faces = [
            cq.Face.makeRuledSurface(apex_helix_wires[0], apex_helix_wires[1]),
            cq.Face.makeRuledSurface(apex_helix_wires[1], root_helix_wires[1]),
            cq.Face.makeRuledSurface(root_helix_wires[1], root_helix_wires[0]),
            cq.Face.makeRuledSurface(root_helix_wires[0], apex_helix_wires[0]),
        ]

        # End cap at position 0 only (position 1 fades to nothing)
        end_cap_wire = cq.Wire.makePolygon(
            [
                apex_helix_wires[0].positionAt(0),
                apex_helix_wires[1].positionAt(0),
                root_helix_wires[1].positionAt(0),
                root_helix_wires[0].positionAt(0),
                apex_helix_wires[0].positionAt(0),
            ]
        )
        end_faces = [cq.Face.makeFromWires(end_cap_wire)]

        return thread_faces, end_faces

    def make_thread_with_faded_ends(self) -> cq.Shape:
        """Build thread with proper fade runout at ends.

        Following cq_warehouse approach EXACTLY:
        - Create cylindrical thread faces at origin, then translate
        - Create fade faces with fade_helix=True, then transform
        - For 2 faded ends: 4 + 4 + 4 = 12 faces, NO end caps
        - For 1 faded end: 4 + 4 + 1 end cap = 9 faces

        Returns:
            Thread solid as CadQuery Shape
        """
        start_finish, end_finish = self.end_finishes
        fade_start = start_finish == "fade"
        fade_end = end_finish == "fade"
        pipe_junction_end = end_finish == "pipe_junction"
        # Count faded ends (both "fade" and "pipe_junction" count)
        number_faded_ends = int(fade_start) + int(fade_end or pipe_junction_end)

        # Calculate cylindrical section length per cq_warehouse formula
        cylindrical_length = self.length + self.pitch * (1 - number_faded_ends)

        if cylindrical_length < 0:
            raise ValueError(f"Thread length {self.length} too short for fade sections")

        # Displacement positions cylindrical section correctly
        cylindrical_displacement = self.pitch / 2 if fade_start else -self.pitch / 2

        # === 1. Create cylindrical thread faces at origin ===
        cyl_apex_wires = [
            cq.Wire.makeHelix(
                pitch=self.pitch,
                height=cylindrical_length,
                radius=self.apex_radius,
                angle=self.taper,
                lefthand=not self.right_hand,
            ).translate(cq.Vector(0, 0, i * self.apex_width))
            for i in [-0.5, 0.5]
        ]

        cyl_root_wires = [
            cq.Wire.makeHelix(
                pitch=self.pitch,
                height=cylindrical_length,
                radius=self.root_radius_adjusted,
                angle=self.taper,
                lefthand=not self.right_hand,
            ).translate(cq.Vector(0, 0, i * self.root_width))
            for i in [-0.5, 0.5]
        ]

        # Create 4 thread faces from the wires
        thread_faces = [
            cq.Face.makeRuledSurface(cyl_apex_wires[0], cyl_apex_wires[1]),
            cq.Face.makeRuledSurface(cyl_apex_wires[1], cyl_root_wires[1]),
            cq.Face.makeRuledSurface(cyl_root_wires[1], cyl_root_wires[0]),
            cq.Face.makeRuledSurface(cyl_root_wires[0], cyl_apex_wires[0]),
        ]

        # Create end cap faces (only used when that end is NOT faded)
        end_cap_wires = [
            cq.Wire.makePolygon([
                cyl_apex_wires[0].positionAt(i),
                cyl_apex_wires[1].positionAt(i),
                cyl_root_wires[1].positionAt(i),
                cyl_root_wires[0].positionAt(i),
                cyl_apex_wires[0].positionAt(i),
            ])
            for i in [0, 1]
        ]
        end_faces = [cq.Face.makeFromWires(w) for w in end_cap_wires]

        # Translate thread faces by cylindrical_displacement
        thread_faces = [f.translate(cq.Vector(0, 0, cylindrical_displacement)) for f in thread_faces]
        end_faces = [f.translate(cq.Vector(0, 0, cylindrical_displacement)) for f in end_faces]

        # Calculate rotation angle for top fade
        cylindrical_angle = (360 if self.right_hand else -360) * cylindrical_length / self.pitch

        # === 2. Create fade faces using fade_helix=True ===
        # Use the SIMPLIFIED fade_helix (no taper correction during fade)
        def fade_helix_simple(t: float, apex: bool, vertical_displacement: float) -> tuple[float, float, float]:
            """Simplified fade_helix per cq_warehouse (no taper during fade)."""
            if self.external:
                radius = self.apex_radius - math.sin(t * math.pi / 2) * self.tooth_height if apex else self.root_radius
            else:
                radius = self.apex_radius + math.sin(t * math.pi / 2) * self.tooth_height if apex else self.root_radius

            z_pos = t * self.pitch / 4 + t * vertical_displacement
            x_pos = radius * math.cos(t * math.pi / 2)
            y_pos = radius * math.sin(t * math.pi / 2)
            return (x_pos, y_pos, z_pos)

        def pipe_junction_fade_simple(t: float, apex: bool, vertical_displacement: float) -> tuple[float, float, float]:
            """Fade where both apex and root converge to blend_radius (pipe OD)."""
            if self.blend_radius is None:
                raise ValueError("blend_radius must be set for pipe_junction fade")

            fade_factor = math.sin(t * math.pi / 2)
            if apex:
                # Apex transitions to blend_radius
                if self.external:
                    radius = self.apex_radius + fade_factor * (self.blend_radius - self.apex_radius)
                else:
                    radius = self.apex_radius - fade_factor * (self.apex_radius - self.blend_radius)
            else:
                # Root transitions to blend_radius
                if self.external:
                    radius = self.root_radius + fade_factor * (self.blend_radius - self.root_radius)
                else:
                    radius = self.root_radius - fade_factor * (self.root_radius - self.blend_radius)

            z_pos = t * self.pitch / 4 + t * vertical_displacement
            x_pos = radius * math.cos(t * math.pi / 2)
            y_pos = radius * math.sin(t * math.pi / 2)
            return (x_pos, y_pos, z_pos)

        # Create fade wires (same for both ends, then transformed differently)
        fade_apex_wires: list[cq.Wire] = []
        for i in [-0.5, 0.5]:
            i_val = i  # Capture for closure

            def make_apex_curve(t: float, _i: float = i_val) -> tuple[float, float, float]:
                return fade_helix_simple(t, apex=True, vertical_displacement=0)

            edge = cq.Workplane("XY").parametricCurve(make_apex_curve, N=20).val()
            wire = cq.Wire.assembleEdges([edge]) if isinstance(edge, cq.Edge) else edge
            wire = wire.translate(cq.Vector(0, 0, i_val * self.apex_width))
            fade_apex_wires.append(wire)

        fade_root_wires: list[cq.Wire] = []
        for i in [-0.5, 0.5]:
            i_val = i  # Capture for closure
            vert_disp = -i_val * (self.root_width - self.apex_width)

            def make_root_curve(t: float, vd: float = vert_disp) -> tuple[float, float, float]:
                return fade_helix_simple(t, apex=False, vertical_displacement=vd)

            edge = cq.Workplane("XY").parametricCurve(make_root_curve, N=20).val()
            wire = cq.Wire.assembleEdges([edge]) if isinstance(edge, cq.Edge) else edge
            wire = wire.translate(cq.Vector(0, 0, i_val * self.root_width))
            fade_root_wires.append(wire)

        # Create 4 fade faces from the wires
        fade_faces = [
            cq.Face.makeRuledSurface(fade_apex_wires[0], fade_apex_wires[1]),
            cq.Face.makeRuledSurface(fade_apex_wires[1], fade_root_wires[1]),
            cq.Face.makeRuledSurface(fade_root_wires[1], fade_root_wires[0]),
            cq.Face.makeRuledSurface(fade_root_wires[0], fade_apex_wires[0]),
        ]

        # For left-hand threads, mirror the fade faces
        if not self.right_hand:
            fade_faces = [f.mirror("XZ") for f in fade_faces]

        # === 3. Transform fade faces and collect all faces ===
        all_faces = list(thread_faces)  # Start with cylindrical faces

        if fade_start:
            # Bottom fade: mirror(XZ).mirror(XY).translate(pitch/2)
            fade_faces_bottom = [
                f.mirror("XZ").mirror("XY").translate(cq.Vector(0, 0, self.pitch / 2))
                for f in fade_faces
            ]
            all_faces.extend(fade_faces_bottom)

        if fade_end:
            # Top fade: translate then rotate
            top_z = cylindrical_length + cylindrical_displacement
            fade_faces_top = [
                f.translate(cq.Vector(0, 0, top_z)).rotate(
                    cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), cylindrical_angle
                )
                for f in fade_faces
            ]
            all_faces.extend(fade_faces_top)

        if pipe_junction_end:
            # Pipe junction fade: thread blends to pipe OD
            # Create separate fade wires using pipe_junction_fade_simple
            pj_apex_wires: list[cq.Wire] = []
            for i in [-0.5, 0.5]:
                i_val = i

                def make_pj_apex(t: float, _i: float = i_val) -> tuple[float, float, float]:
                    return pipe_junction_fade_simple(t, apex=True, vertical_displacement=0)

                edge = cq.Workplane("XY").parametricCurve(make_pj_apex, N=20).val()
                wire = cq.Wire.assembleEdges([edge]) if isinstance(edge, cq.Edge) else edge
                wire = wire.translate(cq.Vector(0, 0, i_val * self.apex_width))
                pj_apex_wires.append(wire)

            pj_root_wires: list[cq.Wire] = []
            for i in [-0.5, 0.5]:
                i_val = i
                vert_disp = -i_val * (self.root_width - self.apex_width)

                def make_pj_root(t: float, vd: float = vert_disp) -> tuple[float, float, float]:
                    return pipe_junction_fade_simple(t, apex=False, vertical_displacement=vd)

                edge = cq.Workplane("XY").parametricCurve(make_pj_root, N=20).val()
                wire = cq.Wire.assembleEdges([edge]) if isinstance(edge, cq.Edge) else edge
                wire = wire.translate(cq.Vector(0, 0, i_val * self.root_width))
                pj_root_wires.append(wire)

            # Create 4 pipe junction fade faces
            pj_fade_faces = [
                cq.Face.makeRuledSurface(pj_apex_wires[0], pj_apex_wires[1]),
                cq.Face.makeRuledSurface(pj_apex_wires[1], pj_root_wires[1]),
                cq.Face.makeRuledSurface(pj_root_wires[1], pj_root_wires[0]),
                cq.Face.makeRuledSurface(pj_root_wires[0], pj_apex_wires[0]),
            ]

            if not self.right_hand:
                pj_fade_faces = [f.mirror("XZ") for f in pj_fade_faces]

            # Transform: translate then rotate (same as fade_end but different faces)
            top_z = cylindrical_length + cylindrical_displacement
            pj_faces_top = [
                f.translate(cq.Vector(0, 0, top_z)).rotate(
                    cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), cylindrical_angle
                )
                for f in pj_fade_faces
            ]
            all_faces.extend(pj_faces_top)

        # Add end caps only for NON-faded ends
        if number_faded_ends == 2:
            pass  # No end caps when both ends faded
        elif fade_start:
            all_faces.append(end_faces[1])  # Top end cap only
        elif not (fade_end or pipe_junction_end):
            all_faces.append(end_faces[0])  # Bottom end cap only

        # === 4. Create shell and solid using BRepBuilderAPI_Sewing ===
        # Faces from different sections may not share exact edges, so we need
        # to sew them together with a tolerance
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
        from OCP.TopoDS import TopoDS
        from OCP.TopAbs import TopAbs_SHELL, TopAbs_COMPOUND
        from OCP.TopExp import TopExp_Explorer
        from OCP.ShapeFix import ShapeFix_Shell, ShapeFix_Solid

        sew = BRepBuilderAPI_Sewing(0.5)  # 0.5mm tolerance for sewing (faces don't share edges)
        for face in all_faces:
            if hasattr(face, "wrapped"):
                sew.Add(face.wrapped)
        sew.Perform()
        sewn_shape = sew.SewedShape()

        # Extract shell from sewn result
        shell_ocp = None
        shape_type = sewn_shape.ShapeType()

        if shape_type == TopAbs_SHELL:
            shell_ocp = TopoDS.Shell_s(sewn_shape)
        elif shape_type == TopAbs_COMPOUND:
            # Find first shell in compound
            explorer = TopExp_Explorer(sewn_shape, TopAbs_SHELL)
            if explorer.More():
                shell_ocp = TopoDS.Shell_s(explorer.Current())

        if shell_ocp is not None:
            # Fix the shell
            fix_shell = ShapeFix_Shell(shell_ocp)
            fix_shell.Perform()
            fixed_shell = fix_shell.Shell()

            # Make solid from shell
            try:
                fix_solid = ShapeFix_Solid()
                solid_ocp = fix_solid.SolidFromShell(fixed_shell)
                thread_solid = cq.Solid(solid_ocp)

                if thread_solid.isValid() and thread_solid.Volume() > 1.0:
                    return thread_solid
            except Exception:
                pass

            # Try BRepBuilderAPI_MakeSolid as fallback
            try:
                builder = BRepBuilderAPI_MakeSolid(fixed_shell)
                if builder.IsDone():
                    thread_solid = cq.Solid(builder.Solid())
                    if thread_solid.isValid() and thread_solid.Volume() > 1.0:
                        return thread_solid
            except Exception:
                pass

        # Fallback: raw thread without fade
        print("Warning: fade thread creation failed, falling back to raw thread")
        thread_faces_raw, end_faces_raw = self.make_thread_faces(self.length, fade_helix=False)
        all_raw_faces = thread_faces_raw + end_faces_raw
        try:
            thread_shell = cq.Shell.makeShell(all_raw_faces)
            thread_solid = cq.Solid.makeSolid(thread_shell)
            if thread_solid.isValid():
                return thread_solid
        except Exception:
            pass

        raise ValueError("Failed to create thread: faces did not form a valid shell")

    def build(self) -> cq.Shape:
        """Build the complete thread solid.

        Uses the new faded ends approach when fade is requested, which creates
        proper thread runout geometry. Falls back to raw thread with post-processing
        for other end finishes (raw, square, chamfer).

        Returns:
            Thread solid as CadQuery Shape
        """
        start_finish, end_finish = self.end_finishes

        # Use new faded ends approach when fade is requested
        if "fade" in self.end_finishes:
            return self.make_thread_with_faded_ends()

        # For non-fade finishes, use original approach with raw thread
        thread_faces, end_faces = self.make_thread_faces(self.length, fade_helix=False)
        all_faces = thread_faces + end_faces

        # Create shell and solid using makeShell (simpler and more reliable)
        try:
            thread_shell = cq.Shell.makeShell(all_faces)
            thread_solid = cq.Solid.makeSolid(thread_shell)

            if not thread_solid.isValid():
                raise ValueError("Shell.makeShell produced invalid solid")

        except Exception:
            # Fallback: use BRepBuilderAPI_Sewing for edge connectivity issues
            from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
            from OCP.TopAbs import TopAbs_COMPOUND, TopAbs_SHELL, TopAbs_SOLID
            from OCP.TopExp import TopExp_Explorer

            sew = BRepBuilderAPI_Sewing(1e-3)
            for face in all_faces:
                sew.Add(face.wrapped)
            sew.Perform()

            sewn_shape = sew.SewedShape()
            shape_type = sewn_shape.ShapeType()

            if shape_type == TopAbs_SHELL:
                thread_shell = cq.Shell(sewn_shape)
            elif shape_type == TopAbs_SOLID:
                thread_solid = cq.Solid(sewn_shape)
                if not thread_solid.isValid():
                    raise ValueError("Thread solid is not valid") from None
                thread_solid = self._apply_end_finishes(thread_solid, start_finish, end_finish)
                return thread_solid
            elif shape_type == TopAbs_COMPOUND:
                explorer = TopExp_Explorer(sewn_shape, TopAbs_SHELL)
                if explorer.More():
                    thread_shell = cq.Shell(explorer.Current())
                else:
                    thread_shell = cq.Shell.makeShell([f for f in all_faces if isinstance(f, cq.Face)])
            else:
                thread_shell = cq.Shell.makeShell([f for f in all_faces if isinstance(f, cq.Face)])

            thread_solid = cq.Solid.makeSolid(thread_shell)

            if not thread_solid.isValid():
                raise ValueError("Thread solid is not valid") from None

        # Apply end finishes as post-processing (square, chamfer)
        thread_solid = self._apply_end_finishes(thread_solid, start_finish, end_finish)

        return thread_solid

    def _apply_end_finishes(self, thread_solid: cq.Shape, start_finish: str, end_finish: str) -> cq.Shape:
        """Apply end finishes (square, chamfer, fade) by trimming the raw thread."""
        result = thread_solid

        # Apply square finish (cuts at Z=0 and/or Z=length)
        if start_finish == "square" or end_finish == "square":
            result = self._apply_square_finish_internal(result, start_finish, end_finish)

        # Apply chamfer finish
        if start_finish == "chamfer" or end_finish == "chamfer":
            result = self._apply_chamfer_finish(result)

        # Apply fade finish (intersect with tapered profile)
        if start_finish == "fade" or end_finish == "fade":
            result = self._apply_fade_finish(result, start_finish, end_finish)

        return result

    def _apply_fade_finish(self, thread_solid: cq.Shape, start_finish: str, end_finish: str) -> cq.Shape:
        """Apply fade finish.

        Note: True geometric fade (gradual thread runout) requires complex geometry
        that creates edge connectivity issues. For now, fade is treated as raw
        (the helical threads naturally terminate at the boundaries).

        The raw helix already provides reasonable thread termination - the threads
        start and end at Z=0 and Z=length respectively.
        """
        # Return thread as-is - the raw helix provides adequate termination
        # Cutting operations create Compounds which breaks the geometry
        return thread_solid

    def _apply_square_finish_internal(self, thread_solid: cq.Shape, start_finish: str, end_finish: str) -> cq.Shape:
        """Apply square finish by cutting off thread at boundaries."""
        taper_tan = math.tan(math.radians(self.taper)) if self.taper != 0 else 0.0
        max_radius = max(self.apex_radius, self.root_radius) + self.length * taper_tan
        box_size = 4 * max_radius
        half_box = box_size / 2

        result = thread_solid

        if start_finish == "square":
            cutter = cq.Solid.makeBox(box_size, box_size, self.length, cq.Vector(-half_box, -half_box, -self.length))
            result = result.cut(cutter)

        if end_finish == "square":
            cutter = cq.Solid.makeBox(box_size, box_size, self.length, cq.Vector(-half_box, -half_box, self.length))
            result = result.cut(cutter)

        return result

    def _apply_square_finish(self, thread_solid: cq.Shape) -> cq.Shape:
        """Apply square finish by cutting off thread at boundaries."""
        if "square" not in self.end_finishes:
            return thread_solid

        taper_tan = math.tan(math.radians(self.taper)) if self.taper != 0 else 0.0
        max_radius = max(self.apex_radius, self.root_radius) + self.length * taper_tan
        box_size = 4 * max_radius
        half_box = box_size / 2

        start_finish, end_finish = self.end_finishes
        result = thread_solid

        if start_finish == "square":
            cutter = cq.Solid.makeBox(box_size, box_size, self.length, cq.Vector(-half_box, -half_box, -self.length))
            result = result.cut(cutter)

        if end_finish == "square":
            cutter = cq.Solid.makeBox(box_size, box_size, self.length, cq.Vector(-half_box, -half_box, self.length))
            result = result.cut(cutter)

        return result

    def _apply_chamfer_finish(self, thread_solid: cq.Shape) -> cq.Shape:
        """Apply chamfer finish by intersecting with chamfered annular cylinder."""
        if "chamfer" not in self.end_finishes:
            return thread_solid

        start_finish, end_finish = self.end_finishes
        taper_tan = math.tan(math.radians(self.taper)) if self.taper != 0 else 0.0

        apex_end = self.apex_radius + self.length * taper_tan
        root_end = self.root_radius_adjusted + self.length * taper_tan

        if self.external:
            inner_r_start, outer_r_start = self.root_radius_adjusted, self.apex_radius
            inner_r_end, outer_r_end = root_end, apex_end
        else:
            inner_r_start, outer_r_start = self.apex_radius, self.root_radius_adjusted
            inner_r_end, outer_r_end = apex_end, root_end

        try:
            outer_cone = _make_tapered_cylinder(outer_r_start, outer_r_end, self.length)
            inner_cone = _make_tapered_cylinder(inner_r_start, inner_r_end, self.length)
            annular = outer_cone.cut(inner_cone)

            cutter = cq.Workplane("XY").add(annular)

            chamfer_length = self.tooth_height * 0.5
            chamfer_depth = self.tooth_height * 0.75
            edge_selector = 1 if self.external else 0

            if start_finish == "chamfer":
                cutter = cutter.faces("<Z").edges(cq.selectors.RadiusNthSelector(edge_selector)).chamfer(chamfer_length, chamfer_depth)

            if end_finish == "chamfer":
                cutter = cutter.faces(">Z").edges(cq.selectors.RadiusNthSelector(edge_selector)).chamfer(chamfer_length, chamfer_depth)

            cutter_shape: cq.Shape = cutter.val()  # type: ignore[assignment]
            return thread_solid.intersect(cutter_shape)

        except Exception:
            return thread_solid


def _make_thread_solid(
    apex_radius: float,
    root_radius: float,
    pitch: float,
    length: float,
    taper_angle: float = 0.0,
    external: bool = True,
    end_finishes: tuple[str, str] = ("fade", "fade"),
    blend_radius: float | None = None,
) -> cq.Shape:
    """Create thread solid using ruled surfaces between helical wires.

    This follows cq_warehouse's proven approach:
    1. Create 4 helical wires (2 at apex, 2 at root) with Z-offsets for width
    2. Connect wires with ruled surfaces to form thread flanks
    3. Add end caps and fade sections as needed
    4. Create shell and solid

    Args:
        apex_radius: Radius at thread crest (tips) at Z=0
        root_radius: Radius at thread root (valleys) at Z=0
        pitch: Thread pitch (mm)
        length: Thread length (mm)
        taper_angle: Helix taper half-angle in degrees (0 = no taper)
        external: True for external threads (apex > root)
        end_finishes: Tuple of (start_finish, end_finish) where each is:
            - "raw": Unfinished thread extending beyond boundaries
            - "fade": Thread profile fades to zero over 90° arc
            - "pipe_junction": Thread blends to blend_radius (pipe OD)
            - "square": Clean cut at thread boundary
            - "chamfer": Conical chamfer at thread ends
        blend_radius: Target radius for "pipe_junction" fade (typically pipe OD)

    Returns:
        Thread solid as CadQuery Shape
    """
    try:
        builder = ThreadBuilder(
            apex_radius=apex_radius,
            root_radius=root_radius,
            pitch=pitch,
            length=length,
            taper_angle=taper_angle,
            external=external,
            end_finishes=end_finishes,
            blend_radius=blend_radius,
        )
        return builder.build()

    except Exception as e:
        # Fallback: simplified thread as tapered annulus
        import traceback

        traceback.print_exc()
        print(f"Thread creation failed: {e}, using simplified geometry")

        taper_tan = math.tan(math.radians(taper_angle)) if taper_angle != 0 else 0.0
        r_apex_end = apex_radius + length * taper_tan
        r_root_end = root_radius + length * taper_tan

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
    end_finishes: tuple[str, str] = ("fade", "fade"),
) -> cq.Shape:
    """Create external NPT thread geometry for a pipe end.

    The thread is oriented with:
    - Axis along +Z
    - Z=0 is the pipe end (free end) - SMALLEST diameter (canonical orientation)
    - Thread extends in +Z direction (into the pipe body)
    - Diameter INCREASES as Z increases (natural helix taper direction)

    For external threads on a pipe:
    - Thread crests (apex) are at the pipe OD
    - Thread roots (valleys) are cut into the pipe wall
    - The result is a HOLLOW threaded section (if pipe_id is provided)

    Args:
        nps: Nominal pipe size (e.g., "1/2", "1", "2")
        thread_length: Length of thread in mm. Defaults to L2 (effective thread length).
        simple: If True, return simplified representation (tapered cylinder envelope)
        pipe_id: Inner diameter of pipe (for hollow threaded section). If None, solid.
        end_finishes: Tuple of (start_finish, end_finish) where each is:
            - "raw": Unfinished thread extending beyond boundaries (fastest)
            - "fade": Thread profile fades to zero over 90° arc (default, fast)
            - "square": Clean cut at thread boundary (moderate)
            - "chamfer": Conical chamfer at thread ends (slowest)

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
            apex_radius=apex_radius_end,  # Start at small end
            root_radius=root_radius_end,
            pitch=pitch,
            length=thread_length,
            taper_angle=NPT_HALF_ANGLE_DEG,  # POSITIVE = expanding as Z increases
            external=True,
            end_finishes=end_finishes,
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
    end_finishes: tuple[str, str] = ("fade", "fade"),
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
        end_finishes: Tuple of (start_finish, end_finish) where each is:
            - "raw": Unfinished thread
            - "fade": Thread profile fades smoothly (default)
            - "square": Clean cut at thread boundary
            - "chamfer": Conical chamfer at thread ends

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
            end_finishes=end_finishes,
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
