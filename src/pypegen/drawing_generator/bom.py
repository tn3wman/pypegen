"""
Bill of Materials and Balloon Leader System

This module provides:
- ComponentRecord: Tracks component metadata during pipe route building
- BOMEntry: Aggregated entry for the BOM table
- Balloon: Positioned balloon with leader line
- BalloonPlacer: Smart placement algorithm with collision avoidance
- BOMTable: Generates BOM table SVG
"""

import math
from dataclasses import dataclass, field
from typing import Optional

import cadquery as cq
import numpy as np

from .constants import BORDER_COLOR, THIN_LINE_WIDTH
from .view_area import ViewArea

# =============================================================================
# BALLOON STYLING CONSTANTS
# =============================================================================

BALLOON_RADIUS = 4.0           # mm - radius of balloon circle
BALLOON_STROKE_WIDTH = 0.35    # mm - balloon circle stroke
LEADER_STROKE_WIDTH = 0.25     # mm - leader line stroke
LEADER_DOT_RADIUS = 0.8        # mm - dot at component attachment
MIN_LEADER_LENGTH = 8.0        # mm - minimum leader length
MAX_LEADER_LENGTH = 30.0       # mm - maximum leader length
BALLOON_FONT_SIZE = 3.0        # mm - item number font size

BOM_ROW_HEIGHT = 6.0           # mm - height of each BOM row
BOM_HEADER_HEIGHT = 8.0        # mm - height of BOM header row


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def get_bounding_box_corners(
    bb: cq.BoundBox
) -> list[tuple[float, float, float]]:
    """
    Get all 8 corners of a CadQuery BoundingBox.

    Args:
        bb: CadQuery BoundingBox object

    Returns:
        List of 8 corner points as (x, y, z) tuples
    """
    return [
        (bb.xmin, bb.ymin, bb.zmin),
        (bb.xmax, bb.ymin, bb.zmin),
        (bb.xmin, bb.ymax, bb.zmin),
        (bb.xmax, bb.ymax, bb.zmin),
        (bb.xmin, bb.ymin, bb.zmax),
        (bb.xmax, bb.ymin, bb.zmax),
        (bb.xmin, bb.ymax, bb.zmax),
        (bb.xmax, bb.ymax, bb.zmax),
    ]


def mm_to_arch_feet_inches(mm: float | None) -> str:
    """
    Convert millimeters to architectural feet and inches (to nearest 1/16").

    Examples:
        914.4 mm -> 3'-0"
        457.2 mm -> 1'-6"
        304.8 mm -> 1'-0"
        100.0 mm -> 3 15/16"
    """
    if mm is None or mm <= 0:
        return ""

    # Convert mm to inches
    total_inches = mm / 25.4

    # Get feet and remaining inches
    feet = int(total_inches // 12)
    remaining_inches = total_inches % 12

    # Get whole inches and fractional part
    whole_inches = int(remaining_inches)
    fractional = remaining_inches - whole_inches

    # Convert fractional to nearest 1/16"
    sixteenths = round(fractional * 16)

    # Handle rollover (16/16 = 1 inch)
    if sixteenths == 16:
        sixteenths = 0
        whole_inches += 1
        if whole_inches == 12:
            whole_inches = 0
            feet += 1

    # Build fractional string (reduced to lowest terms)
    if sixteenths == 0:
        frac_str = ""
    elif sixteenths == 8:
        frac_str = "1/2"
    elif sixteenths == 4:
        frac_str = "1/4"
    elif sixteenths == 12:
        frac_str = "3/4"
    elif sixteenths % 4 == 0:
        frac_str = f"{sixteenths // 4}/4"
    elif sixteenths % 2 == 0:
        frac_str = f"{sixteenths // 2}/8"
    else:
        frac_str = f"{sixteenths}/16"

    # Format output (space between inches and fraction)
    if feet > 0:
        if frac_str:
            return f"{feet}'-{whole_inches} {frac_str}\""
        else:
            return f"{feet}'-{whole_inches}\""
    else:
        if frac_str:
            if whole_inches > 0:
                return f"{whole_inches} {frac_str}\""
            else:
                return f"{frac_str}\""
        else:
            return f"{whole_inches}\""


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class ComponentRecord:
    """
    Metadata for a placed component in the pipe assembly.

    Created during PipeRoute.build() for each fitting and pipe segment.
    Used for BOM generation and balloon placement.

    Coordinate Systems:
    - world_position_3d: Position in CadQuery 3D world (X=East, Y=North, Z=Up)
    - svg_position_2d: Position in final SVG drawing coordinates (mm on sheet)
    - svg_bounds_2d: Bounding box in SVG coordinates (min_x, min_y, max_x, max_y)

    The SVG positions are computed during drawing generation and stored here
    for use by balloon placement, weld symbols, and other annotations.
    """
    item_number: int                          # BOM item number (1, 2, 3...)
    component_type: str                       # "flange", "elbow", "pipe"
    nps: str                                  # Nominal pipe size ("2", "1-1/2")
    description: str                          # Human-readable ("WN FLANGE", "90° SW ELL")
    schedule_class: str                       # "Sch 40", "Class 3000", "Class 300"
    length_mm: float | None = None         # For pipes only
    shape: cq.Shape | None = None          # The CadQuery geometry
    world_transform: np.ndarray | None = None  # 4x4 matrix for world position

    # Connection directions (for smart balloon placement)
    # These are world directions: "east", "west", "north", "south", "up", "down"
    connected_directions: list[str] = field(default_factory=list)
    pipe_direction: str | None = None      # For pipes: the direction the pipe runs

    # 3D World Position (computed from shape bounding box)
    world_position_3d: tuple[float, float, float] | None = None

    # 2D SVG Position (computed during drawing generation)
    # These are in final SVG coordinates (mm on the drawing sheet)
    svg_position_2d: tuple[float, float] | None = None
    svg_bounds_2d: tuple[float, float, float, float] | None = None  # (min_x, min_y, max_x, max_y)

    # Actual SVG geometry points (extracted from rendered SVG paths)
    # These are the actual edge points, not just bounding box corners
    svg_geometry_points: list[tuple[float, float]] = field(default_factory=list)

    @property
    def center_3d(self) -> tuple[float, float, float]:
        """Get the 3D centroid of the component's bounding box."""
        if self.world_position_3d is not None:
            return self.world_position_3d
        if self.shape is None:
            return (0.0, 0.0, 0.0)
        bb = self.shape.BoundingBox()
        return (
            (bb.xmin + bb.xmax) / 2,
            (bb.ymin + bb.ymax) / 2,
            (bb.zmin + bb.zmax) / 2
        )

    def set_world_position_from_shape(self):
        """Compute and store the 3D world position from the shape's bounding box.

        Applies world_transform to get the correct world position.
        The shape is stored in local coordinates, so we need to transform
        the local bounding box center to get the world position.
        """
        if self.shape is not None:
            bb = self.shape.BoundingBox()
            local_center = np.array([
                (bb.xmin + bb.xmax) / 2,
                (bb.ymin + bb.ymax) / 2,
                (bb.zmin + bb.zmax) / 2,
                1.0  # homogeneous coordinate
            ])

            if self.world_transform is not None:
                world_center = self.world_transform @ local_center
                self.world_position_3d = (world_center[0], world_center[1], world_center[2])
            else:
                self.world_position_3d = (local_center[0], local_center[1], local_center[2])

    @property
    def size_display(self) -> str:
        """Format size for BOM display."""
        return f'{self.nps}"'

    @property
    def description_short(self) -> str:
        """Short description for BOM table."""
        if self.component_type == "pipe" and self.length_mm:
            return f"PIPE {self.length_mm:.0f}mm"
        return self.description


@dataclass
class WeldRecord:
    """
    Metadata for a weld joint in the pipe assembly.

    Created during PipeRoute.build() at each connection point.
    Used for weld numbering per ASME B31.3 requirements.

    Weld types:
    - "BW": Butt weld
    - "SW": Socket weld
    - "FW": Fillet weld
    """
    weld_number: int                              # Sequential weld number (W1, W2, W3...)
    weld_type: str                                # "BW", "SW", or "FW"
    description: str                              # "FLANGE TO PIPE", "PIPE TO ELBOW", etc.

    # 3D World Position of the weld (at centerline)
    world_position_3d: tuple[float, float, float]

    # Pipe direction at this weld (for smart marker placement)
    # e.g., "east", "north", "up", etc.
    pipe_direction: str | None = None

    # Direction to avoid when placing marker (where connected fitting/pipe is)
    # e.g., if the next pipe goes "up", avoid placing marker upward
    avoid_direction: str | None = None

    # Attachment point direction (up, down, east, west, north, south)
    # Used to determine label direction in drawing
    attachment_direction: str | None = None

    # Pipe outer radius in mm (for computing edge position)
    pipe_radius_mm: float | None = None

    # 2D SVG Position (computed during drawing generation)
    svg_position_2d: tuple[float, float] | None = None

    @property
    def label(self) -> str:
        """Weld label for drawing (e.g., 'W1', 'W2')."""
        return f"W{self.weld_number}"


@dataclass
class BOMEntry:
    """
    Aggregated entry for the Bill of Materials table.

    Multiple identical components are grouped into one entry with quantity.
    """
    item_number: int           # Display item number
    quantity: int              # Count of identical components
    nps: str                   # Nominal pipe size
    description: str           # Component description
    schedule_class: str        # Schedule or class
    length_mm: float | None = None  # For pipes: length in mm
    material: str = ""         # Material specification
    component_ids: list[int] = field(default_factory=list)  # Original item numbers


@dataclass
class Balloon:
    """
    A balloon annotation with leader line.

    Attributes:
        item_number: The BOM item number to display
        center: 2D center of the balloon circle (in drawing coords)
        radius: Radius of the balloon circle
        leader_start: 2D point where leader attaches to component
        leader_end: 2D point where leader meets balloon circle
    """
    item_number: int
    center: tuple[float, float]
    radius: float
    leader_start: tuple[float, float]
    leader_end: tuple[float, float]


@dataclass
class WeldMarker:
    """
    A weld number marker for ASME B31.3 compliance.

    Displayed as a flag/triangle pointing to the weld location with
    a number inside (W1, W2, etc.).

    Attributes:
        weld_number: The sequential weld number
        weld_type: "BW", "SW", or "FW"
        position: 2D position of the weld on the drawing
        label_position: 2D position for the label (offset from weld)
    """
    weld_number: int
    weld_type: str
    position: tuple[float, float]
    label_position: tuple[float, float]

    @property
    def label(self) -> str:
        """Weld label (e.g., 'W1')."""
        return f"W{self.weld_number}"


@dataclass
class WeldPlacementContext:
    """
    Spatial context for intelligent weld marker placement.

    This class aggregates information about the piping system geometry
    to enable collision-aware marker placement.
    """
    # 2D pipe segments in screen space: list of ((x1,y1), (x2,y2)) tuples
    pipe_segments_2d: list[tuple[tuple[float, float], tuple[float, float]]]

    # Already placed markers (to avoid overlap)
    placed_markers: list['WeldMarker'] = field(default_factory=list)

    # Already placed BOM balloons (to avoid overlap)
    placed_balloons: list['Balloon'] = field(default_factory=list)


@dataclass
class AnnotationPlacementContext:
    """
    Unified context for coordinating balloon and weld marker placement.

    This class:
    1. Tracks all placed annotations (balloons + weld markers)
    2. Computes effective sizes based on fit_scale
    3. Provides collision detection across all annotation types
    """
    view_area: 'ViewArea'
    fit_scale: float
    pipe_segments_2d: list[tuple[tuple[float, float], tuple[float, float]]] = field(
        default_factory=list
    )

    # Placed annotations
    placed_balloons: list['Balloon'] = field(default_factory=list)
    placed_weld_markers: list['WeldMarker'] = field(default_factory=list)

    # Effective sizes (computed in __post_init__)
    effective_balloon_radius: float = field(init=False)
    effective_weld_size: float = field(init=False)
    effective_balloon_leader: float = field(init=False)
    effective_weld_offset: float = field(init=False)

    def __post_init__(self) -> None:
        self._compute_effective_sizes()

    def _compute_effective_sizes(self) -> None:
        """Compute annotation sizes that scale with fit_scale."""
        # Scale factor with bounds: use sqrt for gentler scaling
        # This means at fit_scale=0.25 (large assembly), size_scale=0.5
        # At fit_scale=1.0, size_scale=1.0
        size_scale = max(0.6, min(1.2, self.fit_scale ** 0.5))

        # Minimum sizes for readability
        min_balloon_radius = 3.0
        min_weld_size = 2.5
        min_leader = 6.0

        self.effective_balloon_radius = max(
            min_balloon_radius, BALLOON_RADIUS * size_scale
        )
        self.effective_weld_size = max(
            min_weld_size, WELD_MARKER_SIZE * size_scale
        )

        # Leader lengths scale inversely (longer when geometry is smaller)
        # This spreads annotations further apart on scaled-down drawings
        leader_scale = 1.0 / max(0.5, size_scale)
        self.effective_balloon_leader = max(min_leader, 12.0 * leader_scale)
        self.effective_weld_offset = max(
            min_leader + 6, WELD_MARKER_OFFSET * leader_scale
        )

    def has_collision(
        self, center: tuple[float, float], radius: float
    ) -> bool:
        """Check if position collides with any placed annotation."""
        margin = 2.0

        # Check balloon collisions
        for balloon in self.placed_balloons:
            min_dist = radius + balloon.radius + margin
            dist = math.sqrt(
                (center[0] - balloon.center[0]) ** 2
                + (center[1] - balloon.center[1]) ** 2
            )
            if dist < min_dist:
                return True

        # Check weld marker collisions
        for marker in self.placed_weld_markers:
            min_dist = radius + self.effective_weld_size + margin
            dist = math.sqrt(
                (center[0] - marker.label_position[0]) ** 2
                + (center[1] - marker.label_position[1]) ** 2
            )
            if dist < min_dist:
                return True

        return False


def _cross_product_2d(
    o: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float]
) -> float:
    """
    Compute 2D cross product of vectors OA and OB.

    Returns positive if OAB makes a counter-clockwise turn,
    negative if clockwise, zero if collinear.
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _line_segments_intersect(
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    p4: tuple[float, float]
) -> bool:
    """
    Check if line segment p1-p2 intersects line segment p3-p4.

    Uses the cross product method to detect proper intersection
    (not touching at endpoints).
    """
    d1 = _cross_product_2d(p3, p4, p1)
    d2 = _cross_product_2d(p3, p4, p2)
    d3 = _cross_product_2d(p1, p2, p3)
    d4 = _cross_product_2d(p1, p2, p4)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def _leader_crosses_pipe(
    start_2d: tuple[float, float],
    end_2d: tuple[float, float],
    pipe_segments: list[tuple[tuple[float, float], tuple[float, float]]],
    exclude_threshold: float = 5.0
) -> bool:
    """
    Check if a leader line from start_2d to end_2d crosses any pipe segment.

    Args:
        start_2d: Starting point of leader line (at weld)
        end_2d: End point of leader line (at label)
        pipe_segments: List of 2D pipe segments to check against
        exclude_threshold: Distance threshold to exclude nearby segments
                          (the weld's own pipe should be excluded)

    Returns:
        True if the leader would cross a pipe segment
    """
    for seg_start, seg_end in pipe_segments:
        # Skip segments that are very close to the start point
        # (the weld's own pipe segment)
        dist_to_start = min(
            math.sqrt((start_2d[0] - seg_start[0])**2 + (start_2d[1] - seg_start[1])**2),
            math.sqrt((start_2d[0] - seg_end[0])**2 + (start_2d[1] - seg_end[1])**2)
        )
        if dist_to_start < exclude_threshold:
            continue

        if _line_segments_intersect(start_2d, end_2d, seg_start, seg_end):
            return True
    return False


# Weld marker styling constants
WELD_MARKER_SIZE = 3.5         # mm - size of weld marker oval
WELD_MARKER_OFFSET = 18.0      # mm - offset from weld point to label (increased for clarity)
WELD_MARKER_STROKE = 0.3       # mm - stroke width
WELD_FONT_SIZE = 2.5           # mm - font size for weld number


# =============================================================================
# COORDINATE MAPPING SYSTEM
# =============================================================================

@dataclass
class CoordinateMapper:
    """
    Maps coordinates between 3D CadQuery world and 2D SVG drawing space.

    This class is the single source of truth for coordinate transformations.
    It stores all the parameters needed to convert any 3D point to its
    corresponding 2D position on the SVG drawing.

    The transformation chain is:
    1. 3D World → Rotated 3D (apply model rotation for isometric view)
    2. Rotated 3D → Raw 2D Projection (orthographic projection from (1,1,1))
    3. Raw 2D → CadQuery SVG space (apply CadQuery's scale and translate)
    4. CadQuery SVG → Final Screen (apply fit scale and centering)

    Usage:
        mapper = CoordinateMapper()
        mapper.set_from_assembly_svg(raw_svg, view_area)
        screen_pos = mapper.world_to_svg(x, y, z)
    """
    # Model rotation (typically -90° around X for piping isometric)
    apply_model_rotation: bool = True

    # CadQuery's SVG transform parameters
    cq_scale_x: float = 1.0
    cq_scale_y: float = -1.0
    cq_translate_x: float = 0.0
    cq_translate_y: float = 0.0

    # Assembly geometry center (in CadQuery's transformed space)
    geometry_center_x: float = 0.0
    geometry_center_y: float = 0.0

    # View area parameters
    view_center_x: float = 0.0
    view_center_y: float = 0.0

    # Fit scale (to fit geometry in view area)
    fit_scale: float = 1.0

    # Assembly bounds in raw CadQuery projection space
    raw_bounds: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    # Is the mapper initialized?
    _initialized: bool = False

    def world_to_svg(self, x: float, y: float, z: float) -> tuple[float, float]:
        """
        Convert a 3D world position to 2D SVG screen coordinates.

        NOTE: This method uses mathematical projection which may not exactly match
        CadQuery's rendering. For precise positioning, use actual SVG geometry
        from rendered components (like balloon placement does).

        Args:
            x, y, z: Position in CadQuery 3D world (X=East, Y=North, Z=Up)

        Returns:
            (svg_x, svg_y): Position in SVG coordinates (mm on drawing sheet)
        """
        if not self._initialized:
            raise ValueError("CoordinateMapper not initialized. Call set_from_assembly_svg first.")

        # Step 1: Apply model rotation (if enabled)
        if self.apply_model_rotation:
            # Rotate -90° around X: (x, y, z) → (x, -z, y)
            rx, ry, rz = x, -z, y
        else:
            rx, ry, rz = x, y, z

        # Step 2: Orthographic projection from (1,1,1) direction
        # Right vector: (1, -1, 0) / sqrt(2)
        # Up vector: (-1, -1, 2) / sqrt(6)
        sqrt2 = 1.4142135623730951
        sqrt6 = 2.449489742783178

        raw_x = (rx - ry) / sqrt2
        raw_y = (-rx - ry + 2 * rz) / sqrt6

        # Step 3: Apply CadQuery's SVG transform
        # Transform is: scale(sx, sy) translate(tx, ty)
        # Result: (sx * (raw_x + tx), sy * (raw_y + ty))
        cq_x = self.cq_scale_x * (raw_x + self.cq_translate_x)
        cq_y = self.cq_scale_y * (raw_y + self.cq_translate_y)

        # Step 4: Apply our fit transform
        # Transform: translate(view_center) scale(fit_scale) translate(-geometry_center)
        svg_x = self.view_center_x + self.fit_scale * (cq_x - self.geometry_center_x)
        svg_y = self.view_center_y + self.fit_scale * (cq_y - self.geometry_center_y)

        return (svg_x, svg_y)

    def world_bounds_to_svg(
        self,
        bb_min: tuple[float, float, float],
        bb_max: tuple[float, float, float]
    ) -> tuple[float, float, float, float]:
        """
        Convert 3D bounding box to 2D SVG bounds.

        Projects all 8 corners and returns the enclosing 2D rectangle.

        Returns:
            (min_x, min_y, max_x, max_y) in SVG coordinates
        """
        corners = [
            (bb_min[0], bb_min[1], bb_min[2]),
            (bb_max[0], bb_min[1], bb_min[2]),
            (bb_min[0], bb_max[1], bb_min[2]),
            (bb_max[0], bb_max[1], bb_min[2]),
            (bb_min[0], bb_min[1], bb_max[2]),
            (bb_max[0], bb_min[1], bb_max[2]),
            (bb_min[0], bb_max[1], bb_max[2]),
            (bb_max[0], bb_max[1], bb_max[2]),
        ]

        projected = [self.world_to_svg(x, y, z) for x, y, z in corners]
        xs = [p[0] for p in projected]
        ys = [p[1] for p in projected]

        return (min(xs), min(ys), max(xs), max(ys))


# =============================================================================
# PROJECTION HELPERS
# =============================================================================

def compute_isometric_projection_matrix() -> list[list[float]]:
    """
    Compute the 2D projection matrix for standard piping isometric view.

    After model rotation -90 degrees around X (to fix Y/Z swap):
    - X axis (East) projects to lower-right at 30 degrees below horizontal
    - Y axis (North) projects to upper-right at 30 degrees above horizontal
    - Z axis (Up) projects straight up

    Returns a 2x3 matrix that transforms (x, y, z) to (screen_x, screen_y)
    """
    angle_30 = math.radians(30)
    cos30 = math.cos(angle_30)
    sin30 = math.sin(angle_30)

    # After model rotation: (x, y, z) -> (x, -z, y)
    # Then isometric projection
    return [
        [cos30, 0, -cos30],      # screen_x coefficients
        [sin30, -1, sin30]       # screen_y coefficients
    ]


def project_point_3d_to_2d(
    point_3d: tuple[float, float, float],
    projection_matrix: list[list[float]],
    apply_model_rotation: bool = True
) -> tuple[float, float]:
    """
    Project a 3D point to 2D using the isometric projection matrix.

    Args:
        point_3d: (x, y, z) in world coordinates
        projection_matrix: 2x3 isometric projection matrix
        apply_model_rotation: Apply -90 degree X rotation (Y/Z swap fix)

    Returns:
        (screen_x, screen_y) in projection units
    """
    x, y, z = point_3d

    if apply_model_rotation:
        # Apply model rotation: (x, y, z) -> (x, -z, y)
        x, y, z = x, -z, y

    screen_x = (projection_matrix[0][0] * x +
                projection_matrix[0][1] * y +
                projection_matrix[0][2] * z)
    screen_y = (projection_matrix[1][0] * x +
                projection_matrix[1][1] * y +
                projection_matrix[1][2] * z)

    return (screen_x, screen_y)


def project_bounds_3d_to_2d(
    shape: cq.Shape,
    projection_matrix: list[list[float]]
) -> tuple[float, float, float, float]:
    """
    Project a 3D shape's bounding box to 2D.

    Returns: (min_x, min_y, max_x, max_y) in projection units
    """
    bb = shape.BoundingBox()
    corners = get_bounding_box_corners(bb)

    # Project each corner
    projected = [project_point_3d_to_2d(c, projection_matrix) for c in corners]

    xs = [p[0] for p in projected]
    ys = [p[1] for p in projected]

    return (min(xs), min(ys), max(xs), max(ys))


def transform_to_screen_coords(
    point_proj: tuple[float, float],
    view_area: ViewArea,
    fit_scale: float,
    geometry_center: tuple[float, float]
) -> tuple[float, float]:
    """
    Transform a projected point to screen coordinates (mm on drawing).

    Applies the same transform chain as the isometric view rendering.
    """
    view_cx, view_cy = view_area.center
    geom_cx, geom_cy = geometry_center

    screen_x = view_cx + fit_scale * (point_proj[0] - geom_cx)
    screen_y = view_cy + fit_scale * (point_proj[1] - geom_cy)

    return (screen_x, screen_y)


# =============================================================================
# WORLD DIRECTION TO SCREEN DIRECTION MAPPING
# =============================================================================

# Standard isometric view from (1,1,1) with -90° X rotation.
# These are approximate screen offsets for placing balloons in each world direction.
# Screen coordinates: X increases right, Y increases down (SVG convention).
#
# The mapping was derived from the isometric projection matrix:
#   screen_x = cos30 * x' - cos30 * z'
#   screen_y = sin30 * x' - y' + sin30 * z'
# where (x', y', z') = (x, -z, y) after -90° X rotation.

WORLD_DIR_TO_SCREEN_OFFSET = {
    # Direction: (screen_dx, screen_dy) - normalized direction on screen
    # These match the axis indicator in the drawing:
    #   - X (East): 30° below horizontal (right and down)
    #   - Y (North): 30° above horizontal (right and up)
    #   - Z (Up): straight up
    "east":  (0.866, 0.5),     # +X → right-down (30° below horizontal)
    "west":  (-0.866, -0.5),   # -X → left-up (opposite of east)
    "north": (0.866, -0.5),    # +Y → right-up (30° above horizontal)
    "south": (-0.866, 0.5),    # -Y → left-down (opposite of north)
    "up":    (0.0, -1.0),      # +Z → up (straight up, negative Y in SVG)
    "down":  (0.0, 1.0),       # -Z → down (straight down, positive Y in SVG)
}

# Alternative names
WORLD_DIR_TO_SCREEN_OFFSET["+x"] = WORLD_DIR_TO_SCREEN_OFFSET["east"]
WORLD_DIR_TO_SCREEN_OFFSET["-x"] = WORLD_DIR_TO_SCREEN_OFFSET["west"]
WORLD_DIR_TO_SCREEN_OFFSET["+y"] = WORLD_DIR_TO_SCREEN_OFFSET["north"]
WORLD_DIR_TO_SCREEN_OFFSET["-y"] = WORLD_DIR_TO_SCREEN_OFFSET["south"]
WORLD_DIR_TO_SCREEN_OFFSET["+z"] = WORLD_DIR_TO_SCREEN_OFFSET["up"]
WORLD_DIR_TO_SCREEN_OFFSET["-z"] = WORLD_DIR_TO_SCREEN_OFFSET["down"]


def get_opposite_direction(direction: str) -> str:
    """Get the opposite world direction."""
    opposites = {
        "east": "west", "west": "east",
        "north": "south", "south": "north",
        "up": "down", "down": "up",
        "+x": "-x", "-x": "+x",
        "+y": "-y", "-y": "+y",
        "+z": "-z", "-z": "+z",
    }
    return opposites.get(direction.lower(), direction)


def get_free_directions(connected_directions: list[str]) -> list[str]:
    """
    Get world directions that are free (not connected to anything).

    Args:
        connected_directions: List of directions where pipes/fittings connect

    Returns:
        List of free directions, ordered by preference for balloon placement
    """
    all_directions = ["up", "down", "north", "south", "east", "west"]
    connected_set = {d.lower() for d in connected_directions}

    # Filter to free directions
    free = [d for d in all_directions if d not in connected_set]

    # Prefer vertical directions (up, down) and then horizontal
    # This tends to place balloons above/below the pipe run
    priority = {"up": 0, "down": 1, "north": 2, "south": 3, "east": 4, "west": 5}
    free.sort(key=lambda d: priority.get(d, 99))

    return free


def get_perpendicular_directions(pipe_direction: str) -> list[str]:
    """
    Get directions perpendicular to a pipe's axis.

    For balloon placement on pipes, we want to place balloons perpendicular
    to the pipe's run direction.
    """
    pipe_dir = pipe_direction.lower()

    if pipe_dir in ("east", "west", "+x", "-x"):
        # Pipe runs E-W, perpendicular directions are N, S, up, down
        return ["up", "down", "north", "south"]
    elif pipe_dir in ("north", "south", "+y", "-y"):
        # Pipe runs N-S, perpendicular directions are E, W, up, down
        return ["up", "down", "east", "west"]
    elif pipe_dir in ("up", "down", "+z", "-z"):
        # Pipe runs vertically, perpendicular directions are E, W, N, S
        return ["north", "south", "east", "west"]
    else:
        # Unknown, return all horizontal
        return ["up", "down", "north", "south", "east", "west"]


def compute_pipe_perpendicular_screen_direction(pipe_direction: str) -> tuple[float, float]:
    """
    Compute the screen direction perpendicular to a pipe's visual appearance.

    Uses the ACTUAL screen directions as shown in the drawing's axis indicator:
        - X (East): 30° below horizontal (right and down)
        - Y (North): 30° above horizontal (right and up)
        - Z (Up): straight up (vertical)

    The perpendicular is computed by rotating the screen axis 90° CCW,
    then choosing the direction that points generally upward.

    Args:
        pipe_direction: World direction of pipe axis ("east", "north", "up", etc.)

    Returns:
        (dx, dy) normalized screen direction perpendicular to pipe wall
    """
    # Standard isometric angle
    angle_30 = math.radians(30)
    cos30 = math.cos(angle_30)  # ≈ 0.866
    sin30 = math.sin(angle_30)  # ≈ 0.5

    pipe_dir = pipe_direction.lower()

    # Use consistent isometric directions for balloon/weld placement
    # based on pipe direction and isometric visibility
    if pipe_dir in ("up", "down"):
        # Vertical pipes: prefer SOUTH direction (left-down on screen)
        # This keeps balloons away from typical east-facing branches
        return (-cos30, sin30)  # South: (-0.866, 0.5)
    elif pipe_dir in ("east", "west"):
        # E-W pipes: use isometric "north" direction (right and up)
        # North is the visible front side in isometric view
        return (cos30, -sin30)  # North: (0.866, -0.5)
    else:
        # N-S pipes: use isometric "east" direction (right and down)
        return (cos30, sin30)  # East: (0.866, 0.5)


def compute_elbow_balloon_direction(
    incoming_direction: str,
    turn_direction: str
) -> str:
    """
    Compute the best balloon direction for an elbow based on its turn.

    The balloon should point toward the "outside" of the bend, which is
    perpendicular to the plane of the elbow and visible in isometric view.

    Args:
        incoming_direction: Direction the pipe was coming from (e.g., "east")
        turn_direction: Direction the pipe turns to (e.g., "up")

    Returns:
        World direction for balloon placement (e.g., "south", "north")
    """
    inc = incoming_direction.lower()
    turn = turn_direction.lower()

    # The elbow lies in a plane defined by incoming and turn directions
    # The balloon should be perpendicular to this plane, on the visible side

    # Horizontal to vertical elbows (E-W to up/down)
    if inc in ("east", "west") and turn in ("up", "down"):
        # Elbow in the east-vertical plane: balloon goes south (visible in iso)
        return "south"
    if turn in ("east", "west") and inc in ("up", "down"):
        # Same plane but reversed: balloon goes north (visible in iso)
        return "north"

    # Vertical to horizontal elbows (up/down to N-S or E-W)
    if inc in ("up", "down") and turn in ("north", "south"):
        # Elbow in the vertical-NS plane: balloon goes east (visible)
        return "east"
    if turn in ("up", "down") and inc in ("north", "south"):
        # Same plane reversed
        return "west"

    # Horizontal plane elbows (E-W to N-S)
    if inc in ("east", "west") and turn in ("north", "south"):
        # Elbow in horizontal plane: balloon goes up
        return "up"
    if inc in ("north", "south") and turn in ("east", "west"):
        # Same plane reversed
        return "up"

    # Default fallback
    return "north"


def compute_tee_balloon_direction(
    run_direction: str,
    branch_direction: str
) -> str:
    """
    Compute the best balloon direction for a tee.

    Similar to elbows, the balloon should be perpendicular to the plane
    of the tee (run + branch directions) and on the visible side in
    isometric view.

    Args:
        run_direction: Direction of the main run (e.g., "up")
        branch_direction: Direction of the branch (e.g., "east")

    Returns:
        World direction for balloon placement
    """
    run = run_direction.lower()
    branch = branch_direction.lower()

    # Tee lies in a plane defined by run and branch directions
    # Balloon should be perpendicular to this plane, on the visible side

    # Vertical run with E-W branch: tee in vertical-EW plane, balloon goes south
    if run in ("up", "down") and branch in ("east", "west"):
        return "south"

    # Vertical run with N-S branch: tee in vertical-NS plane, balloon goes east
    if run in ("up", "down") and branch in ("north", "south"):
        return "east"

    # E-W run with vertical branch: tee in EW-vertical plane, balloon goes south
    if run in ("east", "west") and branch in ("up", "down"):
        return "south"

    # E-W run with N-S branch: tee in horizontal plane, balloon goes up
    if run in ("east", "west") and branch in ("north", "south"):
        return "up"

    # N-S run with vertical branch: tee in NS-vertical plane, balloon goes east
    if run in ("north", "south") and branch in ("up", "down"):
        return "east"

    # N-S run with E-W branch: tee in horizontal plane, balloon goes up
    if run in ("north", "south") and branch in ("east", "west"):
        return "up"

    # Default: prefer south (visible in isometric)
    return "south"


def compute_flange_balloon_direction(connected_direction: str) -> str:
    """
    Compute the best balloon direction for a flange.

    The balloon should be perpendicular to the connected pipe direction
    and on the visible side in isometric view.

    Args:
        connected_direction: Direction of the connected pipe

    Returns:
        World direction for balloon placement
    """
    conn = connected_direction.lower()

    # For E-W connected pipes, balloon goes north (front visible side)
    if conn in ("east", "west"):
        return "north"

    # For N-S connected pipes, balloon goes east (visible)
    if conn in ("north", "south"):
        return "east"

    # For vertical connected pipes, balloon goes north
    if conn in ("up", "down"):
        return "north"

    return "north"


def compute_weld_label_screen_direction(
    pipe_direction: str,
    attachment_direction: str | None = None,
    avoid_direction: str | None = None
) -> tuple[float, float]:
    """
    Compute the screen direction for weld marker labels.

    Weld labels should generally point toward the viewer in isometric view,
    which is the "south" direction (left-down on screen).

    Special cases:
    - If attachment is on "up" side, label points north (away from pipe)
    - If attachment is on "east" side for horizontal branches, label points north
    - If pipe is vertical and avoid_direction is horizontal (elbow at top), label points north

    Args:
        pipe_direction: World direction of pipe axis ("east", "north", "up", etc.)
        attachment_direction: Direction of attachment point ("up", "down", "north", etc.)
        avoid_direction: Direction to avoid (where next fitting/pipe is)

    Returns:
        (dx, dy) normalized screen direction for weld label
    """
    # Standard isometric angle
    angle_30 = math.radians(30)
    cos30 = math.cos(angle_30)  # ≈ 0.866
    sin30 = math.sin(angle_30)  # ≈ 0.5

    # Screen directions for each world direction
    SCREEN_DIRS = {
        "north": (cos30, -sin30),    # Right-up
        "south": (-cos30, sin30),    # Left-down
        "east": (cos30, sin30),      # Right-down
        "west": (-cos30, -sin30),    # Left-up
        "up": (0.0, -1.0),           # Straight up
        "down": (0.0, 1.0),          # Straight down
    }

    pipe_dir = pipe_direction.lower() if pipe_direction else "east"
    avoid_dir = avoid_direction.lower() if avoid_direction else None

    # Handle special cases based on attachment direction
    if attachment_direction:
        attach = attachment_direction.lower()

        # If attached on "east" side (for branches), label points north
        # This keeps branch weld labels visible and consistent
        if attach == "east":
            return SCREEN_DIRS["north"]

        # If attached on "north" side, label points north
        if attach == "north":
            return SCREEN_DIRS["north"]

        # "up" attachments point south (consistent with other vertical pipe welds)
        if attach == "up":
            return SCREEN_DIRS["south"]

    # Default: labels point south (toward viewer in isometric)
    if pipe_dir in ("up", "down"):
        # Vertical pipes: label points south
        return SCREEN_DIRS["south"]
    elif pipe_dir in ("east", "west"):
        # E-W pipes: label points south
        return SCREEN_DIRS["south"]
    else:
        # N-S pipes: label points down (toward viewer)
        return SCREEN_DIRS["down"]


# =============================================================================
# BALLOON PLACER
# =============================================================================

class BalloonPlacer:
    """
    Smart balloon placement with collision avoidance.

    Places balloons near their components while avoiding:
    - Overlapping the pipe geometry
    - Overlapping other balloons
    - Going outside the view area
    """

    # 8 radial directions (unit vectors)
    DIRECTIONS = [
        (0, -1),          # N (up on screen)
        (0.707, -0.707),  # NE
        (1, 0),           # E
        (0.707, 0.707),   # SE
        (0, 1),           # S (down on screen)
        (-0.707, 0.707),  # SW
        (-1, 0),          # W
        (-0.707, -0.707), # NW
    ]

    def __init__(
        self,
        view_area: ViewArea,
        geometry_bounds: list[tuple[float, float, float, float]] | None = None,
        balloon_radius: float = BALLOON_RADIUS,
        min_leader_length: float = MIN_LEADER_LENGTH,
        max_leader_length: float = MAX_LEADER_LENGTH
    ):
        """
        Initialize the balloon placer.

        Args:
            view_area: The drawing view area for bounds checking
            geometry_bounds: List of (min_x, min_y, max_x, max_y) for all components
            balloon_radius: Radius of balloon circles
            min_leader_length: Minimum distance from component to balloon
            max_leader_length: Maximum distance from component to balloon
        """
        self.view_area = view_area
        self.geometry_bounds = geometry_bounds or []
        self.balloon_radius = balloon_radius
        self.min_leader_length = min_leader_length
        self.max_leader_length = max_leader_length
        self.placed_balloons: list[Balloon] = []

    def place_balloon(
        self,
        item_number: int,
        component_center: tuple[float, float],
        component_bounds: tuple[float, float, float, float]
    ) -> Balloon | None:
        """
        Find optimal position for a balloon near a component.

        Args:
            item_number: The BOM item number
            component_center: 2D center of the component
            component_bounds: (min_x, min_y, max_x, max_y) of the component

        Returns:
            Balloon object with placement info, or None if placement failed
        """
        # Determine preferred direction order based on component position
        # Components on left prefer balloons on right, and vice versa
        view_cx = self.view_area.center_x
        if component_center[0] < view_cx:
            direction_order = [2, 1, 3, 0, 4, 7, 5, 6]  # E, NE, SE, N, S, NW, SW, W
        else:
            direction_order = [6, 7, 5, 0, 4, 1, 3, 2]  # W, NW, SW, N, S, NE, SE, E

        best_position = None
        best_score = float('-inf')

        for dir_idx in direction_order:
            dx, dy = self.DIRECTIONS[dir_idx]

            # Find edge point in this direction
            edge_point = self._find_edge_point(
                component_bounds, component_center, (dx, dy)
            )

            # Try placing balloon at various distances
            for distance in [self.min_leader_length,
                           self.min_leader_length + 5,
                           self.min_leader_length + 10,
                           self.max_leader_length]:

                balloon_center = (
                    edge_point[0] + dx * distance,
                    edge_point[1] + dy * distance
                )

                # Validate placement
                if not self._is_in_view_area(balloon_center):
                    continue

                if self._has_geometry_collision(balloon_center):
                    continue

                if self._has_balloon_collision(balloon_center):
                    continue

                # Score this position
                score = self._score_position(balloon_center, distance)

                if score > best_score:
                    best_score = score
                    best_position = (balloon_center, edge_point)

        if best_position is None:
            # Fallback: place in first valid position
            best_position = self._fallback_placement(component_center, component_bounds)

        if best_position is None:
            return None

        balloon_center, edge_point = best_position

        # Calculate leader end point (on balloon circle, toward component)
        dx = edge_point[0] - balloon_center[0]
        dy = edge_point[1] - balloon_center[1]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 0:
            leader_end = (
                balloon_center[0] + (dx / dist) * self.balloon_radius,
                balloon_center[1] + (dy / dist) * self.balloon_radius
            )
        else:
            leader_end = balloon_center

        balloon = Balloon(
            item_number=item_number,
            center=balloon_center,
            radius=self.balloon_radius,
            leader_start=edge_point,
            leader_end=leader_end
        )

        self.placed_balloons.append(balloon)
        return balloon

    def _find_edge_point(
        self,
        bounds: tuple[float, float, float, float],
        center: tuple[float, float],
        direction: tuple[float, float]
    ) -> tuple[float, float]:
        """Find the point on the bounding box edge in the given direction."""
        min_x, min_y, max_x, max_y = bounds
        cx, cy = center
        dx, dy = direction

        # Ray-box intersection
        t_values = []

        if dx != 0:
            t_left = (min_x - cx) / dx
            t_right = (max_x - cx) / dx
            t_values.extend([t_left, t_right])

        if dy != 0:
            t_top = (min_y - cy) / dy
            t_bottom = (max_y - cy) / dy
            t_values.extend([t_top, t_bottom])

        # Find smallest positive t that gives a point inside the box
        valid_t = []
        for t in t_values:
            if t > 0:
                px = cx + t * dx
                py = cy + t * dy
                # Check if point is on box edge (with small tolerance)
                if (min_x - 0.1 <= px <= max_x + 0.1 and
                    min_y - 0.1 <= py <= max_y + 0.1):
                    valid_t.append(t)

        if not valid_t:
            # Fallback to center
            return center

        t = min(valid_t)
        return (cx + t * dx, cy + t * dy)

    def _is_in_view_area(self, point: tuple[float, float]) -> bool:
        """Check if balloon center (with radius) is within view area."""
        x, y = point
        r = self.balloon_radius + 2  # Add margin

        return (x - r >= self.view_area.left and
                x + r <= self.view_area.right and
                y - r >= self.view_area.top and
                y + r <= self.view_area.bottom)

    def _has_geometry_collision(self, point: tuple[float, float]) -> bool:
        """Check if balloon overlaps any geometry bounds."""
        x, y = point
        r = self.balloon_radius + 1  # Add margin

        for min_x, min_y, max_x, max_y in self.geometry_bounds:
            # Check if balloon circle overlaps rectangle
            # Find closest point on rectangle to circle center
            closest_x = max(min_x, min(x, max_x))
            closest_y = max(min_y, min(y, max_y))

            dist = math.sqrt((x - closest_x)**2 + (y - closest_y)**2)
            if dist < r:
                return True

        return False

    def _has_balloon_collision(self, point: tuple[float, float]) -> bool:
        """Check if balloon overlaps any already-placed balloons."""
        x, y = point
        min_dist = self.balloon_radius * 2 + 2  # Two radii plus margin

        for balloon in self.placed_balloons:
            bx, by = balloon.center
            dist = math.sqrt((x - bx)**2 + (y - by)**2)
            if dist < min_dist:
                return True

        return False

    def _score_position(
        self,
        balloon_center: tuple[float, float],
        leader_length: float
    ) -> float:
        """Score a potential balloon position."""
        score = 0.0

        # Prefer shorter leaders
        score -= leader_length * 0.5

        # Prefer distance from other balloons
        for other in self.placed_balloons:
            dist = math.sqrt(
                (balloon_center[0] - other.center[0])**2 +
                (balloon_center[1] - other.center[1])**2
            )
            score += min(dist, 50) * 0.1

        return score

    def _fallback_placement(
        self,
        center: tuple[float, float],
        _bounds: tuple[float, float, float, float]
    ) -> tuple[tuple[float, float], tuple[float, float]] | None:
        """Fallback placement when normal algorithm fails."""
        # Try placing at a corner of the view area
        corners = [
            self.view_area.position_top_left(margin=self.balloon_radius + 5),
            self.view_area.position_top_right(margin=self.balloon_radius + 5),
            self.view_area.position_bottom_left(margin=self.balloon_radius + 5),
            self.view_area.position_bottom_right(margin=self.balloon_radius + 5),
        ]

        for corner in corners:
            if not self._has_balloon_collision(corner):
                # Use component center as leader start
                return (corner, center)

        return None


# =============================================================================
# TOPOLOGY-BASED BALLOON PLACER
# =============================================================================

class TopologyBalloonPlacer:
    """
    Topology-aware balloon placement using component connection information.

    Instead of trying to calculate exact screen positions through projection,
    this placer uses knowledge of the pipe routing topology:
    - Flanges: Place balloon opposite to the connected pipe direction
    - Elbows: Avoid the two pipe directions, place in a free direction
    - Pipes: Place balloon perpendicular to the pipe axis at the midpoint

    The key insight is that we know the world directions and can map them
    to approximate screen directions for the isometric view.
    """

    def __init__(
        self,
        view_area: ViewArea,
        balloon_radius: float = BALLOON_RADIUS,
        leader_length: float = 15.0,  # Fixed leader length in mm
        context: AnnotationPlacementContext | None = None
    ):
        self.view_area = view_area
        self.context = context
        # Use context sizes if available, otherwise use provided defaults
        if context:
            self.balloon_radius = context.effective_balloon_radius
            self.leader_length = context.effective_balloon_leader
        else:
            self.balloon_radius = balloon_radius
            self.leader_length = leader_length
        self.placed_balloons: list[Balloon] = []

    def place_balloon_for_component(
        self,
        comp: 'ComponentRecord',
        screen_center: tuple[float, float],
        screen_bounds: tuple[float, float, float, float]
    ) -> Balloon | None:
        """
        Place a balloon for a component using topology information.

        Uses smart placement algorithms based on component type:
        - Pipes: Perpendicular to pipe axis, preferring west for vertical runs
        - Elbows: Perpendicular to elbow plane, on visible side
        - Tees: Away from branch direction
        - Flanges: Perpendicular to connected pipe, on visible side

        Args:
            comp: The ComponentRecord with connection info
            screen_center: Approximate screen center of the component
            screen_bounds: Screen bounding box (min_x, min_y, max_x, max_y)

        Returns:
            Balloon if placement succeeded, None otherwise
        """
        # Get geometry points if available (for accurate edge finding)
        geometry_points = comp.svg_geometry_points if comp.svg_geometry_points else None

        # Determine preferred directions based on component type
        preferred_dirs: list[str] = []
        use_center_attachment = False

        if comp.component_type == "pipe" and comp.pipe_direction:
            # For pipes, compute perpendicular direction based on pipe axis
            perp_dir = compute_pipe_perpendicular_screen_direction(comp.pipe_direction)
            use_center_attachment = True

            # Try placing with computed perpendicular direction
            balloon = self._try_place_with_screen_direction(
                comp.item_number, screen_center, screen_bounds, perp_dir,
                geometry_points=geometry_points,
                use_center_attachment=True
            )
            if balloon:
                self.placed_balloons.append(balloon)
                return balloon

            # Try opposite perpendicular direction
            opposite_dir = (-perp_dir[0], -perp_dir[1])
            balloon = self._try_place_with_screen_direction(
                comp.item_number, screen_center, screen_bounds, opposite_dir,
                geometry_points=geometry_points,
                use_center_attachment=True
            )
            if balloon:
                self.placed_balloons.append(balloon)
                return balloon

            # Fallback to discrete directions
            preferred_dirs = get_perpendicular_directions(comp.pipe_direction)

        elif comp.component_type == "elbow" and len(comp.connected_directions) >= 2:
            # For elbows, use smart direction based on inlet/outlet
            incoming = comp.connected_directions[0]  # First is inlet direction
            turn = comp.connected_directions[1]      # Second is outlet direction
            best_dir = compute_elbow_balloon_direction(incoming, turn)
            preferred_dirs = [best_dir]
            # Also try free directions as fallback
            free_dirs = get_free_directions(comp.connected_directions)
            preferred_dirs.extend([d for d in free_dirs if d != best_dir])
            use_center_attachment = True  # Connect at center for elbows

        elif comp.component_type == "tee" and len(comp.connected_directions) >= 3:
            # For tees, use smart direction based on run/branch
            # connected_directions typically has: [incoming, run, branch]
            run_dir = comp.connected_directions[1] if len(comp.connected_directions) > 1 else "up"
            branch_dir = comp.connected_directions[2] if len(comp.connected_directions) > 2 else "east"
            best_dir = compute_tee_balloon_direction(run_dir, branch_dir)
            preferred_dirs = [best_dir]
            free_dirs = get_free_directions(comp.connected_directions)
            preferred_dirs.extend([d for d in free_dirs if d != best_dir])
            use_center_attachment = True  # Connect at center for tees

        elif comp.component_type == "flange" and comp.connected_directions:
            # For flanges, use smart direction based on connected pipe
            connected = comp.connected_directions[0]
            best_dir = compute_flange_balloon_direction(connected)
            preferred_dirs = [best_dir]
            free_dirs = get_free_directions(comp.connected_directions)
            preferred_dirs.extend([d for d in free_dirs if d != best_dir])

        elif comp.connected_directions:
            # For other fittings, prefer directions not occupied by connections
            preferred_dirs = get_free_directions(comp.connected_directions)

        else:
            # Fallback: prefer vertical then horizontal
            preferred_dirs = ["up", "down", "north", "south", "east", "west"]

        # Try each preferred direction
        for world_dir in preferred_dirs:
            balloon = self._try_place_in_direction(
                comp.item_number, screen_center, screen_bounds, world_dir,
                geometry_points=geometry_points,
                use_center_attachment=use_center_attachment
            )
            if balloon:
                self.placed_balloons.append(balloon)
                return balloon

        # Fallback: try any direction
        all_dirs = ["up", "down", "north", "south", "east", "west"]
        for world_dir in all_dirs:
            if world_dir not in preferred_dirs:
                balloon = self._try_place_in_direction(
                    comp.item_number, screen_center, screen_bounds, world_dir,
                    geometry_points=geometry_points,
                    use_center_attachment=use_center_attachment
                )
                if balloon:
                    self.placed_balloons.append(balloon)
                    return balloon

        return None

    def _try_place_with_screen_direction(
        self,
        item_number: int,
        screen_center: tuple[float, float],
        screen_bounds: tuple[float, float, float, float],
        screen_direction: tuple[float, float],
        geometry_points: list[tuple[float, float]] | None = None,
        use_center_attachment: bool = False
    ) -> Balloon | None:
        """
        Try to place a balloon in the given screen direction.

        This is used for pipes where we compute the exact perpendicular direction
        to the pipe's visual axis using linear algebra, rather than using discrete
        world directions.

        Args:
            item_number: BOM item number
            screen_center: Screen center of component
            screen_bounds: Screen bounding box
            screen_direction: (dx, dy) normalized screen direction for balloon
            geometry_points: Optional list of actual SVG geometry points
            use_center_attachment: If True, attach at geometry point nearest to center

        Returns:
            Balloon if placement is valid, None otherwise
        """
        dx, dy = screen_direction

        # Find the edge point
        if geometry_points and len(geometry_points) > 0:
            if use_center_attachment:
                edge_point = self._find_center_edge_point(
                    geometry_points, screen_center, (dx, dy), screen_bounds
                )
            else:
                edge_point = self._find_edge_point_from_geometry(geometry_points, (dx, dy))
        else:
            edge_point = self._find_edge_point_in_direction(
                screen_bounds, screen_center, (dx, dy)
            )

        # Compute balloon center at fixed distance from edge
        balloon_center = (
            edge_point[0] + dx * self.leader_length,
            edge_point[1] + dy * self.leader_length
        )

        # Validate placement
        if not self._is_valid_placement(balloon_center):
            return None

        # Compute leader end point (on balloon circle toward component)
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 0:
            leader_end = (
                balloon_center[0] - (dx / dist) * self.balloon_radius,
                balloon_center[1] - (dy / dist) * self.balloon_radius
            )
        else:
            leader_end = balloon_center

        return Balloon(
            item_number=item_number,
            center=balloon_center,
            radius=self.balloon_radius,
            leader_start=edge_point,
            leader_end=leader_end
        )

    def _try_place_in_direction(
        self,
        item_number: int,
        screen_center: tuple[float, float],
        screen_bounds: tuple[float, float, float, float],
        world_direction: str,
        geometry_points: list[tuple[float, float]] | None = None,
        use_center_attachment: bool = False
    ) -> Balloon | None:
        """
        Try to place a balloon in the given world direction.

        Args:
            item_number: BOM item number
            screen_center: Screen center of component
            screen_bounds: Screen bounding box
            world_direction: World direction to try ("up", "east", etc.)
            geometry_points: Optional list of actual SVG geometry points for accurate edge finding
            use_center_attachment: If True, attach at geometry point nearest to center
                                   (better for pipes). If False, attach at furthest edge.

        Returns:
            Balloon if placement is valid, None otherwise
        """
        # Get screen direction corresponding to world direction
        if world_direction not in WORLD_DIR_TO_SCREEN_OFFSET:
            return None

        screen_direction = WORLD_DIR_TO_SCREEN_OFFSET[world_direction]

        return self._try_place_with_screen_direction(
            item_number,
            screen_center,
            screen_bounds,
            screen_direction,
            geometry_points,
            use_center_attachment
        )

    def _find_edge_point_from_geometry(
        self,
        points: list[tuple[float, float]],
        direction: tuple[float, float]
    ) -> tuple[float, float]:
        """
        Find the point on the actual geometry edge furthest in the given direction.

        Uses dot product to find the point that extends furthest in the specified
        screen direction.

        Args:
            points: List of (x, y) geometry points
            direction: (dx, dy) screen direction vector

        Returns:
            The point furthest in the given direction
        """
        if not points:
            return (0.0, 0.0)

        # Normalize direction
        length = math.sqrt(direction[0]**2 + direction[1]**2)
        if length < 1e-6:
            return points[0]
        dx, dy = direction[0] / length, direction[1] / length

        # Find point with maximum dot product (furthest in direction)
        best_point = points[0]
        best_dot = float('-inf')

        for x, y in points:
            dot = x * dx + y * dy
            if dot > best_dot:
                best_dot = dot
                best_point = (x, y)

        return best_point

    def _find_center_edge_point(
        self,
        points: list[tuple[float, float]],
        center: tuple[float, float],
        direction: tuple[float, float],
        _bounds: tuple[float, float, float, float] | None = None
    ) -> tuple[float, float]:
        """
        Find the edge attachment point at the center of a pipe-like component.

        For pipes: attaches at the pipe surface at its midpoint (not at corners).
        Uses ray-casting from center to find where it intersects the geometry edge.

        Args:
            points: List of (x, y) geometry points
            center: Screen center of the component
            direction: (dx, dy) balloon direction vector
            bounds: Optional bounds (not used - we use actual geometry)

        Returns:
            The edge point at center in the given direction
        """
        if not points or len(points) < 2:
            return center

        # Normalize direction
        length = math.sqrt(direction[0]**2 + direction[1]**2)
        if length < 1e-6:
            return center
        dx, dy = direction[0] / length, direction[1] / length

        cx, cy = center

        # Strategy: Cast a ray from center in the given direction
        # Find the geometry point closest to this ray that's in front of center
        #
        # For each point, compute:
        # 1. Distance along ray direction (must be positive = in front)
        # 2. Distance perpendicular to ray (should be small = close to ray)
        # Pick the point with smallest perpendicular distance among those in front

        perp_dx, perp_dy = -dy, dx  # Perpendicular to ray

        candidates = []
        for x, y in points:
            # Vector from center to point
            vx, vy = x - cx, y - cy

            # Distance along ray direction
            along_dist = vx * dx + vy * dy

            # Only consider points in front of center (in ray direction)
            if along_dist <= 0:
                continue

            # Distance perpendicular to ray
            perp_dist = abs(vx * perp_dx + vy * perp_dy)

            candidates.append((perp_dist, along_dist, x, y))

        if not candidates:
            # No points in front - fall back to closest point in direction
            best_point = points[0]
            best_dot = float('-inf')
            for x, y in points:
                dot = (x - cx) * dx + (y - cy) * dy
                if dot > best_dot:
                    best_dot = dot
                    best_point = (x, y)
            return best_point

        # Sort by perpendicular distance (closest to ray first)
        candidates.sort(key=lambda c: c[0])

        # For perpendicular attachment, we want the point closest to the ray
        # Use a tight tolerance to ensure we're on the actual ray line
        tolerance = 1.0  # mm - tight tolerance for accurate perpendicular
        close_candidates = [c for c in candidates if c[0] <= candidates[0][0] + tolerance]

        # Among candidates on the ray, pick the one closest to center (first edge intersection)
        close_candidates.sort(key=lambda c: c[1])  # Sort by along_dist

        selected = close_candidates[0]
        return (selected[2], selected[3])

    def _find_edge_point_in_direction(
        self,
        bounds: tuple[float, float, float, float],
        center: tuple[float, float],
        direction: tuple[float, float]
    ) -> tuple[float, float]:
        """Find the point on the bounding box edge in the given screen direction."""
        min_x, min_y, max_x, max_y = bounds
        cx, cy = center
        dx, dy = direction

        # Normalize direction
        length = math.sqrt(dx * dx + dy * dy)
        if length < 1e-6:
            return center
        dx, dy = dx / length, dy / length

        # Ray-box intersection: find where ray from center hits box edge
        # Use parametric form: point = center + t * direction
        t_values = []

        if abs(dx) > 1e-6:
            # Intersection with left edge (x = min_x)
            t = (min_x - cx) / dx
            if t > 0:
                y = cy + t * dy
                if min_y <= y <= max_y:
                    t_values.append(t)
            # Intersection with right edge (x = max_x)
            t = (max_x - cx) / dx
            if t > 0:
                y = cy + t * dy
                if min_y <= y <= max_y:
                    t_values.append(t)

        if abs(dy) > 1e-6:
            # Intersection with top edge (y = min_y)
            t = (min_y - cy) / dy
            if t > 0:
                x = cx + t * dx
                if min_x <= x <= max_x:
                    t_values.append(t)
            # Intersection with bottom edge (y = max_y)
            t = (max_y - cy) / dy
            if t > 0:
                x = cx + t * dx
                if min_x <= x <= max_x:
                    t_values.append(t)

        if not t_values:
            # Fallback: return point on edge closest to direction
            return (
                cx + dx * min(max_x - min_x, max_y - min_y) / 2,
                cy + dy * min(max_x - min_x, max_y - min_y) / 2
            )

        # Use smallest positive t (first intersection)
        t = min(t_values)
        return (cx + t * dx, cy + t * dy)

    def _is_valid_placement(self, balloon_center: tuple[float, float]) -> bool:
        """Check if a balloon placement is valid."""
        x, y = balloon_center
        r = self.balloon_radius

        # Check bounds with margin
        margin = r + 2
        if (x - margin < self.view_area.left or
            x + margin > self.view_area.right or
            y - margin < self.view_area.top or
            y + margin > self.view_area.bottom):
            return False

        # Check collision with existing balloons (from this placer)
        min_dist = self.balloon_radius * 2 + 3
        for other in self.placed_balloons:
            dist = math.sqrt(
                (x - other.center[0])**2 + (y - other.center[1])**2
            )
            if dist < min_dist:
                return False

        # Check collision with weld markers (from context, if available)
        if self.context:
            if self.context.has_collision(balloon_center, r):
                return False

        return True


# =============================================================================
# BALLOON SVG RENDERING
# =============================================================================

def render_balloon_svg(balloon: Balloon) -> str:
    """
    Render a balloon with leader line as SVG.

    Style:
    - Straight leader line from component edge to balloon edge
    - Small filled dot at component attachment point
    - Circle with item number centered
    """
    cx, cy = balloon.center
    r = balloon.radius
    lx1, ly1 = balloon.leader_start
    lx2, ly2 = balloon.leader_end

    return f'''
    <g class="balloon" data-item="{balloon.item_number}">
        <!-- Leader line -->
        <line x1="{lx1:.2f}" y1="{ly1:.2f}"
              x2="{lx2:.2f}" y2="{ly2:.2f}"
              stroke="{BORDER_COLOR}" stroke-width="{LEADER_STROKE_WIDTH}"/>

        <!-- Attachment dot -->
        <circle cx="{lx1:.2f}" cy="{ly1:.2f}" r="{LEADER_DOT_RADIUS}"
                fill="{BORDER_COLOR}"/>

        <!-- Balloon circle -->
        <circle cx="{cx:.2f}" cy="{cy:.2f}" r="{r:.2f}"
                fill="white" stroke="{BORDER_COLOR}"
                stroke-width="{BALLOON_STROKE_WIDTH}"/>

        <!-- Item number -->
        <text x="{cx:.2f}" y="{cy + BALLOON_FONT_SIZE * 0.35:.2f}"
              text-anchor="middle" font-family="Arial"
              font-size="{BALLOON_FONT_SIZE}" font-weight="bold">
            {balloon.item_number}
        </text>
    </g>
    '''


def render_all_balloons_svg(balloons: list[Balloon]) -> str:
    """Render all balloons as an SVG group."""
    if not balloons:
        return ""

    parts = ['<!-- Balloon Leaders -->', '<g id="balloons">']
    for balloon in balloons:
        parts.append(render_balloon_svg(balloon))
    parts.append('</g>')

    return '\n'.join(parts)


# =============================================================================
# BOM TABLE
# =============================================================================

class BOMTable:
    """
    Generates a Bill of Materials table in SVG.

    The table grows UPWARD from a baseline (same Y as title block).
    Title "BILL OF MATERIALS" is at the bottom, items are above it.

    Columns: ITEM, QTY, SIZE, SCH/CLS, LENGTH, DESCRIPTION, MATERIAL
    """

    def __init__(
        self,
        entries: list[BOMEntry],
        area: ViewArea,
        row_height: float = BOM_ROW_HEIGHT,
        header_height: float = BOM_HEADER_HEIGHT
    ):
        """
        Initialize the BOM table.

        Args:
            entries: List of BOM entries to display
            area: ViewArea defining the available space (y is baseline)
            row_height: Height of each data row
            header_height: Height of the header row
        """
        self.entries = entries
        self.area = area
        self.row_height = row_height
        self.header_height = header_height

        # Column configuration (widths as proportions) - tighter spacing
        # Columns: ITEM, QTY, SIZE, SCH/CLS, LENGTH, DESCRIPTION, MATERIAL
        self.columns = [
            ('ITEM', 0.07),
            ('QTY', 0.06),
            ('SIZE', 0.08),
            ('SCH/CLS', 0.15),
            ('LENGTH', 0.14),
            ('DESC', 0.25),
            ('MATERIAL', 0.25),
        ]

    def generate_svg(self) -> str:
        """Generate SVG content for the BOM table (grows upward) with notes below."""
        if not self.entries:
            return ""

        x = self.area.x
        baseline_y = self.area.y  # Bottom baseline (same as title block top)
        w = self.area.width

        # Calculate column widths
        col_widths = [(name, w * prop) for name, prop in self.columns]

        # Calculate total table height
        table_height = self.header_height + len(self.entries) * self.row_height

        # Table starts from this Y (grows upward from baseline)
        table_top_y = baseline_y - table_height

        svg_parts = ['<!-- Bill of Materials -->', '<g id="bom-table">']

        # Header row (at the top of the table, which is table_top_y)
        svg_parts.append(self._render_header(x, table_top_y, col_widths))

        # Data rows (below header, growing downward to baseline)
        current_y = table_top_y + self.header_height
        for entry in self.entries:
            svg_parts.append(self._render_row(entry, x, current_y, col_widths))
            current_y += self.row_height

        # Outer border around the table
        svg_parts.append(
            f'<rect x="{x}" y="{table_top_y}" width="{w}" height="{table_height}" '
            f'fill="none" stroke="{BORDER_COLOR}" stroke-width="0.35"/>'
        )

        # Notes section below the table (in title block area) with bold border
        from .constants import BORDER_WIDTH, TITLE_BLOCK_HEIGHT
        notes_height = TITLE_BLOCK_HEIGHT
        notes_y = baseline_y

        # Bold border around notes (same as title block)
        svg_parts.append(
            f'<rect x="{x}" y="{notes_y}" width="{w}" height="{notes_height}" '
            f'fill="none" stroke="{BORDER_COLOR}" stroke-width="{BORDER_WIDTH * 2}"/>'
        )

        # Notes text
        svg_parts.append(
            f'<text x="{x + 3}" y="{notes_y + 10}" '
            f'font-family="Arial" font-size="2" font-weight="bold">NOTES:</text>'
        )
        svg_parts.append(
            f'<text x="{x + 3}" y="{notes_y + 17}" '
            f'font-family="Arial" font-size="1.8">1. ALL DIMS IN MM</text>'
        )
        svg_parts.append(
            f'<text x="{x + 3}" y="{notes_y + 24}" '
            f'font-family="Arial" font-size="1.8">2. WELD PER SPEC</text>'
        )
        svg_parts.append(
            f'<text x="{x + 3}" y="{notes_y + 31}" '
            f'font-family="Arial" font-size="1.8">3. AWS SYMBOLS</text>'
        )

        svg_parts.append('</g>')
        return '\n'.join(svg_parts)

    def _render_header(
        self,
        x: float,
        y: float,
        col_widths: list[tuple[str, float]]
    ) -> str:
        """Render the BOM table header row."""
        cells = []
        col_x = x

        for name, width in col_widths:
            # Cell background
            cells.append(
                f'<rect x="{col_x}" y="{y}" width="{width}" '
                f'height="{self.header_height}" fill="#E8E8E8" '
                f'stroke="{BORDER_COLOR}" stroke-width="{THIN_LINE_WIDTH}"/>'
            )
            # Header text
            cells.append(
                f'<text x="{col_x + width/2}" y="{y + self.header_height * 0.65}" '
                f'text-anchor="middle" font-family="Arial" '
                f'font-size="2" font-weight="bold">{name}</text>'
            )
            col_x += width

        return '\n'.join(cells)

    def _render_row(
        self,
        entry: BOMEntry,
        x: float,
        y: float,
        col_widths: list[tuple[str, float]]
    ) -> str:
        """Render a single BOM entry row."""
        # Format length - show in architectural feet-inches for pipes, blank for fittings
        length_str = mm_to_arch_feet_inches(entry.length_mm)

        values = [
            str(entry.item_number),
            str(entry.quantity),
            f'{entry.nps}"',
            entry.schedule_class[:12].upper(),  # Truncate long class names, all caps
            length_str,
            entry.description[:20],  # Truncate long descriptions
            entry.material[:20] if entry.material else "",  # Material
        ]

        cells = []
        col_x = x

        for (_, width), value in zip(col_widths, values, strict=True):
            # Cell border
            cells.append(
                f'<rect x="{col_x}" y="{y}" width="{width}" '
                f'height="{self.row_height}" fill="none" '
                f'stroke="{BORDER_COLOR}" stroke-width="{THIN_LINE_WIDTH}"/>'
            )
            # Value text
            cells.append(
                f'<text x="{col_x + 1.5}" y="{y + self.row_height * 0.7}" '
                f'font-family="Arial" font-size="2">{value}</text>'
            )
            col_x += width

        return '\n'.join(cells)

    @property
    def total_height(self) -> float:
        """Calculate total height of the BOM table."""
        return self.header_height + len(self.entries) * self.row_height


# =============================================================================
# BOM AGGREGATION
# =============================================================================

def aggregate_components_to_bom(
    components: list[ComponentRecord],
    material: str = ""
) -> list[BOMEntry]:
    """
    Aggregate component records into BOM entries.

    Groups identical components (same type, NPS, class) and sums quantities.
    Pipes are NOT aggregated (each pipe segment is a separate entry).

    Args:
        components: List of ComponentRecord objects
        material: Default material specification for all components
    """
    if not components:
        return []

    # Group non-pipe components by type, NPS, and class
    groups: dict[tuple[str, str, str], list[ComponentRecord]] = {}
    pipe_entries: list[BOMEntry] = []

    for comp in components:
        if comp.component_type == "pipe":
            # Each pipe is a separate entry with length
            pipe_entries.append(BOMEntry(
                item_number=comp.item_number,
                quantity=1,
                nps=comp.nps,
                description=comp.description_short,
                schedule_class=comp.schedule_class,
                length_mm=comp.length_mm,
                material=material,
                component_ids=[comp.item_number]
            ))
        else:
            # Group fittings
            key = (comp.component_type, comp.nps, comp.schedule_class)
            if key not in groups:
                groups[key] = []
            groups[key].append(comp)

    # Create entries for grouped fittings
    fitting_entries: list[BOMEntry] = []
    for (_comp_type, nps, schedule_class), comps in groups.items():
        fitting_entries.append(BOMEntry(
            item_number=comps[0].item_number,  # Use first item's number
            quantity=len(comps),
            nps=nps,
            description=comps[0].description,
            schedule_class=schedule_class,
            length_mm=None,  # Fittings don't have length
            material=material,
            component_ids=[c.item_number for c in comps]
        ))

    # Combine and renumber
    all_entries = fitting_entries + pipe_entries
    all_entries.sort(key=lambda e: e.item_number)

    # Renumber sequentially
    for i, entry in enumerate(all_entries, start=1):
        entry.item_number = i

    return all_entries


# =============================================================================
# WELD MARKER RENDERING
# =============================================================================

def render_weld_markers_svg(weld_markers: list[WeldMarker]) -> str:
    """
    Render weld markers as SVG elements.

    Each weld marker is a slot shape (stadium/pill shape) with "WELD #" inside,
    connected to the weld location by a leader line.

    Args:
        weld_markers: List of WeldMarker objects with positions set

    Returns:
        SVG string containing all weld marker elements
    """
    if not weld_markers:
        return ""

    svg_parts = ['<!-- Weld Markers -->']

    # Slot dimensions - height matches BOM balloon diameter
    slot_height = BALLOON_RADIUS * 2  # Same diameter as BOM balloons (8mm)
    slot_radius = slot_height / 2     # Radius for rounded ends
    slot_width = 14.0                 # Width to fit "WELD #" text

    for marker in weld_markers:
        wx, wy = marker.position
        lx, ly = marker.label_position

        # Slot rectangle bounds (centered on label position)
        slot_left = lx - slot_width / 2
        slot_top = ly - slot_height / 2

        # Leader line from weld point to slot edge
        # Calculate point on slot edge closest to weld point
        dx = wx - lx
        dy = wy - ly
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 0:
            # Approximate edge point (use slot_radius for simplicity)
            edge_x = lx + (dx / dist) * (slot_width / 2)
            edge_y = ly + (dy / dist) * slot_radius
        else:
            edge_x, edge_y = lx, ly

        # Label text: "WELD #" instead of "W#"
        label_text = f"WELD {marker.weld_number}"

        svg_parts.append(f'''
        <g class="weld-marker" id="weld-{marker.weld_number}">
            <!-- Leader line -->
            <line x1="{wx}" y1="{wy}" x2="{edge_x}" y2="{edge_y}"
                  stroke="#000000" stroke-width="{WELD_MARKER_STROKE}"/>
            <!-- Weld point dot -->
            <circle cx="{wx}" cy="{wy}" r="0.8"
                    fill="#000000"/>
            <!-- Slot background (stadium/pill shape) -->
            <rect x="{slot_left}" y="{slot_top}" width="{slot_width}" height="{slot_height}"
                  rx="{slot_radius}" ry="{slot_radius}"
                  fill="white" stroke="#000000" stroke-width="{WELD_MARKER_STROKE}"/>
            <!-- Weld label -->
            <text x="{lx}" y="{ly + WELD_FONT_SIZE * 0.35}"
                  text-anchor="middle" font-family="Arial" font-size="{WELD_FONT_SIZE}"
                  font-weight="bold">{label_text}</text>
        </g>''')

    return '\n'.join(svg_parts)


def create_weld_markers(
    welds: list['WeldRecord'],
    coord_mapper: 'CoordinateMapper',
    view_area: Optional['ViewArea'] = None,
    existing_markers: list[WeldMarker] | None = None,
    pipe_segments_2d: list[tuple[tuple[float, float], tuple[float, float]]] | None = None,
    placed_balloons: list['Balloon'] | None = None,
    context: AnnotationPlacementContext | None = None
) -> list[WeldMarker]:
    """
    Create WeldMarker objects from WeldRecords with smart placement.

    Uses the pipe direction at each weld to compute a perpendicular
    screen direction for marker placement (like BOM balloons).
    The marker is placed perpendicular to the pipe, with the leader
    pointing to the pipe's outer edge.

    Args:
        welds: List of WeldRecord objects with 3D positions
        coord_mapper: CoordinateMapper for 3D to 2D projection
        view_area: Optional view area for bounds checking
        existing_markers: Previously placed markers to avoid
        pipe_segments_2d: 2D pipe segments for collision detection
        placed_balloons: Already placed BOM balloons to avoid
        context: Optional AnnotationPlacementContext for scale-aware sizing

    Returns:
        List of WeldMarker objects ready for rendering
    """
    markers = []
    placed_positions = []  # Track placed label positions to avoid overlap

    # Use context sizes if available, otherwise use defaults
    if context:
        weld_offset = context.effective_weld_offset
        weld_size = context.effective_weld_size
    else:
        weld_offset = WELD_MARKER_OFFSET
        weld_size = WELD_MARKER_SIZE

    if existing_markers:
        placed_positions = [m.label_position for m in existing_markers]

    # Fallback direction candidates (if pipe_direction not available)
    fallback_directions = [
        (1, -1),   # up-right
        (-1, -1),  # up-left
        (1, 1),    # down-right
        (-1, 1),   # down-left
    ]

    for weld in welds:
        # Project 3D centerline position to 2D
        x3d, y3d, z3d = weld.world_position_3d
        svg_center = coord_mapper.world_to_svg(x3d, y3d, z3d)

        if svg_center is None:
            continue

        cx, cy = svg_center
        best_weld_pos = None
        best_label_pos = None

        # If we have pipe direction, compute perpendicular screen direction
        if weld.pipe_direction:
            # Get the perpendicular direction in screen space
            perp_dir = compute_pipe_perpendicular_screen_direction(weld.pipe_direction)

            # Check if we need to avoid a direction (e.g., where next pipe goes)
            # If the computed perpendicular aligns with the avoid direction, flip it
            if weld.avoid_direction and weld.avoid_direction in WORLD_DIR_TO_SCREEN_OFFSET:
                avoid_screen = WORLD_DIR_TO_SCREEN_OFFSET[weld.avoid_direction]
                # Check if perpendicular points toward the avoid direction
                # Use dot product: positive means same general direction
                dot = perp_dir[0] * avoid_screen[0] + perp_dir[1] * avoid_screen[1]
                if dot > 0.3:  # If pointing toward avoid direction, flip
                    perp_dir = (-perp_dir[0], -perp_dir[1])

            # Try the (possibly flipped) perpendicular direction first
            directions_to_try = [
                perp_dir,
                (-perp_dir[0], -perp_dir[1]),  # Opposite perpendicular
            ]
        else:
            directions_to_try = fallback_directions

        # Edge offset: compute the visible pipe edge in screen space
        # The 3D weld position is at the centerline; we need to offset to the
        # visible pipe edge. In isometric projection, the visible half-width
        # of a pipe is: pipe_radius * fit_scale / 2 (the /2 is the isometric
        # foreshortening factor for perpendicular offsets).
        #
        # This matches how BOM balloons find edges using actual SVG geometry.
        if weld.pipe_radius_mm is not None and coord_mapper.fit_scale > 0:
            # Compute visible edge offset using proper coordinate transformation
            edge_offset = weld.pipe_radius_mm * coord_mapper.fit_scale / 2.0
        elif weld.weld_type == "BW":
            edge_offset = 8.0  # Fallback for butt welds without radius info
        else:
            edge_offset = 6.0  # Fallback for socket welds without radius info

        for direction in directions_to_try:
            dx, dy = direction

            # Normalize direction
            norm = math.sqrt(dx**2 + dy**2)
            if norm > 0:
                dx, dy = dx / norm, dy / norm

            # Weld point is at the pipe edge (offset from centerline)
            weld_x = cx + dx * edge_offset
            weld_y = cy + dy * edge_offset

            # Label is further out along the same direction
            label_x = weld_x + dx * weld_offset
            label_y = weld_y + dy * weld_offset

            # Check bounds (if view_area provided)
            if view_area:
                margin = weld_size * 1.5
                if (label_x - margin < view_area.x or
                    label_x + margin > view_area.x + view_area.width or
                    label_y - margin < view_area.y or
                    label_y + margin > view_area.y + view_area.height):
                    continue  # Out of bounds, try next direction

            # Check overlap with existing markers
            overlap = False
            min_dist = weld_size * 3  # Minimum distance between labels
            for px, py in placed_positions:
                dist = math.sqrt((label_x - px)**2 + (label_y - py)**2)
                if dist < min_dist:
                    overlap = True
                    break

            # Check overlap with BOM balloons
            if not overlap and placed_balloons:
                balloon_margin = weld_size * 2
                for balloon in placed_balloons:
                    bx, by = balloon.center
                    dist = math.sqrt((label_x - bx)**2 + (label_y - by)**2)
                    if dist < balloon.radius + balloon_margin:
                        overlap = True
                        break

            # Check if leader line crosses any pipe geometry
            if not overlap and pipe_segments_2d:
                if _leader_crosses_pipe(
                    (weld_x, weld_y),
                    (label_x, label_y),
                    pipe_segments_2d
                ):
                    overlap = True

            if not overlap:
                best_weld_pos = (weld_x, weld_y)
                best_label_pos = (label_x, label_y)
                break

        # If no good position found, use first direction as fallback
        if best_label_pos is None:
            direction = directions_to_try[0] if directions_to_try else fallback_directions[0]
            norm = math.sqrt(direction[0]**2 + direction[1]**2)
            dx, dy = direction[0] / norm, direction[1] / norm
            weld_x = cx + dx * edge_offset
            weld_y = cy + dy * edge_offset
            best_weld_pos = (weld_x, weld_y)
            best_label_pos = (weld_x + dx * weld_offset, weld_y + dy * weld_offset)

        placed_positions.append(best_label_pos)
        # best_weld_pos is always set: either in the loop above or in the fallback
        assert best_weld_pos is not None
        markers.append(WeldMarker(
            weld_number=weld.weld_number,
            weld_type=weld.weld_type,
            position=best_weld_pos,
            label_position=best_label_pos
        ))

    return markers


def optimize_annotation_positions(
    context: AnnotationPlacementContext,
    max_iterations: int = 10
) -> None:
    """
    Apply force-directed optimization to resolve remaining annotation overlaps.

    Uses a simple repulsion algorithm where overlapping annotations push each other
    apart. Modifies balloon centers and weld marker label positions in place.

    The algorithm:
    1. Collect all annotation positions (balloons and weld markers)
    2. For each pair of overlapping annotations, apply repulsion force
    3. Move annotations apart proportionally to their overlap
    4. Repeat until converged or max iterations reached

    Args:
        context: AnnotationPlacementContext with placed_balloons and placed_weld_markers
        max_iterations: Maximum number of optimization iterations (default 10)
    """
    for _iteration in range(max_iterations):
        total_movement = 0.0

        # Collect all annotation positions
        # Each entry: {'pos': [x, y], 'radius': r, 'type': str, 'obj': object}
        annotations: list[dict] = []
        for b in context.placed_balloons:
            annotations.append({
                'pos': [b.center[0], b.center[1]],  # Mutable list for updating
                'radius': b.radius,
                'type': 'balloon',
                'obj': b
            })
        for m in context.placed_weld_markers:
            annotations.append({
                'pos': [m.label_position[0], m.label_position[1]],
                'radius': context.effective_weld_size,
                'type': 'weld',
                'obj': m
            })

        if len(annotations) < 2:
            return  # Nothing to optimize

        # Check all pairs and apply repulsion
        for i, ann1 in enumerate(annotations):
            for ann2 in annotations[i + 1:]:
                min_dist = ann1['radius'] + ann2['radius'] + 3.0
                dx = ann1['pos'][0] - ann2['pos'][0]
                dy = ann1['pos'][1] - ann2['pos'][1]
                actual_dist = math.sqrt(dx * dx + dy * dy)

                if actual_dist < min_dist and actual_dist > 0.1:
                    # Compute repulsion - push apart
                    overlap = min_dist - actual_dist
                    nudge = overlap * 0.25  # Damping factor
                    nx, ny = dx / actual_dist, dy / actual_dist

                    # Move both annotations apart (each moves half the distance)
                    ann1['pos'][0] += nx * nudge
                    ann1['pos'][1] += ny * nudge
                    ann2['pos'][0] -= nx * nudge
                    ann2['pos'][1] -= ny * nudge
                    total_movement += nudge * 2

        # Apply updated positions back to objects
        for ann in annotations:
            if ann['type'] == 'balloon':
                b = ann['obj']
                new_center = (ann['pos'][0], ann['pos'][1])
                # Recompute leader_end to point from new center toward leader_start
                dx = new_center[0] - b.leader_start[0]
                dy = new_center[1] - b.leader_start[1]
                dist = math.sqrt(dx * dx + dy * dy)
                if dist > 0:
                    # leader_end is on the balloon circle, toward the component
                    leader_end = (
                        new_center[0] - (dx / dist) * b.radius,
                        new_center[1] - (dy / dist) * b.radius
                    )
                else:
                    leader_end = new_center
                # Update balloon (dataclass is not frozen)
                b.center = new_center
                b.leader_end = leader_end
            else:
                # Weld marker - update label position
                m = ann['obj']
                m.label_position = (ann['pos'][0], ann['pos'][1])

        # Stop if converged (very little movement)
        if total_movement < 0.5:
            break
