"""
Dimension System for Pipe Engineering Drawings

This module provides ASME Y14.5 compliant dimensioning for piping drawings:
- Centerline-to-centerline (C-L to C-L) dimensions between fittings
- Pipe cut-length dimensions for fabrication
- Support for both isometric and orthographic views

Components:
- DimensionPoint: A point that can be used as a dimension endpoint
- DimensionRecord: A single linear dimension
- DimensionStyle: ASME-compliant styling configuration
- DimensionPlacer: Placement algorithm with collision avoidance
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable

import numpy as np

from .bom import mm_to_arch_feet_inches
from .constants import THIN_LINE_WIDTH, BORDER_COLOR
from .view_area import ViewArea

if TYPE_CHECKING:
    from .bom import Balloon, ComponentRecord, CoordinateMapper, WeldMarker, WeldRecord
    from .view_projection import ViewProjection


# =============================================================================
# DIMENSION STYLING CONSTANTS
# =============================================================================

DIMENSION_LINE_WIDTH = 0.25        # mm - dimension and extension line stroke
EXTENSION_LINE_GAP = 3.0           # mm - gap between geometry and extension line start
EXTENSION_LINE_OVERSHOOT = 2.0     # mm - overshoot past dimension line
ARROW_LENGTH = 2.5                 # mm - arrowhead length
ARROW_WIDTH = 0.8                  # mm - arrowhead width at base
DIMENSION_FONT_SIZE = 3.0          # mm - dimension value text size
DIMENSION_TEXT_OFFSET = 1.0        # mm - gap between dimension line and text
MIN_DIMENSION_OFFSET = 10.0        # mm - minimum offset from geometry
DIMENSION_LAYER_SPACING = 7.0      # mm - spacing between stacked dimension layers


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class DimensionPoint:
    """
    A point that can be used as a dimension endpoint.

    Represents a specific location in 3D world space that can be dimensioned to,
    such as a flange face, elbow center, or pipe endpoint.

    Attributes:
        world_position_3d: Position in 3D world coordinates (X=East, Y=North, Z=Up)
        svg_position_2d: Computed position in SVG drawing coordinates (set during placement)
        point_type: Type of point - "centerline", "face", "endpoint"
        component_ref: Item number reference to the associated ComponentRecord
        description: Human-readable description (e.g., "Elbow 2 center")
    """
    world_position_3d: tuple[float, float, float]
    svg_position_2d: tuple[float, float] | None = None
    point_type: str = "centerline"  # "centerline", "face", "endpoint"
    component_ref: int | None = None
    description: str = ""


@dataclass
class DimensionRecord:
    """
    Represents a single linear dimension on the drawing.

    Dimension types:
    - "centerline": C-L to C-L distance between pipe run centerlines
    - "pipe_length": Actual fabrication cut length of a pipe segment
    - "overall": Overall dimension spanning multiple segments
    - "fitting": Face-to-centerline or face-to-face fitting dimension

    The dimension consists of:
    - Two extension lines from the geometry to the dimension line
    - A dimension line with arrows at each end
    - A dimension value text

    Attributes:
        dimension_id: Unique identifier for this dimension
        dimension_type: Type of dimension (see above)
        start_point: Starting point for the dimension
        end_point: Ending point for the dimension
        value_mm: Dimension value in millimeters
        display_value: Formatted display string (e.g., "3'-6\"")
        direction: Dimension orientation ("horizontal", "vertical", "aligned")
        pipe_direction: World direction of pipe for aligned dimensions
        offset_distance: Distance from geometry to dimension line (mm)
        layer: Stacking layer (0 = closest to geometry)
    """
    dimension_id: int
    dimension_type: str  # "centerline", "pipe_length", "overall", "fitting"
    start_point: DimensionPoint
    end_point: DimensionPoint
    value_mm: float
    display_value: str = ""

    # Dimension orientation and placement
    direction: str = "horizontal"  # "horizontal", "vertical", "aligned"
    pipe_direction: str | None = None  # For aligned dimensions: "east", "north", etc.
    offset_distance: float = MIN_DIMENSION_OFFSET
    offset_side: str = "auto"  # "top", "bottom", "left", "right", "auto"
    layer: int = 0  # Stacking layer (0 = closest to geometry)

    # SVG computed positions (set during placement/rendering)
    svg_start: tuple[float, float] | None = None
    svg_end: tuple[float, float] | None = None
    svg_dim_line_pos: float | None = None  # Y for horizontal, X for vertical
    svg_text_position: tuple[float, float] | None = None

    # Geometry edge positions for extension lines (set during placement)
    # These are where the extension lines should start FROM (outer edge of geometry)
    svg_start_geometry_edge: tuple[float, float] | None = None
    svg_end_geometry_edge: tuple[float, float] | None = None

    def __post_init__(self):
        """Format display value if not provided."""
        if not self.display_value and self.value_mm > 0:
            self.display_value = mm_to_arch_feet_inches(self.value_mm)


@dataclass
class DimensionStyle:
    """
    ASME Y14.5 compliant dimension styling configuration.

    Provides all styling parameters for rendering dimensions including
    line weights, arrow styles, text formatting, and placement rules.
    """
    # Line styling
    line_stroke_width: float = DIMENSION_LINE_WIDTH
    line_color: str = BORDER_COLOR
    extension_line_gap: float = EXTENSION_LINE_GAP
    extension_line_overshoot: float = EXTENSION_LINE_OVERSHOOT

    # Arrow styling
    arrow_length: float = ARROW_LENGTH
    arrow_width: float = ARROW_WIDTH
    arrow_style: str = "filled"  # "filled", "open", "tick"

    # Text styling
    font_family: str = "Arial, sans-serif"
    font_size: float = DIMENSION_FONT_SIZE
    font_weight: str = "normal"
    text_offset: float = DIMENSION_TEXT_OFFSET
    text_gap: float = 1.5  # Gap on each side of text in the dimension line

    # Placement parameters
    min_offset_from_geometry: float = MIN_DIMENSION_OFFSET
    layer_spacing: float = DIMENSION_LAYER_SPACING

    # Units and format
    unit_system: str = "imperial"  # "imperial" or "metric"
    show_units: bool = False  # Units typically not shown on pipe isometrics
    precision_mm: int = 0  # Decimal places for mm
    precision_inch_denominator: int = 16  # Fraction denominator (16 = 1/16")


@dataclass
class DimensionPlacementContext:
    """
    Context for dimension placement with collision information.

    Tracks existing annotations to avoid overlaps during dimension placement.
    """
    view_area: ViewArea
    fit_scale: float = 1.0

    # Existing annotations to avoid
    placed_balloons: list = field(default_factory=list)
    placed_weld_markers: list = field(default_factory=list)
    placed_dimensions: list[DimensionRecord] = field(default_factory=list)

    # Geometry bounds for placing dimensions outside
    geometry_bounds_2d: tuple[float, float, float, float] | None = None

    # View-specific coordinate mapper
    coord_mapper: "CoordinateMapper | ViewProjection | None" = None

    # Components for looking up geometry bounds at dimension points
    components: list["ComponentRecord"] = field(default_factory=list)


# =============================================================================
# DIMENSION POINT EXTRACTION
# =============================================================================

def get_direction_vector(direction_name: str) -> tuple[float, float, float]:
    """Convert direction name to unit vector."""
    vectors = {
        "east": (1.0, 0.0, 0.0),
        "west": (-1.0, 0.0, 0.0),
        "north": (0.0, 1.0, 0.0),
        "south": (0.0, -1.0, 0.0),
        "up": (0.0, 0.0, 1.0),
        "down": (0.0, 0.0, -1.0),
    }
    return vectors.get(direction_name, (1.0, 0.0, 0.0))


def get_fitting_centerline_point(
    comp: "ComponentRecord",
) -> tuple[float, float, float] | None:
    """
    Get the centerline reference point for a fitting.

    For piping dimensions, we need specific reference points:
    - Flanges: The pipe connection face (where the weld attaches)
    - Elbows: The elbow center (intersection of the two pipe centerlines)
    - Other fittings: The geometric center

    These points are used for C-L to C-L dimensioning.

    Args:
        comp: ComponentRecord for a fitting

    Returns:
        3D world position of the centerline reference point, or None if unavailable
    """
    if comp.shape is None or comp.world_transform is None:
        # Fall back to stored world_position_3d (bounding box center)
        return comp.world_position_3d

    bb = comp.shape.BoundingBox()

    if comp.component_type == "flange":
        # For flanges, the flange FACE is at local Z=max (the raised face sealing surface)
        # The hub/weld end extends in the -Z direction
        # Piping fabrication drawings dimension to the flange face, not the weld end
        local_point = np.array([0, 0, bb.zmax, 1.0])
        world_point = comp.world_transform @ local_point
        return (float(world_point[0]), float(world_point[1]), float(world_point[2]))

    elif comp.component_type == "elbow":
        # For elbows, the center is at local origin (0, 0, 0)
        # This is where the two pipe centerlines intersect
        local_point = np.array([0, 0, 0, 1.0])
        world_point = comp.world_transform @ local_point
        return (float(world_point[0]), float(world_point[1]), float(world_point[2]))

    elif comp.component_type == "reducer":
        # For reducers, the center is at local origin (0, 0, 0)
        # This is on the centerline of the pipe run
        local_point = np.array([0, 0, 0, 1.0])
        world_point = comp.world_transform @ local_point
        return (float(world_point[0]), float(world_point[1]), float(world_point[2]))

    else:
        # For other fittings, use bounding box center
        return comp.world_position_3d


def extract_dimension_points(
    components: list["ComponentRecord"],
) -> list[DimensionPoint]:
    """
    Extract all potential dimension points from the route components.

    Returns points for:
    - Flange faces (for face-to-centerline dims)
    - Elbow centers (for C-L to C-L dims)
    - Pipe segment endpoints (for cut lengths)

    Args:
        components: List of ComponentRecord from PipeRoute

    Returns:
        List of DimensionPoint objects
    """
    points: list[DimensionPoint] = []

    for comp in components:
        if comp.world_position_3d is None:
            continue

        if comp.component_type == "flange":
            # Flange center point (weld end)
            points.append(DimensionPoint(
                world_position_3d=comp.world_position_3d,
                point_type="face",
                component_ref=comp.item_number,
                description=f"Flange {comp.item_number}",
            ))

        elif comp.component_type == "elbow":
            # Elbow center point
            points.append(DimensionPoint(
                world_position_3d=comp.world_position_3d,
                point_type="centerline",
                component_ref=comp.item_number,
                description=f"Elbow {comp.item_number} center",
            ))

        elif comp.component_type == "pipe":
            # Pipe center point (for pipe length dimensions)
            points.append(DimensionPoint(
                world_position_3d=comp.world_position_3d,
                point_type="centerline",
                component_ref=comp.item_number,
                description=f"Pipe {comp.item_number} center",
            ))

    return points


def compute_pipe_endpoints(
    comp: "ComponentRecord",
) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    """
    Compute the 3D endpoints of a pipe segment from its ComponentRecord.

    Uses the pipe's world_position_3d (center), pipe_direction, and length_mm
    to calculate the start and end points along the centerline.

    Args:
        comp: ComponentRecord for a pipe segment

    Returns:
        Tuple of (start_point_3d, end_point_3d) or None if data is missing
    """
    if comp.component_type != "pipe":
        return None
    if comp.world_position_3d is None or comp.length_mm is None:
        return None
    if comp.pipe_direction is None:
        return None

    center = comp.world_position_3d
    direction = get_direction_vector(comp.pipe_direction)
    half_length = comp.length_mm / 2.0

    start = (
        center[0] - direction[0] * half_length,
        center[1] - direction[1] * half_length,
        center[2] - direction[2] * half_length,
    )
    end = (
        center[0] + direction[0] * half_length,
        center[1] + direction[1] * half_length,
        center[2] + direction[2] * half_length,
    )

    return (start, end)


# =============================================================================
# DIMENSION GENERATION
# =============================================================================

def generate_centerline_dimensions(
    components: list["ComponentRecord"],
) -> list[DimensionRecord]:
    """
    Generate centerline-to-centerline dimensions between fittings.

    Creates dimensions between consecutive fittings (flanges, elbows)
    along each pipe run axis.

    Args:
        components: List of ComponentRecord from PipeRoute

    Returns:
        List of DimensionRecord for centerline dimensions
    """
    dimensions: list[DimensionRecord] = []
    dim_id = 1

    # Group components by type and find consecutive fitting pairs
    # Include reducers as they are dimensioning reference points
    fittings = [c for c in components if c.component_type in ("flange", "elbow", "reducer")]

    # Also need pipe components to know the direction between fittings
    pipes = [c for c in components if c.component_type == "pipe"]

    # For each pipe, create a C-L dimension from the fitting before to fitting after
    for pipe in pipes:
        if pipe.world_position_3d is None or pipe.length_mm is None:
            continue

        # Find the fittings connected to this pipe by looking at pipe direction
        # The pipe connects the fitting before it to the fitting after it
        pipe_dir = pipe.pipe_direction
        if pipe_dir is None:
            continue

        endpoints = compute_pipe_endpoints(pipe)
        if endpoints is None:
            continue

        start_3d, end_3d = endpoints

        # Find fittings near the endpoints
        start_fitting = _find_nearest_fitting(fittings, start_3d)
        end_fitting = _find_nearest_fitting(fittings, end_3d)

        if start_fitting is None or end_fitting is None:
            continue

        # Get the proper centerline reference points for each fitting
        # (flange face for flanges, center for elbows)
        start_cl_point = get_fitting_centerline_point(start_fitting)
        end_cl_point = get_fitting_centerline_point(end_fitting)

        if start_cl_point is None or end_cl_point is None:
            continue

        # Calculate the actual C-L to C-L distance
        cl_distance = _distance_3d(start_cl_point, end_cl_point)

        # Skip dimensions that are too small (likely same fitting found twice)
        if cl_distance < 1.0:  # Less than 1mm is not meaningful
            continue

        # Determine dimension direction based on pipe direction
        direction = _pipe_direction_to_dim_direction(pipe_dir)

        start_point = DimensionPoint(
            world_position_3d=start_cl_point,
            point_type="centerline",
            component_ref=start_fitting.item_number,
            description=f"{start_fitting.component_type.title()} {start_fitting.item_number}",
        )

        end_point = DimensionPoint(
            world_position_3d=end_cl_point,
            point_type="centerline",
            component_ref=end_fitting.item_number,
            description=f"{end_fitting.component_type.title()} {end_fitting.item_number}",
        )

        dimensions.append(DimensionRecord(
            dimension_id=dim_id,
            dimension_type="centerline",
            start_point=start_point,
            end_point=end_point,
            value_mm=cl_distance,
            direction=direction,
            pipe_direction=pipe_dir,
        ))
        dim_id += 1

    return dimensions


def generate_pipe_length_dimensions(
    components: list["ComponentRecord"],
) -> list[DimensionRecord]:
    """
    Generate fabrication cut-length dimensions for each pipe segment.

    Creates dimensions showing the actual cut length of each pipe,
    positioned perpendicular to the pipe axis.

    Args:
        components: List of ComponentRecord from PipeRoute

    Returns:
        List of DimensionRecord for pipe length dimensions
    """
    dimensions: list[DimensionRecord] = []
    dim_id = 100  # Start at 100 to distinguish from centerline dims

    for comp in components:
        if comp.component_type != "pipe":
            continue
        if comp.length_mm is None or comp.length_mm <= 0:
            continue

        endpoints = compute_pipe_endpoints(comp)
        if endpoints is None:
            continue

        start_3d, end_3d = endpoints

        # Determine dimension direction based on pipe direction
        direction = _pipe_direction_to_dim_direction(comp.pipe_direction)

        start_point = DimensionPoint(
            world_position_3d=start_3d,
            point_type="endpoint",
            component_ref=comp.item_number,
            description=f"Pipe {comp.item_number} start",
        )

        end_point = DimensionPoint(
            world_position_3d=end_3d,
            point_type="endpoint",
            component_ref=comp.item_number,
            description=f"Pipe {comp.item_number} end",
        )

        dimensions.append(DimensionRecord(
            dimension_id=dim_id,
            dimension_type="pipe_length",
            start_point=start_point,
            end_point=end_point,
            value_mm=comp.length_mm,
            direction=direction,
            pipe_direction=comp.pipe_direction,
        ))
        dim_id += 1

    return dimensions


def generate_pipe_dimensions(
    components: list["ComponentRecord"],
    dimension_types: list[str] | None = None,
) -> list[DimensionRecord]:
    """
    Generate all requested dimension types for the pipe route.

    Args:
        components: List of ComponentRecord from PipeRoute
        dimension_types: List of dimension types to generate.
            Options: ["centerline", "pipe_length"]
            Default: both types

    Returns:
        Combined list of DimensionRecord
    """
    if dimension_types is None:
        dimension_types = ["centerline", "pipe_length"]

    dimensions: list[DimensionRecord] = []

    if "centerline" in dimension_types:
        dimensions.extend(generate_centerline_dimensions(components))

    if "pipe_length" in dimension_types:
        dimensions.extend(generate_pipe_length_dimensions(components))

    return dimensions


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def _distance_3d(
    p1: tuple[float, float, float],
    p2: tuple[float, float, float],
) -> float:
    """Calculate Euclidean distance between two 3D points."""
    return math.sqrt(
        (p2[0] - p1[0]) ** 2 +
        (p2[1] - p1[1]) ** 2 +
        (p2[2] - p1[2]) ** 2
    )


def _find_nearest_fitting(
    fittings: list["ComponentRecord"],
    point: tuple[float, float, float],
    max_distance: float = 500.0,  # mm - reasonable search radius
) -> "ComponentRecord | None":
    """Find the fitting nearest to the given point."""
    nearest = None
    nearest_dist = float('inf')

    for fitting in fittings:
        if fitting.world_position_3d is None:
            continue
        dist = _distance_3d(fitting.world_position_3d, point)
        if dist < nearest_dist and dist < max_distance:
            nearest_dist = dist
            nearest = fitting

    return nearest


def _pipe_direction_to_dim_direction(pipe_dir: str | None) -> str:
    """
    Convert pipe direction to dimension direction.

    East-West pipes get horizontal dimensions.
    North-South pipes get horizontal dimensions (in top view, vertical in front).
    Up-Down pipes get vertical dimensions.

    For isometric views, we use "aligned" to follow pipe axis.
    """
    if pipe_dir is None:
        return "horizontal"

    if pipe_dir in ("east", "west"):
        return "horizontal"
    elif pipe_dir in ("north", "south"):
        return "horizontal"  # Will be adjusted per view
    elif pipe_dir in ("up", "down"):
        return "vertical"
    else:
        return "aligned"


# =============================================================================
# DIMENSION PLACEMENT
# =============================================================================

class DimensionPlacer:
    """
    Places dimensions with collision avoidance.

    Strategy:
    1. Project dimension endpoints to 2D using the coordinate mapper
    2. Group dimensions by orientation (horizontal/vertical)
    3. Sort by position along perpendicular axis
    4. Assign to layers (closest to furthest from geometry)
    5. Check for collisions with balloons/welds
    6. Compute final SVG positions
    """

    def __init__(
        self,
        context: DimensionPlacementContext,
        style: DimensionStyle | None = None,
    ):
        self.context = context
        self.style = style or DimensionStyle()

    def place_dimensions(
        self,
        dimensions: list[DimensionRecord],
    ) -> list[DimensionRecord]:
        """
        Place all dimensions with proper offsets and collision avoidance.

        Args:
            dimensions: List of DimensionRecord to place

        Returns:
            Same list with SVG positions computed
        """
        if not dimensions:
            return dimensions

        if self.context.coord_mapper is None:
            return dimensions

        # First pass: project all points to 2D
        for dim in dimensions:
            self._project_dimension_points(dim)

        # Group by direction
        horizontal_dims = [d for d in dimensions if d.direction == "horizontal"]
        vertical_dims = [d for d in dimensions if d.direction == "vertical"]
        aligned_dims = [d for d in dimensions if d.direction == "aligned"]

        # Place each group
        self._place_dimension_group(horizontal_dims, "horizontal")
        self._place_dimension_group(vertical_dims, "vertical")
        self._place_aligned_dimensions(aligned_dims)

        return dimensions

    def _project_dimension_points(self, dim: DimensionRecord) -> None:
        """Project dimension start/end points to 2D SVG coordinates.

        Also computes geometry edge positions for extension lines by looking up
        the component bounds at each dimension endpoint.
        """
        mapper = self.context.coord_mapper
        if mapper is None:
            return

        # Project start point
        start_3d = dim.start_point.world_position_3d
        dim.start_point.svg_position_2d = mapper.world_to_svg(*start_3d)
        dim.svg_start = dim.start_point.svg_position_2d

        # Project end point
        end_3d = dim.end_point.world_position_3d
        dim.end_point.svg_position_2d = mapper.world_to_svg(*end_3d)
        dim.svg_end = dim.end_point.svg_position_2d

        # Find geometry edge positions for extension lines
        # Look up component bounds by item number reference
        dim.svg_start_geometry_edge = self._find_geometry_edge(
            dim.start_point, dim.direction
        )
        dim.svg_end_geometry_edge = self._find_geometry_edge(
            dim.end_point, dim.direction
        )

    def _find_geometry_edge(
        self,
        point: DimensionPoint,
        direction: str,
    ) -> tuple[float, float] | None:
        """Find the geometry edge position for extension line start.

        For a dimension point, finds the outer edge of the associated component's
        geometry in the direction perpendicular to the dimension line.

        This computes view-specific bounds by projecting the component's 3D
        bounding box corners to SVG coordinates using the current view's
        coordinate mapper.

        Args:
            point: The dimension point (with svg_position_2d set)
            direction: Dimension direction ("horizontal", "vertical", "aligned")

        Returns:
            SVG coordinates of the geometry edge, or None if not found
        """
        if point.svg_position_2d is None or point.component_ref is None:
            return point.svg_position_2d

        # Find the component by item number
        comp = None
        for c in self.context.components:
            if c.item_number == point.component_ref:
                comp = c
                break

        if comp is None or comp.shape is None:
            return point.svg_position_2d

        # Compute view-specific bounds by projecting 3D bounding box corners
        mapper = self.context.coord_mapper
        if mapper is None:
            return point.svg_position_2d

        bb = comp.shape.BoundingBox()

        # Get all 8 corners of the 3D bounding box
        corners_3d = [
            (bb.xmin, bb.ymin, bb.zmin),
            (bb.xmax, bb.ymin, bb.zmin),
            (bb.xmin, bb.ymax, bb.zmin),
            (bb.xmax, bb.ymax, bb.zmin),
            (bb.xmin, bb.ymin, bb.zmax),
            (bb.xmax, bb.ymin, bb.zmax),
            (bb.xmin, bb.ymax, bb.zmax),
            (bb.xmax, bb.ymax, bb.zmax),
        ]

        # Transform corners to world coordinates if component has a transform
        if comp.world_transform is not None:
            transformed_corners = []
            for cx, cy, cz in corners_3d:
                local_pt = np.array([cx, cy, cz, 1.0])
                world_pt = comp.world_transform @ local_pt
                transformed_corners.append((world_pt[0], world_pt[1], world_pt[2]))
            corners_3d = transformed_corners

        # Project all corners to SVG coordinates
        svg_corners = [mapper.world_to_svg(*corner) for corner in corners_3d]
        svg_x_coords = [c[0] for c in svg_corners]
        svg_y_coords = [c[1] for c in svg_corners]

        # Compute view-specific bounds
        min_x, max_x = min(svg_x_coords), max(svg_x_coords)
        min_y, max_y = min(svg_y_coords), max(svg_y_coords)

        px, py = point.svg_position_2d

        # For horizontal dimensions, extension lines go vertically
        # Return the Y coordinate at the geometry edge (bottom or top of bounds)
        if direction == "horizontal":
            # Return point with Y at the bottom edge of geometry (max_y in SVG coords)
            return (px, max_y)

        # For vertical dimensions, extension lines go horizontally
        # Return the X coordinate at the geometry edge (left or right of bounds)
        elif direction == "vertical":
            # Return point with X at the right edge of geometry (max_x)
            return (max_x, py)

        # For aligned dimensions, use the point as-is for now
        return point.svg_position_2d

    def _place_dimension_group(
        self,
        dims: list[DimensionRecord],
        direction: str,
    ) -> None:
        """Place a group of dimensions with the same orientation.

        Handles stacking of overlapping dimensions by tracking used zones
        and assigning higher layers to dimensions that overlap with already
        placed ones.

        Standard practice: shorter dimensions are placed closer to the geometry,
        longer dimensions are placed further out. This is achieved by sorting
        dimensions by their span length before placement.
        """
        if not dims:
            return

        # Determine offset side based on geometry bounds
        bounds = self.context.geometry_bounds_2d

        # Sort dimensions by span length (shortest first)
        # This ensures shorter dims are closer to geometry, longer ones outside
        def get_span_length(dim: DimensionRecord) -> float:
            if dim.svg_start is None or dim.svg_end is None:
                return 0.0
            if direction == "horizontal":
                return abs(dim.svg_end[0] - dim.svg_start[0])
            else:
                return abs(dim.svg_end[1] - dim.svg_start[1])

        sorted_dims = sorted(dims, key=get_span_length)

        # Track placed dimension zones: list of (min_coord, max_coord, dim_line_pos)
        # For horizontal dims: (min_x, max_x, y_pos)
        # For vertical dims: (min_y, max_y, x_pos)
        placed_zones: list[tuple[float, float, float]] = []

        for dim in sorted_dims:
            if dim.svg_start is None or dim.svg_end is None:
                continue

            # Determine which side to place the dimension
            if direction == "horizontal":
                # Check if dimension should go above or below
                mid_y = (dim.svg_start[1] + dim.svg_end[1]) / 2
                if bounds:
                    geom_mid_y = (bounds[1] + bounds[3]) / 2
                    # Place above if dimension is in upper half, below otherwise
                    offset_side = "top" if mid_y < geom_mid_y else "bottom"
                else:
                    offset_side = "bottom"

                # Get the X range this dimension spans
                dim_min_x = min(dim.svg_start[0], dim.svg_end[0])
                dim_max_x = max(dim.svg_start[0], dim.svg_end[0])
                dim_base_y = max(dim.svg_start[1], dim.svg_end[1]) if offset_side == "bottom" else min(dim.svg_start[1], dim.svg_end[1])

                # Find the required layer to avoid overlapping other dimensions
                layer = self._find_free_layer(
                    placed_zones, dim_min_x, dim_max_x, direction, offset_side
                )
                dim.layer = layer

                # Compute Y position for dimension line with stacking
                offset = self._compute_offset(dim, direction, offset_side)
                if offset_side == "top":
                    dim.svg_dim_line_pos = dim_base_y - offset
                else:
                    dim.svg_dim_line_pos = dim_base_y + offset

                # Record this dimension's zone for future collision checking
                placed_zones.append((dim_min_x, dim_max_x, dim.svg_dim_line_pos))

            else:  # vertical
                # Check if dimension should go left or right
                mid_x = (dim.svg_start[0] + dim.svg_end[0]) / 2
                if bounds:
                    geom_mid_x = (bounds[0] + bounds[2]) / 2
                    offset_side = "left" if mid_x < geom_mid_x else "right"
                else:
                    offset_side = "right"

                # Get the Y range this dimension spans
                dim_min_y = min(dim.svg_start[1], dim.svg_end[1])
                dim_max_y = max(dim.svg_start[1], dim.svg_end[1])
                dim_base_x = max(dim.svg_start[0], dim.svg_end[0]) if offset_side == "right" else min(dim.svg_start[0], dim.svg_end[0])

                # Find the required layer to avoid overlapping other dimensions
                layer = self._find_free_layer(
                    placed_zones, dim_min_y, dim_max_y, direction, offset_side
                )
                dim.layer = layer

                # Compute X position for dimension line with stacking
                offset = self._compute_offset(dim, direction, offset_side)
                if offset_side == "left":
                    dim.svg_dim_line_pos = dim_base_x - offset
                else:
                    dim.svg_dim_line_pos = dim_base_x + offset

                # Record this dimension's zone for future collision checking
                placed_zones.append((dim_min_y, dim_max_y, dim.svg_dim_line_pos))

            dim.offset_side = offset_side

    def _find_free_layer(
        self,
        placed_zones: list[tuple[float, float, float]],
        dim_min: float,
        dim_max: float,
        direction: str,
        offset_side: str,
    ) -> int:
        """Find the lowest layer where this dimension doesn't overlap others.

        Args:
            placed_zones: List of (min_coord, max_coord, line_pos) for placed dims
            dim_min: Minimum coordinate of the new dimension span
            dim_max: Maximum coordinate of the new dimension span
            direction: "horizontal" or "vertical"
            offset_side: Which side dimensions are placed on

        Returns:
            Layer number (0 = closest to geometry)
        """
        # Check if this dimension's range overlaps with any placed dimension
        # If so, we need a higher layer
        required_layer = 0
        tolerance = 2.0  # mm overlap tolerance

        for zone_min, zone_max, zone_line_pos in placed_zones:
            # Check if ranges overlap (with tolerance for slight overlaps)
            if not (dim_max < zone_min - tolerance or dim_min > zone_max + tolerance):
                # Ranges overlap, need to be at a different layer
                # Calculate what layer the existing zone is at
                # (This is approximate since we don't store layer directly)
                required_layer += 1

        return required_layer

    def _place_aligned_dimensions(self, dims: list[DimensionRecord]) -> None:
        """Place dimensions that align with pipe axes (for isometric view)."""
        for dim in dims:
            if dim.svg_start is None or dim.svg_end is None:
                continue

            # For aligned dimensions, compute perpendicular offset
            dx = dim.svg_end[0] - dim.svg_start[0]
            dy = dim.svg_end[1] - dim.svg_start[1]
            length = math.sqrt(dx * dx + dy * dy)

            if length < 0.001:
                continue

            # Perpendicular direction (rotate 90 degrees)
            perp_x = -dy / length
            perp_y = dx / length

            # Offset in perpendicular direction
            offset = self.style.min_offset_from_geometry + dim.layer * self.style.layer_spacing

            # Compute dimension line endpoints
            mid_x = (dim.svg_start[0] + dim.svg_end[0]) / 2
            mid_y = (dim.svg_start[1] + dim.svg_end[1]) / 2

            # Store the offset direction for rendering
            dim.svg_dim_line_pos = offset
            dim.offset_side = "perpendicular"

    def _compute_offset(
        self,
        dim: DimensionRecord,
        direction: str,
        offset_side: str,
    ) -> float:
        """Compute the offset distance for a dimension, avoiding collisions."""
        base_offset = self.style.min_offset_from_geometry
        layer_offset = dim.layer * self.style.layer_spacing
        offset = base_offset + layer_offset

        # Check for collisions and increase offset if needed
        max_iterations = 5
        for _ in range(max_iterations):
            if not self._has_collision(dim, direction, offset, offset_side):
                break
            offset += self.style.layer_spacing
            dim.layer += 1

        return offset

    def _has_collision(
        self,
        dim: DimensionRecord,
        direction: str,
        offset: float,
        offset_side: str,
    ) -> bool:
        """Check if dimension at given offset collides with existing annotations."""
        if dim.svg_start is None or dim.svg_end is None:
            return False

        # Compute dimension line bounds at this offset
        if direction == "horizontal":
            dim_line_y = (
                min(dim.svg_start[1], dim.svg_end[1]) - offset
                if offset_side == "top"
                else max(dim.svg_start[1], dim.svg_end[1]) + offset
            )
            dim_min_x = min(dim.svg_start[0], dim.svg_end[0])
            dim_max_x = max(dim.svg_start[0], dim.svg_end[0])

            # Check balloon collisions
            for balloon in self.context.placed_balloons:
                if hasattr(balloon, 'center') and balloon.center is not None:
                    bx, by = balloon.center
                    radius = getattr(balloon, 'radius', 4.0) + 3.0  # Add margin
                    if dim_min_x - radius < bx < dim_max_x + radius:
                        if abs(by - dim_line_y) < radius:
                            return True

        else:  # vertical
            dim_line_x = (
                min(dim.svg_start[0], dim.svg_end[0]) - offset
                if offset_side == "left"
                else max(dim.svg_start[0], dim.svg_end[0]) + offset
            )
            dim_min_y = min(dim.svg_start[1], dim.svg_end[1])
            dim_max_y = max(dim.svg_start[1], dim.svg_end[1])

            # Check balloon collisions
            for balloon in self.context.placed_balloons:
                if hasattr(balloon, 'center') and balloon.center is not None:
                    bx, by = balloon.center
                    radius = getattr(balloon, 'radius', 4.0) + 3.0
                    if dim_min_y - radius < by < dim_max_y + radius:
                        if abs(bx - dim_line_x) < radius:
                            return True

        return False


# =============================================================================
# SVG RENDERING
# =============================================================================

def render_dimension_svg(
    dim: DimensionRecord,
    style: DimensionStyle | None = None,
) -> str:
    """
    Render a single dimension as SVG.

    Generates:
    1. Extension lines from geometry to dimension line
    2. Dimension line with arrows
    3. Dimension value text

    Args:
        dim: DimensionRecord with SVG positions computed
        style: DimensionStyle configuration

    Returns:
        SVG string for the complete dimension
    """
    if style is None:
        style = DimensionStyle()

    if dim.svg_start is None or dim.svg_end is None:
        return ""

    parts: list[str] = []

    # Render based on direction
    if dim.direction == "horizontal":
        parts.append(_render_horizontal_dimension(dim, style))
    elif dim.direction == "vertical":
        parts.append(_render_vertical_dimension(dim, style))
    else:  # aligned
        parts.append(_render_aligned_dimension(dim, style))

    return "\n".join(parts)


def _render_horizontal_dimension(
    dim: DimensionRecord,
    style: DimensionStyle,
) -> str:
    """Render a horizontal dimension with ASME-style formatting.

    Features:
    - Extension lines start from geometry edge (not centerline) with small gap
    - Extension lines with overshoot past dimension line
    - Dimension line broken at center with text in the gap
    - Arrows pointing outward from text (standard ASME style)
    """
    if dim.svg_start is None or dim.svg_end is None or dim.svg_dim_line_pos is None:
        return ""

    parts: list[str] = []
    x1, y1 = dim.svg_start
    x2, y2 = dim.svg_end
    dim_y = dim.svg_dim_line_pos

    # Get geometry edge positions (where extension lines should start from)
    # These are the outer edges of the component geometry
    geom_edge1 = dim.svg_start_geometry_edge
    geom_edge2 = dim.svg_end_geometry_edge

    # Fall back to centerline positions if geometry edges not available
    geom_y1 = geom_edge1[1] if geom_edge1 else y1
    geom_y2 = geom_edge2[1] if geom_edge2 else y2

    # Ensure x1 < x2 (and swap geometry edges accordingly)
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        geom_y1, geom_y2 = geom_y2, geom_y1

    # Extension lines
    gap = style.extension_line_gap
    overshoot = style.extension_line_overshoot

    # Extension line starts with a small gap from the GEOMETRY EDGE (not centerline)
    # For "bottom" offset: dimension line is BELOW geometry, so extension goes DOWN
    # For "top" offset: dimension line is ABOVE geometry, so extension goes UP
    if dim.offset_side == "bottom":
        # Extension line starts just below geometry edge, goes to dimension line
        ext1_start_y = geom_y1 + gap
        ext1_end_y = dim_y + overshoot
        ext2_start_y = geom_y2 + gap
        ext2_end_y = dim_y + overshoot
    else:  # top
        # Extension line starts just above geometry edge, goes to dimension line
        ext1_start_y = geom_y1 - gap
        ext1_end_y = dim_y - overshoot
        ext2_start_y = geom_y2 - gap
        ext2_end_y = dim_y - overshoot

    # Left extension line
    parts.append(
        f'<line x1="{x1:.2f}" y1="{ext1_start_y:.2f}" '
        f'x2="{x1:.2f}" y2="{ext1_end_y:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )

    # Right extension line
    parts.append(
        f'<line x1="{x2:.2f}" y1="{ext2_start_y:.2f}" '
        f'x2="{x2:.2f}" y2="{ext2_end_y:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )

    # Calculate text width (approximate) and gap for dimension line break
    text_width_approx = len(dim.display_value) * style.font_size * 0.6
    text_gap = style.text_gap
    text_center_x = (x1 + x2) / 2
    gap_left = text_center_x - text_width_approx / 2 - text_gap
    gap_right = text_center_x + text_width_approx / 2 + text_gap

    # Dimension line - split into two segments with gap for text
    # Left segment: from left extension line to gap
    parts.append(
        f'<line x1="{x1:.2f}" y1="{dim_y:.2f}" '
        f'x2="{gap_left:.2f}" y2="{dim_y:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )
    # Right segment: from gap to right extension line
    parts.append(
        f'<line x1="{gap_right:.2f}" y1="{dim_y:.2f}" '
        f'x2="{x2:.2f}" y2="{dim_y:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )

    # Arrows pointing OUTWARD (away from text) - standard ASME style
    # Left arrow points left, right arrow points right
    parts.append(_render_arrow_svg(x1, dim_y, "left", style))
    parts.append(_render_arrow_svg(x2, dim_y, "right", style))

    # Dimension text - centered on dimension line
    text_x = text_center_x
    # Position text vertically centered on the dimension line
    text_y = dim_y + style.font_size * 0.35  # Baseline adjustment for vertical centering

    parts.append(
        f'<text x="{text_x:.2f}" y="{text_y:.2f}" '
        f'text-anchor="middle" '
        f'font-family="{style.font_family}" '
        f'font-size="{style.font_size}" '
        f'font-weight="{style.font_weight}">'
        f'{dim.display_value}</text>'
    )

    return "\n".join(parts)


def _render_vertical_dimension(
    dim: DimensionRecord,
    style: DimensionStyle,
) -> str:
    """Render a vertical dimension with ASME-style formatting.

    Features:
    - Extension lines start from geometry edge (not centerline) with small gap
    - Extension lines with overshoot past dimension line
    - Dimension line broken at center with text in the gap
    - Arrows pointing outward from text (standard ASME style)
    - Text rotated 90 degrees for vertical reading
    """
    if dim.svg_start is None or dim.svg_end is None or dim.svg_dim_line_pos is None:
        return ""

    parts: list[str] = []
    x1, y1 = dim.svg_start
    x2, y2 = dim.svg_end
    dim_x = dim.svg_dim_line_pos

    # Get geometry edge positions (where extension lines should start from)
    geom_edge1 = dim.svg_start_geometry_edge
    geom_edge2 = dim.svg_end_geometry_edge

    # Fall back to centerline positions if geometry edges not available
    geom_x1 = geom_edge1[0] if geom_edge1 else x1
    geom_x2 = geom_edge2[0] if geom_edge2 else x2

    # Ensure y1 < y2 (y1 is top in SVG coordinates)
    if y1 > y2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        geom_x1, geom_x2 = geom_x2, geom_x1

    # Extension lines
    gap = style.extension_line_gap
    overshoot = style.extension_line_overshoot

    # Extension line starts with a small gap from the GEOMETRY EDGE (not centerline)
    # For "right" offset: dimension line is to the RIGHT of geometry
    # For "left" offset: dimension line is to the LEFT of geometry
    if dim.offset_side == "right":
        # Extension line starts just right of geometry edge, goes to dimension line
        ext1_start_x = geom_x1 + gap
        ext1_end_x = dim_x + overshoot
        ext2_start_x = geom_x2 + gap
        ext2_end_x = dim_x + overshoot
    else:  # left
        # Extension line starts just left of geometry edge, goes to dimension line
        ext1_start_x = geom_x1 - gap
        ext1_end_x = dim_x - overshoot
        ext2_start_x = geom_x2 - gap
        ext2_end_x = dim_x - overshoot

    # Top extension line (at y1)
    parts.append(
        f'<line x1="{ext1_start_x:.2f}" y1="{y1:.2f}" '
        f'x2="{ext1_end_x:.2f}" y2="{y1:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )

    # Bottom extension line (at y2)
    parts.append(
        f'<line x1="{ext2_start_x:.2f}" y1="{y2:.2f}" '
        f'x2="{ext2_end_x:.2f}" y2="{y2:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )

    # Calculate text width (approximate) and gap for dimension line break
    text_width_approx = len(dim.display_value) * style.font_size * 0.6
    text_gap = style.text_gap
    text_center_y = (y1 + y2) / 2
    gap_top = text_center_y - text_width_approx / 2 - text_gap
    gap_bottom = text_center_y + text_width_approx / 2 + text_gap

    # Dimension line - split into two segments with gap for text
    # Top segment: from top extension line to gap
    parts.append(
        f'<line x1="{dim_x:.2f}" y1="{y1:.2f}" '
        f'x2="{dim_x:.2f}" y2="{gap_top:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )
    # Bottom segment: from gap to bottom extension line
    parts.append(
        f'<line x1="{dim_x:.2f}" y1="{gap_bottom:.2f}" '
        f'x2="{dim_x:.2f}" y2="{y2:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )

    # Arrows pointing OUTWARD (away from text) - standard ASME style
    # Top arrow points up, bottom arrow points down
    parts.append(_render_arrow_svg(dim_x, y1, "up", style))
    parts.append(_render_arrow_svg(dim_x, y2, "down", style))

    # Dimension text - centered on dimension line with rotation
    text_x = dim_x + style.font_size * 0.35  # Baseline adjustment after rotation
    text_y = text_center_y

    # Rotate text for vertical reading (always 90 degrees, text reads bottom-to-top)
    parts.append(
        f'<text x="{text_x:.2f}" y="{text_y:.2f}" '
        f'text-anchor="middle" '
        f'font-family="{style.font_family}" '
        f'font-size="{style.font_size}" '
        f'font-weight="{style.font_weight}" '
        f'transform="rotate(-90, {text_x:.2f}, {text_y:.2f})">'
        f'{dim.display_value}</text>'
    )

    return "\n".join(parts)


def _render_aligned_dimension(
    dim: DimensionRecord,
    style: DimensionStyle,
) -> str:
    """Render a dimension aligned with an angled pipe axis.

    Features:
    - Gap between geometry and extension line start
    - Extension lines with overshoot past dimension line
    - Dimension line broken at center with text in the gap
    - Arrows pointing inward toward text
    - Text rotated to align with dimension line
    """
    if dim.svg_start is None or dim.svg_end is None:
        return ""

    parts: list[str] = []
    x1, y1 = dim.svg_start
    x2, y2 = dim.svg_end

    # Calculate the angle and length
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx * dx + dy * dy)

    if length < 0.001:
        return ""

    # Unit vectors along and perpendicular to dimension
    ux, uy = dx / length, dy / length
    px, py = -uy, ux  # Perpendicular (rotated 90 degrees CCW)

    # Offset dimension line perpendicular to pipe axis
    offset = dim.svg_dim_line_pos or style.min_offset_from_geometry

    # Dimension line endpoints
    dl_x1 = x1 + px * offset
    dl_y1 = y1 + py * offset
    dl_x2 = x2 + px * offset
    dl_y2 = y2 + py * offset

    # Extension lines
    gap = style.extension_line_gap
    overshoot = style.extension_line_overshoot

    # Start extension line
    ext1_start_x = x1 + px * gap
    ext1_start_y = y1 + py * gap
    ext1_end_x = dl_x1 + px * overshoot
    ext1_end_y = dl_y1 + py * overshoot
    parts.append(
        f'<line x1="{ext1_start_x:.2f}" y1="{ext1_start_y:.2f}" '
        f'x2="{ext1_end_x:.2f}" y2="{ext1_end_y:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )

    # End extension line
    ext2_start_x = x2 + px * gap
    ext2_start_y = y2 + py * gap
    ext2_end_x = dl_x2 + px * overshoot
    ext2_end_y = dl_y2 + py * overshoot
    parts.append(
        f'<line x1="{ext2_start_x:.2f}" y1="{ext2_start_y:.2f}" '
        f'x2="{ext2_end_x:.2f}" y2="{ext2_end_y:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )

    # Calculate text width and gap for dimension line break
    text_width_approx = len(dim.display_value) * style.font_size * 0.6
    text_gap = style.text_gap
    half_gap = (text_width_approx / 2 + text_gap)

    # Dimension line center
    dl_cx = (dl_x1 + dl_x2) / 2
    dl_cy = (dl_y1 + dl_y2) / 2

    # Gap points along dimension line (offset from center along line direction)
    gap_x1 = dl_cx - ux * half_gap
    gap_y1 = dl_cy - uy * half_gap
    gap_x2 = dl_cx + ux * half_gap
    gap_y2 = dl_cy + uy * half_gap

    # Dimension line - split into two segments with gap for text
    # First segment: from start to gap
    parts.append(
        f'<line x1="{dl_x1:.2f}" y1="{dl_y1:.2f}" '
        f'x2="{gap_x1:.2f}" y2="{gap_y1:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )
    # Second segment: from gap to end
    parts.append(
        f'<line x1="{gap_x2:.2f}" y1="{gap_y2:.2f}" '
        f'x2="{dl_x2:.2f}" y2="{dl_y2:.2f}" '
        f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
    )

    # Arrows pointing OUTWARD (away from text) - standard ASME style
    # First arrow points away from the dimension line (opposite direction)
    # Second arrow points away in the other direction
    angle = math.degrees(math.atan2(dy, dx))
    parts.append(_render_arrow_svg(dl_x1, dl_y1, angle + 180, style, is_angle=True))
    parts.append(_render_arrow_svg(dl_x2, dl_y2, angle, style, is_angle=True))

    # Dimension text - centered on dimension line
    text_x = dl_cx
    text_y = dl_cy

    # Rotate text to be readable (avoid upside-down text)
    text_angle = angle
    if text_angle > 90:
        text_angle -= 180
    elif text_angle < -90:
        text_angle += 180

    parts.append(
        f'<text x="{text_x:.2f}" y="{text_y:.2f}" '
        f'text-anchor="middle" '
        f'dominant-baseline="middle" '
        f'font-family="{style.font_family}" '
        f'font-size="{style.font_size}" '
        f'font-weight="{style.font_weight}" '
        f'transform="rotate({text_angle:.1f}, {text_x:.2f}, {text_y:.2f})">'
        f'{dim.display_value}</text>'
    )

    return "\n".join(parts)


def _render_arrow_svg(
    x: float,
    y: float,
    direction: str | float,
    style: DimensionStyle,
    is_angle: bool = False,
) -> str:
    """
    Render an arrowhead at the specified position.

    Args:
        x, y: Arrow tip position
        direction: Either a string ("left", "right", "up", "down") or angle in degrees
        style: DimensionStyle configuration
        is_angle: If True, direction is interpreted as an angle in degrees

    Returns:
        SVG path for the arrowhead
    """
    al = style.arrow_length
    aw = style.arrow_width / 2

    if is_angle:
        # Direction is an angle in degrees
        angle_rad = math.radians(direction)
        # Arrow points in the direction of the angle
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
    else:
        # Direction is a string
        directions = {
            "left": (-1, 0),
            "right": (1, 0),
            "up": (0, -1),
            "down": (0, 1),
        }
        dx, dy = directions.get(direction, (1, 0))

    # Calculate arrow base points perpendicular to direction
    px, py = -dy, dx  # Perpendicular direction

    # Arrow tip is at (x, y), base is at (x - dx*al, y - dy*al)
    base_x = x - dx * al
    base_y = y - dy * al

    # Two base corners
    b1_x = base_x + px * aw
    b1_y = base_y + py * aw
    b2_x = base_x - px * aw
    b2_y = base_y - py * aw

    if style.arrow_style == "filled":
        return (
            f'<polygon points="{x:.2f},{y:.2f} {b1_x:.2f},{b1_y:.2f} {b2_x:.2f},{b2_y:.2f}" '
            f'fill="{style.line_color}" stroke="none"/>'
        )
    elif style.arrow_style == "tick":
        # 45-degree tick mark (architectural style)
        tick_len = al * 0.7
        return (
            f'<line x1="{x - tick_len:.2f}" y1="{y - tick_len:.2f}" '
            f'x2="{x + tick_len:.2f}" y2="{y + tick_len:.2f}" '
            f'stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
        )
    else:  # open
        return (
            f'<polyline points="{b1_x:.2f},{b1_y:.2f} {x:.2f},{y:.2f} {b2_x:.2f},{b2_y:.2f}" '
            f'fill="none" stroke="{style.line_color}" stroke-width="{style.line_stroke_width}"/>'
        )


def render_all_dimensions_svg(
    dimensions: list[DimensionRecord],
    style: DimensionStyle | None = None,
    group_id: str = "dimensions",
) -> str:
    """
    Render all dimensions as an SVG group.

    Args:
        dimensions: List of DimensionRecord with SVG positions computed
        style: DimensionStyle configuration
        group_id: ID for the SVG group element

    Returns:
        SVG string containing all dimensions
    """
    if style is None:
        style = DimensionStyle()

    if not dimensions:
        return ""

    parts: list[str] = [f'<g id="{group_id}">']

    for dim in dimensions:
        dim_svg = render_dimension_svg(dim, style)
        if dim_svg:
            parts.append(f'  <!-- Dim {dim.dimension_id}: {dim.dimension_type} {dim.display_value} -->')
            parts.append("  " + dim_svg.replace("\n", "\n  "))

    parts.append("</g>")

    return "\n".join(parts)
