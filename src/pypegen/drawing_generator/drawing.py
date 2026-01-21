"""
Main piping drawing generator.

Generates piping fabrication drawings from CadQuery pipe assemblies.
"""

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import cadquery as cq
import numpy as np
from cadquery.occ_impl.exporters.svg import getSVG

# Make PDF export optional using svglib + reportlab (pure Python, no Cairo needed)
try:
    from svglib.svglib import svg2rlg
    from reportlab.graphics import renderPDF
    SVGLIB_AVAILABLE = True
except ImportError:
    SVGLIB_AVAILABLE = False

from .axis_indicator import create_axis_indicator_svg
from .bom import (
    AnnotationPlacementContext,
    BOMTable,
    ComponentRecord,
    CoordinateMapper,
    TopologyBalloonPlacer,
    WeldRecord,
    _leader_crosses_pipe,
    aggregate_components_to_bom,
    create_weld_markers,
    get_bounding_box_corners,
    optimize_annotation_positions,
    render_all_balloons_svg,
    render_weld_markers_svg,
)
from .constants import (
    BOM_WIDTH,
    BOM_X,
    BOM_Y,
    BORDER_COLOR,
    BORDER_WIDTH,
    DRAWING_AREA_HEIGHT,
    DRAWING_AREA_WIDTH,
    DRAWING_AREA_X,
    DRAWING_AREA_Y,
    HIDDEN_COLOR,
    PIPING_ISO_DIRECTION,
    ROTATE_MODEL_FOR_PROJECTION,
    SHEET_HEIGHT_MM,
    SHEET_WIDTH_MM,
    VISIBLE_COLOR,
    VISIBLE_STROKE_WIDTH,
)
from .title_block import TitleBlock, TitleBlockInfo
from .view_area import ViewArea

# Import transform utility for applying world transforms to shapes
from ..fittings.pipe_fittings import apply_transform_to_shape


def load_step_file(filepath: str) -> cq.Shape:
    """Load a STEP file and return as CadQuery shape."""
    result = cq.importers.importStep(filepath)
    shape = result.val()
    # Cast to Shape - importStep always returns a Compound/Shape
    assert isinstance(shape, cq.Shape)
    return shape


def strip_svg_namespaces(svg_content: str) -> str:
    """Remove namespace prefixes from SVG content."""
    cleaned = re.sub(r'<ns\d+:', '<', svg_content)
    cleaned = re.sub(r'</ns\d+:', '</', cleaned)
    cleaned = re.sub(r'\s*xmlns:ns\d+="[^"]*"', '', cleaned)
    return cleaned


def _world_normal_to_direction_label(normal: tuple[float, float, float]) -> str:
    """Map a world normal vector to the closest cardinal direction label.

    Args:
        normal: (nx, ny, nz) world normal direction

    Returns:
        Single character label: N, S, E, W, U, or D
    """
    nx, ny, nz = normal

    # Find the axis with the largest magnitude
    abs_vals = [abs(nx), abs(ny), abs(nz)]
    max_idx = abs_vals.index(max(abs_vals))

    if max_idx == 0:  # X-axis dominant
        return "E" if nx > 0 else "W"
    elif max_idx == 1:  # Y-axis dominant
        return "N" if ny > 0 else "S"
    else:  # Z-axis dominant
        return "U" if nz > 0 else "D"


def _screen_direction_to_compass_label(dx: float, dy: float) -> str:
    """Map a 2D screen direction to the closest compass direction label.

    In the isometric projection used for piping drawings:
    - East (+X world): screen direction (0.866, 0.5) - lower right at 30° below horizontal
    - North (+Y world): screen direction (0.866, -0.5) - upper right at 30° above horizontal
    - Up (+Z world): screen direction (0.0, -1.0) - straight up

    Args:
        dx: Screen X direction (positive = right)
        dy: Screen Y direction (positive = down, SVG convention)

    Returns:
        Single character label: N, S, E, W, U, or D
    """
    # Normalize the direction
    length = math.sqrt(dx * dx + dy * dy)
    if length < 1e-6:
        return "?"
    dx, dy = dx / length, dy / length

    # Compass directions in screen coordinates (from WORLD_DIR_TO_SCREEN_OFFSET)
    # Note: SVG Y increases downward, so "up" has negative dy
    compass_dirs = {
        "E": (0.866, 0.5),    # +X world: right and down (30° below horizontal)
        "W": (-0.866, -0.5),  # -X world: left and up
        "N": (0.866, -0.5),   # +Y world: right and up (30° above horizontal)
        "S": (-0.866, 0.5),   # -Y world: left and down
        "U": (0.0, -1.0),     # +Z world: straight up
        "D": (0.0, 1.0),      # -Z world: straight down
    }

    # Find the compass direction with the highest dot product
    best_label = "?"
    best_dot = -2.0
    for label, (cx, cy) in compass_dirs.items():
        dot = dx * cx + dy * cy
        if dot > best_dot:
            best_dot = dot
            best_label = label

    return best_label


def is_point_in_bounds(
    x: float,
    y: float,
    margin: float,
    view_area: ViewArea
) -> bool:
    """
    Check if a point with margin is within the view area bounds.

    Args:
        x: X coordinate of the point
        y: Y coordinate of the point
        margin: Margin around the point that must also be within bounds
        view_area: The view area to check against

    Returns:
        True if the point (with margin) is within bounds
    """
    return (x - margin >= view_area.x and
            x + margin <= view_area.x + view_area.width and
            y - margin >= view_area.y and
            y + margin <= view_area.y + view_area.height)


def find_svg_geometry_group(root: ET.Element) -> ET.Element | None:
    """
    Find the main geometry group in an SVG element tree.

    CadQuery generates SVG with a <g> element that contains a scale transform.
    This function finds that group.

    Args:
        root: The root element of the parsed SVG

    Returns:
        The geometry group element, or None if not found
    """
    for child in root:
        tag = child.tag.split('}')[-1] if '}' in child.tag else child.tag
        if tag == 'g':
            transform = child.get('transform', '')
            if 'scale' in transform:
                return child
    return None


def generate_view_svg(
    shape: cq.Shape,
    direction: tuple[float, float, float],
    width: float = 600,
    height: float = 400,
    show_hidden: bool = True,
) -> str:
    """Generate SVG of shape from a specific view direction."""
    opts = {
        "width": width,
        "height": height,
        "marginLeft": 10,
        "marginTop": 10,
        "projectionDir": direction,
        "showAxes": False,
        "strokeWidth": VISIBLE_STROKE_WIDTH,
        "strokeColor": (0, 0, 0),
        "hiddenColor": (136, 136, 136),
        "showHidden": show_hidden,
    }
    return getSVG(shape, opts)


def compute_isometric_projection_matrix() -> list[list[float]]:
    """
    Compute the 2D projection matrix for standard piping isometric view.

    For isometric projection with camera at (1,1,1) looking at origin:
    - X axis (East) projects to lower-right at 30 degrees below horizontal
    - Y axis (North) projects to upper-right at 30 degrees above horizontal
    - Z axis (Up) projects straight up

    After rotating model -90 degrees around X (to fix Y/Z swap):
    - Original Y becomes -Z, Original Z becomes Y

    Returns a 2x3 matrix that transforms (x, y, z) to (screen_x, screen_y)
    """
    # Standard isometric angles
    angle_30 = math.radians(30)

    # After model rotation -90 around X:
    # New coords: (x', y', z') = (x, -z, y)
    # Then project with (1,1,1) isometric:
    # screen_x = x' * cos(30) + y' * cos(30) = cos(30) * (x - z)
    # screen_y = -z' + (x' - y') * sin(30) = -y + (x + z) * sin(30)

    cos30 = math.cos(angle_30)
    sin30 = math.sin(angle_30)

    # Matrix: [screen_x] = [a b c] [x]
    #         [screen_y]   [d e f] [y]
    #                              [z]
    # screen_x = cos30 * x + 0 * y - cos30 * z
    # screen_y = sin30 * x - 1 * y + sin30 * z

    return [
        [cos30, 0, -cos30],      # screen_x coefficients
        [sin30, -1, sin30]       # screen_y coefficients (negative because SVG Y is down)
    ]


def project_point_isometric(x: float, y: float, z: float,
                            matrix: list[list[float]]) -> tuple[float, float]:
    """Project a 3D point to 2D using the isometric projection matrix."""
    screen_x = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z
    screen_y = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z
    return (screen_x, screen_y)


def get_projected_bounds(shape: cq.Shape) -> tuple[float, float, float, float, float, float]:
    """
    Calculate the 2D bounding box of the shape in isometric projection.

    Returns: (min_x, min_y, max_x, max_y, center_x, center_y) in projection units
    """
    bb = shape.BoundingBox()
    corners = get_bounding_box_corners(bb)

    # Project each corner
    matrix = compute_isometric_projection_matrix()
    projected = [project_point_isometric(x, y, z, matrix) for x, y, z in corners]

    # Find 2D bounds
    xs = [p[0] for p in projected]
    ys = [p[1] for p in projected]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    return (min_x, min_y, max_x, max_y, center_x, center_y)


def parse_svg_path_coords(path_d: str, interpolate_lines: bool = True) -> list[tuple[float, float]]:
    """
    Parse an SVG path 'd' attribute and return all coordinate points.

    Handles M, L, C, A commands (the main ones CadQuery uses).
    For line segments (M...L...), interpolates intermediate points.
    Returns list of (x, y) coordinate tuples.

    Args:
        path_d: SVG path 'd' attribute string
        interpolate_lines: If True, add intermediate points along line segments
    """
    if not path_d:
        return []

    # Extract all numeric coordinates from the path
    numbers = re.findall(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?', path_d)
    if len(numbers) < 2:
        return []

    # Parse as x,y pairs
    raw_coords = []
    i = 0
    while i < len(numbers) - 1:
        try:
            x = float(numbers[i])
            y = float(numbers[i + 1])
            raw_coords.append((x, y))
            i += 2
        except (ValueError, IndexError):
            i += 1

    # If this is a simple line segment (2 points), interpolate
    if interpolate_lines and len(raw_coords) == 2:
        x1, y1 = raw_coords[0]
        x2, y2 = raw_coords[1]

        # Calculate line length
        length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # Interpolate if line is long enough
        if length > 10:  # Only interpolate lines > 10 units
            num_segments = max(2, int(length / 5))  # ~5 units between points
            coords = []
            for j in range(num_segments + 1):
                t = j / num_segments
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                coords.append((x, y))
            return coords

    return raw_coords


def parse_svg_path_bounds(path_d: str) -> tuple[float, float, float, float]:
    """
    Parse an SVG path 'd' attribute and return bounding box (min_x, min_y, max_x, max_y).
    """
    coords = parse_svg_path_coords(path_d)
    if not coords:
        return (0, 0, 0, 0)

    xs = [c[0] for c in coords]
    ys = [c[1] for c in coords]

    return (min(xs), min(ys), max(xs), max(ys))


def get_svg_geometry_points(svg_group: ET.Element) -> list[tuple[float, float]]:
    """
    Extract all coordinate points from SVG geometry in a group.

    Returns list of (x, y) points from all paths, circles, lines, ellipses.
    """
    all_points = []

    def process_element(elem):
        tag = elem.tag.split('}')[-1] if '}' in elem.tag else elem.tag

        if tag == 'path':
            d = elem.get('d', '')
            if d:
                all_points.extend(parse_svg_path_coords(d))

        elif tag == 'circle':
            cx = float(elem.get('cx', 0))
            cy = float(elem.get('cy', 0))
            r = float(elem.get('r', 0))
            # Sample points around the circle
            import math
            for angle in range(0, 360, 10):
                rad = math.radians(angle)
                all_points.append((cx + r * math.cos(rad), cy + r * math.sin(rad)))

        elif tag == 'line':
            x1 = float(elem.get('x1', 0))
            y1 = float(elem.get('y1', 0))
            x2 = float(elem.get('x2', 0))
            y2 = float(elem.get('y2', 0))
            all_points.append((x1, y1))
            all_points.append((x2, y2))

        elif tag == 'ellipse':
            cx = float(elem.get('cx', 0))
            cy = float(elem.get('cy', 0))
            rx = float(elem.get('rx', 0))
            ry = float(elem.get('ry', 0))
            # Sample points around the ellipse
            import math
            for angle in range(0, 360, 10):
                rad = math.radians(angle)
                all_points.append((cx + rx * math.cos(rad), cy + ry * math.sin(rad)))

        # Recurse into groups
        for child in elem:
            process_element(child)

    process_element(svg_group)
    return all_points


def find_edge_point_in_direction(
    points: list[tuple[float, float]],
    direction: tuple[float, float]
) -> tuple[float, float] | None:
    """
    Find the point that is furthest in a given direction.

    Args:
        points: List of (x, y) coordinate points
        direction: (dx, dy) direction vector (will be normalized)

    Returns:
        The point furthest in the given direction, or None if no points
    """
    if not points:
        return None

    # Normalize direction
    import math
    length = math.sqrt(direction[0]**2 + direction[1]**2)
    if length < 1e-6:
        return None
    dx, dy = direction[0] / length, direction[1] / length

    # Find point with maximum dot product with direction
    best_point = None
    best_dot = float('-inf')

    for x, y in points:
        dot = x * dx + y * dy
        if dot > best_dot:
            best_dot = dot
            best_point = (x, y)

    return best_point


def get_svg_geometry_bounds(svg_group: ET.Element) -> tuple[float, float, float, float]:
    """
    Find the bounding box of all geometry in an SVG group.

    Returns: (min_x, min_y, max_x, max_y)
    """
    points = get_svg_geometry_points(svg_group)

    if not points:
        return (0, 0, 0, 0)

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    return (min(xs), min(ys), max(xs), max(ys))


@dataclass
class PipingDrawing:
    """
    Generates a piping fabrication drawing from a 3D pipe assembly.

    Focuses on isometric view which is standard for piping drawings.

    Attributes:
        shape: CadQuery shape to render (either this or step_file required)
        step_file: Path to STEP file to load (either this or shape required)
        title: Drawing title
        drawing_number: Drawing number/identifier
        revision: Revision number
        scale_text: Scale indicator text
        description: Verbose description of the drawing
        material: Material specification (for BOM)
        project_number: Project number
        fund_number: Fund number
        drawn_by: Name of person who created drawing
        drawn_date: Date drawing was created
        checked_by: Name of person who checked drawing
        checked_date: Date drawing was checked
        show_pe_stamp: Whether to show PE stamp area
        show_hidden: Whether to show hidden lines
        components: List of ComponentRecord for BOM and balloon generation
        show_bom: Whether to show the bill of materials table
        show_balloons: Whether to show balloon leaders
    """
    shape: cq.Shape | None = None
    step_file: str | None = None
    title: str = "PIPING ASSEMBLY"
    drawing_number: str = "SPOOL-001"
    revision: str = "0"
    scale_text: str = "NOT TO SCALE"
    description: str = ""
    material: str = ""  # For BOM only
    project_number: str = ""
    fund_number: str = ""
    drawn_by: str = ""
    drawn_date: str = ""
    checked_by: str = ""
    checked_date: str = ""
    company_name: str = ""  # Company name for title block
    company_logo_path: str | None = None  # Path to company logo image
    show_hidden: bool = True
    debug_center: bool = False  # Show crosshair at view area center
    components: list[ComponentRecord] = field(default_factory=list)
    welds: list[WeldRecord] = field(default_factory=list)
    fittings: list = field(default_factory=list)  # List[(Fitting, world_transform)] for debug visualization
    show_bom: bool = True
    show_balloons: bool = True
    show_welds: bool = True
    show_debug: bool = True  # Show debug markers (component centers, bounds, attachment points)

    # Internal state - initialized in __post_init__
    _svg_content: str = field(default="", repr=False)
    _model_view_area: ViewArea = field(init=False, repr=False)
    _title_block: TitleBlock = field(init=False, repr=False)

    # Coordinate mapper - single source of truth for 3D→2D transformations
    _coord_mapper: CoordinateMapper = field(init=False, repr=False)

    # Legacy projection state (kept for debugging, will be removed)
    _fit_scale: float = field(default=1.0, repr=False)
    _geometry_center: tuple[float, float] = field(default=(0.0, 0.0), repr=False)
    _cq_scale_x: float = field(default=1.0, repr=False)
    _cq_scale_y: float = field(default=-1.0, repr=False)
    _cq_trans_x: float = field(default=0.0, repr=False)
    _cq_trans_y: float = field(default=0.0, repr=False)
    _raw_geom_center: tuple[float, float] = field(default=(0.0, 0.0), repr=False)
    _raw_geom_bounds: tuple[float, float, float, float] = field(default=(0.0, 0.0, 0.0, 0.0), repr=False)
    _proj_to_cq_scale: float = field(default=1.0, repr=False)
    _proj_to_cq_offset_x: float = field(default=0.0, repr=False)
    _proj_to_cq_offset_y: float = field(default=0.0, repr=False)

    def __post_init__(self):
        """Initialize shape and layout areas."""
        # Load shape from STEP file if provided
        if self.shape is None and self.step_file:
            self.shape = load_step_file(self.step_file)
        elif self.shape is None:
            raise ValueError("Either shape or step_file must be provided")

        # Initialize the model view area (main drawing area)
        self._model_view_area = ViewArea(
            x=DRAWING_AREA_X,
            y=DRAWING_AREA_Y,
            width=DRAWING_AREA_WIDTH,
            height=DRAWING_AREA_HEIGHT
        )

        # Initialize title block
        self._title_block = TitleBlock(
            info=TitleBlockInfo(
                title=self.title,
                drawing_number=self.drawing_number,
                revision=self.revision,
                scale_text=self.scale_text,
                description=self.description,
                project_number=self.project_number,
                fund_number=self.fund_number,
                drawn_by=self.drawn_by,
                drawn_date=self.drawn_date,
                checked_by=self.checked_by,
                checked_date=self.checked_date,
                company_name=self.company_name,
                company_logo_path=self.company_logo_path,
            )
        )

        # Initialize coordinate mapper
        self._coord_mapper = CoordinateMapper(
            apply_model_rotation=ROTATE_MODEL_FOR_PROJECTION
        )

        # Compute and store 3D world positions for all components
        for comp in self.components:
            comp.set_world_position_from_shape()

    @property
    def model_view_area(self) -> ViewArea:
        """The main view area for the CAD model and coordinate indicator."""
        return self._model_view_area

    def _create_border(self) -> str:
        """Create the drawing border using the model view area."""
        return f'''
        <!-- Drawing Border -->
        {self._model_view_area.svg_rect(stroke=BORDER_COLOR, stroke_width=BORDER_WIDTH)}
        '''

    def _create_center_crosshair(self) -> str:
        """Create a debug crosshair at the view area center."""
        if not self.debug_center:
            return ""

        cx, cy = self._model_view_area.center
        size = 10  # Crosshair arm length

        return f'''
        <!-- Debug: View Area Center Crosshair -->
        <g id="debug-center" stroke="#FF0000" stroke-width="0.3">
            <line x1="{cx - size}" y1="{cy}" x2="{cx + size}" y2="{cy}"/>
            <line x1="{cx}" y1="{cy - size}" x2="{cx}" y2="{cy + size}"/>
            <circle cx="{cx}" cy="{cy}" r="2" fill="none"/>
            <text x="{cx + 3}" y="{cy - 3}" font-size="2" fill="#FF0000">
                CENTER ({cx:.1f}, {cy:.1f})
            </text>
        </g>
        '''

    def _create_isometric_view(self) -> str:
        """Create the main isometric view of the pipe assembly."""
        if self.shape is None:
            return "<!-- No shape to render -->"

        area = self._model_view_area

        # Rotate model to fix Y/Z swap in CadQuery's projection
        shape_to_render: cq.Shape = self.shape
        if ROTATE_MODEL_FOR_PROJECTION:
            shape_to_render = self.shape.rotate((0, 0, 0), (1, 0, 0), -90)

        # Generate SVG from CadQuery
        raw_svg = generate_view_svg(
            shape_to_render,
            PIPING_ISO_DIRECTION,
            width=1000,
            height=800,
            show_hidden=self.show_hidden
        )

        # Parse the SVG
        try:
            root = ET.fromstring(raw_svg)
        except ET.ParseError:
            return "<!-- Failed to parse SVG -->"

        _svg_width = float(root.get('width', '1000').replace('px', ''))
        _svg_height = float(root.get('height', '800').replace('px', ''))

        # Find the main geometry group with transform
        main_group = find_svg_geometry_group(root)
        if main_group is None:
            return "<!-- No geometry group found -->"

        transform_str = main_group.get('transform', '')

        # Parse the actual geometry bounds from SVG paths
        # These are in the model coordinate space (before CadQuery's transform)
        geom_min_x, geom_min_y, geom_max_x, geom_max_y = get_svg_geometry_bounds(main_group)
        geom_width = geom_max_x - geom_min_x
        geom_height = geom_max_y - geom_min_y
        geom_center_x = (geom_min_x + geom_max_x) / 2
        geom_center_y = (geom_min_y + geom_max_y) / 2

        # Parse CadQuery's transform to get scale factors
        scale_x, scale_y = 1.0, -1.0
        scale_match = re.search(r'scale\(([-\d.]+),\s*([-\d.]+)\)', transform_str)
        if scale_match:
            scale_x = float(scale_match.group(1))
            scale_y = float(scale_match.group(2))

        # Apply CadQuery's transform to the geometry center
        # Transform is: scale(sx, sy) translate(tx, ty)
        # For a point P: result = scale(P + translate) = (sx*(Px+tx), sy*(Py+ty))
        trans_x, trans_y = 0.0, 0.0
        trans_match = re.search(r'translate\(([-\d.]+),\s*([-\d.]+)\)', transform_str)
        if trans_match:
            trans_x = float(trans_match.group(1))
            trans_y = float(trans_match.group(2))

        # Calculate where the geometry center ends up after CadQuery's transform
        transformed_center_x = scale_x * (geom_center_x + trans_x)
        transformed_center_y = scale_y * (geom_center_y + trans_y)

        # Calculate the transformed geometry dimensions
        transformed_width = abs(scale_x) * geom_width
        transformed_height = abs(scale_y) * geom_height

        # Extract geometry content
        geometry_parts = []
        for child in main_group:
            content = ET.tostring(child, encoding='unicode')
            content = strip_svg_namespaces(content)
            geometry_parts.append(content)
        geometry = '\n'.join(geometry_parts)

        # View area center is our target
        view_center_x, view_center_y = area.center

        # Calculate scale to fit transformed geometry in view area with margin
        margin_factor = 0.85
        if transformed_width > 0 and transformed_height > 0:
            scale_to_width = (area.width * margin_factor) / transformed_width
            scale_to_height = (area.height * margin_factor) / transformed_height
            fit_scale = min(scale_to_width, scale_to_height)
        else:
            fit_scale = 1.0

        # Store projection parameters for balloon placement
        self._fit_scale = fit_scale
        self._geometry_center = (transformed_center_x, transformed_center_y)
        self._cq_scale_x = scale_x
        self._cq_scale_y = scale_y
        self._cq_trans_x = trans_x
        self._cq_trans_y = trans_y

        # Also store the raw geometry bounds for debugging
        self._raw_geom_center = (geom_center_x, geom_center_y)
        self._raw_geom_bounds = (geom_min_x, geom_min_y, geom_max_x, geom_max_y)

        # Calibrate projection by comparing 3D bounding box projection with CadQuery's 2D bounds
        bb = self.shape.BoundingBox()

        # Project all 8 corners of the 3D bounding box using CadQuery-style isometric
        # CadQuery uses standard isometric from (1,1,1), but we need to match its exact formula
        corners_3d = get_bounding_box_corners(bb)

        # CadQuery's isometric projection from direction (1,1,1)
        # After -90° X rotation: (X, Y, Z) -> (X, -Z, Y)
        #
        # For orthographic projection from camera at (1,1,1):
        # - Right vector: (1, -1, 0) / sqrt(2)
        # - Up vector: (-1, -1, 2) / sqrt(6)
        #
        # screen_x = dot(point, right) = (rx - ry) / sqrt(2)
        # screen_y = dot(point, up) = (-rx - ry + 2*rz) / sqrt(6)
        #
        # Note: CadQuery applies scale(sx, -sy) which flips Y, so positive screen_y goes up
        sqrt2 = 1.4142135623730951
        sqrt6 = 2.449489742783178

        def project_3d_to_2d(X, Y, Z):
            if ROTATE_MODEL_FOR_PROJECTION:
                # After -90° X rotation: (X, Y, Z) -> (X, -Z, Y)
                rx, ry, rz = X, -Z, Y
            else:
                rx, ry, rz = X, Y, Z
            # Orthographic projection from (1,1,1)
            px = (rx - ry) / sqrt2
            py = (-rx - ry + 2 * rz) / sqrt6
            return (px, py)

        corners_2d = [project_3d_to_2d(X, Y, Z) for X, Y, Z in corners_3d]
        my_xs = [c[0] for c in corners_2d]
        my_ys = [c[1] for c in corners_2d]
        my_min_x, my_max_x = min(my_xs), max(my_xs)
        my_min_y, my_max_y = min(my_ys), max(my_ys)
        my_center_x = (my_min_x + my_max_x) / 2
        my_center_y = (my_min_y + my_max_y) / 2
        my_width = my_max_x - my_min_x
        my_height = my_max_y - my_min_y

        # CadQuery's geometry bounds
        cq_width = geom_max_x - geom_min_x
        cq_height = geom_max_y - geom_min_y

        # Compute scale factor between my projection and CadQuery's
        # They should be proportional since both are isometric
        if my_width > 0 and my_height > 0:
            scale_to_cq_x = cq_width / my_width
            scale_to_cq_y = cq_height / my_height
            # Use average (they should be equal for true isometric)
            self._proj_to_cq_scale = (scale_to_cq_x + scale_to_cq_y) / 2
        else:
            self._proj_to_cq_scale = 1.0

        # Compute offset: CadQuery_center = scale * My_center + offset
        self._proj_to_cq_offset_x = geom_center_x - self._proj_to_cq_scale * my_center_x
        self._proj_to_cq_offset_y = geom_center_y - self._proj_to_cq_scale * my_center_y

        # Initialize the coordinate mapper with all parameters
        self._coord_mapper.cq_scale_x = scale_x
        self._coord_mapper.cq_scale_y = scale_y
        self._coord_mapper.cq_translate_x = trans_x
        self._coord_mapper.cq_translate_y = trans_y
        self._coord_mapper.geometry_center_x = transformed_center_x
        self._coord_mapper.geometry_center_y = transformed_center_y
        self._coord_mapper.view_center_x = view_center_x
        self._coord_mapper.view_center_y = view_center_y
        self._coord_mapper.fit_scale = fit_scale
        self._coord_mapper.raw_bounds = (geom_min_x, geom_min_y, geom_max_x, geom_max_y)
        self._coord_mapper._initialized = True

        # Compute and store SVG positions for all components
        self._compute_component_svg_positions()

        return f'''
        <!-- Isometric View -->
        <!-- View area: {area} -->
        <!-- View area center: ({view_center_x:.1f}, {view_center_y:.1f}) -->
        <!-- Raw geometry bounds: ({geom_min_x:.1f}, {geom_min_y:.1f}) to ({geom_max_x:.1f}, {geom_max_y:.1f}) -->
        <!-- Raw geometry center: ({geom_center_x:.1f}, {geom_center_y:.1f}) -->
        <!-- Transformed geometry center: ({transformed_center_x:.1f}, {transformed_center_y:.1f}) -->
        <!-- Transformed geometry size: {transformed_width:.1f} x {transformed_height:.1f} -->
        <!-- CadQuery transform: {transform_str} -->
        <!-- Fit scale: {fit_scale:.4f} -->
        <g id="isometric-view"
           transform="translate({view_center_x}, {view_center_y})
                      scale({fit_scale})
                      translate({-transformed_center_x}, {-transformed_center_y})">
            <g transform="{transform_str}">
                {geometry}
            </g>
        </g>
        '''

    def _compute_component_svg_positions(self):
        """
        Compute and store SVG positions for all components.

        For each component:
        1. Render it through CadQuery's SVG pipeline
        2. Parse the actual SVG path coordinates to get true geometry bounds AND points
        3. Map those to final screen coordinates

        This gives us the actual rendered geometry edges, not just projected 3D bounding boxes.
        The geometry points are stored for accurate balloon leader attachment.
        """
        if not self._coord_mapper._initialized:
            print("WARNING: CoordinateMapper not initialized, cannot compute SVG positions")
            return

        print("\n--- Computing SVG Positions for Components ---")
        for comp in self.components:
            if comp.shape is None:
                print(f"  Component {comp.item_number} ({comp.component_type}): No shape available")
                continue

            # Apply world transform to get shape in world coordinates
            # The raw comp.shape is at local origin; we need to transform it
            # to its actual world position before computing SVG bounds
            if comp.world_transform is not None:
                transformed_shape = apply_transform_to_shape(comp.shape, comp.world_transform)
            else:
                transformed_shape = comp.shape

            # Render component through CadQuery to get actual SVG geometry AND points
            result = self._get_component_actual_svg_bounds_and_points(transformed_shape)
            if result is None:
                print(f"  Component {comp.item_number} ({comp.component_type}): Failed to get SVG data")
                continue

            svg_bounds, svg_points = result
            min_x, min_y, max_x, max_y = svg_bounds
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2

            comp.svg_position_2d = (center_x, center_y)
            comp.svg_bounds_2d = svg_bounds
            comp.svg_geometry_points = svg_points  # Store actual geometry points!

            # Also store 3D position for reference
            if comp.world_position_3d is None:
                comp.set_world_position_from_shape()

            print(f"  Component {comp.item_number} ({comp.component_type}): "
                  f"SVG center=({center_x:.1f}, {center_y:.1f}), "
                  f"bounds=({min_x:.1f}, {min_y:.1f}) to ({max_x:.1f}, {max_y:.1f}), "
                  f"points={len(svg_points)}")

    def _get_component_actual_svg_bounds_and_points(
        self, shape: cq.Shape
    ) -> tuple[tuple[float, float, float, float], list[tuple[float, float]]] | None:
        """
        Get the actual SVG geometry bounds AND geometry points for a component.

        Strategy:
        1. Render the component through the same pipeline as the assembly
        2. Apply the same transformation chain to map to final screen coordinates
        3. The key is to use the assembly's transformation parameters consistently

        Returns: ((min_x, min_y, max_x, max_y), [list of (x,y) points]) or None if failed
        """
        # Render component to get its actual SVG geometry
        if ROTATE_MODEL_FOR_PROJECTION:
            rotated_shape = shape.rotate((0, 0, 0), (1, 0, 0), -90)
        else:
            rotated_shape = shape

        raw_svg = generate_view_svg(
            rotated_shape,
            PIPING_ISO_DIRECTION,
            width=1000,
            height=800,
            show_hidden=False
        )

        try:
            root = ET.fromstring(raw_svg)
        except ET.ParseError:
            return None

        # Find geometry group
        main_group = find_svg_geometry_group(root)
        if main_group is None:
            return None

        # Parse component's scale and translate from CadQuery's SVG output
        comp_transform_str = main_group.get('transform', '')
        _comp_scale_x, _comp_scale_y = 1.0, -1.0
        scale_match = re.search(r'scale\(([-\d.]+),\s*([-\d.]+)\)', comp_transform_str)
        if scale_match:
            _comp_scale_x = float(scale_match.group(1))
            _comp_scale_y = float(scale_match.group(2))

        _comp_trans_x, _comp_trans_y = 0.0, 0.0
        trans_match = re.search(r'translate\(([-\d.]+),\s*([-\d.]+)\)', comp_transform_str)
        if trans_match:
            _comp_trans_x = float(trans_match.group(1))
            _comp_trans_y = float(trans_match.group(2))

        # Get raw geometry points from SVG paths
        raw_points = get_svg_geometry_points(main_group)
        if not raw_points:
            return None

        # =================================================================
        # COORDINATE SYSTEM TRACKING
        # =================================================================
        #
        # Both component and assembly renders use the same projection:
        # - 3D model → isometric projection → "model projection space" (mm)
        # - CadQuery then applies: scale(sx,sy) translate(tx,ty) to fit in viewport
        #
        # The RAW SVG paths (before CadQuery's transform) are in model projection space.
        # This space is COMMON to both component and assembly renders.
        #
        # Transform chain to screen:
        # 1. raw_point (model projection mm)
        # 2. → CadQuery transform: cq_point = scale * (raw_point + translate)
        # 3. → Our outer transform: screen = view_center + fit_scale * (cq_point - geom_center)
        #
        # Key: Use ASSEMBLY's transform parameters, not component's!
        # =================================================================

        view_cx, view_cy = self._model_view_area.center
        geom_cx, geom_cy = self._geometry_center

        screen_points = []
        for rx, ry in raw_points:
            # Step 1: Apply assembly's CadQuery transform (scale + translate)
            # CadQuery transform: result = scale * (point + translate)
            cq_x = self._cq_scale_x * (rx + self._cq_trans_x)
            cq_y = self._cq_scale_y * (ry + self._cq_trans_y)

            # Step 2: Apply our outer transform (center in view area)
            screen_x = view_cx + self._fit_scale * (cq_x - geom_cx)
            screen_y = view_cy + self._fit_scale * (cq_y - geom_cy)

            screen_points.append((screen_x, screen_y))

        if not screen_points:
            return None

        screen_xs = [p[0] for p in screen_points]
        screen_ys = [p[1] for p in screen_points]
        screen_bounds = (min(screen_xs), min(screen_ys), max(screen_xs), max(screen_ys))

        return (screen_bounds, screen_points)

    def _get_component_actual_svg_bounds(self, shape: cq.Shape) -> tuple[float, float, float, float] | None:
        """
        Get the actual SVG geometry bounds for a component.

        Returns: (min_x, min_y, max_x, max_y) or None if failed
        """
        result = self._get_component_actual_svg_bounds_and_points(shape)
        if result is None:
            return None
        return result[0]

    def _get_component_screen_bounds(self, shape: cq.Shape) -> tuple[float, float, float, float, float, float]:
        """
        Get screen coordinates for a component shape using CadQuery's actual projection.

        Renders the component through CadQuery's SVG pipeline to get accurate 2D bounds,
        then maps those to the assembly's coordinate space.

        Returns: (min_x, min_y, max_x, max_y, center_x, center_y) in screen coordinates
        """
        # Step 1: Apply same rotation as main assembly
        if ROTATE_MODEL_FOR_PROJECTION:
            rotated_shape = shape.rotate((0, 0, 0), (1, 0, 0), -90)
        else:
            rotated_shape = shape

        # Step 2: Generate SVG through CadQuery with same projection
        raw_svg = generate_view_svg(
            rotated_shape,
            PIPING_ISO_DIRECTION,
            width=1000,
            height=800,
            show_hidden=False
        )

        # Step 3: Parse SVG to get raw geometry bounds and transform
        try:
            root = ET.fromstring(raw_svg)
        except ET.ParseError:
            return (0, 0, 0, 0, 0, 0)

        # Find the geometry group with transform
        main_group = find_svg_geometry_group(root)
        if main_group is None:
            return (0, 0, 0, 0, 0, 0)

        # Get the component's own transform
        comp_transform_str = main_group.get('transform', '')

        # Parse component's scale and translate
        comp_scale_x, comp_scale_y = 1.0, -1.0
        scale_match = re.search(r'scale\(([-\d.]+),\s*([-\d.]+)\)', comp_transform_str)
        if scale_match:
            comp_scale_x = float(scale_match.group(1))
            comp_scale_y = float(scale_match.group(2))

        comp_trans_x, comp_trans_y = 0.0, 0.0
        trans_match = re.search(r'translate\(([-\d.]+),\s*([-\d.]+)\)', comp_transform_str)
        if trans_match:
            comp_trans_x = float(trans_match.group(1))
            comp_trans_y = float(trans_match.group(2))

        # Get raw geometry bounds
        raw_bounds = get_svg_geometry_bounds(main_group)
        raw_min_x, raw_min_y, raw_max_x, raw_max_y = raw_bounds
        raw_center_x = (raw_min_x + raw_max_x) / 2
        raw_center_y = (raw_min_y + raw_max_y) / 2
        raw_width = raw_max_x - raw_min_x
        raw_height = raw_max_y - raw_min_y

        # Apply the component's own transform to get its center in CadQuery's output space
        comp_cq_center_x = comp_scale_x * (raw_center_x + comp_trans_x)
        comp_cq_center_y = comp_scale_y * (raw_center_y + comp_trans_y)

        # The component's CadQuery output is centered in a 1000x800 viewport
        # We need to map this to the same coordinate space as the assembly
        # The key insight: raw geometry coordinates are consistent, but transforms differ

        # Apply our outer transform to get final screen coordinates
        # Use the component's transformed center relative to the assembly's geometry center
        view_cx, view_cy = self._model_view_area.center
        geom_cx, geom_cy = self._geometry_center

        # Scale the component's position relative to its own viewport center (500, 400)
        # then map to assembly space
        comp_viewport_cx = 500.0  # CadQuery centers in 1000x800 viewport
        comp_viewport_cy = 400.0

        # The component's offset from its viewport center
        comp_offset_x = comp_cq_center_x - comp_viewport_cx
        comp_offset_y = comp_cq_center_y - comp_viewport_cy

        # This offset should map to the same position in the assembly's coordinate system
        # But we need to account for different scales between component and assembly renders

        # Use the ratio of assembly scale to component scale
        scale_ratio_x = abs(self._cq_scale_x / comp_scale_x) if abs(comp_scale_x) > 0.001 else 1.0
        scale_ratio_y = abs(self._cq_scale_y / comp_scale_y) if abs(comp_scale_y) > 0.001 else 1.0

        # Map to assembly's CadQuery space
        assembly_cq_center_x = self._geometry_center[0] + comp_offset_x * scale_ratio_x
        assembly_cq_center_y = self._geometry_center[1] + comp_offset_y * scale_ratio_y

        # Apply our outer transform
        screen_center_x = view_cx + self._fit_scale * (assembly_cq_center_x - geom_cx)
        screen_center_y = view_cy + self._fit_scale * (assembly_cq_center_y - geom_cy)

        # Compute screen bounds
        half_width = abs(comp_scale_x) * raw_width * self._fit_scale * scale_ratio_x / 2
        half_height = abs(comp_scale_y) * raw_height * self._fit_scale * scale_ratio_y / 2

        screen_min_x = screen_center_x - half_width
        screen_max_x = screen_center_x + half_width
        screen_min_y = screen_center_y - half_height
        screen_max_y = screen_center_y + half_height

        return (screen_min_x, screen_min_y, screen_max_x, screen_max_y,
                screen_center_x, screen_center_y)

    def _create_balloons(self) -> str:
        """Create balloon leaders for all components using stored SVG positions."""
        if not self.show_balloons or not self.components:
            return ""

        # Create topology-based balloon placer
        placer = TopologyBalloonPlacer(
            view_area=self._model_view_area,
            balloon_radius=4.0,
            leader_length=12.0
        )

        balloons = []
        debug_markers = []  # Debug: circles at computed component centers
        print("\n--- Placing Balloons ---")

        for comp in self.components:
            # Use stored SVG positions from coordinate mapper
            if comp.svg_position_2d is None:
                print(f"  Balloon {comp.item_number} ({comp.component_type}): No SVG position stored")
                continue

            screen_center = comp.svg_position_2d

            # Add debug marker at computed center (small red circle)
            debug_markers.append(
                f'<circle cx="{screen_center[0]:.2f}" cy="{screen_center[1]:.2f}" '
                f'r="2" fill="red" opacity="0.7"/>'
            )

            # Use stored SVG bounds if available, otherwise create a small box around center
            if comp.svg_bounds_2d is not None:
                screen_bounds = comp.svg_bounds_2d
                # Also add debug rectangle for bounds
                min_x, min_y, max_x, max_y = screen_bounds
                debug_markers.append(
                    f'<rect x="{min_x:.2f}" y="{min_y:.2f}" '
                    f'width="{max_x - min_x:.2f}" height="{max_y - min_y:.2f}" '
                    f'fill="none" stroke="blue" stroke-width="0.3" opacity="0.5"/>'
                )
            else:
                # Create a small bounding box around the center
                half_size = 10.0
                screen_bounds = (
                    screen_center[0] - half_size,
                    screen_center[1] - half_size,
                    screen_center[0] + half_size,
                    screen_center[1] + half_size
                )

            # Use topology-aware placement
            balloon = placer.place_balloon_for_component(
                comp=comp,
                screen_center=screen_center,
                screen_bounds=screen_bounds
            )

            if balloon:
                balloons.append(balloon)
                print(f"  Balloon {comp.item_number} ({comp.component_type}): "
                      f"SVG center=({screen_center[0]:.1f}, {screen_center[1]:.1f}), "
                      f"placed at ({balloon.center[0]:.1f}, {balloon.center[1]:.1f})")
            else:
                print(f"  Balloon {comp.item_number} ({comp.component_type}): FAILED to place")

        # Include debug markers in output (only if show_debug is enabled)
        if self.show_debug:
            debug_svg = '\n'.join(['<!-- DEBUG: Component center markers -->',
                                  '<g id="debug-markers">'] + debug_markers + ['</g>'])
            # Add geometry edge highlighting for component 2 (pipe) for debugging
            edge_highlight = self._create_geometry_edge_highlight(self.components[1] if len(self.components) > 1 else None)
            return debug_svg + '\n' + edge_highlight + '\n' + render_all_balloons_svg(balloons)
        else:
            return render_all_balloons_svg(balloons)

    def _create_geometry_edge_highlight(self, comp: ComponentRecord | None) -> str:
        """Create SVG highlighting the outer edge of a component's geometry points."""
        if comp is None or not comp.svg_geometry_points:
            return ""

        points = comp.svg_geometry_points
        if len(points) < 3:
            return ""

        # Create a path connecting all points to visualize the geometry outline
        # We'll draw small dots at each point
        svg_parts = [
            f'<!-- DEBUG: Geometry edge highlight for {comp.component_type} (item {comp.item_number}) -->',
            '<g id="geometry-edge-highlight" opacity="0.8">'
        ]

        # Draw dots at each geometry point
        for x, y in points:
            # Use green for the points
            svg_parts.append(
                f'<circle cx="{x:.2f}" cy="{y:.2f}" r="0.3" fill="lime" stroke="green" stroke-width="0.1"/>'
            )

        # Also draw the convex hull or just connect consecutive points to show the shape
        # For now, let's just show all points as dots - this should outline the shape

        svg_parts.append('</g>')
        return '\n'.join(svg_parts)

    def _get_svg_position_for_3d_point(self, x: float, y: float, z: float) -> tuple[float, float] | None:
        """
        Get the actual SVG screen position for a 3D world point.

        Uses the same approach as component SVG positioning: renders a small test shape
        at the given 3D position through CadQuery's SVG pipeline to get the actual
        screen coordinates.

        Args:
            x, y, z: 3D world coordinates

        Returns:
            (svg_x, svg_y) screen coordinates, or None if failed
        """
        # Create a small sphere at the 3D position
        test_shape = cq.Solid.makeSphere(1.0, cq.Vector(x, y, z))

        # Apply same rotation as main assembly
        if ROTATE_MODEL_FOR_PROJECTION:
            test_shape = test_shape.rotate((0, 0, 0), (1, 0, 0), -90)

        # Render through CadQuery
        raw_svg = generate_view_svg(
            test_shape,
            PIPING_ISO_DIRECTION,
            width=1000,
            height=800,
            show_hidden=False
        )

        try:
            root = ET.fromstring(raw_svg)
        except ET.ParseError:
            return None

        # Find geometry group
        main_group = find_svg_geometry_group(root)
        if main_group is None:
            return None

        # Get raw geometry points from SVG
        raw_points = get_svg_geometry_points(main_group)
        if not raw_points:
            return None

        # Calculate centroid of the test shape's SVG geometry
        raw_xs = [p[0] for p in raw_points]
        raw_ys = [p[1] for p in raw_points]
        raw_center_x = sum(raw_xs) / len(raw_xs)
        raw_center_y = sum(raw_ys) / len(raw_ys)

        # Apply the same transformation chain as the assembly
        view_cx, view_cy = self._model_view_area.center
        geom_cx, geom_cy = self._geometry_center

        # Step 1: Apply assembly's CadQuery transform
        cq_x = self._cq_scale_x * (raw_center_x + self._cq_trans_x)
        cq_y = self._cq_scale_y * (raw_center_y + self._cq_trans_y)

        # Step 2: Apply our outer transform
        screen_x = view_cx + self._fit_scale * (cq_x - geom_cx)
        screen_y = view_cy + self._fit_scale * (cq_y - geom_cy)

        return (screen_x, screen_y)

    def _get_perpendicular_3d_directions(self, pipe_direction: str, weld_type: str = "BW") -> list[tuple[float, float, float]]:
        """
        Get 3D unit vectors perpendicular to a pipe's axis for OD point placement.

        Different placement for different weld types:
        - Butt welds (BW): Prefer north/east (front side in iso view, like BOM balloons)
        - Socket welds (SW): Prefer south/west (back side in iso view, to avoid crossing fittings)

        Args:
            pipe_direction: World direction of pipe axis ("east", "north", "up", etc.)
            weld_type: "BW" for butt weld, "SW" for socket weld

        Returns:
            List of (dx, dy, dz) unit vectors perpendicular to pipe axis
        """
        pipe_dir = pipe_direction.lower() if pipe_direction else "east"

        if weld_type == "SW":
            # Socket welds: prefer back side to avoid crossing connected fittings
            perpendiculars: dict[str, list[tuple[float, float, float]]] = {
                "east":  [(0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)],   # south, up, north, down
                "west":  [(0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)],
                "north": [(-1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)],   # west, up, east, down
                "south": [(-1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)],
                "up":    [(-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],   # west, south, east, north
                "down":  [(-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
            }
        else:
            # Butt welds: prefer front side (like BOM balloons) - north for E-W pipes
            perpendiculars = {
                "east":  [(0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)],   # north, up, south, down
                "west":  [(0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)],
                "north": [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)],   # east, up, west, down
                "south": [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)],
                "up":    [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)],   # east, north, west, south
                "down":  [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)],
            }

        return perpendiculars.get(pipe_dir, [(0.0, 1.0, 0.0), (0.0, 0.0, 1.0)])

    def _create_weld_markers(self) -> str:
        """Create weld number markers for ASME B31.3 compliance.

        Weld positions now come from fitting attachment points which are already
        at the correct OD position (on the pipe/fitting surface). This is more
        accurate than computing perpendicular offsets from centerline positions.

        Butt welds (BW): Attachment point at flange hub OD
        Socket welds (SW): Attachment point at socket opening OD (fitting body surface)
        """
        if not self.show_welds or not self.welds:
            return ""

        print("\n--- Placing Weld Markers (using attachment point positions) ---")

        from .bom import WELD_MARKER_OFFSET, WELD_MARKER_SIZE, WORLD_DIR_TO_SCREEN_OFFSET, WeldMarker, compute_weld_label_screen_direction, render_weld_markers_svg

        # Extract 2D pipe segments for collision detection
        pipe_segments_2d: list[tuple[tuple[float, float], tuple[float, float]]] = []
        for comp in self.components:
            if comp.component_type == "pipe" and comp.svg_bounds_2d:
                min_x, min_y, max_x, max_y = comp.svg_bounds_2d
                # For pipes, use the bounding box diagonal as the segment
                # This approximates the pipe's centerline in screen space
                pipe_segments_2d.append(((min_x, min_y), (max_x, max_y)))
                # Also add the other diagonal for better coverage
                pipe_segments_2d.append(((min_x, max_y), (max_x, min_y)))

        if pipe_segments_2d:
            print(f"  Extracted {len(pipe_segments_2d)} pipe segments for collision detection")

        markers = []
        debug_markers = []  # Debug visualization of weld positions
        placed_positions = []  # Track placed label positions to avoid overlap

        for weld in self.welds:
            # Weld position is now already at OD (from fitting attachment point)
            x3d, y3d, z3d = weld.world_position_3d

            print(f"  {weld.label} ({weld.weld_type}): OD 3D=({x3d:.1f}, {y3d:.1f}, {z3d:.1f})")

            # Get SVG position of weld point (already at OD)
            svg_pos = self._get_svg_position_for_3d_point(x3d, y3d, z3d)
            if svg_pos is None:
                print("    WARNING: Could not project weld position to SVG")
                continue

            weld_x, weld_y = svg_pos
            print(f"    -> SVG=({weld_x:.1f}, {weld_y:.1f})")

            # Add debug marker at weld position (magenta circle with label)
            if self.show_debug:
                debug_markers.append(
                    f'<circle cx="{weld_x:.2f}" cy="{weld_y:.2f}" '
                    f'r="3" fill="none" stroke="magenta" stroke-width="0.5" opacity="0.9"/>'
                )
                debug_markers.append(
                    f'<text x="{weld_x + 4:.2f}" y="{weld_y - 2:.2f}" '
                    f'font-family="Arial" font-size="2.5" fill="magenta" font-weight="bold">'
                    f'{weld.label}</text>'
                )

            # Get screen direction for label placement
            # Weld labels should point toward the viewer (south direction in isometric)
            # unless the attachment is on a special side (up, east, north)
            # or it's at an elbow turning from vertical to horizontal
            if weld.pipe_direction:
                screen_perp = compute_weld_label_screen_direction(
                    weld.pipe_direction,
                    weld.attachment_direction,
                    weld.avoid_direction
                )
            else:
                # Default: south direction (toward viewer)
                screen_perp = (-0.866, 0.5)

            # Compute label position from weld point
            dx, dy = screen_perp
            label_x = weld_x + dx * WELD_MARKER_OFFSET
            label_y = weld_y + dy * WELD_MARKER_OFFSET

            best_weld_pos = (weld_x, weld_y)
            best_label_pos = (label_x, label_y)

            # Check if this attachment direction should never be flipped
            # Attachments on up, east, or north side should keep their north-pointing labels
            never_flip = weld.attachment_direction in ("up", "east", "north")

            # Check bounds
            margin = WELD_MARKER_SIZE * 1.5
            in_bounds = is_point_in_bounds(label_x, label_y, margin, self._model_view_area)

            if not in_bounds and not never_flip:
                # Try opposite direction (only if allowed to flip)
                label_x = weld_x - dx * WELD_MARKER_OFFSET
                label_y = weld_y - dy * WELD_MARKER_OFFSET
                in_bounds = is_point_in_bounds(label_x, label_y, margin, self._model_view_area)
                if in_bounds:
                    best_label_pos = (label_x, label_y)
                else:
                    print("    WARNING: Label position out of bounds")

            # Check if leader line crosses any pipe geometry
            leader_crosses = False
            if pipe_segments_2d:
                leader_crosses = _leader_crosses_pipe(
                    best_weld_pos, best_label_pos, pipe_segments_2d
                )
                if leader_crosses and not never_flip:
                    # Try opposite direction
                    alt_label_x = weld_x - dx * WELD_MARKER_OFFSET
                    alt_label_y = weld_y - dy * WELD_MARKER_OFFSET
                    alt_crosses = _leader_crosses_pipe(
                        best_weld_pos, (alt_label_x, alt_label_y), pipe_segments_2d
                    )
                    if not alt_crosses:
                        best_label_pos = (alt_label_x, alt_label_y)
                        leader_crosses = False
                        print(f"    -> Flipped to avoid crossing pipe")

            # Check overlap with existing markers and try alternative position if needed
            for px, py in placed_positions:
                if math.sqrt((best_label_pos[0] - px)**2 + (best_label_pos[1] - py)**2) < WELD_MARKER_SIZE * 3:
                    if never_flip:
                        # Don't flip these, just accept the overlap
                        break
                    # Try opposite direction
                    alt_label_x = weld_x - dx * WELD_MARKER_OFFSET
                    alt_label_y = weld_y - dy * WELD_MARKER_OFFSET
                    alt_overlap = any(
                        math.sqrt((alt_label_x - px2)**2 + (alt_label_y - py2)**2) < WELD_MARKER_SIZE * 3
                        for px2, py2 in placed_positions
                    )
                    if not alt_overlap:
                        best_label_pos = (alt_label_x, alt_label_y)
                    break

            # Add marker
            placed_positions.append(best_label_pos)
            markers.append(WeldMarker(
                weld_number=weld.weld_number,
                weld_type=weld.weld_type,
                position=best_weld_pos,
                label_position=best_label_pos
            ))

            print(f"    -> weld=({best_weld_pos[0]:.1f}, {best_weld_pos[1]:.1f}), "
                  f"label=({best_label_pos[0]:.1f}, {best_label_pos[1]:.1f})")

            # Add debug line from weld point to label position
            if self.show_debug:
                debug_markers.append(
                    f'<line x1="{best_weld_pos[0]:.2f}" y1="{best_weld_pos[1]:.2f}" '
                    f'x2="{best_label_pos[0]:.2f}" y2="{best_label_pos[1]:.2f}" '
                    f'stroke="magenta" stroke-width="0.3" stroke-dasharray="1,1" opacity="0.7"/>'
                )

        # Include debug markers if enabled
        result = render_weld_markers_svg(markers)
        if self.show_debug and debug_markers:
            debug_svg = '\n'.join([
                '<!-- DEBUG: Weld position markers (magenta) -->',
                '<g id="debug-weld-markers">'
            ] + debug_markers + ['</g>'])
            result = debug_svg + '\n' + result

        return result

    def _create_annotations(self) -> str:
        """Create all annotations (balloons and weld markers) with unified placement.

        This method coordinates placement between BOM balloons and weld markers to:
        1. Use scale-aware annotation sizing based on fit_scale
        2. Place balloons first, then place weld markers avoiding balloons
        3. Apply force-directed optimization to resolve remaining overlaps

        Returns:
            Combined SVG string for all annotations
        """
        if not self.show_balloons and not self.show_welds:
            return ""

        svg_parts = []

        # Extract 2D pipe segments for collision detection
        pipe_segments_2d: list[tuple[tuple[float, float], tuple[float, float]]] = []
        for comp in self.components:
            if comp.component_type == "pipe" and comp.svg_bounds_2d:
                min_x, min_y, max_x, max_y = comp.svg_bounds_2d
                pipe_segments_2d.append(((min_x, min_y), (max_x, max_y)))
                pipe_segments_2d.append(((min_x, max_y), (max_x, min_y)))

        # Create unified placement context with scale-aware sizing
        context = AnnotationPlacementContext(
            view_area=self._model_view_area,
            fit_scale=self._fit_scale,
            pipe_segments_2d=pipe_segments_2d
        )

        print(f"\n--- Unified Annotation Placement (fit_scale={self._fit_scale:.3f}) ---")
        print(f"  Effective balloon radius: {context.effective_balloon_radius:.2f}mm")
        print(f"  Effective balloon leader: {context.effective_balloon_leader:.2f}mm")
        print(f"  Effective weld size: {context.effective_weld_size:.2f}mm")
        print(f"  Effective weld offset: {context.effective_weld_offset:.2f}mm")

        # Phase 1: Place BOM balloons
        if self.show_balloons and self.components:
            placer = TopologyBalloonPlacer(
                view_area=self._model_view_area,
                context=context
            )

            print("\n--- Placing Balloons (with context) ---")
            for comp in self.components:
                if comp.svg_position_2d is None:
                    print(f"  Balloon {comp.item_number} ({comp.component_type}): No SVG position stored")
                    continue

                screen_center = comp.svg_position_2d
                screen_bounds = comp.svg_bounds_2d or (
                    screen_center[0] - 10, screen_center[1] - 10,
                    screen_center[0] + 10, screen_center[1] + 10
                )

                balloon = placer.place_balloon_for_component(
                    comp=comp,
                    screen_center=screen_center,
                    screen_bounds=screen_bounds
                )

                if balloon:
                    context.placed_balloons.append(balloon)
                    print(f"  Balloon {comp.item_number} ({comp.component_type}): "
                          f"placed at ({balloon.center[0]:.1f}, {balloon.center[1]:.1f})")
                else:
                    print(f"  Balloon {comp.item_number} ({comp.component_type}): FAILED to place")

        # Phase 2: Place weld markers (avoiding balloons)
        if self.show_welds and self.welds:
            print("\n--- Placing Weld Markers (with context, avoiding balloons) ---")
            markers = create_weld_markers(
                welds=self.welds,
                coord_mapper=self._coord_mapper,
                view_area=self._model_view_area,
                pipe_segments_2d=pipe_segments_2d,
                placed_balloons=context.placed_balloons,
                context=context
            )
            context.placed_weld_markers = markers

            for m in markers:
                print(f"  Weld {m.weld_number}: label at ({m.label_position[0]:.1f}, {m.label_position[1]:.1f})")

        # Phase 3: Force-directed optimization to resolve overlaps
        if context.placed_balloons or context.placed_weld_markers:
            print("\n--- Running overlap optimization ---")
            optimize_annotation_positions(context)

        # Render SVG
        if context.placed_balloons:
            svg_parts.append(render_all_balloons_svg(context.placed_balloons))
        if context.placed_weld_markers:
            svg_parts.append(render_weld_markers_svg(context.placed_weld_markers))

        return '\n'.join(svg_parts)

    def _create_attachment_point_debug_markers(self) -> str:
        """Create debug markers showing ALL attachment points on all fittings.

        Draws colored circles at each attachment point:
        - Red circles for weld attachment points
        - Labels show the attachment point name

        This helps visualize where attachment points are located relative to
        the pipe geometry, allowing verification of placement.
        """
        if not self.fittings:
            return ""

        markers = ['<!-- Attachment Point Debug Markers -->']

        # Color scheme for different ports
        port_colors = {
            "weld": "#FF0000",    # Red for flange weld
            "inlet": "#00AA00",   # Green for elbow inlet
            "outlet": "#0000FF",  # Blue for elbow outlet
        }

        for fitting, world_transform in self.fittings:
            # Get all weld attachment points from this fitting
            weld_attachments = fitting.get_weld_attachment_points()

            # Group attachment points by port and compute port centers
            port_centers_3d = {}
            port_attachments = {}
            for ap in weld_attachments:
                port_name = ap.port_name or "weld"
                if port_name not in port_attachments:
                    port_attachments[port_name] = []
                port_attachments[port_name].append(ap)

            # Compute 3D center for each port's attachment points
            for port_name, aps in port_attachments.items():
                positions = [fitting.get_attachment_point_world_position(ap.name, world_transform) for ap in aps]
                cx = sum(p[0] for p in positions) / len(positions)
                cy = sum(p[1] for p in positions) / len(positions)
                cz = sum(p[2] for p in positions) / len(positions)
                port_centers_3d[port_name] = (cx, cy, cz)

            for ap in weld_attachments:
                # Get world position of this attachment point
                world_pos = fitting.get_attachment_point_world_position(ap.name, world_transform)
                x3d, y3d, z3d = world_pos

                # Project to SVG coordinates
                svg_pos = self._get_svg_position_for_3d_point(x3d, y3d, z3d)
                if svg_pos is None:
                    continue

                sx, sy = svg_pos

                # Pick color based on port name
                port_name = ap.port_name or "weld"
                color = port_colors.get(port_name, "#FF00FF")  # Magenta as fallback

                # Draw a circle at the attachment point position
                markers.append(
                    f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="2.0" '
                    f'fill="{color}" stroke="white" stroke-width="0.4" '
                    f'opacity="0.9"/>'
                )

                # Get the port center in screen coordinates
                port_center_3d = port_centers_3d.get(port_name, world_pos)
                port_center_svg = self._get_svg_position_for_3d_point(*port_center_3d)
                if port_center_svg is None:
                    port_center_svg = (sx, sy)

                # Compute screen direction from port center to attachment point
                # This matches what the user sees relative to the compass
                screen_dx = sx - port_center_svg[0]
                screen_dy = sy - port_center_svg[1]
                short_label = _screen_direction_to_compass_label(screen_dx, screen_dy)

                markers.append(
                    f'<text x="{sx + 3:.1f}" y="{sy - 1:.1f}" '
                    f'font-family="Arial" font-size="2.0" fill="{color}" '
                    f'font-weight="bold">{short_label}</text>'
                )

        return '\n'.join(markers)

    def _create_bom_table(self) -> str:
        """Create the Bill of Materials table (grows upward from baseline)."""
        if not self.show_bom or not self.components:
            return ""

        # Aggregate components into BOM entries, passing material specification
        bom_entries = aggregate_components_to_bom(
            self.components,
            material=self.material
        )

        if not bom_entries:
            return ""

        # Create a view area for the BOM table
        # BOM_Y is the baseline (same as title block top)
        # The table grows upward from there
        bom_area = ViewArea(
            x=BOM_X,
            y=BOM_Y,  # Baseline - table grows upward from here
            width=BOM_WIDTH,
            height=0  # Height is computed by BOMTable based on entries
        )

        bom_table = BOMTable(entries=bom_entries, area=bom_area)

        return bom_table.generate_svg()

    def generate(self) -> str:
        """Generate the complete piping drawing as SVG."""
        area = self._model_view_area

        svg_header = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg"
     width="{SHEET_WIDTH_MM}mm" height="{SHEET_HEIGHT_MM}mm"
     viewBox="0 0 {SHEET_WIDTH_MM} {SHEET_HEIGHT_MM}">

    <defs>
        <style>
            .visible {{ stroke: {VISIBLE_COLOR}; stroke-width: {VISIBLE_STROKE_WIDTH}; fill: none; }}
            .hidden {{ stroke: {HIDDEN_COLOR}; stroke-width: 0.25; fill: none; stroke-dasharray: 2,1; }}
        </style>
    </defs>

    <!-- Background -->
    <rect x="0" y="0" width="{SHEET_WIDTH_MM}" height="{SHEET_HEIGHT_MM}" fill="white"/>
'''

        svg_content = []

        # Add border
        svg_content.append(self._create_border())

        # Add isometric view
        svg_content.append(self._create_isometric_view())

        # Add 3D axis indicator (top left of model view area)
        axis_x, axis_y = area.position_top_left(margin=25)
        svg_content.append(create_axis_indicator_svg(axis_x, axis_y, size=15))

        # Add title block
        svg_content.append(self._title_block.generate_svg())

        # Add BOM table (after title block)
        svg_content.append(self._create_bom_table())

        # Add annotations (balloons + weld markers) after geometry to render on top
        svg_content.append(self._create_annotations())

        # Add debug markers for attachment point positions (only if show_debug is enabled)
        if self.show_debug:
            svg_content.append(self._create_attachment_point_debug_markers())

        # Add debug crosshair if enabled
        svg_content.append(self._create_center_crosshair())

        svg_footer = '''
</svg>'''

        self._svg_content = svg_header + '\n'.join(svg_content) + svg_footer
        return self._svg_content

    def export_svg(self, filepath: str):
        """Export the drawing as SVG file."""
        if not self._svg_content:
            self.generate()

        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(self._svg_content)
        print(f"Exported SVG: {filepath}")

    def export_pdf(self, filepath: str):
        """Export the drawing as PDF file using svglib + reportlab."""
        if not SVGLIB_AVAILABLE:
            raise ImportError(
                "PDF export requires svglib and reportlab. "
                "Install with: pip install svglib reportlab"
            )

        if not self._svg_content:
            self.generate()

        # Write SVG to a temporary file for svglib to read
        import tempfile
        import os

        with tempfile.NamedTemporaryFile(mode='w', suffix='.svg',
                                          encoding='utf-8', delete=False) as tmp:
            tmp.write(self._svg_content)
            tmp_path = tmp.name

        try:
            # Convert SVG to ReportLab drawing
            drawing = svg2rlg(tmp_path)
            if drawing is None:
                raise ValueError("Failed to parse SVG content")

            # Render to PDF
            renderPDF.drawToFile(drawing, filepath)
            print(f"Exported PDF: {filepath}")
        finally:
            # Clean up temp file
            os.unlink(tmp_path)


# Alias for backward compatibility
EngineeringDrawing = PipingDrawing
