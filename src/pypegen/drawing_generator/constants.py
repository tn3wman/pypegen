"""
Drawing generator constants.

Sheet sizes, margins, and styling constants for engineering drawings.
"""

# =============================================================================
# SHEET AND LAYOUT CONSTANTS
# =============================================================================

# 11x17 inch sheet in mm (landscape orientation)
SHEET_WIDTH_MM = 431.8   # 17 inches
SHEET_HEIGHT_MM = 279.4  # 11 inches

# Margins (in mm) - equal left/right for professional appearance
MARGIN = 10  # Uniform margin on all sides

# BOM dimensions (in mm) - bottom left corner, grows upward
# Width is tighter to reduce whitespace
BOM_WIDTH = 115  # Tighter width
BOM_X = MARGIN

# Title block dimensions (in mm) - horizontal strip at bottom right
# About 1.3" tall (35mm), starts at right edge of BOM
TITLE_BLOCK_HEIGHT = 35
TITLE_BLOCK_X = BOM_X + BOM_WIDTH  # Aligns with BOM right edge
TITLE_BLOCK_WIDTH = SHEET_WIDTH_MM - MARGIN - TITLE_BLOCK_X
TITLE_BLOCK_Y = SHEET_HEIGHT_MM - MARGIN - TITLE_BLOCK_HEIGHT

# BOM baseline matches title block
BOM_Y = TITLE_BLOCK_Y

# Drawing area - bottom edge aligns with top of title block
DRAWING_AREA_X = MARGIN
DRAWING_AREA_Y = MARGIN
DRAWING_AREA_WIDTH = SHEET_WIDTH_MM - (2 * MARGIN)
DRAWING_AREA_HEIGHT = TITLE_BLOCK_Y - MARGIN  # Bottom aligns with title block top


# =============================================================================
# SVG STYLING
# =============================================================================

VISIBLE_STROKE_WIDTH = 0.5
HIDDEN_STROKE_WIDTH = 0.25
VISIBLE_COLOR = "#000000"
HIDDEN_COLOR = "#888888"
BORDER_COLOR = "#000000"
BORDER_WIDTH = 0.5
THIN_LINE_WIDTH = 0.25


# =============================================================================
# PIPING ISOMETRIC PROJECTION
# =============================================================================

# Standard piping isometric convention:
# - North arrow points to upper-right (NE direction on paper)
# - Vertical pipes (risers) go straight up on paper
# - East-West pipes go to lower-right / upper-left
# - North-South pipes go to upper-right / lower-left
#
# PyPeGen coordinate system: X=East, Y=North, Z=Up
#
# CadQuery projectionDir = where camera is located relative to object

# Isometric view: (1, 1, 1) with model rotation to fix Y/Z swap
PIPING_ISO_DIRECTION = (1, 1, 1)
ROTATE_MODEL_FOR_PROJECTION = True  # Rotate 90 degrees around X axis

# =============================================================================
# ORTHOGRAPHIC VIEW DIRECTIONS (3rd Angle Projection)
# =============================================================================
# PyPeGen coordinate system: X=East, Y=North, Z=Up
# CadQuery projectionDir = where camera is located relative to object

# Individual view directions
PLAN_VIEW_DIRECTION = (0, 0, 1)       # Top-down (plan view) - camera above
ELEVATION_DIRECTION = (0, -1, 0)      # From South looking North (front elevation)
RIGHT_SIDE_DIRECTION = (1, 0, 0)      # From East looking West (right side view)

# View configuration for 3rd angle projection drawings
# CadQuery projectionDir = direction FROM object TO camera (where camera is located)
#
# PyPeGen coordinates: X=East, Y=North, Z=Up
# Orthographic views do NOT use model rotation - just projection direction
# Only isometric uses the -90 rotation for piping convention
#
VIEW_DIRECTIONS: dict[str, tuple[float, float, float]] = {
    "front": (0, -1, 0),      # Looking North: camera to South (-Y)
    "top": (0, 0, 1),         # Looking Down: camera above (+Z)
    "right": (1, 0, 0),       # Looking West: camera to East (+X)
    "isometric": (1, 1, 1),   # Standard piping isometric (uses rotated model)
}

# Up vectors for each view (defines which direction is "up" on the drawing)
# The up vector must NOT be parallel to the projection direction!
VIEW_UP_VECTORS: dict[str, tuple[float, float, float]] = {
    "front": (0, 0, 1),       # Z is up when looking North
    "top": (0, 1, 0),         # Y (North) is up when looking down
    "right": (0, 0, 1),       # Z is up when looking West
    "isometric": (0, 0, 1),   # Z is up in isometric
}

# View labels for drawing annotations (using "Looking {direction}" convention)
VIEW_LABELS: dict[str, str] = {
    "front": "LOOKING NORTH",
    "top": "LOOKING DOWN",
    "right": "LOOKING WEST",
    "isometric": "ISOMETRIC VIEW",
}

# =============================================================================
# MULTI-VIEW LAYOUT CONSTANTS
# =============================================================================

# Minimum spacing between views (mm)
VIEW_SPACING = 15.0

# View label styling
VIEW_LABEL_FONT_SIZE = 4.0
VIEW_LABEL_OFFSET = 5.0  # Distance below view border
