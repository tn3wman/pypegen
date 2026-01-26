"""
Drawing Generator Module

Generates professional piping fabrication drawings from CadQuery pipe assemblies.
Based on industry conventions for piping isometric and spool drawings.

Features:
- Primary isometric view (standard piping convention)
- 3D coordinate axis indicator
- Professional title block
- 11x17 (ANSI B / Tabloid) landscape format
- PDF and SVG export

Usage:
    from drawing_generator import PipingDrawing

    drawing = PipingDrawing(
        step_file="path/to/file.step",
        title="2\" PROCESS PIPING",
        drawing_number="SPOOL-001",
    )
    drawing.generate()
    drawing.export_pdf("output.pdf")
"""

from .axis_indicator import create_axis_indicator_svg
from .bom import (
    WORLD_DIR_TO_SCREEN_OFFSET,
    Balloon,
    BalloonPlacer,
    BOMEntry,
    BOMTable,
    ComponentRecord,
    CoordinateMapper,
    TopologyBalloonPlacer,
    WeldMarker,
    WeldRecord,
    aggregate_components_to_bom,
    get_free_directions,
    get_perpendicular_directions,
)
from .constants import (
    DRAWING_AREA_HEIGHT,
    DRAWING_AREA_WIDTH,
    MARGIN,
    SHEET_HEIGHT_MM,
    SHEET_WIDTH_MM,
)
from .drawing import EngineeringDrawing, OrthographicPipingDrawing, PipingDrawing
from .layout_engine import ViewLayoutEngine
from .page import DrawingPage, MultiPageDrawing
from .title_block import TitleBlock, TitleBlockInfo
from .view_area import ViewArea
from .view_projection import ViewProjection, create_standard_views

__all__ = [
    # Main classes
    'PipingDrawing',
    'OrthographicPipingDrawing',
    'EngineeringDrawing',
    'ViewArea',
    'ViewProjection',
    'ViewLayoutEngine',
    'DrawingPage',
    'MultiPageDrawing',
    'TitleBlock',
    'TitleBlockInfo',
    # BOM and coordinate classes
    'ComponentRecord',
    'WeldRecord',
    'WeldMarker',
    'CoordinateMapper',
    'BOMEntry',
    'Balloon',
    'BalloonPlacer',
    'TopologyBalloonPlacer',
    'BOMTable',
    # Functions
    'create_axis_indicator_svg',
    'create_standard_views',
    'aggregate_components_to_bom',
    'get_free_directions',
    'get_perpendicular_directions',
    # Constants
    'SHEET_WIDTH_MM',
    'SHEET_HEIGHT_MM',
    'MARGIN',
    'DRAWING_AREA_WIDTH',
    'DRAWING_AREA_HEIGHT',
    'WORLD_DIR_TO_SCREEN_OFFSET',
]
