"""
ViewProjection class for managing orthographic view projections.

This module provides the ViewProjection class which encapsulates all the
parameters and transformations needed to render a single orthographic view
of a 3D assembly onto a 2D drawing sheet.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import cadquery as cq
from cadquery.occ_impl.exporters.svg import getSVG

from .constants import (
    VIEW_DIRECTIONS,
    VIEW_UP_VECTORS,
    VIEW_LABELS,
    VISIBLE_STROKE_WIDTH,
    ROTATE_MODEL_FOR_PROJECTION,
)
from .view_area import ViewArea

if TYPE_CHECKING:
    pass


@dataclass
class ViewProjection:
    """
    Represents a single orthographic view projection.

    This class encapsulates the projection direction, coordinate mapping,
    and view area for rendering one view of a 3D assembly.

    Attributes:
        name: View name ("front", "top", "right", "isometric")
        direction: Camera direction vector (where camera is located)
        up_vector: Which direction is "up" in this view
        view_area: Rectangle defining position on sheet
    """

    name: str
    direction: tuple[float, float, float]
    up_vector: tuple[float, float, float]
    view_area: ViewArea

    # Computed projection parameters (set after rendering geometry)
    _cq_scale_x: float = field(default=1.0, init=False, repr=False)
    _cq_scale_y: float = field(default=-1.0, init=False, repr=False)
    _cq_translate_x: float = field(default=0.0, init=False, repr=False)
    _cq_translate_y: float = field(default=0.0, init=False, repr=False)
    _geometry_center_x: float = field(default=0.0, init=False, repr=False)
    _geometry_center_y: float = field(default=0.0, init=False, repr=False)
    _fit_scale: float = field(default=1.0, init=False, repr=False)
    _raw_bounds: tuple[float, float, float, float] = field(
        default=(0.0, 0.0, 0.0, 0.0), init=False, repr=False
    )
    _initialized: bool = field(default=False, init=False, repr=False)

    # Projection matrices (computed from direction)
    _proj_right: tuple[float, float, float] = field(
        default=(1.0, 0.0, 0.0), init=False, repr=False
    )
    _proj_up: tuple[float, float, float] = field(
        default=(0.0, 1.0, 0.0), init=False, repr=False
    )

    def __post_init__(self) -> None:
        """Compute projection basis vectors from direction and up vector."""
        self._compute_projection_basis()

    def _compute_projection_basis(self) -> None:
        """
        Compute the right and up vectors for orthographic projection.

        For orthographic projection from direction D with up hint U:
        - Right = normalize(D × U)
        - Up = normalize(Right × D)
        """
        dx, dy, dz = self.direction
        ux, uy, uz = self.up_vector

        # Normalize direction
        d_len = math.sqrt(dx * dx + dy * dy + dz * dz)
        dx, dy, dz = dx / d_len, dy / d_len, dz / d_len

        # Right = D × U (cross product)
        rx = dy * uz - dz * uy
        ry = dz * ux - dx * uz
        rz = dx * uy - dy * ux

        r_len = math.sqrt(rx * rx + ry * ry + rz * rz)
        if r_len < 1e-10:
            # Direction parallel to up, use fallback
            if abs(dz) > 0.9:
                rx, ry, rz = 1.0, 0.0, 0.0
            else:
                rx, ry, rz = 0.0, 0.0, 1.0
            # Recalculate cross product
            rx = dy * rz - dz * ry
            ry_new = dz * rx - dx * rz
            rz_new = dx * ry - dy * rx
            rx, ry, rz = rx, ry_new, rz_new
            r_len = math.sqrt(rx * rx + ry * ry + rz * rz)

        rx, ry, rz = rx / r_len, ry / r_len, rz / r_len

        # Up = Right × D (ensures orthogonal)
        upx = ry * dz - rz * dy
        upy = rz * dx - rx * dz
        upz = rx * dy - ry * dx

        up_len = math.sqrt(upx * upx + upy * upy + upz * upz)
        upx, upy, upz = upx / up_len, upy / up_len, upz / up_len

        self._proj_right = (rx, ry, rz)
        self._proj_up = (upx, upy, upz)

    @classmethod
    def from_name(cls, name: str, view_area: ViewArea) -> "ViewProjection":
        """
        Create a ViewProjection from a standard view name.

        Args:
            name: One of "front", "top", "right", "isometric"
            view_area: Rectangle defining position on sheet

        Returns:
            Configured ViewProjection instance
        """
        if name not in VIEW_DIRECTIONS:
            raise ValueError(
                f"Unknown view name: {name}. "
                f"Valid names: {list(VIEW_DIRECTIONS.keys())}"
            )

        return cls(
            name=name,
            direction=VIEW_DIRECTIONS[name],
            up_vector=VIEW_UP_VECTORS[name],
            view_area=view_area,
        )

    @property
    def label(self) -> str:
        """Get the display label for this view."""
        return VIEW_LABELS.get(self.name, self.name.upper())

    @property
    def is_isometric(self) -> bool:
        """Check if this is an isometric view."""
        return self.name == "isometric"

    def project_point(
        self, x: float, y: float, z: float, apply_model_rotation: bool = True
    ) -> tuple[float, float]:
        """
        Project a 3D point to 2D raw projection space.

        This returns coordinates in the projection's local 2D space,
        NOT final SVG coordinates. Use world_to_svg() for SVG coords.

        Args:
            x, y, z: 3D world position (X=East, Y=North, Z=Up)
            apply_model_rotation: Whether to apply -90° X rotation first

        Returns:
            (proj_x, proj_y): 2D coordinates in projection space
        """
        # Apply model rotation if needed (for isometric compatibility)
        if apply_model_rotation and self.is_isometric and ROTATE_MODEL_FOR_PROJECTION:
            # Rotate -90° around X: (x, y, z) → (x, -z, y)
            px, py, pz = x, -z, y
        else:
            px, py, pz = x, y, z

        # Project using basis vectors
        rx, ry, rz = self._proj_right
        ux, uy, uz = self._proj_up

        proj_x = px * rx + py * ry + pz * rz
        proj_y = px * ux + py * uy + pz * uz

        return (proj_x, proj_y)

    def world_to_svg(self, x: float, y: float, z: float) -> tuple[float, float]:
        """
        Convert a 3D world position to 2D SVG screen coordinates.

        This applies the full transformation chain:
        1. Optional model rotation
        2. Orthographic projection
        3. CadQuery transform (scale + translate)
        4. Fit transform (center in view area)

        Args:
            x, y, z: Position in 3D world (X=East, Y=North, Z=Up)

        Returns:
            (svg_x, svg_y): Position in SVG coordinates (mm on sheet)
        """
        if not self._initialized:
            raise ValueError(
                "ViewProjection not initialized. "
                "Call set_from_rendered_svg() first."
            )

        # Get raw projection
        raw_x, raw_y = self.project_point(x, y, z)

        # Apply CadQuery's SVG transform
        cq_x = self._cq_scale_x * (raw_x + self._cq_translate_x)
        cq_y = self._cq_scale_y * (raw_y + self._cq_translate_y)

        # Apply fit transform
        svg_x = self.view_area.center_x + self._fit_scale * (
            cq_x - self._geometry_center_x
        )
        svg_y = self.view_area.center_y + self._fit_scale * (
            cq_y - self._geometry_center_y
        )

        return (svg_x, svg_y)

    def world_bounds_to_svg(
        self,
        bb_min: tuple[float, float, float],
        bb_max: tuple[float, float, float],
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

    def render_geometry_svg(
        self,
        shape: cq.Shape,
        width: float = 600,
        height: float = 400,
        show_hidden: bool = True,
    ) -> str:
        """
        Render geometry to SVG from this view's direction.

        Args:
            shape: CadQuery shape to render
            width: SVG output width (pixels)
            height: SVG output height (pixels)
            show_hidden: Whether to show hidden lines

        Returns:
            SVG string of rendered geometry
        """
        opts = {
            "width": width,
            "height": height,
            "marginLeft": 10,
            "marginTop": 10,
            "projectionDir": self.direction,
            "showAxes": False,
            "strokeWidth": VISIBLE_STROKE_WIDTH,
            "strokeColor": (0, 0, 0),
            "hiddenColor": (136, 136, 136),
            "showHidden": show_hidden,
        }
        return getSVG(shape, opts)

    def set_from_rendered_svg(
        self,
        svg_content: str,
        fit_margin: float = 10.0,
    ) -> None:
        """
        Extract CadQuery transform parameters from rendered SVG.

        This parses the SVG to extract the scale/translate transform
        and computes the fit scale to center geometry in the view area.

        Args:
            svg_content: Raw SVG string from CadQuery
            fit_margin: Margin around geometry (mm)
        """
        # Parse transform from SVG
        self._parse_cq_transform(svg_content)

        # Compute geometry bounds and fit scale
        self._compute_fit_parameters(svg_content, fit_margin)

        self._initialized = True

    def _parse_cq_transform(self, svg_content: str) -> None:
        """Extract scale and translate from CadQuery's SVG transform."""
        # Look for transform="scale(x,y) translate(x,y)"
        # Usually on the main <g> element
        transform_match = re.search(
            r'transform\s*=\s*"([^"]+)"',
            svg_content,
        )

        if transform_match:
            transform_str = transform_match.group(1)

            # Parse scale
            scale_match = re.search(
                r"scale\(([-\d.]+),\s*([-\d.]+)\)",
                transform_str,
            )
            if scale_match:
                self._cq_scale_x = float(scale_match.group(1))
                self._cq_scale_y = float(scale_match.group(2))

            # Parse translate
            trans_match = re.search(
                r"translate\(([-\d.]+),\s*([-\d.]+)\)",
                transform_str,
            )
            if trans_match:
                self._cq_translate_x = float(trans_match.group(1))
                self._cq_translate_y = float(trans_match.group(2))

    def _compute_fit_parameters(
        self,
        svg_content: str,
        margin: float,
    ) -> None:
        """Compute geometry center and fit scale from SVG content."""
        # Extract path coordinates to find bounds
        path_coords = self._extract_path_coordinates(svg_content)

        if not path_coords:
            # Fallback to viewBox or default
            self._geometry_center_x = 0.0
            self._geometry_center_y = 0.0
            self._fit_scale = 1.0
            return

        min_x = min(c[0] for c in path_coords)
        max_x = max(c[0] for c in path_coords)
        min_y = min(c[1] for c in path_coords)
        max_y = max(c[1] for c in path_coords)

        self._raw_bounds = (min_x, min_y, max_x, max_y)
        self._geometry_center_x = (min_x + max_x) / 2
        self._geometry_center_y = (min_y + max_y) / 2

        # Compute scale to fit in view area with margin
        geom_width = max_x - min_x
        geom_height = max_y - min_y

        if geom_width > 0 and geom_height > 0:
            available_width = self.view_area.width - 2 * margin
            available_height = self.view_area.height - 2 * margin

            scale_x = available_width / geom_width
            scale_y = available_height / geom_height
            self._fit_scale = min(scale_x, scale_y)
        else:
            self._fit_scale = 1.0

    def _extract_path_coordinates(
        self, svg_content: str
    ) -> list[tuple[float, float]]:
        """Extract coordinates from SVG path elements.

        Uses proper SVG path command parsing to extract only M (moveto)
        and L (lineto) coordinates, avoiding incorrect parsing of other
        numeric values in the path data.
        """
        coords: list[tuple[float, float]] = []

        # Find all path d="" attributes
        path_pattern = re.compile(r'd\s*=\s*"([^"]+)"')

        for match in path_pattern.finditer(svg_content):
            path_data = match.group(1)
            # Extract M/L command coordinates (x,y pairs after M or L)
            # This properly handles SVG path format: M x,y L x,y L x,y ...
            coord_matches = re.findall(
                r'[ML]([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)',
                path_data
            )
            for x_str, y_str in coord_matches:
                try:
                    x = float(x_str)
                    y = float(y_str)
                    coords.append((x, y))
                except ValueError:
                    continue

        return coords

    def generate_view_label_svg(self, offset_y: float = 5.0) -> str:
        """
        Generate SVG for the view label below the view area.

        Args:
            offset_y: Distance below view border (mm)

        Returns:
            SVG text element for the label
        """
        label_x = self.view_area.center_x
        label_y = self.view_area.bottom + offset_y

        return (
            f'<text x="{label_x}" y="{label_y}" '
            f'text-anchor="middle" '
            f'font-family="Arial, sans-serif" '
            f'font-size="4" '
            f'font-weight="bold">'
            f"{self.label}</text>"
        )

    def generate_view_border_svg(
        self,
        stroke: str = "#000000",
        stroke_width: float = 0.25,
    ) -> str:
        """
        Generate SVG rectangle for the view border.

        Returns:
            SVG rect element for the border
        """
        return self.view_area.svg_rect(
            stroke=stroke,
            stroke_width=stroke_width,
            fill="none",
        )


def create_standard_views(
    drawing_area: ViewArea,
    spacing: float = 15.0,
) -> dict[str, ViewProjection]:
    """
    Create the standard 4-view layout for 3rd angle projection.

    Layout:
    +--------+--------+
    |  TOP   |  ISO   |
    +--------+--------+
    | FRONT  | RIGHT  |
    +--------+--------+

    Args:
        drawing_area: Total available area for all views
        spacing: Gap between views (mm)

    Returns:
        Dictionary of view name to ViewProjection
    """
    # Calculate view dimensions (2x2 grid with spacing)
    view_width = (drawing_area.width - spacing) / 2
    view_height = (drawing_area.height - spacing) / 2

    views = {}

    # Top view (upper-left)
    views["top"] = ViewProjection.from_name(
        "top",
        ViewArea(
            x=drawing_area.x,
            y=drawing_area.y,
            width=view_width,
            height=view_height,
        ),
    )

    # Isometric view (upper-right)
    views["isometric"] = ViewProjection.from_name(
        "isometric",
        ViewArea(
            x=drawing_area.x + view_width + spacing,
            y=drawing_area.y,
            width=view_width,
            height=view_height,
        ),
    )

    # Front view (lower-left)
    views["front"] = ViewProjection.from_name(
        "front",
        ViewArea(
            x=drawing_area.x,
            y=drawing_area.y + view_height + spacing,
            width=view_width,
            height=view_height,
        ),
    )

    # Right view (lower-right)
    views["right"] = ViewProjection.from_name(
        "right",
        ViewArea(
            x=drawing_area.x + view_width + spacing,
            y=drawing_area.y + view_height + spacing,
            width=view_width,
            height=view_height,
        ),
    )

    return views
