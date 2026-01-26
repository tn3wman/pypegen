"""
Layout engine for multi-view orthographic drawings.

This module provides the ViewLayoutEngine class which handles:
- View arrangement in standard 3rd angle projection layout
- Overlap detection between views
- Optimal scale calculation
- Layout optimization to prevent content overlap
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import cadquery as cq

from .constants import VIEW_SPACING
from .view_area import ViewArea
from .view_projection import ViewProjection, create_standard_views

if TYPE_CHECKING:
    from .bom import ComponentRecord


@dataclass
class ComponentBounds2D:
    """2D bounding box for a component in a specific view."""

    component_id: int
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    @property
    def width(self) -> float:
        return self.max_x - self.min_x

    @property
    def height(self) -> float:
        return self.max_y - self.min_y

    @property
    def center(self) -> tuple[float, float]:
        return ((self.min_x + self.max_x) / 2, (self.min_y + self.max_y) / 2)

    def overlaps(self, other: "ComponentBounds2D") -> bool:
        """Check if this bounds overlaps with another."""
        return not (
            self.max_x < other.min_x
            or other.max_x < self.min_x
            or self.max_y < other.min_y
            or other.max_y < self.min_y
        )

    def expand(self, margin: float) -> "ComponentBounds2D":
        """Return a new bounds expanded by margin on all sides."""
        return ComponentBounds2D(
            component_id=self.component_id,
            min_x=self.min_x - margin,
            min_y=self.min_y - margin,
            max_x=self.max_x + margin,
            max_y=self.max_y + margin,
        )


@dataclass
class ViewBounds:
    """Collection of component bounds for a single view."""

    view_name: str
    view_area: ViewArea
    component_bounds: list[ComponentBounds2D] = field(default_factory=list)

    @property
    def content_bounds(self) -> tuple[float, float, float, float] | None:
        """Get the overall content bounds (union of all component bounds)."""
        if not self.component_bounds:
            return None

        min_x = min(b.min_x for b in self.component_bounds)
        min_y = min(b.min_y for b in self.component_bounds)
        max_x = max(b.max_x for b in self.component_bounds)
        max_y = max(b.max_y for b in self.component_bounds)

        return (min_x, min_y, max_x, max_y)


class ViewLayoutEngine:
    """
    Engine for arranging and optimizing multi-view layouts.

    This class handles:
    - Creating view projections for standard 3rd angle layout
    - Computing component bounds in each view
    - Detecting overlaps between views
    - Optimizing layout to eliminate overlaps
    """

    def __init__(
        self,
        drawing_area: ViewArea,
        spacing: float = VIEW_SPACING,
    ):
        """
        Initialize the layout engine.

        Args:
            drawing_area: Total available area for views
            spacing: Minimum gap between views (mm)
        """
        self.drawing_area = drawing_area
        self.spacing = spacing
        self.views: dict[str, ViewProjection] = {}
        self.view_bounds: dict[str, ViewBounds] = {}

    def create_layout(
        self,
        shape: cq.Shape,
        components: list["ComponentRecord"] | None = None,
    ) -> dict[str, ViewProjection]:
        """
        Create the view layout and compute all bounds.

        Args:
            shape: The 3D assembly shape
            components: Optional list of component records for overlap detection

        Returns:
            Dictionary of view name to configured ViewProjection
        """
        # Create initial 2x2 layout
        self.views = create_standard_views(self.drawing_area, self.spacing)

        # Render each view and extract transform parameters
        # Apply the same model rotations that will be used during final rendering
        for name, view in self.views.items():
            rotated_shape = self._apply_view_rotation(shape, name, view.is_isometric)
            svg = view.render_geometry_svg(rotated_shape)
            view.set_from_rendered_svg(svg, fit_margin=10.0)

        # If components provided, compute bounds and check overlaps
        if components:
            self._compute_all_component_bounds(components)

            # Optimize layout if needed
            if self._has_cross_view_overlaps():
                self._optimize_layout(shape, components)

        return self.views

    def _apply_view_rotation(
        self,
        shape: cq.Shape,
        view_name: str,
        is_isometric: bool,
    ) -> cq.Shape:
        """
        Apply the appropriate model rotation for a view.

        This ensures the geometry center is computed for the same rotated
        geometry that will be rendered in the final drawing.

        Args:
            shape: The 3D shape to rotate
            view_name: Name of the view ("front", "top", "right", "isometric")
            is_isometric: Whether this is the isometric view

        Returns:
            The rotated shape
        """
        if is_isometric:
            # Isometric: rotate -90° around X for piping convention
            return shape.rotate((0, 0, 0), (1, 0, 0), -90)
        elif view_name == "front":
            # Front view (Looking North): rotate +90° around Y
            return shape.rotate((0, 0, 0), (0, 1, 0), 90)
        elif view_name == "right":
            # Right view (Looking West): rotate +90° around X
            return shape.rotate((0, 0, 0), (1, 0, 0), 90)
        # Top view: no rotation needed
        return shape

    def _compute_all_component_bounds(
        self,
        components: list["ComponentRecord"],
    ) -> None:
        """Compute 2D bounds for all components in all views."""
        self.view_bounds = {}

        for name, view in self.views.items():
            bounds_list = []

            for i, comp in enumerate(components):
                if comp.shape is None:
                    continue

                # Get 3D bounding box
                try:
                    bb = comp.shape.BoundingBox()
                    bb_min = (bb.xmin, bb.ymin, bb.zmin)
                    bb_max = (bb.xmax, bb.ymax, bb.zmax)

                    # Project to 2D in this view
                    svg_bounds = view.world_bounds_to_svg(bb_min, bb_max)

                    bounds_list.append(
                        ComponentBounds2D(
                            component_id=i,
                            min_x=svg_bounds[0],
                            min_y=svg_bounds[1],
                            max_x=svg_bounds[2],
                            max_y=svg_bounds[3],
                        )
                    )
                except Exception:
                    # Skip components with invalid geometry
                    continue

            self.view_bounds[name] = ViewBounds(
                view_name=name,
                view_area=view.view_area,
                component_bounds=bounds_list,
            )

    def _has_cross_view_overlaps(self) -> bool:
        """
        Check if any component in one view overlaps with any component
        in another view (in sheet coordinates).

        Returns:
            True if overlaps exist between different views
        """
        view_names = list(self.view_bounds.keys())

        for i, name1 in enumerate(view_names):
            for name2 in view_names[i + 1 :]:
                bounds1 = self.view_bounds[name1].component_bounds
                bounds2 = self.view_bounds[name2].component_bounds

                for b1 in bounds1:
                    for b2 in bounds2:
                        if b1.overlaps(b2):
                            return True

        return False

    def _optimize_layout(
        self,
        shape: cq.Shape,
        components: list["ComponentRecord"],
        max_iterations: int = 10,
    ) -> None:
        """
        Optimize layout by adjusting scale until no overlaps.

        Args:
            shape: The 3D assembly shape
            components: Component records
            max_iterations: Maximum optimization iterations
        """
        scale_factor = 0.95  # Reduce by 5% each iteration

        for _ in range(max_iterations):
            # Reduce spacing (effectively scales down views)
            self.spacing *= 1.1  # Increase spacing

            # Recreate layout with new spacing
            self.views = create_standard_views(self.drawing_area, self.spacing)

            # Re-render and recompute bounds
            for name, view in self.views.items():
                svg = view.render_geometry_svg(shape)
                view.set_from_rendered_svg(svg, fit_margin=10.0)

            self._compute_all_component_bounds(components)

            if not self._has_cross_view_overlaps():
                break

    def get_unified_scale(self) -> float:
        """
        Get a unified scale factor that works for all views.

        For consistent appearance, all views should use the same scale.

        Returns:
            Minimum fit_scale across all views
        """
        if not self.views:
            return 1.0

        scales = [v._fit_scale for v in self.views.values() if v._initialized]
        return min(scales) if scales else 1.0

    def apply_unified_scale(self, scale: float) -> None:
        """
        Apply a unified scale to all views.

        Args:
            scale: Scale factor to apply to all views
        """
        for view in self.views.values():
            view._fit_scale = scale

    def get_overlap_report(self) -> list[tuple[str, str, int, int]]:
        """
        Get a report of all cross-view overlaps.

        Returns:
            List of (view1_name, view2_name, comp1_id, comp2_id) tuples
        """
        overlaps = []
        view_names = list(self.view_bounds.keys())

        for i, name1 in enumerate(view_names):
            for name2 in view_names[i + 1 :]:
                bounds1 = self.view_bounds[name1].component_bounds
                bounds2 = self.view_bounds[name2].component_bounds

                for b1 in bounds1:
                    for b2 in bounds2:
                        if b1.overlaps(b2):
                            overlaps.append(
                                (name1, name2, b1.component_id, b2.component_id)
                            )

        return overlaps


def compute_optimal_view_scale(
    shape: cq.Shape,
    view_area: ViewArea,
    direction: tuple[float, float, float],
    margin: float = 10.0,
) -> float:
    """
    Compute the optimal scale for a view to fit geometry with margin.

    Args:
        shape: CadQuery shape to measure
        view_area: Target view area
        direction: Projection direction
        margin: Margin around geometry (mm)

    Returns:
        Scale factor to fit geometry in view
    """
    # Create temporary view projection
    view = ViewProjection(
        name="temp",
        direction=direction,
        up_vector=(0, 0, 1),  # Default up
        view_area=view_area,
    )

    # Render and extract bounds
    svg = view.render_geometry_svg(shape)
    view.set_from_rendered_svg(svg, fit_margin=margin)

    return view._fit_scale


def rectangles_overlap(
    r1: tuple[float, float, float, float],
    r2: tuple[float, float, float, float],
) -> bool:
    """
    Check if two rectangles overlap.

    Args:
        r1, r2: Rectangles as (min_x, min_y, max_x, max_y)

    Returns:
        True if rectangles overlap
    """
    return not (
        r1[2] < r2[0]  # r1 right < r2 left
        or r2[2] < r1[0]  # r2 right < r1 left
        or r1[3] < r2[1]  # r1 bottom < r2 top
        or r2[3] < r1[1]  # r2 bottom < r1 top
    )
