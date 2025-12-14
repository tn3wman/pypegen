"""
ViewArea class for managing rectangular areas on drawing sheets.
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class ViewArea:
    """
    Represents a rectangular area on the drawing sheet for placing content.

    This class manages the geometry of a view area including its position,
    size, and provides helper methods for content placement.

    Attributes:
        x: Left edge position (mm from sheet left)
        y: Top edge position (mm from sheet top)
        width: Width of the area (mm)
        height: Height of the area (mm)
    """
    x: float
    y: float
    width: float
    height: float

    @property
    def left(self) -> float:
        """Left edge x-coordinate."""
        return self.x

    @property
    def right(self) -> float:
        """Right edge x-coordinate."""
        return self.x + self.width

    @property
    def top(self) -> float:
        """Top edge y-coordinate."""
        return self.y

    @property
    def bottom(self) -> float:
        """Bottom edge y-coordinate."""
        return self.y + self.height

    @property
    def center_x(self) -> float:
        """X-coordinate of the center point."""
        return self.x + self.width / 2

    @property
    def center_y(self) -> float:
        """Y-coordinate of the center point."""
        return self.y + self.height / 2

    @property
    def center(self) -> Tuple[float, float]:
        """Center point as (x, y) tuple."""
        return (self.center_x, self.center_y)

    @property
    def size(self) -> Tuple[float, float]:
        """Size as (width, height) tuple."""
        return (self.width, self.height)

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """Bounds as (left, top, right, bottom) tuple."""
        return (self.left, self.top, self.right, self.bottom)

    def inset(self, margin: float) -> 'ViewArea':
        """Return a new ViewArea inset by the given margin on all sides."""
        return ViewArea(
            x=self.x + margin,
            y=self.y + margin,
            width=self.width - 2 * margin,
            height=self.height - 2 * margin
        )

    def inset_sides(self, left: float = 0, top: float = 0,
                    right: float = 0, bottom: float = 0) -> 'ViewArea':
        """Return a new ViewArea inset by different amounts on each side."""
        return ViewArea(
            x=self.x + left,
            y=self.y + top,
            width=self.width - left - right,
            height=self.height - top - bottom
        )

    def position_top_left(self, margin: float = 0) -> Tuple[float, float]:
        """Get position for placing content in top-left corner."""
        return (self.x + margin, self.y + margin)

    def position_top_right(self, margin: float = 0) -> Tuple[float, float]:
        """Get position for placing content in top-right corner."""
        return (self.right - margin, self.y + margin)

    def position_bottom_left(self, margin: float = 0) -> Tuple[float, float]:
        """Get position for placing content in bottom-left corner."""
        return (self.x + margin, self.bottom - margin)

    def position_bottom_right(self, margin: float = 0) -> Tuple[float, float]:
        """Get position for placing content in bottom-right corner."""
        return (self.right - margin, self.bottom - margin)

    def svg_rect(self, stroke: str = "#000000", stroke_width: float = 0.5,
                 fill: str = "none", **attrs) -> str:
        """Generate an SVG rect element for this area."""
        extra = ' '.join(f'{k.replace("_", "-")}="{v}"' for k, v in attrs.items())
        extra_str = f" {extra}" if extra else ""
        return (f'<rect x="{self.x}" y="{self.y}" width="{self.width}" '
                f'height="{self.height}" fill="{fill}" stroke="{stroke}" '
                f'stroke-width="{stroke_width}"{extra_str}/>')

    def __repr__(self) -> str:
        return (f"ViewArea(x={self.x}, y={self.y}, "
                f"w={self.width}, h={self.height}, center={self.center})")
