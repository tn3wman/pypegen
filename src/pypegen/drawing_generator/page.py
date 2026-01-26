"""
Multi-page drawing support.

This module provides classes for managing multi-page engineering drawings,
with separate pages for orthographic views and BOM/notes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from .constants import (
    SHEET_WIDTH_MM,
    SHEET_HEIGHT_MM,
    MARGIN,
    BORDER_COLOR,
    BORDER_WIDTH,
)
from .view_area import ViewArea

if TYPE_CHECKING:
    from .title_block import TitleBlock


@dataclass
class DrawingPage:
    """
    Represents a single page in a multi-page drawing.

    Attributes:
        page_number: Page number (1-indexed)
        total_pages: Total number of pages in the drawing
        title_block: Title block for this page
        content_elements: List of SVG element strings
        page_title: Optional title displayed on the page
    """

    page_number: int
    total_pages: int
    title_block: "TitleBlock | None" = None
    content_elements: list[str] = field(default_factory=list)
    page_title: str = ""

    # Sheet dimensions
    width: float = SHEET_WIDTH_MM
    height: float = SHEET_HEIGHT_MM
    margin: float = MARGIN

    def add_element(self, svg_element: str) -> None:
        """Add an SVG element to the page content."""
        self.content_elements.append(svg_element)

    def add_elements(self, elements: list[str]) -> None:
        """Add multiple SVG elements to the page content."""
        self.content_elements.extend(elements)

    @property
    def drawing_area(self) -> ViewArea:
        """Get the available drawing area (excluding margins and title block)."""
        title_block_height = 35 if self.title_block else 0
        return ViewArea(
            x=self.margin,
            y=self.margin,
            width=self.width - 2 * self.margin,
            height=self.height - 2 * self.margin - title_block_height,
        )

    def generate_svg(self) -> str:
        """
        Generate complete SVG for this page.

        Returns:
            Complete SVG document as string
        """
        svg_parts = [
            self._svg_header(),
            self._svg_styles(),
            self._svg_background(),
            self._svg_border(),
        ]

        # Add page number indicator
        svg_parts.append(self._svg_page_indicator())

        # Add all content elements
        svg_parts.extend(self.content_elements)

        # Add title block if present
        if self.title_block:
            svg_parts.append(self.title_block.generate_svg())

        svg_parts.append("</svg>")

        return "\n".join(svg_parts)

    def _svg_header(self) -> str:
        """Generate SVG header."""
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'width="{self.width}mm" height="{self.height}mm" '
            f'viewBox="0 0 {self.width} {self.height}">'
        )

    def _svg_styles(self) -> str:
        """Generate SVG style definitions."""
        return """
<defs>
  <style type="text/css">
    .visible { stroke: #000000; stroke-width: 0.5; fill: none; }
    .hidden { stroke: #888888; stroke-width: 0.25; fill: none; stroke-dasharray: 2,1; }
    .border { stroke: #000000; stroke-width: 0.5; fill: none; }
    .thin-line { stroke: #000000; stroke-width: 0.25; fill: none; }
    .text { font-family: Arial, sans-serif; fill: #000000; }
    .title { font-size: 6px; font-weight: bold; }
    .label { font-size: 4px; }
    .small { font-size: 3px; }
  </style>
</defs>"""

    def _svg_background(self) -> str:
        """Generate white background."""
        return (
            f'<rect x="0" y="0" width="{self.width}" height="{self.height}" '
            f'fill="white"/>'
        )

    def _svg_border(self) -> str:
        """Generate drawing border."""
        return (
            f'<rect x="{self.margin}" y="{self.margin}" '
            f'width="{self.width - 2 * self.margin}" '
            f'height="{self.height - 2 * self.margin}" '
            f'fill="none" stroke="{BORDER_COLOR}" '
            f'stroke-width="{BORDER_WIDTH}"/>'
        )

    def _svg_page_indicator(self) -> str:
        """Generate page number indicator."""
        if self.total_pages <= 1:
            return ""

        x = self.width - self.margin - 5
        y = self.margin + 5

        return (
            f'<text x="{x}" y="{y}" '
            f'text-anchor="end" '
            f'font-family="Arial, sans-serif" '
            f'font-size="3" '
            f'fill="#666666">'
            f"Page {self.page_number} of {self.total_pages}</text>"
        )


@dataclass
class MultiPageDrawing:
    """
    Container for multi-page engineering drawings.

    Manages a collection of pages and provides methods for
    generating combined output (multiple SVGs or multi-page PDF).
    """

    pages: list[DrawingPage] = field(default_factory=list)
    title: str = ""
    drawing_number: str = ""

    def add_page(self, page: DrawingPage) -> None:
        """Add a page to the drawing."""
        self.pages.append(page)
        self._update_page_numbers()

    def create_page(
        self,
        title_block: "TitleBlock | None" = None,
        page_title: str = "",
    ) -> DrawingPage:
        """
        Create and add a new page.

        Args:
            title_block: Optional title block for the page
            page_title: Optional title for the page

        Returns:
            The created page
        """
        page = DrawingPage(
            page_number=len(self.pages) + 1,
            total_pages=len(self.pages) + 1,
            title_block=title_block,
            page_title=page_title,
        )
        self.add_page(page)
        return page

    def _update_page_numbers(self) -> None:
        """Update page numbers and total count on all pages."""
        total = len(self.pages)
        for i, page in enumerate(self.pages):
            page.page_number = i + 1
            page.total_pages = total

    def generate_svgs(self) -> list[str]:
        """
        Generate SVG for each page.

        Returns:
            List of SVG strings, one per page
        """
        return [page.generate_svg() for page in self.pages]

    def export_svg_files(self, base_path: str) -> list[str]:
        """
        Export each page to a separate SVG file.

        Args:
            base_path: Base file path (without extension)

        Returns:
            List of created file paths
        """
        paths = []

        for i, page in enumerate(self.pages):
            if len(self.pages) == 1:
                path = f"{base_path}.svg"
            else:
                path = f"{base_path}_page{i + 1}.svg"

            with open(path, "w", encoding="utf-8") as f:
                f.write(page.generate_svg())

            paths.append(path)

        return paths

    def export_pdf(self, output_path: str) -> None:
        """
        Export all pages to a single multi-page PDF.

        Args:
            output_path: Output PDF file path
        """
        try:
            from svglib.svglib import renderSVG
            from reportlab.graphics import renderPDF
            from reportlab.pdfgen import canvas
            from reportlab.lib.pagesizes import landscape
            from reportlab.lib.units import mm
            import io
        except ImportError as e:
            raise ImportError(
                "PDF export requires svglib and reportlab. "
                "Install with: pip install svglib reportlab"
            ) from e

        if not self.pages:
            raise ValueError("No pages to export")

        # Get page size from first page
        page_width = self.pages[0].width * mm
        page_height = self.pages[0].height * mm

        # Create PDF canvas
        c = canvas.Canvas(output_path, pagesize=(page_width, page_height))

        for i, page in enumerate(self.pages):
            if i > 0:
                c.showPage()

            # Convert SVG to reportlab drawing
            svg_content = page.generate_svg()
            svg_io = io.StringIO(svg_content)
            drawing = renderSVG(svg_io)

            # Scale drawing to fit page
            scale_x = page_width / drawing.width
            scale_y = page_height / drawing.height
            scale = min(scale_x, scale_y)

            drawing.width *= scale
            drawing.height *= scale
            drawing.scale(scale, scale)

            # Render to PDF
            renderPDF.draw(drawing, c, 0, 0)

        c.save()


def create_orthographic_pages(
    title_block_factory: callable,
    drawing_title: str = "",
    drawing_number: str = "",
) -> MultiPageDrawing:
    """
    Create a multi-page drawing structure for orthographic views.

    Page 1: Orthographic views (Front, Top, Right, Isometric)
    Page 2: BOM and notes

    Args:
        title_block_factory: Callable that creates a TitleBlock
        drawing_title: Drawing title
        drawing_number: Drawing number

    Returns:
        MultiPageDrawing with two pages configured
    """
    drawing = MultiPageDrawing(
        title=drawing_title,
        drawing_number=drawing_number,
    )

    # Page 1: Views
    page1 = drawing.create_page(
        title_block=title_block_factory(
            title=drawing_title,
            drawing_number=drawing_number,
            sheet="1 OF 2",
        ),
        page_title="ORTHOGRAPHIC VIEWS",
    )

    # Page 2: BOM
    page2 = drawing.create_page(
        title_block=title_block_factory(
            title=drawing_title,
            drawing_number=drawing_number,
            sheet="2 OF 2",
        ),
        page_title="BILL OF MATERIALS",
    )

    return drawing
