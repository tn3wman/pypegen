"""
Title block generator for engineering drawings.
"""

from dataclasses import dataclass, field
from datetime import date
from typing import Optional

from .constants import BORDER_COLOR, BORDER_WIDTH, THIN_LINE_WIDTH, TITLE_BLOCK_HEIGHT, TITLE_BLOCK_WIDTH, TITLE_BLOCK_X, TITLE_BLOCK_Y
from .view_area import ViewArea


@dataclass
class TitleBlockInfo:
    """Information displayed in the title block."""
    title: str = "PIPING ASSEMBLY"
    drawing_number: str = "SPOOL-001"
    revision: str = "0"
    scale_text: str = "NOT TO SCALE"
    description: str = ""  # Verbose description of the drawing
    project_number: str = ""
    fund_number: str = ""
    drawn_by: str = ""
    drawn_date: Optional[str] = None  # noqa: UP045
    checked_by: str = ""
    checked_date: str | None = None
    # Company branding
    company_name: str = ""  # Company name to display
    company_logo_path: str | None = None  # Path to logo image (PNG, SVG, etc.)

    def __post_init__(self):
        today = date.today().strftime("%Y-%m-%d")
        if self.drawn_date is None:
            self.drawn_date = today
        if self.checked_date is None:
            self.checked_date = ""


@dataclass
class TitleBlock:
    """
    Generates an engineering drawing title block.

    The title block is positioned on the right side of the drawing
    and contains project information, revision history, and notes.
    """
    info: TitleBlockInfo = field(default_factory=TitleBlockInfo)
    area: ViewArea = field(init=False)

    def __post_init__(self):
        self.area = ViewArea(
            x=TITLE_BLOCK_X,
            y=TITLE_BLOCK_Y,
            width=TITLE_BLOCK_WIDTH,
            height=TITLE_BLOCK_HEIGHT
        )

    def _generate_metadata_row(self, mid_x: float, pe_stamp_x: float, tb_y: float, row_height: float, info: TitleBlockInfo) -> str:
        """Generate equally-spaced metadata columns with vertical dividers."""
        row_width = pe_stamp_x - mid_x

        # Define metadata items: (label, value)
        items = [
            ("PRJ", info.project_number),
            ("DWG", info.drawing_number),
            ("REV", info.revision),
            ("SCALE", info.scale_text),
            ("FUND", info.fund_number),
        ]

        num_cols = len(items)
        col_width = row_width / num_cols
        text_y = tb_y + row_height * 0.6  # Vertically centered text

        svg_parts = []

        for i, (label, value) in enumerate(items):
            col_x = mid_x + i * col_width
            text_x = col_x + 2  # Left-aligned with padding

            # Add vertical divider line (except for first column)
            if i > 0:
                svg_parts.append(
                    f'<line x1="{col_x}" y1="{tb_y}" x2="{col_x}" y2="{tb_y + row_height}" '
                    f'stroke="{BORDER_COLOR}" stroke-width="{THIN_LINE_WIDTH}"/>'
                )

            # Add label and value text, left-aligned in column
            svg_parts.append(
                f'<text x="{text_x}" y="{text_y}" '
                f'font-family="Arial" font-size="1.3">{label}: {value}</text>'
            )

        return '\n            '.join(svg_parts)

    def generate_svg(self) -> str:
        """Generate SVG content for the title block (compact horizontal layout)."""
        tb_x = self.area.x
        tb_y = self.area.y
        tb_width = self.area.width
        tb_height = self.area.height
        info = self.info

        # PE stamp area is 35x35mm square on right (always shown)
        pe_stamp_size = 35.0
        pe_stamp_x = tb_x + tb_width - pe_stamp_size

        # Remaining width for left and middle sections
        remaining_width = pe_stamp_x - tb_x
        left_width = remaining_width * 0.55  # Drawing info
        mid_width = remaining_width * 0.45   # Project/signatures

        left_x = tb_x
        mid_x = left_x + left_width

        # Equal row heights for middle section (4 rows in 35mm)
        row_height = tb_height / 4  # 8.75mm per row
        row1_y = tb_y
        row2_y = tb_y + row_height
        row3_y = tb_y + row_height * 2
        row4_y = tb_y + row_height * 3

        # Text Y positions (vertically centered in each row)
        text_offset = row_height * 0.6
        _text1_y = row1_y + text_offset  # Defined for consistency with other rows
        text2_y = row2_y + text_offset
        text3_y = row3_y + text_offset
        text4_y = row4_y + text_offset

        svg_parts = [f'''
        <!-- Title Block -->
        <g id="title-block">
            <!-- Outer border -->
            <rect x="{tb_x}" y="{tb_y}" width="{tb_width}" height="{tb_height}"
                  fill="none" stroke="{BORDER_COLOR}" stroke-width="{BORDER_WIDTH * 2}"/>

            <!-- LEFT SECTION: Company Branding / Drawing Name -->
            <line x1="{mid_x}" y1="{tb_y}" x2="{mid_x}" y2="{tb_y + tb_height}"
                  stroke="{BORDER_COLOR}" stroke-width="{THIN_LINE_WIDTH}"/>

            <!-- Company branding area (larger upper section) -->
            <text x="{left_x + 2}" y="{tb_y + 7}"
                  font-family="Arial" font-size="2.5" font-weight="bold">{info.company_name}</text>
            <text x="{left_x + left_width/2}" y="{tb_y + 16}"
                  text-anchor="middle" font-family="Arial" font-size="1.5" fill="#AAAAAA">COMPANY BRANDING AREA</text>

            <!-- Horizontal divider (aligned with row 3/4 divider) -->
            <line x1="{left_x}" y1="{row4_y}" x2="{mid_x}" y2="{row4_y}"
                  stroke="{BORDER_COLOR}" stroke-width="{THIN_LINE_WIDTH}"/>

            <!-- Drawing Title (bottom section) -->
            <text x="{left_x + left_width/2}" y="{text4_y}"
                  text-anchor="middle" font-family="Arial" font-size="3" font-weight="bold">
                {info.title}
            </text>

            <!-- MIDDLE SECTION: Metadata + Description + Signatures -->
            <line x1="{pe_stamp_x}" y1="{tb_y}" x2="{pe_stamp_x}" y2="{tb_y + tb_height}"
                  stroke="{BORDER_COLOR}" stroke-width="{THIN_LINE_WIDTH}"/>

            <!-- Row 1: Metadata with equal-spaced columns -->
            {self._generate_metadata_row(mid_x, pe_stamp_x, row1_y, row_height, info)}

            <!-- Horizontal divider (row 1/2) -->
            <line x1="{mid_x}" y1="{row2_y}" x2="{pe_stamp_x}" y2="{row2_y}"
                  stroke="{BORDER_COLOR}" stroke-width="{THIN_LINE_WIDTH}"/>

            <!-- Row 2: Description -->
            <text x="{mid_x + 2}" y="{text2_y}"
                  font-family="Arial" font-size="1.5">{info.description}</text>

            <!-- Horizontal divider (row 2/3) -->
            <line x1="{mid_x}" y1="{row3_y}" x2="{pe_stamp_x}" y2="{row3_y}"
                  stroke="{BORDER_COLOR}" stroke-width="{THIN_LINE_WIDTH}"/>

            <!-- Row 3: Drawn By and Date -->
            <text x="{mid_x + 2}" y="{text3_y}"
                  font-family="Arial" font-size="1.5">DRAWN: {info.drawn_by}</text>
            <text x="{mid_x + mid_width * 0.55}" y="{text3_y}"
                  font-family="Arial" font-size="1.5">DATE: {info.drawn_date}</text>

            <!-- Horizontal divider (row 3/4) -->
            <line x1="{mid_x}" y1="{row4_y}" x2="{pe_stamp_x}" y2="{row4_y}"
                  stroke="{BORDER_COLOR}" stroke-width="{THIN_LINE_WIDTH}"/>

            <!-- Row 4: Checked By and Date -->
            <text x="{mid_x + 2}" y="{text4_y}"
                  font-family="Arial" font-size="1.5">CHECKED: {info.checked_by}</text>
            <text x="{mid_x + mid_width * 0.55}" y="{text4_y}"
                  font-family="Arial" font-size="1.5">DATE: {info.checked_date}</text>
        ''']

        # PE Stamp area (35x35mm square on right) - always shown
        svg_parts.append(f'''
            <!-- PE STAMP AREA (35x35mm) -->
            <text x="{pe_stamp_x + 2}" y="{tb_y + 5}"
                  font-family="Arial" font-size="1.5" fill="#AAAAAA">
                PE STAMP
            </text>
            ''')

        # Company logo (if provided) - placed in branding area (upper left section)
        if info.company_logo_path:
            import base64
            import os
            logo_path = info.company_logo_path
            if os.path.exists(logo_path):
                # Determine MIME type
                ext = os.path.splitext(logo_path)[1].lower()
                mime_types = {
                    '.png': 'image/png',
                    '.jpg': 'image/jpeg',
                    '.jpeg': 'image/jpeg',
                    '.svg': 'image/svg+xml',
                    '.gif': 'image/gif',
                }
                mime_type = mime_types.get(ext, 'image/png')

                # Read and encode the image
                with open(logo_path, 'rb') as f:
                    logo_data = base64.b64encode(f.read()).decode('utf-8')

                # Logo dimensions (fit in branding area, below company name)
                logo_height = 15  # mm (fits in 26mm branding area)
                logo_width = 30  # mm
                logo_x = mid_x - logo_width - 2  # Right side of branding area
                logo_y = tb_y + 9  # Below company name

                svg_parts.append(f'''
            <!-- Company Logo -->
            <image x="{logo_x}" y="{logo_y}" width="{logo_width}" height="{logo_height}"
                   href="data:{mime_type};base64,{logo_data}"
                   preserveAspectRatio="xMidYMid meet"/>
                ''')

        svg_parts.append('</g>')
        return '\n'.join(svg_parts)
