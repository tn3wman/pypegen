"""
3D Coordinate axis indicator for piping isometric drawings.
"""

import math

from .constants import BORDER_COLOR


def create_axis_indicator_svg(x: float, y: float, size: float = 30) -> str:
    """
    Create a 3D coordinate axis indicator for piping isometric drawings.

    Shows X (East), Y (North), Z (Up) axes as they appear in the isometric view.

    PyPeGen coordinate system:
        X = East  (positive X goes East)
        Y = North (positive Y goes North)
        Z = Up    (positive Z goes Up)

    With model rotation and (1,1,1) projection, the on-paper directions are:
        - X (East): lower-right at 30 degrees below horizontal
        - Y (North): upper-right at 30 degrees above horizontal
        - Z (Up): straight up

    Args:
        x: X position of the axis origin on the drawing
        y: Y position of the axis origin on the drawing
        size: Length of each axis line in mm

    Returns:
        SVG string containing the axis indicator
    """
    # Standard isometric angles (30 degrees from horizontal)
    angle_30 = math.radians(30)

    # Arrow head size
    arrow_len = size * 0.25
    math.radians(25)

    # All axes equal length
    # X axis (East): goes to lower-right
    x_angle = -angle_30
    x_end_x = x + size * math.cos(x_angle)
    x_end_y = y - size * math.sin(x_angle)  # SVG Y inverted

    # Y axis (North): goes to upper-right
    y_angle = angle_30
    y_end_x = x + size * math.cos(y_angle)
    y_end_y = y - size * math.sin(y_angle)  # SVG Y inverted

    # Z axis (Up): goes straight up (same length as X and Y)
    z_end_x = x
    z_end_y = y - size

    # Negative axis endpoints
    x_neg_end_x = x - size * math.cos(x_angle)
    x_neg_end_y = y + size * math.sin(x_angle)
    y_neg_end_x = x - size * math.cos(y_angle)
    y_neg_end_y = y + size * math.sin(y_angle)
    z_neg_end_y = y + size

    # Helper function to create arrowhead points
    def arrowhead(tip_x: float, tip_y: float, dx: float, dy: float,
                  color: str, filled: bool = True) -> str:
        """Create an arrowhead polygon at the tip pointing in direction (dx, dy)."""
        # Normalize direction vector
        length = math.sqrt(dx * dx + dy * dy)
        if length == 0:
            return ""
        dx, dy = dx / length, dy / length

        # Perpendicular vector
        px, py = -dy, dx

        # Arrow base center (back from tip)
        base_x = tip_x - arrow_len * dx
        base_y = tip_y - arrow_len * dy

        # Arrow corners
        left_x = base_x + arrow_len * 0.4 * px
        left_y = base_y + arrow_len * 0.4 * py
        right_x = base_x - arrow_len * 0.4 * px
        right_y = base_y - arrow_len * 0.4 * py

        fill = color if filled else "none"
        return (f'<polygon points="{tip_x},{tip_y} {left_x},{left_y} '
                f'{right_x},{right_y}" fill="{fill}" stroke="{color}" '
                f'stroke-width="0.3"/>')

    # Direction vectors for each axis (in SVG coordinates where Y is down)
    x_dx = x_end_x - x
    x_dy = x_end_y - y
    y_dx = y_end_x - x
    y_dy = y_end_y - y
    z_dx = 0
    z_dy = z_end_y - y

    svg = f'''
    <!-- 3D Coordinate Axis Indicator -->
    <g id="axis-indicator" font-family="Arial" font-size="3.5">
        <!-- Origin point -->
        <circle cx="{x}" cy="{y}" r="1" fill="{BORDER_COLOR}"/>

        <!-- X Axis (East) - Red -->
        <line x1="{x}" y1="{y}" x2="{x_end_x}" y2="{x_end_y}"
              stroke="#CC0000" stroke-width="0.6"/>
        {arrowhead(x_end_x, x_end_y, x_dx, x_dy, "#CC0000")}
        <text x="{x_end_x + 3}" y="{x_end_y + 3}" fill="#CC0000" font-weight="bold" font-size="3">E</text>

        <!-- Y Axis (North) - Green -->
        <line x1="{x}" y1="{y}" x2="{y_end_x}" y2="{y_end_y}"
              stroke="#00AA00" stroke-width="0.6"/>
        {arrowhead(y_end_x, y_end_y, y_dx, y_dy, "#00AA00")}
        <text x="{y_end_x + 3}" y="{y_end_y}" fill="#00AA00" font-weight="bold" font-size="3">N</text>

        <!-- Z Axis (Up) - Blue -->
        <line x1="{x}" y1="{y}" x2="{z_end_x}" y2="{z_end_y}"
              stroke="#0000CC" stroke-width="0.6"/>
        {arrowhead(z_end_x, z_end_y, z_dx, z_dy, "#0000CC")}
        <text x="{z_end_x + 3}" y="{z_end_y + 2}" fill="#0000CC" font-weight="bold" font-size="3">UP</text>

        <!-- Negative directions (dashed, same length as positive) -->
        <!-- West (-X) -->
        <line x1="{x}" y1="{y}" x2="{x_neg_end_x}" y2="{x_neg_end_y}"
              stroke="#CC0000" stroke-width="0.4" stroke-dasharray="1.5,1"/>
        {arrowhead(x_neg_end_x, x_neg_end_y, -x_dx, -x_dy, "#CC0000", False)}
        <text x="{x_neg_end_x - 6}" y="{x_neg_end_y + 1}" fill="#CC0000" font-size="2.5">W</text>

        <!-- South (-Y) -->
        <line x1="{x}" y1="{y}" x2="{y_neg_end_x}" y2="{y_neg_end_y}"
              stroke="#00AA00" stroke-width="0.4" stroke-dasharray="1.5,1"/>
        {arrowhead(y_neg_end_x, y_neg_end_y, -y_dx, -y_dy, "#00AA00", False)}
        <text x="{y_neg_end_x - 5}" y="{y_neg_end_y + 1}" fill="#00AA00" font-size="2.5">S</text>

        <!-- Down (-Z) -->
        <line x1="{x}" y1="{y}" x2="{x}" y2="{z_neg_end_y}"
              stroke="#0000CC" stroke-width="0.4" stroke-dasharray="1.5,1"/>
        {arrowhead(x, z_neg_end_y, -z_dx, -z_dy, "#0000CC", False)}
        <text x="{x + 2}" y="{z_neg_end_y + 4}" fill="#0000CC" font-size="2.5">DN</text>
    </g>
    '''
    return svg
