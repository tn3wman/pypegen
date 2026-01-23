#!/usr/bin/env python3
"""
Export the nested_tee_example to SVG and PDF.
"""

import math
from pathlib import Path

import cadquery as cq

from pypegen.drawing_generator.drawing import PipingDrawing
from pypegen.route_builder import RouteBuilder


def nested_tee_example():
    """Route with nested tees (3+ levels deep)."""
    builder = RouteBuilder(
        nps="2-1/2",
        schedule="40",
        flange_class=300,
        material="316 Stainless Steel",
    )

    builder.start_with_flange("east", bolt_hole_orientation="two_hole")
    builder.add_reducer("2")
    builder.add_pipe("10 in")
    builder.add_elbow("down", "bw")

    p_2 = 121.742999700
    builder.add_pipe(f"{p_2} in")
    builder.add_elbow("east", "bw")

    p_3 = 78.258188976 - 0.792500000 - 10 + 3.062993927
    builder.add_pipe(f"{p_3} in")

    x = 23.694500300
    y = 18.125000000
    angle = math.atan2(y, x) * (180 / math.pi)
    builder.add_elbow("down", "bw", roll=-angle)

    p_4 = math.sqrt(x**2 + y**2)
    builder.add_pipe(f"{p_4} in")



    builder.add_elbow("west", "bw")
    builder.add_reducer("1-1/2")
    p_5 = 16 + 1.375492126 - 0.125 - 5.855000000 - 0.792500000
    builder.add_pipe(f"{p_5} in")
    builder.add_flange(bolt_hole_orientation="two_hole")
    # builder.add_pipe("100 in")
    # builder.add_elbow("east", "bw")
    # builder.add_pipe("63.5 in")
    # builder.add_elbow("down", "bw", angle=45)
    # builder.add_pipe("12.254986620 in")
    # builder.add_elbow("down", "bw")
    # pl = 12.254986620+2.049069382
    # builder.add_pipe(f"{pl} in")
    # builder.add_elbow("west", "bw")
    # builder.add_pipe("30 in")

    return builder.build()


def main():
    output_dir = Path(__file__).parent / "output"
    output_dir.mkdir(exist_ok=True)

    step_output = Path("U:\\ENGINEERING\\_DRAWINGS\\WR\\W800-899\\W803 - Niron\\06_Mechanical Integrity\\MFC Enclosure")

    print("Building nested tee example...")
    route = nested_tee_example()
    route.export(str(step_output / "hydrogen_out_2.step"))

    print(f"  Parts: {len(route.parts)}")
    print(f"  Components: {len(route.components)}")
    print(f"  Welds: {len(route.welds)}")
    print(f"  Fittings: {len(route.fittings)}")

    if not route.parts:
        print("Error: No parts in route")
        return 1

    # Combine all parts into a single shape for rendering
    combined_shape = cq.Compound.makeCompound(route.parts)

    # Create fabrication drawing
    drawing = PipingDrawing(
        shape=combined_shape,
        title='Hydrogen Outlet Piping',
        drawing_number="####",
        revision="0",
        description='Hydrogen Outlet Piping to MFCs',
        material="316 Stainless Steel",
        project_number="W803",
        drawn_by="T. Newman",
        show_hidden=True,
        components=route.components,
        welds=route.welds,
        fittings=route.fittings,
        show_bom=True,
        show_balloons=True,
        show_welds=True,
        show_debug=True,
    )

    drawing.generate()

    # Export SVG and PDF
    svg_path = output_dir / "hydrogen_out_2.svg"
    pdf_path = output_dir / "hydrogen_out_2.pdf"

    drawing.export_svg(str(svg_path))
    drawing.export_pdf(str(pdf_path))

    print(f"\nExported SVG: {svg_path}")
    print(f"Exported PDF: {pdf_path}")
    print("\nDone!")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
