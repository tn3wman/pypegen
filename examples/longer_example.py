#!/usr/bin/env python3
"""
Export the nested_tee_example to SVG and PDF.
"""

from pathlib import Path

import cadquery as cq

from pypegen.drawing_generator.drawing import PipingDrawing
from pypegen.route_builder import RouteBuilder


def nested_tee_example():
    """Route with nested tees (3+ levels deep)."""
    builder = RouteBuilder(
        nps="1-1/4",
        schedule="40",
        flange_class=300,
        material="316 Stainless Steel",
    )

    builder.start_with_flange("east", bolt_hole_orientation="two_hole")
    builder.add_pipe("12 in")
    builder.add_reducer("2")
    builder.add_elbow("up", "bw", roll=-4.72379102)
    builder.add_pipe("13.6875 in")
    builder.add_elbow("west", "bw")
    pl1 = 25.274914359 + 1.919087087 + 62.72161206 - 7.530495396 - 4.126929134
    builder.add_pipe(f"{pl1} in")
    builder.add_elbow("up", "bw")
    p_3 = 100 + 21.742999700
    builder.add_pipe(f"{p_3} in")
    builder.add_elbow("west", "bw")
    builder.add_reducer("2-1/2")
    builder.add_flange(bolt_hole_orientation="two_hole")
    # builder.add_pipe("63.5 in")
    # builder.add_elbow("down", "bw")
    # pl = 12.254986620+2.049069382
    # builder.add_pipe(f"{pl} in")
    # builder.add_elbow("west", "bw")
    # builder.add_pipe("30 in")

    return builder.build()


def main():
    output_dir = Path(__file__).parent / "output"
    output_dir.mkdir(exist_ok=True)

    step_path = output_dir / "longer_example.step"


    print("Building nested tee example...")
    route = nested_tee_example()
    route.export(str(step_path))

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
        title='Longer Example Piping',
        drawing_number="####",
        revision="0",
        description='Longer Example Piping to MFCs',
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
    svg_path = output_dir / "longer_example.svg"
    pdf_path = output_dir / "longer_example.pdf"

    drawing.export_svg(str(svg_path))
    drawing.export_pdf(str(pdf_path))

    print(f"\nExported SVG: {svg_path}")
    print(f"Exported PDF: {pdf_path}")
    print("\nDone!")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
