#!/usr/bin/env python3
"""
Export reducer example demonstrating contraction and expansion.

This example shows how the RouteBuilder handles reducers that can both
contract (go from larger to smaller pipe) and expand (go from smaller
to larger pipe).
"""

from pathlib import Path

import cadquery as cq

from pypegen.drawing_generator.drawing import PipingDrawing
from pypegen.route_builder import RouteBuilder


def reducer_example():
    """Route demonstrating reducer contraction and expansion."""
    builder = RouteBuilder(
        nps="4",
        schedule="40",
        flange_class=300,
        material="316 Stainless Steel",
    )

    # Start with a 4" flange
    builder.start_with_flange("east")
    builder.add_pipe("3 ft")

    # Contraction: 4" -> 2" (reducer)
    builder.add_reducer("2")
    builder.add_pipe("3 ft")
    builder.add_elbow("up", "bw")
    builder.add_pipe("2 ft")

    # Expansion: 2" -> 4" (same physical fitting, reversed orientation)
    builder.add_reducer("4")
    builder.add_pipe("3 ft")

    # End with a 4" flange
    builder.add_flange()

    return builder.build()


def main():
    output_dir = Path(__file__).parent / "output"
    output_dir.mkdir(exist_ok=True)

    print("Building reducer example...")
    route = reducer_example()
    route.export(str(output_dir / "reducer_example.step"))

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
        title='4" REDUCER EXAMPLE',
        drawing_number="SPOOL-REDUCER-001",
        revision="0",
        description='4" NPS WITH CONTRACTION AND EXPANSION',
        material="316 Stainless Steel",
        project_number="PYPEGEN-001",
        drawn_by="CAD",
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
    svg_path = output_dir / "reducer_example.svg"
    pdf_path = output_dir / "reducer_example.pdf"

    drawing.export_svg(str(svg_path))
    drawing.export_pdf(str(pdf_path))

    print(f"\nExported STEP: {output_dir / 'reducer_example.step'}")
    print(f"Exported SVG: {svg_path}")
    print(f"Exported PDF: {pdf_path}")
    print("\nDone!")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
