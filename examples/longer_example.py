#!/usr/bin/env python3
"""
Export the longer example to multi-view orthographic drawing.

Generates a 3rd angle projection drawing with:
- Front view (elevation)
- Top view (plan)
- Right side view (profile)
- Isometric view (reference)

The BOM is placed on a separate page (Page 2).
"""

from pathlib import Path

import cadquery as cq

from pypegen.drawing_generator.drawing import OrthographicPipingDrawing
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

    return builder.build()


def main():
    output_dir = Path(__file__).parent / "output"
    output_dir.mkdir(exist_ok=True)

    step_path = output_dir / "longer_example.step"

    print("Building longer example...")
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

    # Create orthographic fabrication drawing (3rd angle projection)
    drawing = OrthographicPipingDrawing(
        shape=combined_shape,
        title='LONGER EXAMPLE - ORTHOGRAPHIC',
        drawing_number="SPOOL-LONGER-001",
        revision="0",
        description='1-1/4" to 2-1/2" NPS 316SS PIPING',
        material="316 Stainless Steel",
        project_number="W803",
        drawn_by="T. Newman",
        show_hidden=True,
        components=route.components,
        welds=route.welds,
        show_balloons=True,
        show_welds=True,
        show_dimensions=True,
        primary_annotation_view="front",
    )

    # Generate the multi-page drawing
    svgs = drawing.generate()

    print(f"\nGenerated {len(svgs)} pages")

    # Export SVG files
    svg_base = output_dir / "longer_example"
    svg_paths = drawing.export_svg(str(svg_base))
    for path in svg_paths:
        print(f"Exported SVG: {path}")

    # Export multi-page PDF
    pdf_path = output_dir / "longer_example.pdf"
    try:
        drawing.export_pdf(str(pdf_path))
        print(f"Exported PDF: {pdf_path}")
    except ImportError as e:
        print(f"PDF export not available: {e}")

    print("\n" + "=" * 50)
    print("Done!")
    print("\nView layout:")
    print("  Page 1: Orthographic views (Looking North, Looking Down, Looking West)")
    print("          Isometric reference view, and Notes")
    print("  Page 2: Bill of Materials")

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
