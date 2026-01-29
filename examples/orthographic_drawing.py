#!/usr/bin/env python3
"""
Orthographic Drawing Example

Generates a multi-view 3rd angle projection drawing with:
- Front view (elevation)
- Top view (plan)
- Right side view (profile)
- Isometric view (reference)

The BOM is placed on a separate page (Page 2).

Outputs:
- SVG files (one per page)
- Multi-page PDF fabrication drawing
"""

from pathlib import Path

from pypegen.drawing_generator.drawing import OrthographicPipingDrawing
from pypegen.pipe_router import PipeRoute


def main():
    # Create output directories
    output_dir = Path(__file__).parent / "output"
    output_dir.mkdir(exist_ok=True)

    print("Orthographic Drawing Example")
    print("=" * 50)

    # Create a simple route: flange -> east 3ft -> up 2ft
    route = PipeRoute(
        nps="2",
        start_fitting="flange",
        moves=[
            ("east", "3 ft"),
            ("up", "2 ft"),
        ],
        debug=True,
        track_components=True,
    )

    # Build the route
    route.build()

    if not route.parts:
        print("Error: No parts in route")
        return

    # Combine all parts into a single shape
    combined_shape = route.parts[0]
    for part in route.parts[1:]:
        combined_shape = combined_shape.fuse(part)

    # Export STEP file
    step_path = output_dir / "orthographic_example.step"
    route.export(str(step_path))
    print(f"\nExported STEP: {step_path}")

    # Create orthographic fabrication drawing (3rd angle projection)
    drawing = OrthographicPipingDrawing(
        shape=combined_shape,
        title='2" PROCESS LINE - ORTHOGRAPHIC',
        drawing_number="SPOOL-ORTHO-001",
        revision="0",
        description='2" NPS CARBON STEEL PIPING',
        material="CS A106 GR.B",
        project_number="PYPEGEN-001",
        drawn_by="CAD",
        show_hidden=True,
        components=route.components,
        welds=route.welds,
        show_balloons=True,
        show_welds=True,
        show_dimensions=True,  # Enable dimensions
        primary_annotation_view="front",  # Balloons on front view
    )

    # Generate the multi-page drawing
    svgs = drawing.generate()

    print(f"\nGenerated {len(svgs)} pages")

    # Export SVG files
    svg_base = output_dir / "orthographic_example"
    svg_paths = drawing.export_svg(str(svg_base))
    for path in svg_paths:
        print(f"Exported SVG: {path}")

    # Export multi-page PDF
    pdf_path = output_dir / "orthographic_example.pdf"
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


if __name__ == "__main__":
    main()
