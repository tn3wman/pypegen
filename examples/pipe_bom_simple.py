#!/usr/bin/env python3
"""
Simple Pipe BOM Example

Generates a simple pipe route with:
- Weld neck flange
- Horizontal pipe going east (3 ft)
- 90° socket weld elbow
- Vertical pipe going up (2 ft)

Outputs:
- STEP file for 3D CAD
- SVG fabrication drawing with BOM and weld markers
- PDF fabrication drawing
"""

from pathlib import Path

from pypegen.drawing_generator.drawing import PipingDrawing
from pypegen.pipe_router import PipeRoute


def main():
    # Create output directories
    output_dir = Path(__file__).parent / "output"
    output_dir.mkdir(exist_ok=True)

    print("Pipe BOM Simple Example")
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

    # Build the route (creates geometry and tracks components)
    route.build()

    if not route.parts:
        print("Error: No parts in route")
        return

    # Combine all parts into a single shape for rendering
    combined_shape = route.parts[0]
    for part in route.parts[1:]:
        combined_shape = combined_shape.fuse(part)

    # Export STEP file
    step_path = output_dir / "pipe_bom_simple.step"
    route.export(str(step_path))
    print(f"\nExported STEP: {step_path}")

    # Create fabrication drawing
    drawing = PipingDrawing(
        shape=combined_shape,
        title='2" PROCESS LINE - SIMPLE',
        drawing_number="SPOOL-PIPE_B-001",
        revision="0",
        description='2" NPS CARBON STEEL PIPING',
        material="CS A106 GR.B",
        project_number="PYPEGEN-001",
        drawn_by="CAD",
        show_hidden=True,
        components=route.components,
        welds=route.welds,
        fittings=route.fittings,
        show_bom=True,
        show_balloons=True,
        show_welds=True,
        show_debug=False,  # Clean output without debug markers
    )

    drawing.generate()

    # Export SVG and PDF
    svg_path = output_dir / "pipe_bom_simple.svg"
    pdf_path = output_dir / "pipe_bom_simple.pdf"

    drawing.export_svg(str(svg_path))
    drawing.export_pdf(str(pdf_path))

    print(f"Exported SVG: {svg_path}")
    print(f"Exported PDF: {pdf_path}")
    print("\n" + "=" * 50)
    print("Done!")


if __name__ == "__main__":
    main()
