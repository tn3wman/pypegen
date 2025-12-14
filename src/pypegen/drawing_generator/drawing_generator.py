#!/usr/bin/env python3
"""
Piping Fabrication Drawing Generator

This is a convenience script that uses the drawing_generator module.
For direct imports, use: from drawing_generator import PipingDrawing

Usage:
    python drawing_generator.py
"""

import os

import cadquery as cq

from ..pipe_router import PipeRoute
from .drawing import PipingDrawing


def process_step_file(step_path: str) -> None:
    """Process a standalone STEP file without BOM (legacy mode)."""
    print(f"\nProcessing: {step_path}")

    base_name = os.path.basename(step_path).replace('.step', '')
    title = base_name.replace('_', ' ').upper()

    drawing = PipingDrawing(
        step_file=step_path,
        title=title,
        drawing_number=f"SPOOL-{base_name[:3].upper()}-001",
        revision="0",
        material="CS A106 GR.B",
        project_number="PYPEGEN-001",
        drawn_by="CAD",
        show_hidden=True,
        show_bom=False,  # No component data
        show_balloons=False,
    )

    drawing.generate()

    svg_path = f"drawings/{base_name}.svg"
    pdf_path = f"drawings/{base_name}.pdf"

    drawing.export_svg(svg_path)
    drawing.export_pdf(pdf_path)


def process_pipe_route(route: PipeRoute, name: str, title: str) -> None:
    """Process a PipeRoute with full BOM and balloon support."""
    print(f"\nProcessing PipeRoute: {name}")

    # Build the route (this creates component tracking)
    route.build()

    # Create combined shape for rendering
    if not route.parts:
        print(f"  Warning: No parts in route {name}")
        return

    combined_shape = route.parts[0]
    for part in route.parts[1:]:
        combined_shape = combined_shape.fuse(part)

    # Export the STEP file for reference
    step_path = f"step/{name}.step"
    route.export(step_path)

    # Create drawing with components and welds
    drawing = PipingDrawing(
        shape=combined_shape,
        title=title,
        drawing_number=f"SPOOL-{name[:6].upper()}-001",
        revision="0",
        description="2\" NPS CARBON STEEL PIPING",
        material="CS A106 GR.B",
        project_number="PYPEGEN-001",
        fund_number="",
        drawn_by="CAD",
        show_hidden=True,
        components=route.components,  # Pass component data for BOM
        welds=route.welds,  # Pass weld data for weld markers
        fittings=route.fittings,  # Pass fitting data for attachment point debug visualization
        show_bom=True,
        show_balloons=True,
        show_welds=True,
    )

    drawing.generate()

    svg_path = f"drawings/{name}.svg"
    pdf_path = f"drawings/{name}.pdf"

    drawing.export_svg(svg_path)
    drawing.export_pdf(pdf_path)


def main():
    """Generate piping drawings with BOM and balloons."""
    print("Piping Fabrication Drawing Generator")
    print("=" * 50)

    # Create output directories
    os.makedirs("drawings", exist_ok=True)
    os.makedirs("step", exist_ok=True)

    # Example 1: Simple route with BOM
    simple_route = PipeRoute(
        nps="2",
        start_fitting="flange",
        moves=[
            ("east", "3 ft"),
            ("up", "2 ft"),
        ],
        debug=True,
        track_components=True,
    )
    process_pipe_route(simple_route, "pipe_bom_simple", "2\" PROCESS LINE - SIMPLE")

    # Example 2: Three-segment route
    three_seg_route = PipeRoute(
        nps="2",
        start_fitting="flange",
        moves=[
            ("east", "3 ft"),
            ("up", "2 ft"),
            ("north", "4 ft"),
        ],
        debug=True,
        track_components=True,
    )
    process_pipe_route(three_seg_route, "pipe_bom_three_seg", "2\" PROCESS LINE - THREE SEG")

    # Example 3: Complex route
    complex_route = PipeRoute(
        nps="2",
        start_fitting="flange",
        moves=[
            ("east", "2 ft"),
            ("up", "1.5 ft"),
            ("north", "3 ft"),
            ("east", "2 ft"),
        ],
        debug=True,
        track_components=True,
    )
    process_pipe_route(complex_route, "pipe_bom_complex", "2\" PROCESS LINE - COMPLEX")

    # Also process legacy STEP files without BOM
    print("\n--- Legacy STEP Files (without BOM) ---")
    step_files = [
        "step/pipe_route_simple.step",
        "step/pipe_route_three_segments.step",
        "step/pipe_route_imperial.step",
    ]

    for step_path in step_files:
        if os.path.exists(step_path):
            process_step_file(step_path)

    print("\n" + "=" * 50)
    print("Done! Check the 'drawings' directory.")


if __name__ == "__main__":
    main()
