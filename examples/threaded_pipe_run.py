#!/usr/bin/env python3
"""
Threaded Pipe Run Example

Generates a pipe route using NPT threaded fittings:
- 6" vertical pipe going up
- 90° threaded elbow
- 6" horizontal pipe going north
- 90° threaded elbow
- 6" horizontal pipe going east

All connections are NPT threaded (no welding).

Outputs:
- STEP file for 3D CAD
- SVG fabrication drawing with BOM
- PDF fabrication drawing
- Text BOM (Bill of Materials)
"""

from pathlib import Path
from dataclasses import dataclass

from pypegen.drawing_generator.drawing import PipingDrawing
from pypegen.pipe_router import PipeRoute


INCH = 25.4  # mm per inch


@dataclass
class BOMItem:
    """Bill of Materials item."""
    item_no: int
    qty: int
    description: str
    size: str
    material: str


def main():
    # Create output directories
    output_dir = Path(__file__).parent / "output"
    output_dir.mkdir(exist_ok=True)

    print("Threaded Pipe Run Example")
    print("=" * 60)

    # Configuration
    nps = "1/2"  # Nominal pipe size

    print(f"NPS: {nps}")
    print(f"Connection Type: NPT Threaded")

    # =========================================================================
    # Create the pipe route using PipeRoute with threaded connections
    # =========================================================================

    print("\nBuilding route...")

    route = PipeRoute(
        nps=nps,
        start_fitting="none",  # No flange for threaded routes
        moves=[
            ("up", "6 in"),     # 6" vertical going up
            ("north", "6 in"),  # 6" horizontal going north
            ("east", "6 in"),   # 6" horizontal going east
        ],
        debug=True,
        track_components=True,
        connection_type="threaded",  # Use NPT threaded fittings
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

    # =========================================================================
    # Export STEP file
    # =========================================================================

    step_path = output_dir / "threaded_pipe_run.step"
    route.export(str(step_path))
    print(f"\nExported STEP: {step_path}")

    # =========================================================================
    # Create fabrication drawing
    # =========================================================================

    drawing = PipingDrawing(
        shape=combined_shape,
        title=f'{nps}" THREADED PIPE RUN',
        drawing_number="SPOOL-THREADED-001",
        revision="0",
        description=f'{nps}" NPS NPT THREADED PIPING',
        material="CS A106 GR.B",
        project_number="PYPEGEN-002",
        drawn_by="CAD",
        show_hidden=True,
        components=route.components,
        welds=[],  # No welds for threaded connections
        fittings=route.fittings,
        show_bom=True,
        show_balloons=True,
        show_welds=False,  # No welds for threaded
        show_debug=False,
    )

    drawing.generate()

    # Export SVG and PDF
    svg_path = output_dir / "threaded_pipe_run.svg"
    pdf_path = output_dir / "threaded_pipe_run.pdf"

    drawing.export_svg(str(svg_path))
    drawing.export_pdf(str(pdf_path))

    print(f"Exported SVG: {svg_path}")
    print(f"Exported PDF: {pdf_path}")

    # =========================================================================
    # Generate BOM
    # =========================================================================

    # Count components by type
    pipe_count = sum(1 for c in route.components if c.component_type == "pipe")
    elbow_count = sum(1 for c in route.components if c.component_type == "elbow")

    bom = [
        BOMItem(1, pipe_count, f"PIPE, {nps}\" NPT THREADED BOTH ENDS", f'{nps}" x 6"', "CS A106 GR.B"),
        BOMItem(2, elbow_count, f"ELBOW 90°, {nps}\" NPT THREADED", f'{nps}"', "CS A105 CLASS 3000"),
    ]

    print("\n" + "=" * 60)
    print("BILL OF MATERIALS")
    print("=" * 60)
    print(f"{'Item':<6} {'Qty':<5} {'Description':<35} {'Size':<12} {'Material'}")
    print("-" * 60)
    for item in bom:
        print(f"{item.item_no:<6} {item.qty:<5} {item.description:<35} {item.size:<12} {item.material}")
    print("=" * 60)

    # =========================================================================
    # Export BOM to text file
    # =========================================================================

    bom_path = output_dir / "threaded_pipe_run_bom.txt"
    with open(bom_path, "w") as f:
        f.write("THREADED PIPE RUN - BILL OF MATERIALS\n")
        f.write("=" * 60 + "\n")
        f.write(f"Route: 6\" Up -> 6\" North -> 6\" East\n")
        f.write(f"Pipe Size: NPS {nps}\n")
        f.write(f"Connection Type: NPT Threaded\n")
        f.write("=" * 60 + "\n\n")
        f.write(f"{'Item':<6} {'Qty':<5} {'Description':<35} {'Size':<12} {'Material'}\n")
        f.write("-" * 75 + "\n")
        for item in bom:
            f.write(f"{item.item_no:<6} {item.qty:<5} {item.description:<35} {item.size:<12} {item.material}\n")
        f.write("\n" + "=" * 60 + "\n")
        f.write("Notes:\n")
        f.write("- All threads per ASME B1.20.1 (NPT)\n")
        f.write("- Elbows per ASME B16.11 Class 3000\n")
        f.write("- Apply thread sealant per specification\n")

    print(f"Exported BOM: {bom_path}")

    # =========================================================================
    # Summary
    # =========================================================================

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Total pipes: {pipe_count}")
    print(f"Total elbows: {elbow_count}")
    print(f"Connection type: NPT Threaded")
    print(f"Assembly volume: {combined_shape.Volume():.2f} mm³")
    print("\nDone!")


if __name__ == "__main__":
    main()
