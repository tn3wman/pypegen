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
        nps="2",
        schedule="40",
        flange_class=300,
        material="316 Stainless Steel",
    )

    builder.start_with_flange("west")
    builder.add_pipe("6 in")
    builder.add_elbow("up", "bw")
    builder.add_pipe("6 in")

    with builder.add_tee("east", "bw") as tee1:
        run = tee1.run()
        branch = tee1.branch()
        
        run.add_pipe("4 in")
        branch.add_pipe("4 in")
        branch.add_flange()

        with run.add_tee("east", "bw") as tee2:
            run2 = tee2.run()
            branch2 = tee2.branch()

            run2.add_pipe("3 in")
            run2.add_elbow("west", "bw")

            branch2.add_pipe("4 in")

    # # Level 1 tee
    # with builder.add_tee(branch_direction="north") as tee1:
    #     # Level 2 tee on the run
    #     with tee1.run().add_pipe("5 ft").add_tee(branch_direction="up") as tee2:
    #         # Level 3 - end the run
    #         tee2.run().add_pipe("3 ft").add_cap()

    #         # Level 3 tee on the branch
    #         with tee2.branch().add_pipe("2 ft").add_tee(branch_direction="west") as tee3:
    #             # Level 4 - end both branches
    #             tee3.run().add_pipe("2 ft").add_cap()
    #             tee3.branch().add_pipe("1 ft").add_cap()

    #     # Level 2 - north branch
    #     tee1.branch().add_pipe("4 ft").add_cap()

    return builder.build()


def main():
    output_dir = Path(__file__).parent / "output"
    output_dir.mkdir(exist_ok=True)

    print("Building nested tee example...")
    route = nested_tee_example()

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
        title='2" NESTED TEE EXAMPLE',
        drawing_number="SPOOL-NESTED-TEE-001",
        revision="0",
        description='2" NPS BRANCHING PIPING WITH NESTED TEES',
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
    svg_path = output_dir / "nested_tee.svg"
    pdf_path = output_dir / "nested_tee.pdf"

    drawing.export_svg(str(svg_path))
    drawing.export_pdf(str(pdf_path))

    print(f"\nExported SVG: {svg_path}")
    print(f"Exported PDF: {pdf_path}")
    print("\nDone!")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
