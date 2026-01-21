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
        nps="1-1/2",
        schedule="40",
        flange_class=300,
        material="316 Stainless Steel",
    )

    builder.start_with_flange("west", bolt_hole_orientation="two_hole")
    builder.add_pipe("16 in")
    builder.add_reducer("2")
    builder.add_elbow("up", "bw")
    builder.add_pipe("10 in")

    with builder.add_tee("east", "bw") as tee1:
        run = tee1.run()
        branch = tee1.branch()

        run.add_pipe("10 in")
        branch.add_reducer("1-1/2")
        branch.add_pipe("16.562992126 in")
        branch.add_flange(bolt_hole_orientation="two_hole")

        with run.add_tee(weld_type="bw", enter_via_branch=True, run_direction="east") as tee2:
            run_a = tee2.run_a()  # Goes east
            run_b = tee2.run_b()  # Goes west

            # East side
            run_a.add_reducer("1-1/2")
            run_a.add_pipe("16.562992126 in")
            run_a.add_flange(bolt_hole_orientation="two_hole")

            # West side
            run_b.add_pipe("25.5 in")
            run_b.add_elbow("north", "sw")
            pl = 23.373500313 - 0.429498867
            run_b.add_pipe(f"{pl} in")
            run_b.add_elbow("up", "bw")
            run_b.add_pipe("17.242999700 in")
            run_b.add_elbow("west", "bw")
            run_b.add_pipe("87.379389802 in")
            # run_b.add_flange()

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

    step_output = Path("U:\\ENGINEERING\\_DRAWINGS\\WR\\W800-899\\W803 - Niron\\06_Mechanical Integrity\\MFC Enclosure")

    print("Building nested tee example...")
    route = nested_tee_example()
    route.export(str(step_output / "hydrogen_in.step"))

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
        title='H2 PIPE TO MFC',
        drawing_number="5979",
        revision="A",
        description='H2 PIPE TO MFC',
        material="316 Stainless Steel",
        project_number="W803",
        drawn_by="Tyler Newman",
        show_hidden=True,
        components=route.components,
        welds=route.welds,
        fittings=route.fittings,
        show_bom=True,
        show_balloons=False,
        show_welds=True,
        show_debug=False,
    )

    drawing.generate()

    # Export SVG and PDF
    svg_path = output_dir / "5979.svg"
    pdf_path = output_dir / "5979.pdf"

    drawing.export_svg(str(svg_path))
    drawing.export_pdf(str(pdf_path))

    print(f"\nExported SVG: {svg_path}")
    print(f"Exported PDF: {pdf_path}")
    print("\nDone!")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
