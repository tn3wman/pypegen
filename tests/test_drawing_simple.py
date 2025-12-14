#!/usr/bin/env python3
"""
Test for simple pipe drawing generation.

Compares generated SVG against a golden file to ensure consistency.
"""

from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import pytest as pytest_module
    pytest: pytest_module = pytest_module  # type: ignore[name-defined]
else:
    try:
        import pytest
    except ImportError:
        pytest = None  # Allow running without pytest for golden file generation

from pypegen.drawing_generator.drawing import PipingDrawing
from pypegen.pipe_router import PipeRoute

# Directory containing this test file
TEST_DIR = Path(__file__).parent
GOLDEN_DIR = TEST_DIR / "golden"


def create_simple_route() -> PipeRoute:
    """Create the standard simple route for testing."""
    route = PipeRoute(
        nps="2",
        start_fitting="flange",
        moves=[
            ("east", "3 ft"),
            ("up", "2 ft"),
        ],
        debug=False,  # Suppress debug output during tests
        track_components=True,
    )
    return route


def generate_simple_drawing_svg(route: PipeRoute) -> str:
    """Generate SVG for the simple route without debug markers."""
    route.build()

    if not route.parts:
        raise ValueError("No parts in route")

    combined_shape = route.parts[0]
    for part in route.parts[1:]:
        combined_shape = combined_shape.fuse(part)

    drawing = PipingDrawing(
        shape=combined_shape,
        title="2\" PROCESS LINE - SIMPLE",
        drawing_number="SPOOL-PIPE_B-001",
        revision="0",
        description="2\" NPS CARBON STEEL PIPING",
        material="CS A106 GR.B",
        project_number="PYPEGEN-001",
        fund_number="",
        drawn_by="CAD",
        show_hidden=True,
        components=route.components,
        welds=route.welds,
        fittings=route.fittings,
        show_bom=True,
        show_balloons=True,
        show_welds=True,
        show_debug=False,  # No debug markers for golden file comparison
    )

    drawing.generate()
    return drawing._svg_content


def test_simple_drawing_matches_golden():
    """Test that generated SVG matches the golden file."""
    golden_path = GOLDEN_DIR / "pipe_bom_simple.svg"

    if not golden_path.exists():
        pytest.skip(f"Golden file not found: {golden_path}. Run with --update-golden to create it.")

    route = create_simple_route()
    generated_svg = generate_simple_drawing_svg(route)

    with open(golden_path) as f:
        golden_svg = f.read()

    # Compare line by line for better error messages
    generated_lines = generated_svg.strip().split('\n')
    golden_lines = golden_svg.strip().split('\n')

    # Check line count first
    if len(generated_lines) != len(golden_lines):
        # Find first difference
        for i, (gen, gold) in enumerate(zip(generated_lines, golden_lines, strict=False)):
            if gen != gold:
                pytest.fail(
                    f"Line count mismatch (generated: {len(generated_lines)}, golden: {len(golden_lines)}). "
                    f"First difference at line {i+1}:\n"
                    f"  Generated: {gen[:100]}...\n"
                    f"  Golden:    {gold[:100]}..."
                )
        pytest.fail(
            f"Line count mismatch (generated: {len(generated_lines)}, golden: {len(golden_lines)})"
        )

    # Compare each line
    for i, (gen_line, gold_line) in enumerate(zip(generated_lines, golden_lines, strict=True)):
        if gen_line != gold_line:
            pytest.fail(
                f"Mismatch at line {i+1}:\n"
                f"  Generated: {gen_line}\n"
                f"  Golden:    {gold_line}"
            )


def update_golden_file():
    """Update the golden file with current output."""
    route = create_simple_route()
    generated_svg = generate_simple_drawing_svg(route)

    golden_path = GOLDEN_DIR / "pipe_bom_simple.svg"
    golden_path.parent.mkdir(parents=True, exist_ok=True)

    with open(golden_path, 'w') as f:
        f.write(generated_svg)

    print(f"Updated golden file: {golden_path}")


if __name__ == "__main__":
    import sys

    if "--update-golden" in sys.argv:
        update_golden_file()
    else:
        # Run the test
        route = create_simple_route()
        svg = generate_simple_drawing_svg(route)
        print(f"Generated SVG ({len(svg)} bytes)")
        print("Run with --update-golden to update the golden file")
