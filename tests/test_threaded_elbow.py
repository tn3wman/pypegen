#!/usr/bin/env python3
"""
Test for threaded elbow generation.

Compares generated STEP geometry against a golden file to ensure consistency.
"""

import sys
from pathlib import Path

try:
    import pytest
except ImportError:
    pytest = None

import cadquery as cq

from pypegen.fittings.threaded_elbow import make_threaded_elbow_90

TEST_DIR = Path(__file__).parent
GOLDEN_DIR = TEST_DIR / "golden_files"


def generate_threaded_elbow() -> cq.Shape:
    """Generate the threaded elbow for testing."""
    return make_threaded_elbow_90(nps="1/2", include_threads=True)


def test_threaded_elbow_matches_golden():
    """Test that generated threaded elbow matches golden file."""
    golden_path = GOLDEN_DIR / "threaded_elbow_90_half_inch.step"

    if not golden_path.exists():
        pytest.skip(f"Golden file not found: {golden_path}")

    # Generate current elbow
    elbow = generate_threaded_elbow()
    current_volume = elbow.Volume()

    # Load golden file
    golden_wp = cq.importers.importStep(str(golden_path))
    golden = golden_wp.val()
    golden_volume = golden.Volume()

    # Compare volumes (should be very close)
    volume_diff = abs(current_volume - golden_volume)
    volume_pct = volume_diff / golden_volume * 100 if golden_volume > 0 else 0

    assert volume_pct < 0.1, (
        f"Volume mismatch: current={current_volume:.2f} mm³, "
        f"golden={golden_volume:.2f} mm³, diff={volume_pct:.2f}%"
    )


def test_threaded_elbow_is_valid():
    """Test that generated threaded elbow is a valid solid."""
    elbow = generate_threaded_elbow()
    assert elbow.isValid(), "Threaded elbow is not a valid solid"


def test_threaded_elbow_has_reasonable_volume():
    """Test that threaded elbow has reasonable volume for 1/2 inch fitting."""
    elbow = generate_threaded_elbow()
    volume = elbow.Volume()

    # 1/2" threaded elbow should be roughly 40,000-60,000 mm³
    assert 40000 < volume < 60000, f"Unexpected volume: {volume:.2f} mm³"


def update_golden_file():
    """Update the golden file with current output."""
    elbow = generate_threaded_elbow()
    golden_path = GOLDEN_DIR / "threaded_elbow_90_half_inch.step"
    GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(elbow, str(golden_path))
    print(f"Updated golden file: {golden_path}")
    print(f"Volume: {elbow.Volume():.2f} mm³")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--update-golden":
        update_golden_file()
    else:
        # Run basic checks without pytest
        print("Running basic checks...")
        elbow = generate_threaded_elbow()
        print(f"Elbow volume: {elbow.Volume():.2f} mm³")
        print(f"Elbow valid: {elbow.isValid()}")

        golden_path = GOLDEN_DIR / "threaded_elbow_90_half_inch.step"
        if golden_path.exists():
            golden_wp = cq.importers.importStep(str(golden_path))
            golden = golden_wp.val()
            print(f"Golden volume: {golden.Volume():.2f} mm³")
            diff = abs(elbow.Volume() - golden.Volume())
            print(f"Volume difference: {diff:.2f} mm³")
        else:
            print(f"Golden file not found: {golden_path}")
