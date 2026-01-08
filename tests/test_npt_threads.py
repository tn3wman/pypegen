"""Tests for NPT thread generation against golden files.

These tests verify that the male and female NPT thread generation
produces consistent geometry across all sizes (1/8" to 4").
"""

import cadquery as cq
import pytest
from pathlib import Path

GOLDEN_DIR = Path(__file__).parent / "golden_files"


def generate_male_threads_assembly() -> cq.Shape:
    """Generate the male threads assembly matching the golden file."""
    from pypegen.threads.npt_male import make_npt_external_thread, get_npt_spec, NPT_SPECS

    assembly = None
    x_offset = 0

    for nps in NPT_SPECS.keys():
        spec = get_npt_spec(nps)
        thread_length = spec.L2

        pipe_od = spec.od
        wall_thickness = spec.thread_depth * 2
        pipe_id = pipe_od - 2 * wall_thickness

        thread = make_npt_external_thread(
            nps=nps,
            thread_length=thread_length,
            pipe_id=pipe_id,
            end_finishes=("chamfer", "fade"),
        )

        thread = thread.translate(cq.Vector(x_offset, 0, 0))

        if assembly is None:
            assembly = thread
        else:
            assembly = assembly.fuse(thread)

        x_offset += pipe_od * 2 + 20

    return assembly


def generate_female_threads_assembly() -> cq.Shape:
    """Generate the female threads assembly matching the golden file."""
    from pypegen.threads.npt_female import make_npt_internal_thread, get_npt_spec, NPT_SPECS

    assembly = None
    x_offset = 0

    for nps in NPT_SPECS.keys():
        spec = get_npt_spec(nps)
        thread_length = spec.L2

        internal = make_npt_internal_thread(
            nps=nps,
            thread_length=thread_length,
            end_finishes=("chamfer", "fade"),
        )

        housing_od = spec.od * 1.5
        housing = cq.Solid.makeCylinder(housing_od / 2, thread_length)
        threaded_housing = housing.cut(internal)

        threaded_housing = threaded_housing.translate(cq.Vector(x_offset, 0, 0))

        if assembly is None:
            assembly = threaded_housing
        else:
            assembly = assembly.fuse(threaded_housing)

        x_offset += housing_od * 2 + 20

    return assembly


def load_golden_file(filename: str) -> tuple[list, float]:
    """Load a golden STEP file and return solids and total volume.

    Returns:
        Tuple of (list of solids, total volume)
    """
    filepath = GOLDEN_DIR / filename
    if not filepath.exists():
        pytest.skip(f"Golden file not found: {filepath}")
    workplane = cq.importers.importStep(str(filepath))
    # importStep returns a Workplane - extract all solids
    solids = workplane.solids().vals()
    total_volume = sum(s.Volume() for s in solids)
    return solids, total_volume


class TestMaleNPTThreads:
    """Tests for male (external) NPT threads."""

    GOLDEN_FILE = "npt_male_threads_all_sizes.step"
    EXPECTED_VOLUME = 113133.96  # mm³

    def test_male_threads_matches_golden_volume(self):
        """Test that generated male threads match golden file volume."""
        _, golden_volume = load_golden_file(self.GOLDEN_FILE)
        generated = generate_male_threads_assembly()
        generated_volume = generated.Volume()

        # Allow 0.1% tolerance for numerical differences
        tolerance = self.EXPECTED_VOLUME * 0.001
        assert abs(golden_volume - generated_volume) < tolerance, (
            f"Volume mismatch: golden={golden_volume:.2f}, "
            f"generated={generated_volume:.2f}, tolerance={tolerance:.2f}"
        )

    def test_male_threads_is_valid(self):
        """Test that generated male threads produce valid geometry."""
        generated = generate_male_threads_assembly()
        assert generated.isValid(), "Generated male threads geometry is invalid"

    def test_male_threads_has_expected_volume(self):
        """Test that male threads have expected volume within tolerance."""
        generated = generate_male_threads_assembly()
        volume = generated.Volume()

        # Allow 1% tolerance
        tolerance = self.EXPECTED_VOLUME * 0.01
        assert abs(volume - self.EXPECTED_VOLUME) < tolerance, (
            f"Volume {volume:.2f} differs from expected {self.EXPECTED_VOLUME:.2f} "
            f"by more than {tolerance:.2f}"
        )

    def test_all_male_sizes_valid(self):
        """Test that each individual male thread size produces valid geometry."""
        from pypegen.threads.npt_male import make_npt_external_thread, get_npt_spec, NPT_SPECS

        for nps in NPT_SPECS.keys():
            spec = get_npt_spec(nps)
            thread = make_npt_external_thread(
                nps=nps,
                thread_length=spec.L2,
                pipe_id=spec.od - spec.thread_depth * 4,
                end_finishes=("chamfer", "fade"),
            )
            assert thread.isValid(), f"Male thread NPS {nps} is invalid"
            assert thread.Volume() > 0, f"Male thread NPS {nps} has zero volume"


class TestFemaleNPTThreads:
    """Tests for female (internal) NPT threads."""

    GOLDEN_FILE = "npt_female_threads_all_sizes.step"
    EXPECTED_VOLUME = 958475.27  # mm³

    def test_female_threads_matches_golden_volume(self):
        """Test that generated female threads match golden file volume."""
        _, golden_volume = load_golden_file(self.GOLDEN_FILE)
        generated = generate_female_threads_assembly()
        generated_volume = generated.Volume()

        # Allow 0.1% tolerance for numerical differences
        tolerance = self.EXPECTED_VOLUME * 0.001
        assert abs(golden_volume - generated_volume) < tolerance, (
            f"Volume mismatch: golden={golden_volume:.2f}, "
            f"generated={generated_volume:.2f}, tolerance={tolerance:.2f}"
        )

    def test_female_threads_is_valid(self):
        """Test that generated female threads produce valid geometry."""
        generated = generate_female_threads_assembly()
        assert generated.isValid(), "Generated female threads geometry is invalid"

    def test_female_threads_has_expected_volume(self):
        """Test that female threads have expected volume within tolerance."""
        generated = generate_female_threads_assembly()
        volume = generated.Volume()

        # Allow 1% tolerance
        tolerance = self.EXPECTED_VOLUME * 0.01
        assert abs(volume - self.EXPECTED_VOLUME) < tolerance, (
            f"Volume {volume:.2f} differs from expected {self.EXPECTED_VOLUME:.2f} "
            f"by more than {tolerance:.2f}"
        )

    def test_all_female_sizes_valid(self):
        """Test that each individual female thread size produces valid geometry."""
        from pypegen.threads.npt_female import make_npt_internal_thread, get_npt_spec, NPT_SPECS

        for nps in NPT_SPECS.keys():
            spec = get_npt_spec(nps)
            thread = make_npt_internal_thread(
                nps=nps,
                thread_length=spec.L2,
                end_finishes=("chamfer", "fade"),
            )
            assert thread.isValid(), f"Female thread NPS {nps} is invalid"
            assert thread.Volume() > 0, f"Female thread NPS {nps} has zero volume"


class TestThreadCompatibility:
    """Tests for male/female thread compatibility."""

    def test_thread_specs_match(self):
        """Test that male and female thread specs are identical."""
        from pypegen.threads.npt_male import NPT_SPECS as MALE_SPECS
        from pypegen.threads.npt_female import NPT_SPECS as FEMALE_SPECS

        assert set(MALE_SPECS.keys()) == set(FEMALE_SPECS.keys()), "Thread size sets don't match"

        for nps in MALE_SPECS.keys():
            male = MALE_SPECS[nps]
            female = FEMALE_SPECS[nps]
            assert male.tpi == female.tpi, f"TPI mismatch for NPS {nps}"
            assert male.od == female.od, f"OD mismatch for NPS {nps}"
            assert male.L2 == female.L2, f"L2 mismatch for NPS {nps}"
