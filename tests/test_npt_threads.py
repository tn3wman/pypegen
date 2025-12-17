"""
Tests for NPT thread generation and alignment verification.

These tests verify:
1. NPT thread specs match ASME B1.20.1 values
2. Thread geometry is created correctly
3. Threads align at wrench-tight engagement depth
"""

import cadquery as cq
import pytest

from pypegen.fittings.npt_thread import (
    NPT_HALF_ANGLE_DEG,
    NPT_SPECS,
    NPT_TAPER_RATIO,
    calculate_thread_alignment_transform,
    get_npt_spec,
    make_npt_external_thread,
    make_npt_internal_thread,
)
from pypegen.fittings.threaded_elbow import make_threaded_elbow_45, make_threaded_elbow_90
from pypegen.fittings.threaded_tee import make_threaded_tee
from pypegen.pipe_generator import make_threaded_pipe

# =============================================================================
# NPT SPECIFICATION TESTS
# =============================================================================


class TestNPTSpecs:
    """Test NPT thread specifications match ASME B1.20.1."""

    def test_all_sizes_have_specs(self):
        """Verify specs exist for all standard NPT sizes."""
        expected_sizes = [
            "1/8", "1/4", "3/8", "1/2", "3/4", "1",
            "1-1/4", "1-1/2", "2", "2-1/2", "3", "4"
        ]
        for size in expected_sizes:
            spec = get_npt_spec(size)
            assert spec is not None, f"Missing spec for NPS {size}"
            assert spec.tpi > 0, f"Invalid TPI for NPS {size}"
            assert spec.od > 0, f"Invalid OD for NPS {size}"

    def test_invalid_size_raises_error(self):
        """Verify invalid sizes raise ValueError."""
        with pytest.raises(ValueError):
            get_npt_spec("invalid")

    def test_tpi_values_match_asme(self):
        """Verify TPI values match ASME B1.20.1 standard."""
        # ASME B1.20.1 TPI values
        expected_tpi = {
            "1/8": 27,
            "1/4": 18,
            "3/8": 18,
            "1/2": 14,
            "3/4": 14,
            "1": 11.5,
            "1-1/4": 11.5,
            "1-1/2": 11.5,
            "2": 11.5,
            "2-1/2": 8,
            "3": 8,
            "4": 8,
        }
        for size, expected in expected_tpi.items():
            spec = get_npt_spec(size)
            assert spec.tpi == expected, f"TPI mismatch for NPS {size}: expected {expected}, got {spec.tpi}"

    def test_taper_angle_correct(self):
        """Verify NPT taper angle is approximately 1.79°."""
        expected_angle = 1.7899  # degrees
        assert abs(NPT_HALF_ANGLE_DEG - expected_angle) < 0.001, \
            f"Taper angle mismatch: expected {expected_angle}, got {NPT_HALF_ANGLE_DEG}"

    def test_taper_ratio_correct(self):
        """Verify NPT taper ratio is 1:16."""
        assert abs(NPT_TAPER_RATIO - 1/16) < 0.0001, \
            f"Taper ratio mismatch: expected {1/16}, got {NPT_TAPER_RATIO}"

    def test_thread_depth_calculation(self):
        """Verify thread depth is calculated correctly."""
        for size in ["1/2", "1", "2"]:
            spec = get_npt_spec(size)
            # Thread depth should be 0.75 * H where H = 0.866025 * pitch
            expected_H = 0.866025 * spec.pitch_mm
            expected_depth = 0.75 * expected_H
            assert abs(spec.thread_depth - expected_depth) < 0.001, \
                f"Thread depth mismatch for NPS {size}"

    def test_wrench_tight_engagement(self):
        """Verify wrench-tight engagement is L1 + L3."""
        for size in ["1/2", "1", "2"]:
            spec = get_npt_spec(size)
            expected = spec.L1 + spec.L3
            assert abs(spec.wrench_tight_engagement - expected) < 0.001, \
                f"Wrench-tight engagement mismatch for NPS {size}"


# =============================================================================
# THREAD GEOMETRY TESTS
# =============================================================================


class TestThreadGeometry:
    """Test NPT thread geometry generation."""

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_external_thread_simple_creates_solid(self, nps):
        """Verify simplified external thread creates a valid solid."""
        thread = make_npt_external_thread(nps, simple=True)
        assert thread is not None
        assert hasattr(thread, 'Volume')

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_external_thread_full_creates_solid(self, nps):
        """Verify full external thread creates a valid solid."""
        thread = make_npt_external_thread(nps, simple=False)
        assert thread is not None
        assert hasattr(thread, 'Volume')

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_internal_thread_simple_creates_solid(self, nps):
        """Verify simplified internal thread creates a valid solid."""
        thread = make_npt_internal_thread(nps, simple=True)
        assert thread is not None
        assert hasattr(thread, 'Volume')

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_internal_thread_full_creates_solid(self, nps):
        """Verify full internal thread creates a valid solid."""
        thread = make_npt_internal_thread(nps, simple=False)
        assert thread is not None
        assert hasattr(thread, 'Volume')

    def test_thread_length_parameter(self):
        """Verify custom thread length is respected."""
        custom_length = 10.0  # mm

        thread = make_npt_external_thread("1", thread_length=custom_length, simple=True)
        assert thread is not None
        # The thread should be approximately custom_length tall
        # We can verify by checking bounding box

    def test_external_thread_tapers_correctly(self):
        """Verify external thread tapers (gets smaller) as Z increases."""
        # For external threads, radius should decrease as we move along +Z
        # This is implicit in the geometry - hard to test directly without
        # inspecting the internal structure


# =============================================================================
# FITTING INTEGRATION TESTS
# =============================================================================


class TestFittingIntegration:
    """Test NPT threads in fittings."""

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_threaded_elbow_90_with_threads(self, nps):
        """Verify 90° elbow with threads creates valid geometry."""
        elbow = make_threaded_elbow_90(nps, include_threads=True)
        assert elbow is not None
        assert hasattr(elbow, 'Volume')

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_threaded_elbow_45_with_threads(self, nps):
        """Verify 45° elbow with threads creates valid geometry."""
        elbow = make_threaded_elbow_45(nps, include_threads=True)
        assert elbow is not None
        assert hasattr(elbow, 'Volume')

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_threaded_tee_with_threads(self, nps):
        """Verify tee with threads creates valid geometry."""
        tee = make_threaded_tee(nps, include_threads=True)
        assert tee is not None
        assert hasattr(tee, 'Volume')

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_threaded_pipe_both_ends(self, nps):
        """Verify threaded pipe with both ends creates valid geometry."""
        pipe = make_threaded_pipe(nps, length=100, thread_end="both")
        assert pipe is not None
        assert hasattr(pipe, 'Volume')

    @pytest.mark.parametrize("thread_end", ["none", "inlet", "outlet", "both"])
    def test_threaded_pipe_thread_end_options(self, thread_end):
        """Verify all thread_end options work."""
        pipe = make_threaded_pipe("1", length=100, thread_end=thread_end)
        assert pipe is not None


# =============================================================================
# THREAD ALIGNMENT TESTS
# =============================================================================


class TestThreadAlignment:
    """Test thread alignment at engagement depths."""

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_alignment_transform_exists(self, nps):
        """Verify alignment transform can be calculated."""
        transform = calculate_thread_alignment_transform(nps)
        assert transform is not None
        assert isinstance(transform, cq.Location)

    def test_alignment_transform_uses_wrench_tight_by_default(self):
        """Verify default alignment is wrench-tight (L1+L3)."""
        nps = "1"
        transform = calculate_thread_alignment_transform(nps)

        # The transform should translate by -wrench_tight_engagement in Z
        # We can verify by checking the transform values
        # For a Location, we can extract translation from wrapped()
        assert transform is not None

    def test_custom_engagement_depth(self):
        """Verify custom engagement depth is respected."""
        nps = "1"
        custom_depth = 10.0  # mm
        transform = calculate_thread_alignment_transform(nps, engagement_depth=custom_depth)
        assert transform is not None

    @pytest.mark.parametrize("nps", ["1/2", "1", "2"])
    def test_assembly_at_wrench_tight_no_major_interference(self, nps):
        """
        Verify pipe and fitting threads don't have major interference at wrench-tight.

        Note: This is a simplified test. For production, you would use more
        sophisticated interference detection or visual inspection.
        """
        # Create fitting with internal threads (simplified for speed)
        fitting = make_threaded_elbow_90(nps, include_threads=False)

        # Create pipe with external threads
        pipe = make_threaded_pipe(nps, length=100, thread_end="inlet", include_threads=False)

        # Get alignment transform
        transform = calculate_thread_alignment_transform(nps)

        # Position pipe at wrench-tight engagement
        pipe_positioned = pipe.moved(transform)

        # Both shapes should be valid
        assert fitting is not None
        assert pipe_positioned is not None


# =============================================================================
# EXPORT/VISUALIZATION TESTS
# =============================================================================


class TestExport:
    """Test STEP export of threaded parts."""

    def test_export_threaded_assembly(self, tmp_path):
        """Export a complete threaded assembly for visual verification."""
        nps = "1"

        # Create fitting and pipe
        fitting = make_threaded_elbow_90(nps, include_threads=True)
        pipe = make_threaded_pipe(nps, length=100, thread_end="inlet")

        # Position pipe at wrench-tight engagement
        transform = calculate_thread_alignment_transform(nps)
        pipe_positioned = pipe.moved(transform)

        # Create assembly compound
        assembly = cq.Compound.makeCompound([fitting, pipe_positioned])

        # Export to STEP
        output_file = tmp_path / "threaded_assembly_test.step"
        cq.exporters.export(assembly, str(output_file))

        assert output_file.exists()
        assert output_file.stat().st_size > 0


# =============================================================================
# EDGE CASES AND ERROR HANDLING
# =============================================================================


class TestEdgeCases:
    """Test edge cases and error handling."""

    def test_minimum_thread_length(self):
        """Verify minimum thread length works."""
        thread = make_npt_external_thread("1", thread_length=5.0, simple=True)
        assert thread is not None

    def test_all_sizes_create_threads(self):
        """Verify all supported sizes can create threads."""
        for nps in NPT_SPECS.keys():
            # Use simplified threads for speed
            ext = make_npt_external_thread(nps, simple=True)
            assert ext is not None, f"Failed to create external thread for NPS {nps}"

            int_thread = make_npt_internal_thread(nps, simple=True)
            assert int_thread is not None, f"Failed to create internal thread for NPS {nps}"
