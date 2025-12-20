"""Tests for NPT thread generation with fade runout."""

from __future__ import annotations

import math

import pytest
import cadquery as cq

from pypegen.fittings.npt_thread import (
    ThreadBuilder,
    make_npt_external_thread,
    make_npt_internal_thread,
    get_npt_spec,
    NPT_HALF_ANGLE_DEG,
    NPT_SPECS,
)


class TestFadeHelix:
    """Tests for the fade_helix parametric curve with taper correction."""

    def test_fade_helix_starts_at_correct_radius(self):
        """At t=0, apex should be at apex_radius (accounting for taper)."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
        )
        x, y, z = builder.fade_helix(0.0, apex=True, vertical_displacement=0, z_offset=0)
        # At t=0, x = radius * cos(0) = radius, y = radius * sin(0) = 0
        assert abs(x - 10.0) < 0.001
        assert abs(y) < 0.001
        assert abs(z) < 0.001

    def test_fade_helix_ends_at_root_level(self):
        """At t=1, apex wire should fade to root radius level for external threads."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
        )
        x, y, z = builder.fade_helix(1.0, apex=True, vertical_displacement=0, z_offset=0)
        # At t=1, position is at 90 degrees: x = r*cos(pi/2) ≈ 0, y = r*sin(pi/2) = r
        # The radius should have faded from apex to root level
        # Plus taper adjustment for z = pitch/4
        z_at_end = builder.pitch / 4
        taper_delta = z_at_end * math.tan(math.radians(NPT_HALF_ANGLE_DEG))
        expected_root_radius = 8.0 + taper_delta  # root_radius_adjusted is 8.0 - 0.001

        assert abs(x) < 0.01  # cos(pi/2) ≈ 0
        # y should be approximately the faded radius (root level + taper)
        assert abs(y - expected_root_radius) < 0.1
        assert abs(z - z_at_end) < 0.01

    def test_fade_helix_accounts_for_z_offset(self):
        """Radius should increase with z_offset due to NPT taper."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
        )
        # At t=0 with different z_offsets
        x1, _, _ = builder.fade_helix(0.0, apex=True, vertical_displacement=0, z_offset=0)
        x2, _, _ = builder.fade_helix(0.0, apex=True, vertical_displacement=0, z_offset=10.0)

        expected_increase = 10.0 * math.tan(math.radians(NPT_HALF_ANGLE_DEG))
        assert abs((x2 - x1) - expected_increase) < 0.001

    def test_fade_helix_internal_thread(self):
        """Internal threads should expand outward during fade."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=12.0,  # Root is larger for internal
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=False,
        )
        # At t=0, apex is at inner radius
        x0, _, _ = builder.fade_helix(0.0, apex=True, vertical_displacement=0, z_offset=0)
        assert abs(x0 - 10.0) < 0.001

        # At t=1, apex should fade toward root (larger radius)
        x1, y1, _ = builder.fade_helix(1.0, apex=True, vertical_displacement=0, z_offset=0)
        # At 90 degrees, x ≈ 0, y = radius
        assert abs(x1) < 0.01
        # y should be larger than apex_radius (faded toward root)
        assert y1 > 10.0


class TestMakeThreadFacesTaperedFade:
    """Tests for the tapered fade face generation."""

    def test_creates_valid_faces(self):
        """Should create 4 thread faces and 1 end face."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
        )
        thread_faces, end_faces = builder.make_thread_faces_tapered_fade(z_offset=0.0)

        assert len(thread_faces) == 4
        assert len(end_faces) == 1

        # makeRuledSurface can return Face or Shell depending on geometry
        for face in thread_faces:
            assert isinstance(face, (cq.Face, cq.Shell, cq.Shape))
        for face in end_faces:
            assert isinstance(face, (cq.Face, cq.Shape))


class TestMakeThreadWithFadedEnds:
    """Tests for the complete faded thread generation."""

    def test_thread_with_faded_ends_is_valid(self):
        """Generated thread with fade should be a valid solid."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("fade", "fade"),
        )
        thread = builder.make_thread_with_faded_ends()
        assert thread.isValid()

    def test_thread_has_positive_volume(self):
        """Generated thread should have positive volume."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("fade", "fade"),
        )
        thread = builder.make_thread_with_faded_ends()
        assert thread.Volume() > 0

    def test_single_end_fade_start(self):
        """Thread with fade only at start should be valid."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("fade", "raw"),
        )
        thread = builder.build()
        assert thread.isValid()

    def test_single_end_fade_end(self):
        """Thread with fade only at end should be valid."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("raw", "fade"),
        )
        thread = builder.build()
        assert thread.isValid()

    def test_thread_too_short_raises_error(self):
        """Thread shorter than required for fade should raise error."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=0.5,  # Too short for 2 * pitch/4 = 1.0
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("fade", "fade"),
        )
        with pytest.raises(ValueError, match="too short"):
            builder.make_thread_with_faded_ends()


class TestNPTExternalThread:
    """Integration tests for NPT external thread API."""

    @pytest.mark.parametrize("nps", ["1/8", "1/4", "1/2", "1", "2"])
    def test_external_thread_is_valid(self, nps: str):
        """External threads should be valid solids."""
        thread = make_npt_external_thread(nps, end_finishes=("fade", "fade"))
        assert thread.isValid()
        assert thread.Volume() > 0

    def test_external_thread_pipe_junction_is_valid(self):
        """Pipe junction fade should produce a valid external thread."""
        thread = make_npt_external_thread("1/2", end_finishes=("fade", "pipe_junction"))
        assert thread.isValid()
        assert thread.Volume() > 0

    def test_external_thread_simple_mode(self):
        """Simple mode should return simplified geometry."""
        thread = make_npt_external_thread("1/2", simple=True)
        assert thread.isValid()
        assert thread.Volume() > 0

    def test_external_thread_with_bore(self):
        """External thread with bore should be hollow."""
        spec = get_npt_spec("1/2")
        bore_id = spec.od - 2 * spec.thread_depth - 2.0  # Some wall thickness
        thread = make_npt_external_thread("1/2", pipe_id=bore_id, end_finishes=("fade", "fade"))
        assert thread.isValid()


class TestNPTInternalThread:
    """Integration tests for NPT internal thread API."""

    @pytest.mark.parametrize("nps", ["1/8", "1/4", "1/2", "1", "2"])
    def test_internal_thread_is_valid(self, nps: str):
        """Internal threads should be valid solids."""
        thread = make_npt_internal_thread(nps, end_finishes=("fade", "fade"))
        assert thread.isValid()
        assert thread.Volume() > 0

    def test_internal_thread_simple_mode(self):
        """Simple mode should return simplified geometry."""
        thread = make_npt_internal_thread("1/2", simple=True)
        assert thread.isValid()
        assert thread.Volume() > 0


class TestNPTSpecs:
    """Tests for NPT thread specifications."""

    def test_all_specs_have_required_fields(self):
        """All NPT specs should have required dimensional data."""
        for nps, spec in NPT_SPECS.items():
            assert spec.tpi > 0, f"NPS {nps} missing TPI"
            assert spec.od > 0, f"NPS {nps} missing OD"
            assert spec.E0 > 0, f"NPS {nps} missing E0"
            assert spec.L1 > 0, f"NPS {nps} missing L1"
            assert spec.L2 > 0, f"NPS {nps} missing L2"
            assert spec.L3 > 0, f"NPS {nps} missing L3"

    def test_pitch_calculation(self):
        """Pitch should be correctly calculated from TPI."""
        spec = get_npt_spec("1/2")
        expected_pitch = 25.4 / 14  # 1/2" is 14 TPI
        assert abs(spec.pitch_mm - expected_pitch) < 0.001

    def test_thread_depth_calculation(self):
        """Thread depth should be 0.8 * pitch per ASME B1.20.1."""
        spec = get_npt_spec("1/2")
        expected_depth = 0.8 * spec.pitch_mm
        assert abs(spec.thread_depth - expected_depth) < 0.001

    def test_truncation_calculation(self):
        """Truncation should be 0.033 * pitch per ASME B1.20.1."""
        spec = get_npt_spec("1/2")
        expected_truncation = 0.033 * spec.pitch_mm
        assert abs(spec.truncation - expected_truncation) < 0.001

    def test_flat_width_calculation(self):
        """Flat width should be ~0.038 * pitch per ASME B1.20.1."""
        spec = get_npt_spec("1/2")
        expected_flat = 0.038 * spec.pitch_mm
        # Allow 5% tolerance due to tan(30°) calculation
        assert abs(spec.flat_width - expected_flat) / expected_flat < 0.05

    def test_thread_profile_widths(self):
        """ThreadBuilder uses correct NPT profile widths for 30° flank angles."""
        pitch = 2.0
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=pitch,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
        )
        # For NPT: apex_width ≈ 0.038P, root_width ≈ 0.962P
        # These create correct 30° flank angles with 0.8P thread depth
        thread_depth = 0.8 * pitch
        flank_span = thread_depth * math.tan(math.radians(30))
        expected_apex = (pitch - 2 * flank_span) / 2
        expected_root = expected_apex + 2 * flank_span
        assert abs(builder.apex_width - expected_apex) < 0.001
        assert abs(builder.root_width - expected_root) < 0.001


class TestBuildMethodRouting:
    """Tests to verify build() routes correctly based on end_finishes."""

    def test_build_with_fade_uses_faded_ends(self):
        """build() should use make_thread_with_faded_ends when fade is specified."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("fade", "fade"),
        )
        thread = builder.build()
        assert thread.isValid()

    def test_build_with_raw_uses_original_path(self):
        """build() should use original path for raw end finishes."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("raw", "raw"),
        )
        thread = builder.build()
        assert thread.isValid()

    def test_build_with_square_uses_original_path(self):
        """build() should use original path for square end finishes."""
        builder = ThreadBuilder(
            apex_radius=10.0,
            root_radius=8.0,
            pitch=2.0,
            length=20.0,
            taper_angle=NPT_HALF_ANGLE_DEG,
            external=True,
            end_finishes=("square", "square"),
        )
        thread = builder.build()
        assert thread.isValid()
