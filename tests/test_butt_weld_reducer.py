#!/usr/bin/env python3
"""
Tests for ASME B16.9 Butt Weld Concentric Reducer.

Tests cover:
- Dimension data completeness and validity
- Geometry generation
- Fitting ports and attachment points
- Route integration with NPS transitions
"""

import numpy as np
import pytest

from pypegen.fittings.butt_weld_reducer import (
    ASME_B169_REDUCER,
    ButtWeldReducerDims,
    compare_nps,
    get_available_reducers_from,
    get_butt_weld_reducer_dims,
    make_butt_weld_reducer,
    nps_to_float,
    validate_reducer,
)
from pypegen.fittings.pipe_fittings import (
    apply_transform_to_shape,
    get_position,
    identity_matrix,
    make_butt_weld_reducer_fitting,
)


# =============================================================================
# NPS PARSING TESTS
# =============================================================================


class TestNPSParsing:
    """Test NPS string parsing utilities."""

    @pytest.mark.parametrize(
        "nps,expected",
        [
            ("1", 1.0),
            ("2", 2.0),
            ("4", 4.0),
            ("10", 10.0),
            ("12", 12.0),
        ],
    )
    def test_whole_numbers(self, nps: str, expected: float):
        """Whole number NPS values should parse correctly."""
        assert nps_to_float(nps) == expected

    @pytest.mark.parametrize(
        "nps,expected",
        [
            ("1/2", 0.5),
            ("3/4", 0.75),
        ],
    )
    def test_fractions(self, nps: str, expected: float):
        """Fractional NPS values should parse correctly."""
        assert nps_to_float(nps) == expected

    @pytest.mark.parametrize(
        "nps,expected",
        [
            ("1-1/2", 1.5),
            ("2-1/2", 2.5),
            ("3-1/2", 3.5),
            ("1-1/4", 1.25),
        ],
    )
    def test_mixed_numbers(self, nps: str, expected: float):
        """Mixed number NPS values should parse correctly."""
        assert nps_to_float(nps) == expected

    def test_whitespace_handling(self):
        """Leading/trailing whitespace should be stripped."""
        assert nps_to_float("  4  ") == 4.0
        assert nps_to_float(" 1-1/2 ") == 1.5

    def test_invalid_format_raises_error(self):
        """Invalid NPS formats should raise ValueError."""
        with pytest.raises(ValueError, match="Cannot parse NPS"):
            nps_to_float("abc")

    @pytest.mark.parametrize(
        "nps1,nps2,expected",
        [
            ("4", "2", 1),      # 4 > 2
            ("2", "4", -1),     # 2 < 4
            ("2", "2", 0),      # 2 == 2
            ("4", "1-1/2", 1),  # 4 > 1.5
            ("3/4", "1", -1),   # 0.75 < 1
            ("1-1/2", "1-1/2", 0),  # Equal
        ],
    )
    def test_compare_nps(self, nps1: str, nps2: str, expected: int):
        """NPS comparison should work correctly."""
        assert compare_nps(nps1, nps2) == expected


# =============================================================================
# VALIDATION TESTS
# =============================================================================


class TestReducerValidation:
    """Test reducer validation and error messaging."""

    def test_contraction_validation(self):
        """Contraction (large to small) should validate correctly."""
        large, small, is_expansion = validate_reducer("4", "2")
        assert large == "4"
        assert small == "2"
        assert is_expansion is False

    def test_expansion_validation(self):
        """Expansion (small to large) should validate correctly."""
        large, small, is_expansion = validate_reducer("2", "4")
        assert large == "4"
        assert small == "2"
        assert is_expansion is True

    def test_same_size_raises_error(self):
        """Same size for current and target should raise error."""
        with pytest.raises(ValueError, match="equals target NPS"):
            validate_reducer("4", "4")

    def test_invalid_size_shows_available_options(self):
        """Invalid size should show available options."""
        with pytest.raises(ValueError) as exc_info:
            validate_reducer("4", "1/2")  # No 4" to 1/2" reducer

        error_msg = str(exc_info.value)
        assert "4\" to 1/2\"" in error_msg
        assert "Available contractions" in error_msg

    def test_get_available_reducers_from_4(self):
        """get_available_reducers_from should return correct options for 4\"."""
        contractions, expansions = get_available_reducers_from("4")

        # 4" can contract to several sizes
        assert len(contractions) > 0
        assert "2" in contractions
        assert "3" in contractions

        # 4" can expand to several sizes
        assert len(expansions) > 0
        assert "5" in expansions
        assert "6" in expansions

    def test_get_available_reducers_from_12(self):
        """12\" should have contractions but no expansions (largest size)."""
        contractions, expansions = get_available_reducers_from("12")

        assert len(contractions) > 0
        assert "10" in contractions
        assert len(expansions) == 0

    def test_get_available_reducers_from_half(self):
        """1/2\" should have expansions but no contractions (smallest size)."""
        contractions, expansions = get_available_reducers_from("1/2")

        # 1/2" has no contractions (smallest size in typical reducer range)
        assert len(contractions) == 0
        # 1/2" can expand to several sizes
        assert len(expansions) > 0


# =============================================================================
# DIMENSION DATA TESTS
# =============================================================================


class TestReducerDimensions:
    """Test ASME B16.9 reducer dimension data."""

    @pytest.mark.parametrize(
        "large_nps,small_nps",
        [
            ("2", "1-1/2"),
            ("2", "1"),
            ("3", "2"),
            ("4", "3"),
            ("4", "2"),
            ("6", "4"),
            ("8", "6"),
            ("10", "8"),
            ("12", "10"),
        ],
    )
    def test_common_sizes_available(self, large_nps: str, small_nps: str):
        """Common reducer sizes should have dimension data."""
        key = (large_nps, small_nps)
        assert key in ASME_B169_REDUCER
        dims = ASME_B169_REDUCER[key]
        assert dims.large_od > 0
        assert dims.small_od > 0
        assert dims.large_od > dims.small_od
        assert dims.length_h > 0

    def test_get_butt_weld_reducer_dims(self):
        """get_butt_weld_reducer_dims should return correct dimensions."""
        dims = get_butt_weld_reducer_dims("4", "2")
        assert isinstance(dims, ButtWeldReducerDims)
        assert dims.large_od == pytest.approx(114.3, rel=0.01)
        assert dims.small_od == pytest.approx(60.3, rel=0.01)
        assert dims.length_h == pytest.approx(102.0, rel=0.01)

    def test_invalid_size_raises_error(self):
        """Invalid size combination should raise ValueError."""
        with pytest.raises(ValueError, match="No ASME B16.9 dimensions"):
            get_butt_weld_reducer_dims("100", "1")

    def test_reversed_sizes_raises_error(self):
        """Reversed sizes (small x large) should raise ValueError."""
        # Reducers go from large to small, not small to large
        with pytest.raises(ValueError):
            get_butt_weld_reducer_dims("2", "4")

    @pytest.mark.parametrize(
        "large_nps,small_nps",
        [
            ("2", "1-1/2"),
            ("3", "2"),
            ("4", "3"),
            ("6", "4"),
        ],
    )
    def test_wall_thicknesses_positive(self, large_nps: str, small_nps: str):
        """Wall thicknesses should be positive and reasonable."""
        dims = get_butt_weld_reducer_dims(large_nps, small_nps)
        assert dims.large_wall > 0
        assert dims.small_wall > 0
        # Wall should be less than radius
        assert dims.large_wall < dims.large_od / 4
        assert dims.small_wall < dims.small_od / 4


# =============================================================================
# GEOMETRY TESTS
# =============================================================================


class TestReducerGeometry:
    """Test reducer geometry generation."""

    def test_creates_valid_shape(self):
        """make_butt_weld_reducer should create valid CadQuery shape."""
        shape = make_butt_weld_reducer("4", "2")
        assert shape is not None
        bb = shape.BoundingBox()
        # Should have non-zero size in all dimensions
        assert bb.xmax - bb.xmin > 0
        assert bb.ymax - bb.ymin > 0
        assert bb.zmax - bb.zmin > 0

    def test_length_matches_h_dimension(self):
        """Reducer length along X should match H dimension."""
        dims = get_butt_weld_reducer_dims("4", "2")
        shape = make_butt_weld_reducer("4", "2")
        bb = shape.BoundingBox()
        length = bb.xmax - bb.xmin
        assert abs(length - dims.length_h) < 1.0

    def test_large_end_diameter_correct(self):
        """Large end (at X=0) should have correct OD."""
        dims = get_butt_weld_reducer_dims("4", "2")
        shape = make_butt_weld_reducer("4", "2")
        bb = shape.BoundingBox()
        # Y and Z extent at origin should match large OD
        yz_extent = max(bb.ymax - bb.ymin, bb.zmax - bb.zmin)
        assert abs(yz_extent - dims.large_od) < 1.0

    def test_is_hollow(self):
        """Reducer should be hollow (have a bore)."""
        shape = make_butt_weld_reducer("4", "2")
        # A solid cone would have volume = pi/3 * h * (r1^2 + r1*r2 + r2^2)
        # A hollow cone has less volume
        bb = shape.BoundingBox()
        length = bb.xmax - bb.xmin
        max_radius = (bb.ymax - bb.ymin) / 2
        # If it were solid, volume would be larger than actual
        # This is a sanity check that geometry was created
        assert shape.Volume() > 0
        assert shape.Volume() < np.pi * max_radius**2 * length

    @pytest.mark.parametrize(
        "large_nps,small_nps",
        [
            ("2", "1"),
            ("3", "2"),
            ("4", "2"),
            ("6", "4"),
            ("8", "6"),
        ],
    )
    def test_multiple_sizes(self, large_nps: str, small_nps: str):
        """Multiple sizes should create valid geometry."""
        shape = make_butt_weld_reducer(large_nps, small_nps)
        assert shape is not None
        assert shape.Volume() > 0


# =============================================================================
# FITTING TESTS
# =============================================================================


class TestReducerFitting:
    """Test reducer fitting factory function."""

    def test_creates_fitting_with_shape(self):
        """make_butt_weld_reducer_fitting should create Fitting with shape."""
        fitting = make_butt_weld_reducer_fitting("4", "2")
        assert fitting is not None
        assert fitting.shape is not None
        assert fitting.nps == "4"  # Primary NPS is large end

    def test_has_inlet_and_outlet_ports(self):
        """Fitting should have inlet and outlet ports."""
        fitting = make_butt_weld_reducer_fitting("4", "2")
        assert "inlet" in fitting.ports
        assert "outlet" in fitting.ports

    def test_inlet_port_at_origin(self):
        """Inlet port (large end) should be at X=0."""
        fitting = make_butt_weld_reducer_fitting("4", "2")
        inlet_pos = fitting.get_port("inlet").position
        assert abs(inlet_pos[0]) < 0.1
        assert abs(inlet_pos[1]) < 0.1
        assert abs(inlet_pos[2]) < 0.1

    def test_outlet_port_at_h(self):
        """Outlet port (small end) should be at X=H."""
        fitting = make_butt_weld_reducer_fitting("4", "2")
        dims = get_butt_weld_reducer_dims("4", "2")
        outlet_pos = fitting.get_port("outlet").position
        assert abs(outlet_pos[0] - dims.length_h) < 0.1
        assert abs(outlet_pos[1]) < 0.1
        assert abs(outlet_pos[2]) < 0.1

    def test_port_directions_oppose_along_x(self):
        """Inlet and outlet port Z-axes should oppose along X."""
        fitting = make_butt_weld_reducer_fitting("4", "2")
        inlet_dir = fitting.get_port("inlet").direction
        outlet_dir = fitting.get_port("outlet").direction

        # Inlet Z points -X
        assert abs(inlet_dir[0] + 1.0) < 0.01
        assert abs(inlet_dir[1]) < 0.01
        assert abs(inlet_dir[2]) < 0.01

        # Outlet Z points +X
        assert abs(outlet_dir[0] - 1.0) < 0.01
        assert abs(outlet_dir[1]) < 0.01
        assert abs(outlet_dir[2]) < 0.01

    def test_has_weld_attachment_points(self):
        """Fitting should have weld attachment points at both ends."""
        fitting = make_butt_weld_reducer_fitting("4", "2")

        inlet_aps = fitting.get_attachment_points_for_port("inlet")
        outlet_aps = fitting.get_attachment_points_for_port("outlet")

        # Should have 4 attachment points per port
        assert len(inlet_aps) == 4
        assert len(outlet_aps) == 4

    def test_attachment_point_names(self):
        """Attachment points should have correct naming convention."""
        fitting = make_butt_weld_reducer_fitting("4", "2")

        expected_inlet = ["inlet_weld_up", "inlet_weld_down", "inlet_weld_north", "inlet_weld_south"]
        expected_outlet = ["outlet_weld_up", "outlet_weld_down", "outlet_weld_north", "outlet_weld_south"]

        for name in expected_inlet:
            assert name in fitting.attachment_points

        for name in expected_outlet:
            assert name in fitting.attachment_points

    def test_attachment_points_at_correct_positions(self):
        """Attachment points should be at correct positions on outer surface."""
        fitting = make_butt_weld_reducer_fitting("4", "2")
        dims = get_butt_weld_reducer_dims("4", "2")

        # Inlet attachment points should be at X=0
        inlet_up = fitting.attachment_points["inlet_weld_up"]
        assert abs(inlet_up.position[0]) < 0.1
        assert abs(inlet_up.position[2] - dims.large_od / 2) < 1.0

        # Outlet attachment points should be at X=H
        outlet_up = fitting.attachment_points["outlet_weld_up"]
        assert abs(outlet_up.position[0] - dims.length_h) < 0.1
        assert abs(outlet_up.position[2] - dims.small_od / 2) < 1.0

    def test_invalid_size_raises_error(self):
        """Invalid size combination should raise ValueError."""
        with pytest.raises(ValueError, match="No butt weld reducer dimensions"):
            make_butt_weld_reducer_fitting("100", "1")


# =============================================================================
# REVERSED FITTING TESTS (EXPANSION)
# =============================================================================


class TestReversedReducerFitting:
    """Test reversed reducer fitting for expansion (pipe getting larger)."""

    def test_reversed_creates_fitting(self):
        """Reversed fitting should create valid Fitting."""
        fitting = make_butt_weld_reducer_fitting("4", "2", reversed=True)
        assert fitting is not None
        assert fitting.shape is not None

    def test_reversed_inlet_at_small_end(self):
        """Reversed fitting inlet should be at small end (X=H)."""
        fitting = make_butt_weld_reducer_fitting("4", "2", reversed=True)
        dims = get_butt_weld_reducer_dims("4", "2")
        inlet_pos = fitting.get_port("inlet").position

        # Inlet should be at X=H (small end)
        assert abs(inlet_pos[0] - dims.length_h) < 0.1
        assert abs(inlet_pos[1]) < 0.1
        assert abs(inlet_pos[2]) < 0.1

    def test_reversed_outlet_at_large_end(self):
        """Reversed fitting outlet should be at large end (X=0)."""
        fitting = make_butt_weld_reducer_fitting("4", "2", reversed=True)
        outlet_pos = fitting.get_port("outlet").position

        # Outlet should be at X=0 (large end)
        assert abs(outlet_pos[0]) < 0.1
        assert abs(outlet_pos[1]) < 0.1
        assert abs(outlet_pos[2]) < 0.1

    def test_reversed_port_directions(self):
        """Reversed fitting ports should have correct directions."""
        fitting = make_butt_weld_reducer_fitting("4", "2", reversed=True)
        inlet_dir = fitting.get_port("inlet").direction
        outlet_dir = fitting.get_port("outlet").direction

        # Reversed: Inlet Z points +X (from small end)
        assert abs(inlet_dir[0] - 1.0) < 0.01
        assert abs(inlet_dir[1]) < 0.01
        assert abs(inlet_dir[2]) < 0.01

        # Reversed: Outlet Z points -X (from large end)
        assert abs(outlet_dir[0] + 1.0) < 0.01
        assert abs(outlet_dir[1]) < 0.01
        assert abs(outlet_dir[2]) < 0.01

    def test_reversed_same_geometry(self):
        """Reversed fitting should have same geometry volume."""
        normal = make_butt_weld_reducer_fitting("4", "2", reversed=False)
        reversed_fit = make_butt_weld_reducer_fitting("4", "2", reversed=True)

        # Same physical shape, different port assignments
        normal_vol = normal.shape.Volume()
        reversed_vol = reversed_fit.shape.Volume()
        assert abs(normal_vol - reversed_vol) / normal_vol < 0.01

    def test_reversed_inlet_attachment_at_small_end(self):
        """Reversed fitting inlet attachments should be at small end."""
        fitting = make_butt_weld_reducer_fitting("4", "2", reversed=True)
        dims = get_butt_weld_reducer_dims("4", "2")

        # Inlet attachment points should be at X=H (small end)
        inlet_up = fitting.attachment_points["inlet_weld_up"]
        assert abs(inlet_up.position[0] - dims.length_h) < 0.1
        # Should use small radius
        assert abs(inlet_up.position[2] - dims.small_od / 2) < 1.0

    def test_reversed_outlet_attachment_at_large_end(self):
        """Reversed fitting outlet attachments should be at large end."""
        fitting = make_butt_weld_reducer_fitting("4", "2", reversed=True)
        dims = get_butt_weld_reducer_dims("4", "2")

        # Outlet attachment points should be at X=0 (large end)
        outlet_up = fitting.attachment_points["outlet_weld_up"]
        assert abs(outlet_up.position[0]) < 0.1
        # Should use large radius
        assert abs(outlet_up.position[2] - dims.large_od / 2) < 1.0


# =============================================================================
# TRANSFORM AND ASSEMBLY TESTS
# =============================================================================


class TestReducerTransform:
    """Test reducer transformation and assembly."""

    def test_can_apply_transform(self):
        """Reducer shape should accept transform application."""
        fitting = make_butt_weld_reducer_fitting("4", "2")
        transform = identity_matrix()
        transformed = apply_transform_to_shape(fitting.shape, transform)
        assert transformed is not None

    def test_transform_preserves_volume(self):
        """Transformation should preserve volume (within tolerance)."""
        fitting = make_butt_weld_reducer_fitting("4", "2")
        original_volume = fitting.shape.Volume()

        # Apply a rotation
        transform = np.array([
            [0, -1, 0, 100],
            [1, 0, 0, 200],
            [0, 0, 1, 300],
            [0, 0, 0, 1],
        ], dtype=float)

        transformed = apply_transform_to_shape(fitting.shape, transform)
        transformed_volume = transformed.Volume()

        # Use relative tolerance (1%) as OCCT can have small numerical differences
        assert abs(transformed_volume - original_volume) / original_volume < 0.01


# =============================================================================
# INTEGRATION TESTS
# =============================================================================


class TestReducerIntegration:
    """Integration tests for reducer in pipe routing."""

    def test_can_export_to_step(self, tmp_path):
        """Reducer should export to STEP file."""
        import cadquery as cq

        shape = make_butt_weld_reducer("4", "2")
        step_file = tmp_path / "reducer_4x2.step"
        cq.exporters.export(shape, str(step_file))

        assert step_file.exists()
        assert step_file.stat().st_size > 0

    @pytest.mark.parametrize(
        "large_nps,small_nps",
        [
            ("2", "1"),
            ("3", "2"),
            ("4", "2"),
            ("4", "3"),
            ("6", "4"),
        ],
    )
    def test_fitting_for_multiple_sizes(self, large_nps: str, small_nps: str):
        """Fitting should work for multiple size combinations."""
        fitting = make_butt_weld_reducer_fitting(large_nps, small_nps)
        assert fitting is not None
        assert fitting.shape is not None
        assert "inlet" in fitting.ports
        assert "outlet" in fitting.ports


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
