#!/usr/bin/env python3
"""
Tests for ASME gasket features.

Tests cover:
- Gasket dimension data completeness
- Gasket geometry generation
- Port definitions and orientations
- Flange-to-flange mating with gaskets
- Gap verification (gasket thickness = flange gap)
"""

import numpy as np
import pytest

from pypegen.fittings.gasket_flat_ring import (
    ASME_B1621_CLASS150_FLAT,
    ASME_B1621_CLASS300_FLAT,
    make_flat_ring_gasket,
)
from pypegen.fittings.gasket_spiral_wound import (
    ASME_B1620_CLASS300_SPIRAL,
    make_spiral_wound_gasket,
)
from pypegen.fittings.pipe_fittings import (
    apply_transform_to_shape,
    get_gasket_thickness,
    get_position,
    identity_matrix,
    make_gasket,
    make_weld_neck_flange,
    mate_flanges_with_gasket,
)
from pypegen.fittings.weld_neck_flange import ASME_B165_CLASS300_WN


# =============================================================================
# GASKET DIMENSION DATA TESTS
# =============================================================================


class TestFlatRingGasketDimensions:
    """Test ASME B16.21 flat ring gasket dimension data."""

    @pytest.mark.parametrize(
        "nps",
        ["1/2", "3/4", "1", "1-1/4", "1-1/2", "2", "2-1/2", "3", "4", "6", "8", "10", "12"],
    )
    def test_class300_sizes_available(self, nps: str):
        """All common sizes should have Class 300 flat ring gasket data."""
        assert nps in ASME_B1621_CLASS300_FLAT
        dims = ASME_B1621_CLASS300_FLAT[nps]
        assert dims.inner_diameter > 0
        assert dims.outer_diameter > dims.inner_diameter
        assert dims.thickness > 0

    @pytest.mark.parametrize(
        "nps",
        ["1/2", "3/4", "1", "1-1/4", "1-1/2", "2", "2-1/2", "3", "4", "6", "8", "10", "12"],
    )
    def test_class150_sizes_available(self, nps: str):
        """All common sizes should have Class 150 flat ring gasket data."""
        assert nps in ASME_B1621_CLASS150_FLAT
        dims = ASME_B1621_CLASS150_FLAT[nps]
        assert dims.inner_diameter > 0
        assert dims.outer_diameter > dims.inner_diameter

    @pytest.mark.parametrize("nps", ["2", "4", "6", "8"])
    def test_gasket_od_extends_to_bolt_circle(self, nps: str):
        """IBC gasket OD should extend to just inside bolt circle.

        ASME B16.21 defines two gasket types:
        - RF (Raised Face): OD matches raised face
        - IBC (Inner Bolt Circle): OD extends beyond raised face to bolt circle

        Our dimension data uses IBC style, which is the more common type.
        The gasket OD should be less than the bolt circle diameter.
        """
        gasket = ASME_B1621_CLASS300_FLAT[nps]
        flange = ASME_B165_CLASS300_WN[nps]
        # Convert flange bolt circle from inches to mm
        bolt_circle_mm = flange.bolt_circle * 25.4
        # IBC gasket OD should be less than bolt circle
        assert gasket.outer_diameter < bolt_circle_mm

    @pytest.mark.parametrize("nps", ["2", "4", "6"])
    def test_class150_od_smaller_than_class300(self, nps: str):
        """Class 150 gasket OD should be smaller than Class 300."""
        class150 = ASME_B1621_CLASS150_FLAT[nps]
        class300 = ASME_B1621_CLASS300_FLAT[nps]
        assert class150.outer_diameter <= class300.outer_diameter


class TestSpiralWoundGasketDimensions:
    """Test ASME B16.20 spiral wound gasket dimension data."""

    @pytest.mark.parametrize(
        "nps",
        ["1/2", "3/4", "1", "1-1/4", "1-1/2", "2", "2-1/2", "3", "4", "6", "8"],
    )
    def test_class300_sizes_available(self, nps: str):
        """Standard sizes should have spiral wound gasket data."""
        assert nps in ASME_B1620_CLASS300_SPIRAL
        dims = ASME_B1620_CLASS300_SPIRAL[nps]
        assert dims.inner_ring_id > 0
        assert dims.sealing_element_id > dims.inner_ring_id
        assert dims.sealing_element_od > dims.sealing_element_id
        assert dims.outer_ring_od > dims.sealing_element_od
        assert dims.compressed_thickness > 0

    @pytest.mark.parametrize("nps", ["2", "4", "6"])
    def test_spiral_wound_thicker_than_flat_ring(self, nps: str):
        """Spiral wound gaskets should be thicker than flat ring gaskets."""
        flat = ASME_B1621_CLASS300_FLAT[nps]
        spiral = ASME_B1620_CLASS300_SPIRAL[nps]
        assert spiral.compressed_thickness > flat.thickness


# =============================================================================
# GASKET GEOMETRY TESTS
# =============================================================================


class TestFlatRingGasketGeometry:
    """Test flat ring gasket shape generation."""

    def test_creates_valid_shape(self):
        """Flat ring gasket should create valid CadQuery shape."""
        shape = make_flat_ring_gasket("2")
        assert shape is not None
        bb = shape.BoundingBox()
        # Should have non-zero size in all dimensions
        assert bb.xmax - bb.xmin > 0
        assert bb.ymax - bb.ymin > 0
        assert bb.zmax - bb.zmin > 0

    def test_thickness_matches_standard(self):
        """Default thickness should match standard 1.6mm."""
        shape = make_flat_ring_gasket("2")
        bb = shape.BoundingBox()
        thickness = bb.zmax - bb.zmin
        assert abs(thickness - 1.6) < 0.01

    def test_custom_thickness(self):
        """Custom thickness should be respected."""
        shape = make_flat_ring_gasket("2", thickness_mm=3.2)
        bb = shape.BoundingBox()
        thickness = bb.zmax - bb.zmin
        assert abs(thickness - 3.2) < 0.01

    def test_centered_at_origin(self):
        """Gasket should be centered at origin (Z from -t/2 to +t/2)."""
        shape = make_flat_ring_gasket("2")
        bb = shape.BoundingBox()
        # Z should be centered around 0
        assert abs((bb.zmax + bb.zmin) / 2) < 0.01
        # X and Y should also be centered
        assert abs((bb.xmax + bb.xmin) / 2) < 0.01
        assert abs((bb.ymax + bb.ymin) / 2) < 0.01


class TestSpiralWoundGasketGeometry:
    """Test spiral wound gasket shape generation."""

    def test_creates_valid_shape(self):
        """Spiral wound gasket should create valid CadQuery shape."""
        shape = make_spiral_wound_gasket("2")
        assert shape is not None
        bb = shape.BoundingBox()
        assert bb.xmax - bb.xmin > 0
        assert bb.ymax - bb.ymin > 0
        assert bb.zmax - bb.zmin > 0

    def test_thickness_matches_standard(self):
        """Default thickness should match standard ~4.5mm."""
        shape = make_spiral_wound_gasket("2")
        bb = shape.BoundingBox()
        thickness = bb.zmax - bb.zmin
        assert abs(thickness - 4.5) < 0.1


# =============================================================================
# GASKET FITTING TESTS
# =============================================================================


class TestGasketFitting:
    """Test make_gasket() function and Fitting object."""

    def test_flat_ring_creates_fitting(self):
        """make_gasket should create valid Fitting for flat ring."""
        gasket = make_gasket("2", gasket_type="flat_ring")
        assert gasket is not None
        assert gasket.shape is not None
        assert gasket.nps == "2"

    def test_spiral_wound_creates_fitting(self):
        """make_gasket should create valid Fitting for spiral wound."""
        gasket = make_gasket("2", gasket_type="spiral_wound")
        assert gasket is not None
        assert gasket.shape is not None

    def test_has_two_ports(self):
        """Gasket should have face_a and face_b ports."""
        gasket = make_gasket("2")
        assert "face_a" in gasket.ports
        assert "face_b" in gasket.ports

    def test_port_directions_oppose(self):
        """Gasket ports should face opposite directions (opposing Z-axes)."""
        gasket = make_gasket("2")
        dir_a = gasket.get_port("face_a").direction
        dir_b = gasket.get_port("face_b").direction
        # Dot product should be -1 (opposite directions)
        dot = dir_a[0] * dir_b[0] + dir_a[1] * dir_b[1] + dir_a[2] * dir_b[2]
        assert abs(dot + 1.0) < 1e-6

    def test_port_positions_at_faces(self):
        """Port positions should be at gasket faces."""
        gasket = make_gasket("2", gasket_type="flat_ring")
        thickness = get_gasket_thickness("2", gasket_type="flat_ring")

        pos_a = gasket.get_port("face_a").position
        pos_b = gasket.get_port("face_b").position

        # face_a at +Z, face_b at -Z
        assert abs(pos_a[2] - thickness / 2) < 0.01
        assert abs(pos_b[2] + thickness / 2) < 0.01

    def test_no_weld_attachment_points(self):
        """Gaskets should have no weld attachment points (bolted joints)."""
        gasket = make_gasket("2")
        weld_points = gasket.get_weld_attachment_points()
        assert len(weld_points) == 0

    def test_invalid_gasket_type_raises(self):
        """Invalid gasket type should raise ValueError."""
        with pytest.raises(ValueError, match="Unknown gasket type"):
            make_gasket("2", gasket_type="invalid")


# =============================================================================
# FLANGE MATING TESTS
# =============================================================================


class TestFlangeMatingWithGasket:
    """Test mate_flanges_with_gasket() function."""

    def test_returns_three_values(self):
        """Should return flange_b_transform, gasket, gasket_transform."""
        flange_a = make_weld_neck_flange("2")
        flange_b = make_weld_neck_flange("2")

        result = mate_flanges_with_gasket(
            flange_a, identity_matrix(), flange_b, gasket_type="flat_ring"
        )

        assert len(result) == 3
        flange_b_transform, gasket, gasket_transform = result
        assert flange_b_transform.shape == (4, 4)
        assert gasket is not None
        assert gasket_transform.shape == (4, 4)

    def test_gap_equals_gasket_thickness_flat_ring(self):
        """Gap between flange faces should equal flat ring gasket thickness."""
        flange_a = make_weld_neck_flange("2")
        flange_b = make_weld_neck_flange("2")

        flange_a_transform = identity_matrix()
        flange_b_transform, gasket, _ = mate_flanges_with_gasket(
            flange_a, flange_a_transform, flange_b, gasket_type="flat_ring"
        )

        # Get face port positions in world coordinates
        face_a_world = flange_a_transform @ flange_a.get_port("face").transform
        face_b_world = flange_b_transform @ flange_b.get_port("face").transform

        face_a_pos = get_position(face_a_world)
        face_b_pos = get_position(face_b_world)

        # Calculate gap
        gap = np.linalg.norm(np.array(face_b_pos) - np.array(face_a_pos))

        # Expected gap is gasket thickness (1.6mm for flat ring)
        expected_gap = get_gasket_thickness("2", gasket_type="flat_ring")
        assert abs(gap - expected_gap) < 0.1

    def test_gap_equals_gasket_thickness_spiral_wound(self):
        """Gap between flange faces should equal spiral wound gasket thickness."""
        flange_a = make_weld_neck_flange("2")
        flange_b = make_weld_neck_flange("2")

        flange_a_transform = identity_matrix()
        flange_b_transform, gasket, _ = mate_flanges_with_gasket(
            flange_a, flange_a_transform, flange_b, gasket_type="spiral_wound"
        )

        face_a_world = flange_a_transform @ flange_a.get_port("face").transform
        face_b_world = flange_b_transform @ flange_b.get_port("face").transform

        face_a_pos = get_position(face_a_world)
        face_b_pos = get_position(face_b_world)

        gap = np.linalg.norm(np.array(face_b_pos) - np.array(face_a_pos))
        expected_gap = get_gasket_thickness("2", gasket_type="spiral_wound")
        assert abs(gap - expected_gap) < 0.1

    def test_gasket_centered_between_flanges(self):
        """Gasket should be centered between flange faces."""
        flange_a = make_weld_neck_flange("2")
        flange_b = make_weld_neck_flange("2")

        flange_a_transform = identity_matrix()
        flange_b_transform, _, gasket_transform = mate_flanges_with_gasket(
            flange_a, flange_a_transform, flange_b
        )

        # Get positions
        gasket_center = get_position(gasket_transform)
        face_a_world = flange_a_transform @ flange_a.get_port("face").transform
        face_b_world = flange_b_transform @ flange_b.get_port("face").transform
        face_a_pos = get_position(face_a_world)
        face_b_pos = get_position(face_b_world)

        # Gasket center should be midpoint between flange faces
        expected_center = [(a + b) / 2 for a, b in zip(face_a_pos, face_b_pos)]
        for i in range(3):
            assert abs(gasket_center[i] - expected_center[i]) < 0.1

    def test_flanges_facing_each_other(self):
        """Flange faces should point toward each other after mating."""
        flange_a = make_weld_neck_flange("2")
        flange_b = make_weld_neck_flange("2")

        flange_a_transform = identity_matrix()
        flange_b_transform, _, _ = mate_flanges_with_gasket(
            flange_a, flange_a_transform, flange_b
        )

        # Get face port Z-axes (directions they point)
        face_a_world = flange_a_transform @ flange_a.get_port("face").transform
        face_b_world = flange_b_transform @ flange_b.get_port("face").transform

        # Extract Z-axis (column 2) from rotation part
        dir_a = face_a_world[:3, 2]
        dir_b = face_b_world[:3, 2]

        # Faces should point in opposite directions (dot product = -1)
        dot = np.dot(dir_a, dir_b)
        assert abs(dot + 1.0) < 1e-6


# =============================================================================
# UTILITY FUNCTION TESTS
# =============================================================================


class TestGetGasketThickness:
    """Test get_gasket_thickness() utility function."""

    def test_flat_ring_class300(self):
        """Should return 1.6mm for Class 300 flat ring."""
        thickness = get_gasket_thickness("2", gasket_type="flat_ring", pressure_class=300)
        assert abs(thickness - 1.6) < 0.01

    def test_spiral_wound(self):
        """Should return ~4.5mm for spiral wound."""
        thickness = get_gasket_thickness("2", gasket_type="spiral_wound")
        assert abs(thickness - 4.5) < 0.1

    def test_invalid_size_raises(self):
        """Invalid NPS should raise ValueError."""
        with pytest.raises(ValueError):
            get_gasket_thickness("999", gasket_type="flat_ring")


# =============================================================================
# INTEGRATION TESTS
# =============================================================================


class TestGasketIntegration:
    """Integration tests for gasket assembly."""

    def test_assembly_can_be_exported(self, tmp_path):
        """Complete flange-gasket-flange assembly should export to STEP."""
        import cadquery as cq

        flange_a = make_weld_neck_flange("2")
        flange_b = make_weld_neck_flange("2")

        flange_a_transform = identity_matrix()
        flange_b_transform, gasket, gasket_transform = mate_flanges_with_gasket(
            flange_a, flange_a_transform, flange_b
        )

        # Create assembly
        parts = [
            apply_transform_to_shape(flange_a.shape, flange_a_transform),
            apply_transform_to_shape(gasket.shape, gasket_transform),
            apply_transform_to_shape(flange_b.shape, flange_b_transform),
        ]
        assembly = cq.Compound.makeCompound(parts)

        # Export
        step_file = tmp_path / "flange_gasket_assembly.step"
        cq.exporters.export(assembly, str(step_file))

        assert step_file.exists()
        assert step_file.stat().st_size > 0

    @pytest.mark.parametrize("nps", ["1", "2", "4", "6"])
    def test_multiple_sizes(self, nps: str):
        """Assembly should work for multiple pipe sizes."""
        flange_a = make_weld_neck_flange(nps)
        flange_b = make_weld_neck_flange(nps)

        flange_b_transform, gasket, gasket_transform = mate_flanges_with_gasket(
            flange_a, identity_matrix(), flange_b
        )

        # Verify gasket was created with correct NPS
        assert gasket.nps == nps

        # Verify shapes can be transformed
        assert apply_transform_to_shape(gasket.shape, gasket_transform) is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
