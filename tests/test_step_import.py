"""
Tests for STEP file fitting import system.

These tests cover:
- Configuration schema (YAML loading/saving)
- STEP marker detection
- Fitting loader
"""

from pathlib import Path
import tempfile

import pytest

from pypegen.fittings.step_import import (
    AttachmentPointConfig,
    FittingConfig,
    FittingLibraryConfig,
    OrientationConfig,
    PortConfig,
    StepMarkerDetector,
)


class TestConfigSchema:
    """Tests for configuration schema dataclasses."""

    def test_orientation_config_with_z_direction(self):
        """Test OrientationConfig with z_direction."""
        config = OrientationConfig(z_direction=(0.0, 0.0, 1.0))
        assert config.z_direction == (0.0, 0.0, 1.0)
        assert config.x_direction is None
        assert config.euler is None

    def test_orientation_config_list_to_tuple(self):
        """Test that lists are converted to tuples."""
        config = OrientationConfig(z_direction=[0.0, 0.0, 1.0])
        assert config.z_direction == (0.0, 0.0, 1.0)
        assert isinstance(config.z_direction, tuple)

    def test_port_config(self):
        """Test PortConfig creation."""
        config = PortConfig(
            name="inlet",
            position=(0.0, 0.0, -50.0),
            orientation=OrientationConfig(z_direction=(0.0, 0.0, -1.0)),
        )
        assert config.name == "inlet"
        assert config.position == (0.0, 0.0, -50.0)
        assert config.orientation.z_direction == (0.0, 0.0, -1.0)

    def test_port_config_from_dict(self):
        """Test PortConfig creation from dict (YAML loading)."""
        config = PortConfig(
            name="outlet",
            position=[0.0, 0.0, 50.0],
            orientation={"z_direction": [0.0, 0.0, 1.0]},
        )
        assert config.position == (0.0, 0.0, 50.0)
        assert config.orientation.z_direction == (0.0, 0.0, 1.0)

    def test_attachment_point_config(self):
        """Test AttachmentPointConfig creation."""
        config = AttachmentPointConfig(
            name="body_label",
            position=(25.0, 0.0, 0.0),
            normal=(1.0, 0.0, 0.0),
            attachment_type="bom",
        )
        assert config.name == "body_label"
        assert config.attachment_type == "bom"

    def test_fitting_config(self):
        """Test FittingConfig creation."""
        config = FittingConfig(
            id="test_valve",
            step_file="valve.step",
            nps="2",
            description="Test valve",
            ports=[
                PortConfig(
                    name="inlet",
                    position=(0.0, 0.0, -50.0),
                    orientation=OrientationConfig(z_direction=(0.0, 0.0, -1.0)),
                ),
            ],
        )
        assert config.id == "test_valve"
        assert config.nps == "2"
        assert len(config.ports) == 1
        assert config.ports[0].name == "inlet"

    def test_fitting_library_yaml_roundtrip(self):
        """Test saving and loading FittingLibraryConfig to/from YAML."""
        original = FittingLibraryConfig(
            fittings=[
                FittingConfig(
                    id="test_valve",
                    step_file="valve.step",
                    nps="2",
                    ports=[
                        PortConfig(
                            name="inlet",
                            position=(0.0, 0.0, -50.0),
                            orientation=OrientationConfig(z_direction=(0.0, 0.0, -1.0)),
                        ),
                        PortConfig(
                            name="outlet",
                            position=(0.0, 0.0, 50.0),
                            orientation=OrientationConfig(z_direction=(0.0, 0.0, 1.0)),
                        ),
                    ],
                    attachment_points=[
                        AttachmentPointConfig(
                            name="body",
                            position=(25.0, 0.0, 0.0),
                            normal=(1.0, 0.0, 0.0),
                        ),
                    ],
                ),
            ],
        )

        with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
            yaml_path = Path(f.name)

        try:
            # Save
            original.to_yaml(yaml_path)

            # Load
            loaded = FittingLibraryConfig.from_yaml(yaml_path)

            # Verify
            assert len(loaded.fittings) == 1
            fitting = loaded.fittings[0]
            assert fitting.id == "test_valve"
            assert fitting.nps == "2"
            assert len(fitting.ports) == 2
            assert fitting.ports[0].name == "inlet"
            assert fitting.ports[0].position == (0.0, 0.0, -50.0)
            assert fitting.ports[1].name == "outlet"
            assert len(fitting.attachment_points) == 1
        finally:
            yaml_path.unlink()


class TestMarkerDetector:
    """Tests for STEP marker detection."""

    @pytest.fixture
    def sample_step_content(self):
        """Create a minimal STEP file content with markers."""
        return """ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Sample'),'2;1');
FILE_NAME('test.step','2024-01-01',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=CARTESIAN_POINT('origin',(0.0,0.0,0.0));
#2=DIRECTION('z',(0.0,0.0,1.0));
#3=DIRECTION('x',(1.0,0.0,0.0));
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);
#10=CARTESIAN_POINT('',(0.0,0.0,-50.0));
#11=DIRECTION('',(0.0,0.0,-1.0));
#12=DIRECTION('',(1.0,0.0,0.0));
#13=AXIS2_PLACEMENT_3D('PORT_INLET',#10,#11,#12);
#20=CARTESIAN_POINT('',(0.0,0.0,50.0));
#21=DIRECTION('',(0.0,0.0,1.0));
#22=DIRECTION('',(1.0,0.0,0.0));
#23=AXIS2_PLACEMENT_3D('PORT_OUTLET',#20,#21,#22);
#30=CARTESIAN_POINT('',(25.0,0.0,0.0));
#31=DIRECTION('',(1.0,0.0,0.0));
#32=DIRECTION('',(0.0,0.0,1.0));
#33=AXIS2_PLACEMENT_3D('ATTACH_BODY',#30,#31,#32);
ENDSEC;
END-ISO-10303-21;
"""

    def test_detect_markers(self, sample_step_content, tmp_path):
        """Test detecting markers from STEP content."""
        step_file = tmp_path / "test.step"
        step_file.write_text(sample_step_content)

        detector = StepMarkerDetector()
        markers = detector.detect_markers(step_file)

        assert len(markers) == 3

        # Find port markers
        port_markers = [m for m in markers if m.marker_type == "port"]
        assert len(port_markers) == 2

        inlet = next(m for m in port_markers if "INLET" in m.name)
        assert inlet.position == (0.0, 0.0, -50.0)
        assert inlet.z_direction == (0.0, 0.0, -1.0)
        assert inlet.port_name == "inlet"

        outlet = next(m for m in port_markers if "OUTLET" in m.name)
        assert outlet.position == (0.0, 0.0, 50.0)
        assert outlet.z_direction == (0.0, 0.0, 1.0)
        assert outlet.port_name == "outlet"

        # Find attachment marker
        attach_markers = [m for m in markers if m.marker_type == "attachment"]
        assert len(attach_markers) == 1
        assert attach_markers[0].position == (25.0, 0.0, 0.0)
        assert attach_markers[0].attachment_name == "body"

    def test_generate_config(self, sample_step_content, tmp_path):
        """Test generating FittingConfig from markers."""
        step_file = tmp_path / "test.step"
        step_file.write_text(sample_step_content)

        detector = StepMarkerDetector()
        config = detector.generate_config(
            step_path=step_file,
            fitting_id="test_valve",
            nps="2",
            description="Test valve",
        )

        assert config.id == "test_valve"
        assert config.nps == "2"
        assert len(config.ports) == 2
        assert len(config.attachment_points) == 1

        # Verify port names are lowercase without prefix
        port_names = {p.name for p in config.ports}
        assert "inlet" in port_names
        assert "outlet" in port_names

    def test_analyze(self, sample_step_content, tmp_path):
        """Test the analyze method."""
        step_file = tmp_path / "test.step"
        step_file.write_text(sample_step_content)

        detector = StepMarkerDetector()
        analysis = detector.analyze(step_file)

        assert analysis["markers_found"] == 3
        assert len(analysis["ports"]) == 2
        assert len(analysis["attachment_points"]) == 1

    def test_no_markers_found(self, tmp_path):
        """Test behavior when no markers are found."""
        step_content = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Sample'),'2;1');
ENDSEC;
DATA;
#1=CARTESIAN_POINT('origin',(0.0,0.0,0.0));
#2=DIRECTION('z',(0.0,0.0,1.0));
#3=DIRECTION('x',(1.0,0.0,0.0));
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);
ENDSEC;
END-ISO-10303-21;
"""
        step_file = tmp_path / "no_markers.step"
        step_file.write_text(step_content)

        detector = StepMarkerDetector()
        markers = detector.detect_markers(step_file)

        assert len(markers) == 0

    def test_file_not_found(self):
        """Test error when STEP file doesn't exist."""
        detector = StepMarkerDetector()
        with pytest.raises(FileNotFoundError):
            detector.detect_markers("/nonexistent/path/file.step")

    def test_origin_marker(self, tmp_path):
        """Test detecting ORIGIN marker."""
        step_content = """ISO-10303-21;
HEADER;
ENDSEC;
DATA;
#1=CARTESIAN_POINT('',(10.0,20.0,30.0));
#2=DIRECTION('',(0.0,0.0,1.0));
#3=DIRECTION('',(1.0,0.0,0.0));
#4=AXIS2_PLACEMENT_3D('ORIGIN',#1,#2,#3);
ENDSEC;
END-ISO-10303-21;
"""
        step_file = tmp_path / "with_origin.step"
        step_file.write_text(step_content)

        detector = StepMarkerDetector()
        markers = detector.detect_markers(step_file)

        assert len(markers) == 1
        assert markers[0].marker_type == "origin"
        assert markers[0].position == (10.0, 20.0, 30.0)

        analysis = detector.analyze(step_file)
        assert analysis["origin"] == (10.0, 20.0, 30.0)
