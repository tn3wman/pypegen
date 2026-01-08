"""
STEP file marker detector.

This module parses STEP files to detect named coordinate systems that follow
the pypegen port naming convention (PORT_*, ATTACH_*, ORIGIN).

STEP files use the AXIS2_PLACEMENT_3D entity to define coordinate systems:
    #100=AXIS2_PLACEMENT_3D('PORT_INLET',#101,#102,#103);
    #101=CARTESIAN_POINT('',(0.0,0.0,-50.0));
    #102=DIRECTION('',(0.0,0.0,-1.0));  -- Z-axis
    #103=DIRECTION('',(1.0,0.0,0.0));   -- X-axis

Autodesk Inventor exports User Coordinate Systems (UCS) as AXIS2_PLACEMENT_3D
entities with the UCS name preserved.
"""

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from .config_schema import (
    AttachmentPointConfig,
    FittingConfig,
    FittingLibraryConfig,
    OrientationConfig,
    PortConfig,
)


@dataclass
class DetectedMarker:
    """
    A coordinate system marker detected in a STEP file.

    Attributes:
        name: The marker name (e.g., "PORT_INLET", "ATTACH_BODY")
        entity_id: The STEP entity ID (e.g., "#2642")
        position: 3D position (x, y, z)
        z_direction: Z-axis direction vector (points outward for ports)
        x_direction: X-axis direction vector (rotational reference)
        marker_type: Detected type based on name prefix
    """

    name: str
    entity_id: str
    position: tuple[float, float, float]
    z_direction: tuple[float, float, float]
    x_direction: tuple[float, float, float]
    marker_type: Literal["port", "attachment", "origin", "unknown"]

    @property
    def port_name(self) -> str:
        """Extract the port name without the PORT_ prefix."""
        if self.name.startswith("PORT_"):
            return self.name[5:].lower()
        return self.name.lower()

    @property
    def attachment_name(self) -> str:
        """Extract the attachment name without the ATTACH_ prefix."""
        if self.name.startswith("ATTACH_"):
            return self.name[7:].lower()
        return self.name.lower()


class StepMarkerDetector:
    """
    Detect named coordinate systems in STEP files.

    This class parses STEP files using text processing to find AXIS2_PLACEMENT_3D
    entities with names matching the pypegen convention:
    - PORT_<name>: Connection ports
    - ATTACH_<name>: Attachment points for annotations
    - ORIGIN: Optional origin alignment

    Usage:
        detector = StepMarkerDetector()
        markers = detector.detect_markers("valve.step")

        # Generate a config from detected markers
        config = detector.generate_config(
            step_path="valve.step",
            fitting_id="ball_valve_2in",
            nps="2"
        )
    """

    # Naming convention prefixes
    PORT_PREFIX = "PORT_"
    ATTACH_PREFIX = "ATTACH_"
    ORIGIN_NAME = "ORIGIN"

    # Regex patterns for STEP entity parsing
    # Match AXIS2_PLACEMENT_3D with a name
    _AXIS2_PATTERN = re.compile(
        r"(#\d+)\s*=\s*AXIS2_PLACEMENT_3D\s*\(\s*'([^']+)'\s*,\s*(#\d+)\s*,\s*(#\d+)\s*,\s*(#\d+)\s*\)",
        re.IGNORECASE,
    )

    # Match CARTESIAN_POINT
    _POINT_PATTERN = re.compile(
        r"(#\d+)\s*=\s*CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(\s*([^)]+)\s*\)\s*\)",
        re.IGNORECASE,
    )

    # Match DIRECTION
    _DIRECTION_PATTERN = re.compile(
        r"(#\d+)\s*=\s*DIRECTION\s*\(\s*'[^']*'\s*,\s*\(\s*([^)]+)\s*\)\s*\)",
        re.IGNORECASE,
    )

    def detect_markers(self, step_path: str | Path) -> list[DetectedMarker]:
        """
        Find all named markers in a STEP file.

        Args:
            step_path: Path to the STEP file

        Returns:
            List of DetectedMarker objects for each found coordinate system
        """
        step_path = Path(step_path)
        if not step_path.exists():
            raise FileNotFoundError(f"STEP file not found: {step_path}")

        # Read and parse the STEP file
        content = step_path.read_text(encoding="utf-8", errors="replace")

        # Build lookup tables for entities
        points = self._parse_cartesian_points(content)
        directions = self._parse_directions(content)

        # Find AXIS2_PLACEMENT_3D entities with our naming convention
        markers = []
        for match in self._AXIS2_PATTERN.finditer(content):
            entity_id = match.group(1)
            name = match.group(2)
            point_ref = match.group(3)
            z_dir_ref = match.group(4)
            x_dir_ref = match.group(5)

            # Check if this matches our naming convention
            marker_type = self._classify_marker(name)
            if marker_type == "unknown":
                continue

            # Look up the referenced entities
            position = points.get(point_ref)
            z_direction = directions.get(z_dir_ref)
            x_direction = directions.get(x_dir_ref)

            if position is None or z_direction is None or x_direction is None:
                # Could not resolve all references, skip this marker
                continue

            markers.append(
                DetectedMarker(
                    name=name,
                    entity_id=entity_id,
                    position=position,
                    z_direction=z_direction,
                    x_direction=x_direction,
                    marker_type=marker_type,
                )
            )

        return markers

    def _parse_cartesian_points(
        self, content: str
    ) -> dict[str, tuple[float, float, float]]:
        """Parse all CARTESIAN_POINT entities into a lookup table."""
        points = {}
        for match in self._POINT_PATTERN.finditer(content):
            entity_id = match.group(1)
            coords_str = match.group(2)
            coords = self._parse_float_tuple(coords_str)
            if coords and len(coords) >= 3:
                points[entity_id] = (coords[0], coords[1], coords[2])
        return points

    def _parse_directions(
        self, content: str
    ) -> dict[str, tuple[float, float, float]]:
        """Parse all DIRECTION entities into a lookup table."""
        directions = {}
        for match in self._DIRECTION_PATTERN.finditer(content):
            entity_id = match.group(1)
            coords_str = match.group(2)
            coords = self._parse_float_tuple(coords_str)
            if coords and len(coords) >= 3:
                directions[entity_id] = (coords[0], coords[1], coords[2])
        return directions

    def _parse_float_tuple(self, s: str) -> list[float]:
        """Parse a comma-separated string of floats."""
        try:
            return [float(x.strip()) for x in s.split(",")]
        except ValueError:
            return []

    def _classify_marker(
        self, name: str
    ) -> Literal["port", "attachment", "origin", "unknown"]:
        """Classify a marker based on its name."""
        upper_name = name.upper()
        if upper_name.startswith(self.PORT_PREFIX):
            return "port"
        elif upper_name.startswith(self.ATTACH_PREFIX):
            return "attachment"
        elif upper_name == self.ORIGIN_NAME:
            return "origin"
        else:
            return "unknown"

    def generate_config(
        self,
        step_path: str | Path,
        fitting_id: str,
        nps: str,
        description: str = "",
        category: str = "custom",
    ) -> FittingConfig:
        """
        Generate a FittingConfig from detected markers in a STEP file.

        Args:
            step_path: Path to the STEP file
            fitting_id: Unique identifier for this fitting
            nps: Nominal pipe size
            description: Human-readable description
            category: Fitting category

        Returns:
            FittingConfig populated with detected ports and attachment points
        """
        step_path = Path(step_path)
        markers = self.detect_markers(step_path)

        ports = []
        attachment_points = []

        for marker in markers:
            if marker.marker_type == "port":
                ports.append(
                    PortConfig(
                        name=marker.port_name,
                        position=marker.position,
                        orientation=OrientationConfig(
                            z_direction=marker.z_direction,
                            x_direction=marker.x_direction,
                            cad_marker=marker.name,
                        ),
                    )
                )
            elif marker.marker_type == "attachment":
                attachment_points.append(
                    AttachmentPointConfig(
                        name=marker.attachment_name,
                        position=marker.position,
                        normal=marker.z_direction,  # Use Z as outward normal
                        attachment_type="bom",
                    )
                )

        return FittingConfig(
            id=fitting_id,
            step_file=step_path.name,
            nps=nps,
            description=description,
            category=category,
            ports=ports,
            attachment_points=attachment_points,
        )

    def generate_library_config(
        self,
        step_path: str | Path,
        fitting_id: str,
        nps: str,
        description: str = "",
        category: str = "custom",
    ) -> FittingLibraryConfig:
        """
        Generate a complete FittingLibraryConfig from a single STEP file.

        This is a convenience method that wraps generate_config() and returns
        a library config suitable for saving to YAML.

        Args:
            step_path: Path to the STEP file
            fitting_id: Unique identifier for this fitting
            nps: Nominal pipe size
            description: Human-readable description
            category: Fitting category

        Returns:
            FittingLibraryConfig containing the single fitting
        """
        config = self.generate_config(
            step_path=step_path,
            fitting_id=fitting_id,
            nps=nps,
            description=description,
            category=category,
        )
        return FittingLibraryConfig(fittings=[config])

    def analyze(self, step_path: str | Path) -> dict:
        """
        Analyze a STEP file and return a summary.

        Useful for the CLI tool to show what was found.

        Args:
            step_path: Path to the STEP file

        Returns:
            Dictionary with analysis results
        """
        step_path = Path(step_path)
        markers = self.detect_markers(step_path)

        # Count marker types
        ports = [m for m in markers if m.marker_type == "port"]
        attachments = [m for m in markers if m.marker_type == "attachment"]
        origin = [m for m in markers if m.marker_type == "origin"]

        return {
            "file": str(step_path),
            "markers_found": len(markers),
            "ports": [
                {"name": m.name, "position": m.position, "z_direction": m.z_direction}
                for m in ports
            ],
            "attachment_points": [
                {"name": m.name, "position": m.position} for m in attachments
            ],
            "origin": origin[0].position if origin else None,
        }
