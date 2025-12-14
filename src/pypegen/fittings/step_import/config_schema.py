"""
Configuration schema for STEP file fitting definitions.

This module defines the dataclasses used to configure fittings loaded from
STEP files. The configuration can be:
- Generated automatically by marker detection
- Created interactively via the CLI helper
- Written manually in YAML format
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import yaml


def _to_tuple3(value: list | tuple) -> tuple[float, float, float]:
    """Convert a list or tuple to a 3-element float tuple."""
    return (float(value[0]), float(value[1]), float(value[2]))


@dataclass
class OrientationConfig:
    """
    Port orientation specification - supports multiple formats.

    At least one of z_direction or euler must be provided.

    Attributes:
        z_direction: Direction vector for port Z-axis (points outward).
                    This is the most intuitive format for most users.
        x_direction: Optional direction vector for port X-axis (rotational reference).
                    If not provided, computed automatically.
        euler: ZYX Euler angles in degrees. Alternative to direction vectors.
        cad_marker: Reference to a detected CAD marker name.
                   Used when generating config from marker detection.
    """

    z_direction: tuple[float, float, float] | None = None
    x_direction: tuple[float, float, float] | None = None
    euler: tuple[float, float, float] | None = None
    cad_marker: str | None = None

    def __post_init__(self):
        # Convert lists to tuples if needed (from YAML loading)
        if isinstance(self.z_direction, list):
            self.z_direction = _to_tuple3(self.z_direction)
        if isinstance(self.x_direction, list):
            self.x_direction = _to_tuple3(self.x_direction)
        if isinstance(self.euler, list):
            self.euler = _to_tuple3(self.euler)


@dataclass
class PortConfig:
    """
    Configuration for a single port on a fitting.

    Attributes:
        name: Port identifier (e.g., "inlet", "outlet", "branch")
        position: 3D position (x, y, z) in the fitting's local coordinates
        orientation: Port orientation specification
    """

    name: str
    position: tuple[float, float, float]
    orientation: OrientationConfig

    def __post_init__(self):
        # Convert lists to tuples if needed (from YAML loading)
        if isinstance(self.position, list):
            self.position = _to_tuple3(self.position)
        # Handle orientation as dict from YAML
        if isinstance(self.orientation, dict):
            self.orientation = OrientationConfig(**self.orientation)


@dataclass
class AttachmentPointConfig:
    """
    Configuration for an attachment point on a fitting.

    Attachment points are used for placing annotations like weld symbols
    and BOM balloons on the drawing.

    Attributes:
        name: Attachment point identifier
        position: 3D position (x, y, z) in fitting's local coordinates
        normal: Direction vector pointing outward from the surface
        attachment_type: Type of annotation ("weld", "bom", or "label")
        port_name: Associated port name, if any
    """

    name: str
    position: tuple[float, float, float]
    normal: tuple[float, float, float]
    attachment_type: Literal["weld", "bom", "label"] = "bom"
    port_name: str | None = None

    def __post_init__(self):
        # Convert lists to tuples if needed (from YAML loading)
        if isinstance(self.position, list):
            self.position = _to_tuple3(self.position)
        if isinstance(self.normal, list):
            self.normal = _to_tuple3(self.normal)


@dataclass
class TransformConfig:
    """
    Optional transform to apply to STEP geometry before port definitions.

    Useful for aligning the model's coordinate system with pypegen's convention.

    Attributes:
        translation: Translation offset (x, y, z)
        rotation_euler: ZYX Euler angles in degrees (optional)
    """

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_euler: tuple[float, float, float] | None = None

    def __post_init__(self):
        if isinstance(self.translation, list):
            self.translation = _to_tuple3(self.translation)
        if isinstance(self.rotation_euler, list):
            self.rotation_euler = _to_tuple3(self.rotation_euler)


@dataclass
class FittingConfig:
    """
    Complete configuration for a single fitting from a STEP file.

    Attributes:
        id: Unique identifier for this fitting
        step_file: Path to the STEP file (relative to config file or absolute)
        nps: Nominal pipe size (e.g., "2", "1/2")
        description: Human-readable description
        category: Fitting category (e.g., "valve", "sensor", "connector")
        units: Units of the STEP file ("mm" or "inch"). Auto-detected if None.
        origin_transform: Optional transform to align the model
        ports: List of port configurations
        attachment_points: List of attachment point configurations
    """

    id: str
    step_file: str
    nps: str
    description: str = ""
    category: str = "custom"
    units: Literal["mm", "inch"] | None = None
    origin_transform: TransformConfig | None = None
    ports: list[PortConfig] = field(default_factory=list)
    attachment_points: list[AttachmentPointConfig] = field(default_factory=list)

    def __post_init__(self):
        # Handle origin_transform as dict from YAML
        if isinstance(self.origin_transform, dict):
            self.origin_transform = TransformConfig(**self.origin_transform)
        # Handle ports as list of dicts from YAML
        self.ports = [
            PortConfig(**p) if isinstance(p, dict) else p for p in self.ports
        ]
        # Handle attachment_points as list of dicts from YAML
        self.attachment_points = [
            AttachmentPointConfig(**ap) if isinstance(ap, dict) else ap
            for ap in self.attachment_points
        ]


@dataclass
class FittingLibraryConfig:
    """
    Root configuration for a fitting library (multiple fittings in one file).

    Attributes:
        version: Config file version (currently "1.0")
        settings: Global settings dictionary
        fittings: List of fitting configurations
    """

    version: str = "1.0"
    settings: dict = field(default_factory=dict)
    fittings: list[FittingConfig] = field(default_factory=list)

    def __post_init__(self):
        # Handle fittings as list of dicts from YAML
        self.fittings = [
            FittingConfig(**f) if isinstance(f, dict) else f for f in self.fittings
        ]

    @classmethod
    def from_yaml(cls, yaml_path: str | Path) -> "FittingLibraryConfig":
        """Load a fitting library configuration from a YAML file."""
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        return cls(**data)

    def to_yaml(self, yaml_path: str | Path) -> None:
        """Save the fitting library configuration to a YAML file."""
        data = self._to_dict()
        with open(yaml_path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    def _to_dict(self) -> dict[str, Any]:
        """Convert to a dictionary suitable for YAML serialization."""
        return {
            "version": self.version,
            "settings": self.settings,
            "fittings": [self._fitting_to_dict(f) for f in self.fittings],
        }

    def _fitting_to_dict(self, fitting: FittingConfig) -> dict[str, Any]:
        """Convert a FittingConfig to a dictionary."""
        result = {
            "id": fitting.id,
            "step_file": fitting.step_file,
            "nps": fitting.nps,
        }
        if fitting.description:
            result["description"] = fitting.description
        if fitting.category != "custom":
            result["category"] = fitting.category
        if fitting.units:
            result["units"] = fitting.units
        if fitting.origin_transform:
            result["origin_transform"] = {
                "translation": list(fitting.origin_transform.translation),
            }
            if fitting.origin_transform.rotation_euler:
                result["origin_transform"]["rotation_euler"] = list(
                    fitting.origin_transform.rotation_euler
                )
        if fitting.ports:
            result["ports"] = [self._port_to_dict(p) for p in fitting.ports]
        if fitting.attachment_points:
            result["attachment_points"] = [
                self._attachment_to_dict(ap) for ap in fitting.attachment_points
            ]
        return result

    def _port_to_dict(self, port: PortConfig) -> dict[str, Any]:
        """Convert a PortConfig to a dictionary."""
        result = {
            "name": port.name,
            "position": list(port.position),
            "orientation": {},
        }
        if port.orientation.z_direction:
            result["orientation"]["z_direction"] = list(port.orientation.z_direction)
        if port.orientation.x_direction:
            result["orientation"]["x_direction"] = list(port.orientation.x_direction)
        if port.orientation.euler:
            result["orientation"]["euler"] = list(port.orientation.euler)
        if port.orientation.cad_marker:
            result["orientation"]["cad_marker"] = port.orientation.cad_marker
        return result

    def _attachment_to_dict(self, ap: AttachmentPointConfig) -> dict[str, Any]:
        """Convert an AttachmentPointConfig to a dictionary."""
        result = {
            "name": ap.name,
            "position": list(ap.position),
            "normal": list(ap.normal),
            "attachment_type": ap.attachment_type,
        }
        if ap.port_name:
            result["port_name"] = ap.port_name
        return result
