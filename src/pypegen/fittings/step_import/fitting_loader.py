"""
Fitting loader for STEP files.

This module loads STEP files and their configurations to produce pypegen
Fitting objects that can be used in pipe routing.
"""

from collections.abc import Callable
from pathlib import Path

import cadquery as cq
import numpy as np

from ..pipe_fittings import (
    AttachmentPoint,
    Fitting,
    Port,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
)
from .config_schema import (
    FittingConfig,
    FittingLibraryConfig,
    OrientationConfig,
    PortConfig,
)


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    """Normalize a 3D vector."""
    magnitude = (v[0]**2 + v[1]**2 + v[2]**2) ** 0.5
    if magnitude < 1e-10:
        return (0.0, 0.0, 1.0)  # Default to Z-up if zero vector
    return (v[0] / magnitude, v[1] / magnitude, v[2] / magnitude)


def _cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Cross product of two 3D vectors."""
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    """Dot product of two 3D vectors."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


class FittingLoader:
    """
    Load Fitting objects from STEP files using configuration.

    This class handles:
    - Loading STEP geometry via CadQuery
    - Building Port objects with correct transformation matrices
    - Building AttachmentPoint objects
    - Unit conversion (inch to mm)

    Usage:
        loader = FittingLoader()

        # Load from config file
        fittings = loader.load_library("my_fittings.yaml")

        # Load single fitting
        valve = loader.load_fitting(config, base_path="./step_files/")

        # Create fitting factory function (for use with PipeRoute)
        make_valve = loader.create_factory(config)
        valve = make_valve("2")  # Creates fitting for NPS 2"
    """

    # Conversion factor: inches to mm
    INCH_TO_MM = 25.4

    def load_library(
        self,
        config_path: str | Path,
    ) -> dict[str, Fitting]:
        """
        Load all fittings from a library config file.

        Args:
            config_path: Path to the YAML configuration file

        Returns:
            Dictionary mapping fitting IDs to Fitting objects
        """
        config_path = Path(config_path)
        library = FittingLibraryConfig.from_yaml(config_path)
        base_path = config_path.parent

        fittings = {}
        for fitting_config in library.fittings:
            fitting = self.load_fitting(fitting_config, base_path)
            fittings[fitting_config.id] = fitting

        return fittings

    def load_fitting(
        self,
        config: FittingConfig,
        base_path: str | Path | None = None,
    ) -> Fitting:
        """
        Load a single fitting from configuration.

        Args:
            config: FittingConfig with STEP file path and port definitions
            base_path: Base directory for resolving relative STEP file paths

        Returns:
            Fitting object ready for use in pipe routing
        """
        # Resolve STEP file path
        step_path = Path(config.step_file)
        if base_path and not step_path.is_absolute():
            step_path = Path(base_path) / step_path

        # Load geometry
        shape = self._load_step_shape(step_path)

        # Determine unit scale
        scale = 1.0
        if config.units == "inch":
            scale = self.INCH_TO_MM

        # Build ports
        ports = {}
        for port_config in config.ports:
            port = self._build_port(port_config, scale)
            ports[port.name] = port

        # Build attachment points
        attachment_points = {}
        for ap_config in config.attachment_points:
            ap = self._build_attachment_point(ap_config, scale)
            attachment_points[ap.name] = ap

        return Fitting(
            nps=config.nps,
            shape=shape,
            ports=ports,
            attachment_points=attachment_points,
        )

    def create_factory(
        self,
        config: FittingConfig,
        base_path: str | Path | None = None,
    ) -> Callable[[str], Fitting]:
        """
        Create a factory function for a fitting.

        Returns a function that takes NPS and returns a Fitting,
        matching the signature of make_weld_neck_flange(), etc.

        Note: The returned fitting uses the same geometry regardless of NPS.
        This is useful for fittings where the STEP file is specific to one size.

        Args:
            config: FittingConfig with STEP file path and port definitions
            base_path: Base directory for resolving relative STEP file paths

        Returns:
            Factory function: (nps: str) -> Fitting
        """
        # Load the fitting once
        fitting = self.load_fitting(config, base_path)

        def factory(nps: str) -> Fitting:
            # Return the same fitting, updating NPS if different
            if nps != fitting.nps:
                return Fitting(
                    nps=nps,
                    shape=fitting.shape,
                    ports=fitting.ports,
                    attachment_points=fitting.attachment_points,
                )
            return fitting

        return factory

    def _load_step_shape(self, path: Path) -> cq.Shape:
        """Load STEP file and return shape."""
        if not path.exists():
            raise FileNotFoundError(f"STEP file not found: {path}")
        result = cq.importers.importStep(str(path))
        shape = result.val()
        if not isinstance(shape, cq.Shape):
            raise ValueError(f"Expected Shape, got {type(shape)}")
        return shape

    def _build_port(self, config: PortConfig, scale: float) -> Port:
        """Build Port object from config."""
        # Scale position
        pos = (
            config.position[0] * scale,
            config.position[1] * scale,
            config.position[2] * scale,
        )

        # Compute transformation matrix
        transform = self._compute_transform_matrix(pos, config.orientation)

        return Port(name=config.name, transform=transform)

    def _build_attachment_point(
        self, config, scale: float
    ) -> AttachmentPoint:
        """Build AttachmentPoint from config."""
        # Scale position
        pos = (
            config.position[0] * scale,
            config.position[1] * scale,
            config.position[2] * scale,
        )

        # Normalize the normal vector
        normal = _normalize(config.normal)

        return AttachmentPoint(
            name=config.name,
            position=pos,
            normal=normal,
            attachment_type=config.attachment_type,
            port_name=config.port_name,
        )

    def _compute_transform_matrix(
        self,
        position: tuple[float, float, float],
        orientation: OrientationConfig,
    ) -> np.ndarray:
        """
        Compute 4x4 transformation matrix from position and orientation.

        The matrix defines the port's coordinate frame:
        - Origin at position
        - Z-axis from orientation (points outward)
        - X-axis computed or from orientation (rotational reference)
        - Y-axis completes the right-hand system

        Args:
            position: Port origin (x, y, z)
            orientation: Orientation specification

        Returns:
            4x4 numpy transformation matrix
        """
        T = np.eye(4)

        # Set translation
        T[0, 3] = position[0]
        T[1, 3] = position[1]
        T[2, 3] = position[2]

        # Determine orientation
        if orientation.z_direction:
            z_axis = _normalize(orientation.z_direction)

            if orientation.x_direction:
                # Use provided X-axis
                x_axis = _normalize(orientation.x_direction)
                # Make sure X is orthogonal to Z
                y_axis = _normalize(_cross(z_axis, x_axis))
                x_axis = _normalize(_cross(y_axis, z_axis))
            else:
                # Compute X-axis (try to make it "up" when possible)
                world_up = (0.0, 0.0, 1.0)
                if abs(_dot(z_axis, world_up)) > 0.99:
                    # Z is nearly vertical, use world X instead
                    world_up = (1.0, 0.0, 0.0)
                y_axis = _normalize(_cross(z_axis, world_up))
                x_axis = _normalize(_cross(y_axis, z_axis))

            # Build rotation matrix
            T[0, 0] = x_axis[0]
            T[1, 0] = x_axis[1]
            T[2, 0] = x_axis[2]

            y_axis = _normalize(_cross(z_axis, x_axis))
            T[0, 1] = y_axis[0]
            T[1, 1] = y_axis[1]
            T[2, 1] = y_axis[2]

            T[0, 2] = z_axis[0]
            T[1, 2] = z_axis[1]
            T[2, 2] = z_axis[2]

        elif orientation.euler:
            # Apply ZYX Euler angles (in degrees)
            rz, ry, rx = orientation.euler
            R = rotation_matrix_z(rz) @ rotation_matrix_y(ry) @ rotation_matrix_x(rx)
            T[:3, :3] = R[:3, :3]

        return T
