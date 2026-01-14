#!/usr/bin/env python3
"""
Pipe Fittings with Proper Coordinate Frames

This module implements a port-based mating system for pipe fittings using
4x4 homogeneous transformation matrices (like robotics).

================================================================================
STANDARD PORT COORDINATE CONVENTION
================================================================================

Each fitting has "ports" defining connection points. Each port has a coordinate
frame specified by a 4x4 transformation matrix relative to the fitting's origin.

Port Coordinate Frame:
- Origin: At the mating point (for socket weld: socket bottom where pipe tip goes)
- Z-axis: Points OUTWARD toward where the connecting pipe/fitting's body extends
- X-axis: Defines rotational orientation (typically "up" = world +Z when possible)
- Y-axis: Completes right-hand coordinate system

Mating Rule:
When mating port A (on fitting FA) to port B (on fitting FB):
1. Port B's Z-axis must OPPOSE port A's Z-axis (they face each other)
2. Port B's origin aligns with port A's origin (with optional gap along Z)
3. Port B's X-axis aligns with port A's X-axis (same rotational orientation)

Example - Pipe going EAST connecting to elbow inlet:
- Pipe end port: Z points EAST (toward elbow)
- Elbow inlet port: Z points WEST (toward pipe)
- Result: Z's are opposite (EAST vs WEST), ports face each other ✓

Flow Direction Convention:
- Inlet ports: Z points toward incoming flow (opposite to flow direction)
- Outlet ports: Z points toward outgoing flow (same as flow direction)
- When connected: flow is continuous because Z's are opposite

================================================================================

References:
- https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
- https://cad.onshape.com/help/Content/mateconnector.htm (Onshape mate connectors)
- https://cadactive.com/blog/2018/05/23/standardizing-spec-driven-fittings-3/
"""

import math
from dataclasses import dataclass, field

import cadquery as cq
import numpy as np

# Import butt weld fittings
from .butt_weld_elbow import ASME_B169_ELBOW90, make_butt_weld_elbow_90
from .butt_weld_reducer import ASME_B169_REDUCER, make_butt_weld_reducer
from .butt_weld_tee import ASME_B169_TEE, make_butt_weld_tee
from .socket_weld_elbow import (
    ASME_B1611_ELBOW45_CLASS3000,
    ASME_B1611_ELBOW90_CLASS3000,
)
from .socket_weld_elbow import (
    make_socket_weld_elbow_45 as make_socket_weld_elbow_45_shape,
)
from .socket_weld_elbow import (
    make_socket_weld_elbow_90 as make_socket_weld_elbow_90_shape,
)

# Import threaded fittings
from .threaded_elbow import (
    ASME_B1611_THREADED_ELBOW45_CLASS3000,
    ASME_B1611_THREADED_ELBOW90_CLASS3000,
    NPT_THREAD_ENGAGEMENT,
    make_threaded_elbow_45,
    make_threaded_elbow_90,
)
from .threaded_tee import ASME_B1611_THREADED_TEE_CLASS3000, make_threaded_tee

# Import dimension data and shape creation function
from .weld_neck_flange import (
    ASME_B165_CLASS300_WN,
    BoltHoleOrientation,
    make_weld_neck_flange_class300,
)

# =============================================================================
# TRANSFORMATION MATRIX UTILITIES
# =============================================================================


def identity_matrix() -> np.ndarray:
    """Return 4x4 identity matrix."""
    return np.eye(4)


def point_along_direction(point: tuple[float, float, float], direction: tuple[float, float, float], distance: float) -> tuple[float, float, float]:
    """
    Compute a point offset from the given point along a direction.

    Args:
        point: Starting point (x, y, z)
        direction: Direction vector (dx, dy, dz)
        distance: Distance to move along the direction

    Returns:
        New point (x + distance*dx, y + distance*dy, z + distance*dz)
    """
    return (point[0] + distance * direction[0], point[1] + distance * direction[1], point[2] + distance * direction[2])


def translation_matrix(x: float, y: float, z: float) -> np.ndarray:
    """Create 4x4 translation matrix."""
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def rotation_matrix_x(angle_deg: float) -> np.ndarray:
    """Create 4x4 rotation matrix around X axis."""
    angle = math.radians(angle_deg)
    c, s = math.cos(angle), math.sin(angle)
    R = np.eye(4)
    R[1, 1] = c
    R[1, 2] = -s
    R[2, 1] = s
    R[2, 2] = c
    return R


def rotation_matrix_y(angle_deg: float) -> np.ndarray:
    """Create 4x4 rotation matrix around Y axis."""
    angle = math.radians(angle_deg)
    c, s = math.cos(angle), math.sin(angle)
    R = np.eye(4)
    R[0, 0] = c
    R[0, 2] = s
    R[2, 0] = -s
    R[2, 2] = c
    return R


def rotation_matrix_z(angle_deg: float) -> np.ndarray:
    """Create 4x4 rotation matrix around Z axis."""
    angle = math.radians(angle_deg)
    c, s = math.cos(angle), math.sin(angle)
    R = np.eye(4)
    R[0, 0] = c
    R[0, 1] = -s
    R[1, 0] = s
    R[1, 1] = c
    return R


def rotation_from_axis_angle(axis: tuple[float, float, float], angle_deg: float) -> np.ndarray:
    """Create 4x4 rotation matrix from axis-angle representation."""
    angle = math.radians(angle_deg)
    x, y, z = axis
    # Normalize axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-10:
        return np.eye(4)
    x, y, z = x / norm, y / norm, z / norm

    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c

    R = np.eye(4)
    R[0, 0] = t * x * x + c
    R[0, 1] = t * x * y - z * s
    R[0, 2] = t * x * z + y * s
    R[1, 0] = t * x * y + z * s
    R[1, 1] = t * y * y + c
    R[1, 2] = t * y * z - x * s
    R[2, 0] = t * x * z - y * s
    R[2, 1] = t * y * z + x * s
    R[2, 2] = t * z * z + c
    return R


def matrix_from_rotation_translation(R: np.ndarray, t: tuple[float, float, float]) -> np.ndarray:
    """
    Create 4x4 homogeneous matrix from 3x3 rotation and translation.
    """
    T = np.eye(4)
    T[0:3, 0:3] = R[0:3, 0:3] if R.shape[0] == 4 else R
    T[0, 3] = t[0]
    T[1, 3] = t[1]
    T[2, 3] = t[2]
    return T


def get_position(T: np.ndarray) -> tuple[float, float, float]:
    """Extract position from transformation matrix."""
    return (T[0, 3], T[1, 3], T[2, 3])


def get_x_axis(T: np.ndarray) -> tuple[float, float, float]:
    """Extract X-axis direction from transformation matrix."""
    return (T[0, 0], T[1, 0], T[2, 0])


def get_y_axis(T: np.ndarray) -> tuple[float, float, float]:
    """Extract Y-axis direction from transformation matrix."""
    return (T[0, 1], T[1, 1], T[2, 1])


def get_z_axis(T: np.ndarray) -> tuple[float, float, float]:
    """Extract Z-axis direction from transformation matrix."""
    return (T[0, 2], T[1, 2], T[2, 2])


def get_rotation_3x3(T: np.ndarray) -> np.ndarray:
    """Extract 3x3 rotation matrix from 4x4 transformation."""
    return T[0:3, 0:3]


def matrix_to_cadquery_location(T: np.ndarray) -> tuple[cq.Location, tuple[float, float, float]]:
    """Convert 4x4 transformation matrix to CadQuery Location."""
    # Extract rotation and translation
    pos = get_position(T)
    R = get_rotation_3x3(T)

    # Convert rotation matrix to axis-angle or quaternion
    # CadQuery Location can take a transformation matrix directly via occ

    # For now, use a simpler approach: apply translation and rotation separately
    # This may have gimbal lock issues but works for axis-aligned cases

    # Calculate Euler angles (ZYX convention)
    # This is a simplified extraction
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0

    rx_deg = math.degrees(rx)
    ry_deg = math.degrees(ry)
    rz_deg = math.degrees(rz)

    # Create location using CadQuery's built-in methods
    loc = cq.Location(cq.Vector(*pos))

    return loc, (rx_deg, ry_deg, rz_deg)


def apply_transform_to_shape(shape: cq.Shape, T: np.ndarray) -> cq.Shape:
    """
    Apply 4x4 transformation matrix to a CadQuery shape.

    Uses CadQuery's native Matrix class for direct matrix application,
    avoiding Euler angle decomposition issues that can cause incorrect
    orientations for compound rotations.

    This follows the robotics convention (Denavit-Hartenberg) where
    transformations are applied directly as 4x4 homogeneous matrices.
    """
    # Convert numpy 4x4 matrix to CadQuery Matrix format (3x4)
    # CadQuery expects rows 0-2, all 4 columns
    matrix_list = [
        [float(T[0, 0]), float(T[0, 1]), float(T[0, 2]), float(T[0, 3])],
        [float(T[1, 0]), float(T[1, 1]), float(T[1, 2]), float(T[1, 3])],
        [float(T[2, 0]), float(T[2, 1]), float(T[2, 2]), float(T[2, 3])],
    ]
    cq_matrix = cq.Matrix(matrix_list)
    return shape.transformGeometry(cq_matrix)


# =============================================================================
# UP-VECTOR COMPUTATION HELPERS
# =============================================================================


def compute_initial_up_vector(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Compute the initial up-vector for a given direction.

    For horizontal pipes, "up" is world +Z.
    For vertical pipes, we choose a default horizontal "up".

    Args:
        direction: Direction the pipe travels (unit vector)

    Returns:
        Up-vector perpendicular to direction
    """
    dx, dy, dz = direction

    # Check if direction is vertical (up or down)
    if abs(dz) > 0.99:
        # Pipe is vertical, use north (+Y) as "up" reference
        return (0.0, 1.0, 0.0)
    else:
        # Pipe is horizontal, use world +Z as "up"
        return (0.0, 0.0, 1.0)


def compute_up_vector_after_elbow(incoming_direction: tuple[float, float, float], outgoing_direction: tuple[float, float, float], current_up: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Compute the new up-vector after traversing an elbow.

    The elbow turns the pipe from incoming_direction to outgoing_direction.
    The up-vector needs to be rotated consistently.

    The elbow's axis of rotation is perpendicular to both pipe directions.
    The up-vector rotates around this axis along with the pipe direction.

    Args:
        incoming_direction: Direction before the elbow
        outgoing_direction: Direction after the elbow
        current_up: Current up-vector before the elbow

    Returns:
        New up-vector after the elbow
    """
    inc = np.array(incoming_direction)
    out = np.array(outgoing_direction)
    up = np.array(current_up)

    # The elbow rotation axis is perpendicular to both pipe directions
    # axis = incoming × outgoing (points in the direction of the elbow's Z)
    axis = np.cross(inc, out)
    axis_norm = np.linalg.norm(axis)

    if axis_norm < 1e-6:
        # Directions are parallel (shouldn't happen for 90° elbow)
        return current_up

    axis = axis / axis_norm

    # The angle of rotation is the angle between incoming and outgoing (90° for standard elbow)
    dot = np.clip(np.dot(inc, out), -1.0, 1.0)
    angle = np.arccos(dot)  # In radians

    # Rodrigues' rotation formula: rotate 'up' around 'axis' by 'angle'
    # v_rot = v*cos(θ) + (axis × v)*sin(θ) + axis*(axis·v)*(1-cos(θ))
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)

    up_rotated = up * cos_a + np.cross(axis, up) * sin_a + axis * np.dot(axis, up) * (1 - cos_a)

    # Normalize
    up_rotated = up_rotated / np.linalg.norm(up_rotated)

    return (float(up_rotated[0]), float(up_rotated[1]), float(up_rotated[2]))


def compute_up_vector_for_branch(run_direction: tuple[float, float, float], branch_direction: tuple[float, float, float], current_up: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Compute the up-vector for a branch coming off a tee or cross.

    The branch goes in a different direction than the run.
    We need to compute an up-vector for the branch that maintains
    consistency with the fitting's orientation.

    Args:
        run_direction: Direction of the run (continues straight)
        branch_direction: Direction of the branch
        current_up: Up-vector for the run

    Returns:
        Up-vector for the branch
    """
    run = np.array(run_direction)
    branch = np.array(branch_direction)
    up = np.array(current_up)

    # The branch up-vector should be perpendicular to the branch direction
    # and as aligned as possible with the current up-vector

    # Project current_up onto the plane perpendicular to branch
    # up_proj = up - (up · branch) * branch
    up_proj = up - np.dot(up, branch) * branch
    proj_norm = np.linalg.norm(up_proj)

    if proj_norm < 1e-6:
        # Up is parallel to branch, need alternative
        # Use the cross product of branch with run to get a perpendicular
        alt_up = np.cross(branch, run)
        alt_norm = np.linalg.norm(alt_up)
        if alt_norm > 1e-6:
            return tuple(alt_up / alt_norm)
        else:
            # Branch and run are parallel (shouldn't happen for tee)
            # Use world +Z or +Y as fallback
            if abs(branch[2]) < 0.99:
                return (0.0, 0.0, 1.0)
            else:
                return (0.0, 1.0, 0.0)

    up_proj = up_proj / proj_norm
    return (float(up_proj[0]), float(up_proj[1]), float(up_proj[2]))


# =============================================================================
# COORDINATE FRAME VISUALIZATION
# =============================================================================


def create_coordinate_frame_marker(
    transform: np.ndarray,
    scale: float = 25.0,
    arrow_radius: float = 2.0,
    cone_height: float = 8.0,
    cone_radius: float = 4.0,
) -> list[cq.Shape]:
    """
    Create coordinate frame marker arrows at a port location.

    Creates three arrows representing the X, Y, Z axes:
    - X-axis: Red (conventionally)
    - Y-axis: Green (conventionally)
    - Z-axis: Blue (conventionally)

    Note: STEP files don't support colors, but the arrows are distinguishable
    by their directions. X is the first axis, Y second, Z third.

    Args:
        transform: 4x4 transformation matrix for the port
        scale: Length of the arrow shafts in mm
        arrow_radius: Radius of the arrow shaft cylinders
        cone_height: Height of the arrow tip cones
        cone_radius: Radius of the arrow tip cones

    Returns:
        List of CadQuery shapes (3 arrows for X, Y, Z axes)
    """
    position = get_position(transform)
    x_axis = get_x_axis(transform)
    y_axis = get_y_axis(transform)
    z_axis = get_z_axis(transform)

    arrows = []

    for axis_dir in [x_axis, y_axis, z_axis]:
        # Normalize direction
        dx, dy, dz = axis_dir
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-6:
            continue
        dx, dy, dz = dx / length, dy / length, dz / length

        # Create arrow shaft (cylinder from origin along direction)
        shaft = cq.Solid.makeCylinder(
            arrow_radius,
            scale,
            cq.Vector(*position),
            cq.Vector(dx, dy, dz),
        )

        # Create arrow tip (cone at end of shaft)
        cone_start = (
            position[0] + scale * dx,
            position[1] + scale * dy,
            position[2] + scale * dz,
        )
        cone = cq.Solid.makeCone(
            cone_radius,
            0.0,
            cone_height,
            cq.Vector(*cone_start),
            cq.Vector(dx, dy, dz),
        )

        # Combine shaft and tip
        arrow = shaft.fuse(cone)
        arrows.append(arrow)

    return arrows


def create_port_markers_for_fitting(
    fitting: "Fitting",
    world_transform: np.ndarray,
    scale: float = 25.0,
) -> list[cq.Shape]:
    """
    Create coordinate frame markers for all ports on a fitting.

    Args:
        fitting: The Fitting object with ports
        world_transform: World transform of the fitting
        scale: Size of the coordinate frame markers

    Returns:
        List of CadQuery shapes (arrows for each port)
    """
    markers = []

    for port in fitting.ports.values():
        # Get port's world transform
        port_world = world_transform @ port.transform
        # Create markers for this port
        port_markers = create_coordinate_frame_marker(port_world, scale=scale)
        markers.extend(port_markers)

    return markers


# =============================================================================
# PORT DEFINITION
# =============================================================================


@dataclass
class Port:
    """
    A connection point on a fitting.

    The port has a local coordinate frame where:
    - Origin is at the connection point (face of the fitting)
    - Z-axis points OUTWARD (direction a connected pipe would go)
    - X-axis is typically "up" for orientation reference
    """

    name: str
    # 4x4 transformation matrix defining port's coordinate frame
    # relative to the fitting's origin
    transform: np.ndarray

    @property
    def position(self) -> tuple[float, float, float]:
        """Port position in fitting's local coordinates."""
        return get_position(self.transform)

    @property
    def direction(self) -> tuple[float, float, float]:
        """Direction the port faces (Z-axis of port frame)."""
        return get_z_axis(self.transform)


# =============================================================================
# ATTACHMENT POINT DEFINITION
# =============================================================================


@dataclass
class AttachmentPoint:
    """
    An attachment point on a fitting for BOM balloons and weld symbols.

    Unlike ports (which are connection points for mating), attachment points
    define where annotation symbols should attach visually. These are on the
    OUTER SURFACE of the fitting body.

    For socket weld fittings:
    - Weld attachments are at the socket OPENING (outer face where fillet weld is visible)
    - Points are at orthogonal positions around the pipe OD circle (N, S, E, W)

    For butt weld fittings:
    - Weld attachments are at the weld seam location on the outer surface

    Attributes:
        name: Identifier (e.g., "inlet_weld_north", "outlet_weld_up")
        position: 3D position in fitting local coordinates (on outer surface)
        normal: Direction pointing outward from the fitting surface
        attachment_type: "weld", "bom", or "label"
        port_name: Which port this attachment is associated with (e.g., "inlet", "outlet")
    """

    name: str
    position: tuple[float, float, float]  # Local coordinates on fitting surface
    normal: tuple[float, float, float]  # Outward direction from surface
    attachment_type: str = "weld"  # "weld", "bom", or "label"
    port_name: str | None = None  # Associated port name


# =============================================================================
# FITTING BASE CLASS
# =============================================================================


@dataclass
class Fitting:
    """Base class for pipe fittings with ports and attachment points."""

    nps: str
    shape: cq.Shape | None = None
    ports: dict[str, Port] = field(default_factory=dict)
    attachment_points: dict[str, AttachmentPoint] = field(default_factory=dict)

    def get_port(self, name: str) -> Port:
        """Get a port by name."""
        if name not in self.ports:
            raise ValueError(f"Port '{name}' not found. Available: {list(self.ports.keys())}")
        return self.ports[name]

    def get_attachment_point(self, name: str) -> AttachmentPoint:
        """Get an attachment point by name."""
        if name not in self.attachment_points:
            raise ValueError(f"Attachment point '{name}' not found. Available: {list(self.attachment_points.keys())}")
        return self.attachment_points[name]

    def get_attachment_points_for_port(self, port_name: str) -> list[AttachmentPoint]:
        """Get all attachment points associated with a given port."""
        return [ap for ap in self.attachment_points.values() if ap.port_name == port_name]

    def get_weld_attachment_points(self) -> list[AttachmentPoint]:
        """Get all weld-type attachment points."""
        return [ap for ap in self.attachment_points.values() if ap.attachment_type == "weld"]

    def get_shape_at_transform(self, T: np.ndarray) -> cq.Shape:
        """Get the fitting's shape transformed to a world position."""
        if self.shape is None:
            raise ValueError("Fitting has no shape")
        return apply_transform_to_shape(self.shape, T)

    def get_attachment_point_world_position(self, point_name: str, fitting_transform: np.ndarray) -> tuple[float, float, float]:
        """
        Get the world position of an attachment point given the fitting's world transform.

        Args:
            point_name: Name of the attachment point
            fitting_transform: 4x4 world transform of the fitting

        Returns:
            (x, y, z) world position of the attachment point
        """
        ap = self.get_attachment_point(point_name)

        # Create a homogeneous position vector
        local_pos = np.array([ap.position[0], ap.position[1], ap.position[2], 1.0])

        # Transform to world coordinates
        world_pos = fitting_transform @ local_pos

        return (world_pos[0], world_pos[1], world_pos[2])

    def get_attachment_point_world_normal(self, point_name: str, fitting_transform: np.ndarray) -> tuple[float, float, float]:
        """
        Get the world normal of an attachment point given the fitting's world transform.

        Args:
            point_name: Name of the attachment point
            fitting_transform: 4x4 world transform of the fitting

        Returns:
            (nx, ny, nz) world normal direction of the attachment point
        """
        ap = self.get_attachment_point(point_name)

        # Extract rotation part of transform (3x3)
        R = fitting_transform[:3, :3]

        # Transform normal (direction vector, no translation)
        local_normal = np.array([ap.normal[0], ap.normal[1], ap.normal[2]])
        world_normal = R @ local_normal

        # Normalize
        length = np.linalg.norm(world_normal)
        if length > 0:
            world_normal = world_normal / length

        return (world_normal[0], world_normal[1], world_normal[2])


# =============================================================================
# WELD NECK FLANGE
# =============================================================================


def make_weld_neck_flange(
    nps: str,
    units: str = "mm",
    bolt_hole_orientation: BoltHoleOrientation = "single_hole",
) -> Fitting:
    """
    Create a weld neck flange with defined ports.

    Coordinate system:
    - Flange face is at Z=0, facing +Z
    - Raised face extends to Z=rf_height
    - Hub extends in -Z direction to Z=-hub_length
    - Pipe connects at the weld end (bottom of hub)

    Ports (using standard convention - Z points OUTWARD toward connecting fitting):
    - "face": At top of raised face (Z=rf_height), Z points +Z (toward mating flange)
    - "weld": At weld end (Z=-hub_length), Z points -Z (toward connecting pipe)

    Args:
        nps: Nominal pipe size (e.g., "2", "4", "6")
        units: "inch" or "mm" - output units
        bolt_hole_orientation: Bolt hole orientation relative to the XZ reference plane.
            - "single_hole": One bolt hole on the reference plane (at angle 0°)
            - "two_hole": Two bolt holes symmetric about the reference plane
            Default is "two_hole" which is the most common orientation.

    When mating:
    - Face port connects to another flange's face port (Z's oppose)
    - Weld port connects to pipe's start port (Z's oppose)
    """
    dims = ASME_B165_CLASS300_WN.get(nps)
    if dims is None:
        raise ValueError(f"No flange dimensions for NPS {nps}")

    scale = 25.4 if units == "mm" else 1.0

    # Get dimensions needed for ports and attachment points
    hub_length = dims.hub_length * scale
    rf_height = dims.raised_face_height * scale
    hub_weld_od = dims.hub_od_at_weld * scale
    r_hub_weld = hub_weld_od / 2

    # Create the flange shape using the shared function
    flange_shape = make_weld_neck_flange_class300(nps, units, bolt_hole_orientation)

    # Define ports
    ports = {}

    # Face port: at Z = rf_height (top of raised face), Z-axis points +Z
    face_transform = translation_matrix(0, 0, rf_height)
    ports["face"] = Port("face", face_transform)

    # Weld port: at Z = -hub_length (bottom of hub)
    # Z-axis points -Z (outward, direction pipe goes)
    # We need to rotate 180° around X to flip Z-axis
    weld_transform = translation_matrix(0, 0, -hub_length) @ rotation_matrix_x(180)
    ports["weld"] = Port("weld", weld_transform)

    # ==========================================================================
    # WELD ATTACHMENT POINTS
    # ==========================================================================
    # Butt weld is at the hub weld end (Z = -hub_length).
    # Attachment points are at orthogonal positions around the hub OD at the weld.
    # The hub tapers, so we use r_hub_weld (the radius at the weld end).

    attachment_points = {}

    # Weld attachment points around the hub end (a circle in the XY plane at Z=-hub_length)
    weld_attachment_positions = [
        # (name suffix, X offset, Y offset, normal_x, normal_y)
        ("east", r_hub_weld, 0.0, 1.0, 0.0),  # +X
        ("west", -r_hub_weld, 0.0, -1.0, 0.0),  # -X
        ("north", 0.0, r_hub_weld, 0.0, 1.0),  # +Y
        ("south", 0.0, -r_hub_weld, 0.0, -1.0),  # -Y
    ]

    for suffix, x_off, y_off, nx, ny in weld_attachment_positions:
        name = f"weld_{suffix}"
        attachment_points[name] = AttachmentPoint(name=name, position=(x_off, y_off, -hub_length), normal=(nx, ny, 0.0), attachment_type="weld", port_name="weld")

    return Fitting(nps=nps, shape=flange_shape, ports=ports, attachment_points=attachment_points)


# =============================================================================
# SOCKET WELD ELBOW
# =============================================================================


def make_socket_weld_elbow(nps: str, units: str = "mm") -> Fitting:  # noqa: ARG001
    """
    Create a socket weld 90° elbow with defined ports.

    Coordinate system:
    - Elbow center (where the two legs meet) is at origin
    - Leg 1 extends along +X axis (inlet port at +X end)
    - Leg 2 extends along +Y axis (outlet port at +Y end)

    Ports:
    - "inlet": Socket end on +X axis, pipe enters from +X direction
    - "outlet": Socket end on +Y axis, pipe exits in +Y direction

    Port Z-axes point OUTWARD (direction pipe goes when connected).
    """
    dims = ASME_B1611_ELBOW90_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No elbow dimensions for NPS {nps}")

    # Dimensions (already in mm in our data)
    A = dims.center_to_end
    B = (dims.socket_bore_max + dims.socket_bore_min) / 2.0
    J = dims.socket_depth

    r_socket_inner = B / 2.0

    # Get elbow shape from socket_weld_elbow module
    elbow = make_socket_weld_elbow_90_shape(nps)

    # Define ports using standard convention:
    # - Port origin: at the mating point (socket bottom + typical gap allowance)
    # - Port Z-axis: points OUTWARD toward where the connecting pipe's body extends
    # - Port X-axis: defines rotational orientation (typically "up" = +Z world)
    #
    # For mating: port Z-axes should be OPPOSITE (facing each other)
    #
    # Elbow geometry:
    # - Leg 1 along +X: socket opening at (A+J, 0, 0), socket bottom at (A, 0, 0)
    # - Leg 2 along +Y: socket opening at (0, A+J, 0), socket bottom at (0, A, 0)
    #
    # Port positions at socket bottom (where pipe tip ends up):
    # - Inlet: at (A, 0, 0) - pipe body extends in +X direction from here
    # - Outlet: at (0, A, 0) - pipe body extends in +Y direction from here

    ports = {}

    # Inlet port: at socket bottom of leg 1
    # Z-axis points +X (outward, toward where incoming pipe body extends)
    # X-axis points +Z (up, for rotational reference)
    inlet_transform = translation_matrix(A, 0, 0) @ rotation_matrix_y(90)
    ports["inlet"] = Port("inlet", inlet_transform)

    # Outlet port: at socket bottom of leg 2
    # Z-axis points +Y (outward, toward where outgoing pipe body extends)
    # X-axis points +Z (up, for rotational reference)
    outlet_transform = translation_matrix(0, A, 0) @ rotation_matrix_x(-90)
    ports["outlet"] = Port("outlet", outlet_transform)

    # ==========================================================================
    # WELD ATTACHMENT POINTS
    # ==========================================================================
    # Socket weld fillet welds are at the SOCKET OPENING - the OUTER face
    # where the pipe enters the socket. This is where the welder applies
    # the fillet weld to seal the joint.
    #
    # Elbow geometry:
    # - Body extends from center (0,0,0) to x=A for leg 1, y=A for leg 2
    # - Socket counterbore extends OUTWARD from body surface:
    #   - Inlet: from x=A to x=A+J (socket depth J extending in +X)
    #   - Outlet: from y=A to y=A+J (socket depth J extending in +Y)
    # - Socket BOTTOM (innermost, where pipe tip stops) is at x=A / y=A
    # - Socket OPENING (outermost, where fillet weld is visible) is at x=A+J / y=A+J
    #
    # Port positions are at socket BOTTOM (x=A, y=A) per convention.
    # Weld attachment points must be at socket OPENING (x=A+J, y=A+J).

    attachment_points = {}

    # Socket opening position (where fillet weld goes)
    socket_opening_x = A + J  # For inlet (on +X axis)
    socket_opening_y = A + J  # For outlet (on +Y axis)

    # Inlet socket opening is at (A+J, 0, 0) - a circle in the YZ plane
    # Points at orthogonal positions around the socket ID (where pipe enters and fillet weld is applied)
    inlet_attachment_positions = [
        # (name suffix, Y offset, Z offset, normal_y, normal_z)
        ("up", 0.0, r_socket_inner, 0.0, 1.0),  # +Z
        ("down", 0.0, -r_socket_inner, 0.0, -1.0),  # -Z
        ("north", r_socket_inner, 0.0, 1.0, 0.0),  # +Y
        ("south", -r_socket_inner, 0.0, -1.0, 0.0),  # -Y
    ]

    for suffix, y_off, z_off, ny, nz in inlet_attachment_positions:
        name = f"inlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(name=name, position=(socket_opening_x, y_off, z_off), normal=(0.0, ny, nz), attachment_type="weld", port_name="inlet")

    # Outlet socket opening is at (0, A+J, 0) - a circle in the XZ plane
    # Points at orthogonal positions around the socket ID (where pipe enters and fillet weld is applied)
    outlet_attachment_positions = [
        # (name suffix, X offset, Z offset, normal_x, normal_z)
        ("up", 0.0, r_socket_inner, 0.0, 1.0),  # +Z
        ("down", 0.0, -r_socket_inner, 0.0, -1.0),  # -Z
        ("east", r_socket_inner, 0.0, 1.0, 0.0),  # +X
        ("west", -r_socket_inner, 0.0, -1.0, 0.0),  # -X
    ]

    for suffix, x_off, z_off, nx, nz in outlet_attachment_positions:
        name = f"outlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(name=name, position=(x_off, socket_opening_y, z_off), normal=(nx, 0.0, nz), attachment_type="weld", port_name="outlet")

    return Fitting(nps=nps, shape=elbow, ports=ports, attachment_points=attachment_points)


def make_socket_weld_elbow_45(nps: str, units: str = "mm") -> Fitting:  # noqa: ARG001
    """
    Create a socket weld 45° elbow with defined ports.

    Coordinate system:
    - Elbow center (where the two legs meet) is at origin
    - Leg 1 extends along +X axis (inlet port at +X end)
    - Leg 2 extends at 45° toward +Y (outlet port)

    Ports:
    - "inlet": Socket end on +X axis, pipe enters from +X direction
    - "outlet": Socket end at 45°, pipe exits at 45° from inlet

    Port Z-axes point OUTWARD (direction pipe goes when connected).
    """
    dims = ASME_B1611_ELBOW45_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No 45° elbow dimensions for NPS {nps}")

    # Dimensions (already in mm in our data)
    A = dims.center_to_end
    B = (dims.socket_bore_max + dims.socket_bore_min) / 2.0
    J = dims.socket_depth

    r_socket_inner = B / 2.0

    # Get elbow shape from socket_weld_elbow module
    elbow = make_socket_weld_elbow_45_shape(nps)

    # Direction for 45° leg (45° turn from -X toward +Y)
    # Outlet direction: rotate -X by 45° counterclockwise = 135° from +X
    angle_rad = math.radians(135)
    dir_x = math.cos(angle_rad)  # ~-0.707
    dir_y = math.sin(angle_rad)  # ~0.707

    # Outlet port position (at end of leg 2)
    outlet_x = A * dir_x
    outlet_y = A * dir_y

    # Define ports
    ports = {}

    # Inlet port: at socket bottom of leg 1
    # Z-axis points +X (outward, toward where incoming pipe body extends)
    # X-axis points +Z (up, for rotational reference)
    inlet_transform = translation_matrix(A, 0, 0) @ rotation_matrix_y(90)
    ports["inlet"] = Port("inlet", inlet_transform)

    # Outlet port: at socket bottom of leg 2 (at 45° position)
    # Z-axis points in outlet direction (outward)
    # Need rotation that aligns Z with (dir_x, dir_y, 0)
    # Rotate around Z by 135° from +X, then rotate around Y by 90° to point Z outward
    outlet_transform = translation_matrix(outlet_x, outlet_y, 0) @ rotation_matrix_z(135) @ rotation_matrix_y(90)
    ports["outlet"] = Port("outlet", outlet_transform)

    # ==========================================================================
    # WELD ATTACHMENT POINTS
    # ==========================================================================
    attachment_points = {}

    # Socket opening positions (where fillet weld goes)
    socket_opening_inlet_x = A + J
    socket_opening_outlet_x = outlet_x + J * dir_x
    socket_opening_outlet_y = outlet_y + J * dir_y

    # Inlet socket opening is at (A+J, 0, 0) - a circle in the YZ plane
    inlet_attachment_positions = [
        # (name suffix, Y offset, Z offset, normal_y, normal_z)
        ("up", 0.0, r_socket_inner, 0.0, 1.0),
        ("down", 0.0, -r_socket_inner, 0.0, -1.0),
        ("north", r_socket_inner, 0.0, 1.0, 0.0),
        ("south", -r_socket_inner, 0.0, -1.0, 0.0),
    ]

    for suffix, y_off, z_off, ny, nz in inlet_attachment_positions:
        name = f"inlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(name=name, position=(socket_opening_inlet_x, y_off, z_off), normal=(0.0, ny, nz), attachment_type="weld", port_name="inlet")

    # Outlet socket opening - perpendicular to outlet direction
    # The outlet direction is (dir_x, dir_y, 0), so perpendicular directions are:
    # - Up/down: (0, 0, ±1)
    # - Sideways in XY plane: (-dir_y, dir_x, 0) and (dir_y, -dir_x, 0)
    perp_x = -dir_y  # perpendicular in XY plane
    perp_y = dir_x

    outlet_attachment_positions = [
        # (name suffix, local offset perpendicular to outlet dir, normal direction)
        ("up", (0.0, 0.0, r_socket_inner), (0.0, 0.0, 1.0)),
        ("down", (0.0, 0.0, -r_socket_inner), (0.0, 0.0, -1.0)),
        ("left", (perp_x * r_socket_inner, perp_y * r_socket_inner, 0.0), (perp_x, perp_y, 0.0)),
        ("right", (-perp_x * r_socket_inner, -perp_y * r_socket_inner, 0.0), (-perp_x, -perp_y, 0.0)),
    ]

    for suffix, offset, normal in outlet_attachment_positions:
        name = f"outlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(
            name=name, position=(socket_opening_outlet_x + offset[0], socket_opening_outlet_y + offset[1], offset[2]), normal=normal, attachment_type="weld", port_name="outlet"
        )

    return Fitting(nps=nps, shape=elbow, ports=ports, attachment_points=attachment_points)


# =============================================================================
# BUTT WELD ELBOW
# =============================================================================


def make_butt_weld_elbow_fitting(nps: str, units: str = "mm") -> Fitting:  # noqa: ARG001
    """
    Create a butt weld 90° elbow with defined ports.

    Coordinate system (original butt weld geometry):
    - Arc centered at origin, from (A, 0, 0) to (0, A, 0)
    - At (A, 0, 0): tangent is +Y, so pipe connects along Y axis
    - At (0, A, 0): tangent is -X, so pipe connects along X axis

    Ports:
    - "inlet": At (A, 0, 0), pipe extends in -Y direction
    - "outlet": At (0, A, 0), pipe extends in -X direction

    Port Z-axes point OUTWARD (direction pipe goes when connected).
    """
    dims = ASME_B169_ELBOW90.get(nps)
    if dims is None:
        raise ValueError(f"No butt weld elbow dimensions for NPS {nps}")

    # Dimensions (in mm)
    A = dims.center_to_end_lr  # Center to end (long radius) = bend radius
    r_outer = dims.od / 2.0  # Outer radius for attachment points

    # Get original butt weld elbow shape (arc centered at origin)
    elbow_shape = make_butt_weld_elbow_90(nps, radius_type="LR")

    # Define ports matching the actual geometry face orientations:
    #
    # Original butt weld elbow geometry:
    # - Arc goes counterclockwise from (A, 0, 0) to (0, A, 0)
    # - At (A, 0, 0): tangent is +Y, face perpendicular to Y
    # - At (0, A, 0): tangent is -X, face perpendicular to X
    #
    # Port Z points toward where connecting pipe body extends:
    # - Inlet at (A, 0, 0): pipe body extends in -Y direction (opposite arc direction)
    # - Outlet at (0, A, 0): pipe body extends in -X direction (along arc tangent)

    ports = {}

    # Inlet port: at (A, 0, 0), pipe connects along Y axis
    # Z-axis points -Y (opposite of arc tangent direction at inlet)
    # The arc tangent at inlet is +Y (flow direction into elbow)
    # Port Z points opposite to flow = toward incoming pipe body
    # rotation_matrix_x(90) rotates Z from +Z to -Y
    inlet_transform = translation_matrix(A, 0, 0) @ rotation_matrix_x(90)
    ports["inlet"] = Port("inlet", inlet_transform)

    # Outlet port: at (0, A, 0), pipe connects along X axis
    # Z-axis points -X (toward where outgoing pipe body extends)
    # rotation_matrix_y(-90) rotates Z from +Z to -X
    outlet_transform = translation_matrix(0, A, 0) @ rotation_matrix_y(-90)
    ports["outlet"] = Port("outlet", outlet_transform)

    # ==========================================================================
    # WELD ATTACHMENT POINTS
    # ==========================================================================
    # Butt weld joints are at the weld end faces.
    # Faces are perpendicular to the arc tangent at each end.

    attachment_points = {}

    # Inlet weld attachment points at (A, 0, 0), face in XZ plane (perpendicular to Y)
    inlet_attachment_positions = [
        ("up", 0.0, r_outer, 0.0, 1.0),  # +Z
        ("down", 0.0, -r_outer, 0.0, -1.0),  # -Z
        ("east", r_outer, 0.0, 1.0, 0.0),  # +X
        ("west", -r_outer, 0.0, -1.0, 0.0),  # -X
    ]

    for suffix, x_off, z_off, nx, nz in inlet_attachment_positions:
        name = f"inlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(
            name=name,
            position=(A + x_off, 0, z_off),
            normal=(nx, 0.0, nz),
            attachment_type="weld",
            port_name="inlet",
        )

    # Outlet weld attachment points at (0, A, 0), face in YZ plane (perpendicular to X)
    outlet_attachment_positions = [
        ("up", 0.0, r_outer, 0.0, 1.0),  # +Z
        ("down", 0.0, -r_outer, 0.0, -1.0),  # -Z
        ("north", r_outer, 0.0, 1.0, 0.0),  # +Y
        ("south", -r_outer, 0.0, -1.0, 0.0),  # -Y
    ]

    for suffix, y_off, z_off, ny, nz in outlet_attachment_positions:
        name = f"outlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(
            name=name,
            position=(0, A + y_off, z_off),
            normal=(0.0, ny, nz),
            attachment_type="weld",
            port_name="outlet",
        )

    return Fitting(nps=nps, shape=elbow_shape, ports=ports, attachment_points=attachment_points)


# =============================================================================
# BUTT WELD TEE
# =============================================================================


def make_butt_weld_tee_fitting(nps: str, units: str = "mm") -> Fitting:  # noqa: ARG001
    """
    Create a butt weld equal tee with defined ports.

    Coordinate system:
    - Tee center (where run and branch intersect) is at origin
    - Run extends along X axis (inlet at -X, run-out at +X)
    - Branch extends along +Y axis

    Ports:
    - "inlet": End of run at -X, pipe enters from -X direction
    - "run": End of run at +X, pipe exits in +X direction
    - "branch": End of branch at +Y, pipe exits in +Y direction

    Port Z-axes point OUTWARD (direction pipe goes when connected).
    """
    dims = ASME_B169_TEE.get(nps)
    if dims is None:
        raise ValueError(f"No butt weld tee dimensions for NPS {nps}")

    # Dimensions (in mm)
    C_run = dims.center_to_end_run  # Center to end (run)
    M_branch = dims.center_to_end_branch  # Center to end (branch)
    r_outer = dims.od / 2.0  # Outer radius for attachment points

    # Get tee shape
    tee_shape = make_butt_weld_tee(nps)

    # Define ports using standard convention:
    # - Port origin: at the weld end (where pipe connects)
    # - Port Z-axis: points OUTWARD toward where the connecting pipe's body extends
    # - Port X-axis: defines rotational orientation (typically "up" = +Z world)
    #
    # Tee geometry:
    # - Run along X axis: from (-C_run, 0, 0) to (+C_run, 0, 0)
    # - Branch along +Y axis: from (0, 0, 0) to (0, M_branch, 0)

    ports = {}

    # Inlet port: at -X end of run
    # Z-axis points -X (outward, toward where incoming pipe body extends)
    # X-axis points +Z (up, for rotational reference)
    # rotation_matrix_y(-90) rotates +Z to point in -X direction
    inlet_transform = translation_matrix(-C_run, 0, 0) @ rotation_matrix_y(-90)
    ports["inlet"] = Port("inlet", inlet_transform)

    # Run port: at +X end of run
    # Z-axis points +X (outward, toward where outgoing pipe body extends)
    # X-axis points +Z (up, for rotational reference)
    run_transform = translation_matrix(C_run, 0, 0) @ rotation_matrix_y(90)
    ports["run"] = Port("run", run_transform)

    # Branch port: at +Y end of branch
    # Z-axis points +Y (outward, toward where branch pipe body extends)
    # X-axis points +Z (up, for rotational reference)
    branch_transform = translation_matrix(0, M_branch, 0) @ rotation_matrix_x(-90)
    ports["branch"] = Port("branch", branch_transform)

    # ==========================================================================
    # WELD ATTACHMENT POINTS
    # ==========================================================================
    # Butt weld attachment points are at the weld seam location on outer surface.
    # Each port has 4 attachment points around the OD circle.

    attachment_points = {}

    # Inlet weld attachment points at (-C_run, 0, 0) - a circle in the YZ plane
    inlet_attachment_positions = [
        # (name suffix, Y offset, Z offset, normal_y, normal_z)
        ("up", 0.0, r_outer, 0.0, 1.0),  # +Z
        ("down", 0.0, -r_outer, 0.0, -1.0),  # -Z
        ("north", r_outer, 0.0, 1.0, 0.0),  # +Y
        ("south", -r_outer, 0.0, -1.0, 0.0),  # -Y
    ]

    for suffix, y_off, z_off, ny, nz in inlet_attachment_positions:
        name = f"inlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(name=name, position=(-C_run, y_off, z_off), normal=(0.0, ny, nz), attachment_type="weld", port_name="inlet")

    # Run weld attachment points at (+C_run, 0, 0) - a circle in the YZ plane
    run_attachment_positions = [
        # (name suffix, Y offset, Z offset, normal_y, normal_z)
        ("up", 0.0, r_outer, 0.0, 1.0),  # +Z
        ("down", 0.0, -r_outer, 0.0, -1.0),  # -Z
        ("north", r_outer, 0.0, 1.0, 0.0),  # +Y
        ("south", -r_outer, 0.0, -1.0, 0.0),  # -Y
    ]

    for suffix, y_off, z_off, ny, nz in run_attachment_positions:
        name = f"run_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(name=name, position=(C_run, y_off, z_off), normal=(0.0, ny, nz), attachment_type="weld", port_name="run")

    # Branch weld attachment points at (0, M_branch, 0) - a circle in the XZ plane
    branch_attachment_positions = [
        # (name suffix, X offset, Z offset, normal_x, normal_z)
        ("up", 0.0, r_outer, 0.0, 1.0),  # +Z
        ("down", 0.0, -r_outer, 0.0, -1.0),  # -Z
        ("east", r_outer, 0.0, 1.0, 0.0),  # +X
        ("west", -r_outer, 0.0, -1.0, 0.0),  # -X
    ]

    for suffix, x_off, z_off, nx, nz in branch_attachment_positions:
        name = f"branch_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(name=name, position=(x_off, M_branch, z_off), normal=(nx, 0.0, nz), attachment_type="weld", port_name="branch")

    return Fitting(nps=nps, shape=tee_shape, ports=ports, attachment_points=attachment_points)


# =============================================================================
# BUTT WELD REDUCER
# =============================================================================


def make_butt_weld_reducer_fitting(
    large_nps: str,
    small_nps: str,
    units: str = "mm",  # noqa: ARG001
    reversed: bool = False,
) -> Fitting:
    """
    Create a butt weld concentric reducer with defined ports.

    Coordinate system:
    - Reducer centerline is along the X-axis
    - Large end at X=0
    - Small end at X=H (overall length)

    Port configuration depends on `reversed` parameter:

    Normal mode (reversed=False, for contraction - pipe getting smaller):
    - "inlet": At large end (X=0), Z-axis points -X
    - "outlet": At small end (X=H), Z-axis points +X

    Reversed mode (reversed=True, for expansion - pipe getting larger):
    - "inlet": At small end (X=H), Z-axis points +X
    - "outlet": At large end (X=0), Z-axis points -X

    This is the same physical fitting, just with ports swapped for expansion use.
    Port Z-axes always point OUTWARD (direction pipe goes when connected).

    Args:
        large_nps: Nominal pipe size of large end (e.g., "4", "6")
        small_nps: Nominal pipe size of small end (e.g., "2", "4")
        units: Units for dimensions (currently only "mm" supported)
        reversed: If True, swap inlet/outlet for expansion (pipe getting larger)

    Returns:
        Fitting object with shape, ports, and attachment points

    Raises:
        ValueError: If size combination is not available
    """
    dims = ASME_B169_REDUCER.get((large_nps, small_nps))
    if dims is None:
        raise ValueError(f"No butt weld reducer dimensions for {large_nps} x {small_nps}")

    # Dimensions (in mm)
    H = dims.length_h  # Overall length
    r_large_outer = dims.large_od / 2.0  # Large end outer radius
    r_small_outer = dims.small_od / 2.0  # Small end outer radius

    # Get reducer shape
    reducer_shape = make_butt_weld_reducer(large_nps, small_nps)

    # Define ports using standard convention:
    # - Port origin: at the weld end (where pipe connects)
    # - Port Z-axis: points OUTWARD toward where the connecting pipe's body extends
    # - Port X-axis: defines rotational orientation (typically "up" = +Z world)
    #
    # Reducer geometry:
    # - Large end at X=0, small end at X=H
    # - Both weld faces are perpendicular to X-axis

    ports = {}
    attachment_points = {}

    if not reversed:
        # Normal mode (contraction): inlet at large end, outlet at small end
        # Inlet port: at large end (X=0)
        # Z-axis points -X (outward, toward where incoming large pipe body extends)
        inlet_transform = translation_matrix(0, 0, 0) @ rotation_matrix_y(-90)
        ports["inlet"] = Port("inlet", inlet_transform)

        # Outlet port: at small end (X=H)
        # Z-axis points +X (outward, toward where outgoing small pipe body extends)
        outlet_transform = translation_matrix(H, 0, 0) @ rotation_matrix_y(90)
        ports["outlet"] = Port("outlet", outlet_transform)

        # Inlet attachment points at large end (X=0)
        inlet_radius = r_large_outer
        inlet_x = 0
        # Outlet attachment points at small end (X=H)
        outlet_radius = r_small_outer
        outlet_x = H
    else:
        # Reversed mode (expansion): inlet at small end, outlet at large end
        # Inlet port: at small end (X=H)
        # Z-axis points +X (outward, toward where incoming small pipe body extends)
        inlet_transform = translation_matrix(H, 0, 0) @ rotation_matrix_y(90)
        ports["inlet"] = Port("inlet", inlet_transform)

        # Outlet port: at large end (X=0)
        # Z-axis points -X (outward, toward where outgoing large pipe body extends)
        outlet_transform = translation_matrix(0, 0, 0) @ rotation_matrix_y(-90)
        ports["outlet"] = Port("outlet", outlet_transform)

        # Inlet attachment points at small end (X=H)
        inlet_radius = r_small_outer
        inlet_x = H
        # Outlet attachment points at large end (X=0)
        outlet_radius = r_large_outer
        outlet_x = 0

    # ==========================================================================
    # WELD ATTACHMENT POINTS
    # ==========================================================================
    # Butt weld attachment points are at the weld seam location on outer surface.
    # Each port has 4 attachment points around the OD circle in the YZ plane.

    # Inlet weld attachment points
    inlet_attachment_positions = [
        # (name suffix, Y offset, Z offset, normal_y, normal_z)
        ("up", 0.0, inlet_radius, 0.0, 1.0),  # +Z
        ("down", 0.0, -inlet_radius, 0.0, -1.0),  # -Z
        ("north", inlet_radius, 0.0, 1.0, 0.0),  # +Y
        ("south", -inlet_radius, 0.0, -1.0, 0.0),  # -Y
    ]

    for suffix, y_off, z_off, ny, nz in inlet_attachment_positions:
        name = f"inlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(
            name=name,
            position=(inlet_x, y_off, z_off),
            normal=(0.0, ny, nz),
            attachment_type="weld",
            port_name="inlet",
        )

    # Outlet weld attachment points
    outlet_attachment_positions = [
        # (name suffix, Y offset, Z offset, normal_y, normal_z)
        ("up", 0.0, outlet_radius, 0.0, 1.0),  # +Z
        ("down", 0.0, -outlet_radius, 0.0, -1.0),  # -Z
        ("north", outlet_radius, 0.0, 1.0, 0.0),  # +Y
        ("south", -outlet_radius, 0.0, -1.0, 0.0),  # -Y
    ]

    for suffix, y_off, z_off, ny, nz in outlet_attachment_positions:
        name = f"outlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(
            name=name,
            position=(outlet_x, y_off, z_off),
            normal=(0.0, ny, nz),
            attachment_type="weld",
            port_name="outlet",
        )

    # Store large_nps as the primary NPS (for BOM lookup)
    # The description field will be used to encode both sizes
    return Fitting(nps=large_nps, shape=reducer_shape, ports=ports, attachment_points=attachment_points)


# =============================================================================
# NPT THREADED ELBOW
# =============================================================================


def make_threaded_elbow_90_fitting(nps: str, units: str = "mm") -> Fitting:  # noqa: ARG001
    """
    Create a 90° NPT threaded elbow with defined ports.

    Coordinate system:
    - Elbow center (where the two legs meet) is at origin
    - Leg 1 extends along +X axis (inlet port at +X end)
    - Leg 2 extends along +Y axis (outlet port at +Y end)

    Ports:
    - "inlet": Thread end on +X axis, pipe enters from +X direction
    - "outlet": Thread end on +Y axis, pipe exits in +Y direction

    Port Z-axes point OUTWARD (direction pipe goes when connected).
    Note: No weld attachment points for threaded fittings.
    """
    dims = ASME_B1611_THREADED_ELBOW90_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No threaded 90° elbow dimensions for NPS {nps}")

    thread_spec = NPT_THREAD_ENGAGEMENT.get(nps)
    if thread_spec is None:
        raise ValueError(f"No NPT thread spec for NPS {nps}")

    # Dimensions
    A = dims.center_to_end  # Center to thread end
    thread_engagement = thread_spec.engagement  # How far pipe screws in

    # Get elbow shape with threads
    elbow_shape = make_threaded_elbow_90(nps, include_threads=True)

    # Port position: where pipe tip ends up after threading in
    # Port is at center_to_end minus thread engagement
    port_offset = A - thread_engagement

    ports = {}

    # Inlet port: at end of leg 1 (accounting for thread engagement)
    # Z-axis points +X (outward, toward where incoming pipe body extends)
    inlet_transform = translation_matrix(port_offset, 0, 0) @ rotation_matrix_y(90)
    ports["inlet"] = Port("inlet", inlet_transform)

    # Outlet port: at end of leg 2 (accounting for thread engagement)
    # Z-axis points +Y (outward, toward where outgoing pipe body extends)
    outlet_transform = translation_matrix(0, port_offset, 0) @ rotation_matrix_x(-90)
    ports["outlet"] = Port("outlet", outlet_transform)

    # No attachment points for threaded fittings (no welds)
    attachment_points: dict[str, AttachmentPoint] = {}

    return Fitting(nps=nps, shape=elbow_shape, ports=ports, attachment_points=attachment_points)


def make_threaded_elbow_45_fitting(nps: str, units: str = "mm") -> Fitting:  # noqa: ARG001
    """
    Create a 45° NPT threaded elbow with defined ports.

    Coordinate system:
    - Elbow center is at origin
    - Leg 1 extends along +X axis (inlet port at +X end)
    - Leg 2 extends at 45° toward +Y (outlet port)

    Ports:
    - "inlet": Thread end on +X axis, pipe enters from +X direction
    - "outlet": Thread end at 45°, pipe exits at 45° from inlet

    Port Z-axes point OUTWARD (direction pipe goes when connected).
    Note: No weld attachment points for threaded fittings.
    """
    dims = ASME_B1611_THREADED_ELBOW45_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No threaded 45° elbow dimensions for NPS {nps}")

    thread_spec = NPT_THREAD_ENGAGEMENT.get(nps)
    if thread_spec is None:
        raise ValueError(f"No NPT thread spec for NPS {nps}")

    # Dimensions
    A = dims.center_to_end  # Center to thread end
    thread_engagement = thread_spec.engagement

    # Get elbow shape with threads
    elbow_shape = make_threaded_elbow_45(nps, include_threads=True)

    # Port position accounting for thread engagement
    port_offset = A - thread_engagement

    # Direction for 45° leg
    angle_rad = math.radians(135)
    dir_x = math.cos(angle_rad)
    dir_y = math.sin(angle_rad)

    # Outlet port position
    outlet_x = port_offset * dir_x
    outlet_y = port_offset * dir_y

    ports = {}

    # Inlet port
    inlet_transform = translation_matrix(port_offset, 0, 0) @ rotation_matrix_y(90)
    ports["inlet"] = Port("inlet", inlet_transform)

    # Outlet port at 45°
    outlet_transform = translation_matrix(outlet_x, outlet_y, 0) @ rotation_matrix_z(135) @ rotation_matrix_y(90)
    ports["outlet"] = Port("outlet", outlet_transform)

    # No attachment points for threaded fittings
    attachment_points: dict[str, AttachmentPoint] = {}

    return Fitting(nps=nps, shape=elbow_shape, ports=ports, attachment_points=attachment_points)


# =============================================================================
# NPT THREADED TEE
# =============================================================================


def make_threaded_tee_fitting(nps: str, units: str = "mm") -> Fitting:  # noqa: ARG001
    """
    Create an NPT threaded equal tee with defined ports.

    Coordinate system:
    - Tee center (where run and branch intersect) is at origin
    - Run extends along X axis (inlet at -X, run-out at +X)
    - Branch extends along +Y axis

    Ports:
    - "inlet": End of run at -X, pipe enters from -X direction
    - "run": End of run at +X, pipe exits in +X direction
    - "branch": End of branch at +Y, pipe exits in +Y direction

    Port Z-axes point OUTWARD (direction pipe goes when connected).
    Note: No weld attachment points for threaded fittings.
    """
    dims = ASME_B1611_THREADED_TEE_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No threaded tee dimensions for NPS {nps}")

    thread_spec = NPT_THREAD_ENGAGEMENT.get(nps)
    if thread_spec is None:
        raise ValueError(f"No NPT thread spec for NPS {nps}")

    # Dimensions
    A = dims.center_to_end  # Center to thread end
    thread_engagement = thread_spec.engagement

    # Get tee shape with threads
    tee_shape = make_threaded_tee(nps, include_threads=True)

    # Port position accounting for thread engagement
    port_offset = A - thread_engagement

    ports = {}

    # Inlet port: at -X end of run
    inlet_transform = translation_matrix(-port_offset, 0, 0) @ rotation_matrix_y(-90)
    ports["inlet"] = Port("inlet", inlet_transform)

    # Run port: at +X end of run
    run_transform = translation_matrix(port_offset, 0, 0) @ rotation_matrix_y(90)
    ports["run"] = Port("run", run_transform)

    # Branch port: at +Y end of branch
    branch_transform = translation_matrix(0, port_offset, 0) @ rotation_matrix_x(-90)
    ports["branch"] = Port("branch", branch_transform)

    # No attachment points for threaded fittings (no welds)
    attachment_points: dict[str, AttachmentPoint] = {}

    return Fitting(nps=nps, shape=tee_shape, ports=ports, attachment_points=attachment_points)


# =============================================================================
# PIPE SEGMENT
# =============================================================================


@dataclass
class PipeSpec:
    """Pipe dimensions."""

    nps: str
    od_mm: float
    wall_mm: float
    id_mm: float


PIPE_SPECS: dict[str, PipeSpec] = {
    "1/2": PipeSpec("1/2", 21.3, 3.73, 13.84),
    "3/4": PipeSpec("3/4", 26.7, 3.91, 18.88),
    "1": PipeSpec("1", 33.4, 4.55, 24.30),
    "1-1/4": PipeSpec("1-1/4", 42.2, 4.85, 32.50),
    "1-1/2": PipeSpec("1-1/2", 48.3, 5.08, 38.14),
    "2": PipeSpec("2", 60.3, 5.54, 49.22),
    "2-1/2": PipeSpec("2-1/2", 73.0, 7.01, 58.98),
    "3": PipeSpec("3", 88.9, 7.62, 73.66),
    "4": PipeSpec("4", 114.3, 8.56, 97.18),
}


def make_pipe(nps: str, length_mm: float) -> Fitting:
    """
    Create a pipe segment with defined ports.

    Coordinate system:
    - Pipe centerline is along Z-axis
    - Pipe body from Z=0 to Z=length
    - Flow direction: from start (Z=0) toward end (Z=length)

    Ports (using standard convention - Z points OUTWARD toward connecting fitting):
    - "start": At Z=0, Z-axis points -Z (toward fitting that connects at start)
    - "end": At Z=length, Z-axis points +Z (toward fitting that connects at end)

    When mating:
    - Pipe start's Z (-Z) should oppose previous fitting's outlet Z
    - Pipe end's Z (+Z) should oppose next fitting's inlet Z
    """
    spec = PIPE_SPECS.get(nps)
    if spec is None:
        raise ValueError(f"No pipe spec for NPS {nps}")

    r_outer = spec.od_mm / 2
    r_inner = spec.id_mm / 2

    outer = cq.Solid.makeCylinder(r_outer, length_mm, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
    inner = cq.Solid.makeCylinder(r_inner, length_mm, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
    pipe = outer.cut(inner)

    ports = {}

    # Start port at Z=0
    # Z-axis points -Z (outward toward the fitting connecting at this end)
    # X-axis points +X (for rotational reference)
    start_transform = rotation_matrix_x(180)  # Flip so Z points -Z
    ports["start"] = Port("start", start_transform)

    # End port at Z=length
    # Z-axis points +Z (outward toward the fitting connecting at this end)
    # X-axis points +X (for rotational reference)
    end_transform = translation_matrix(0, 0, length_mm)
    ports["end"] = Port("end", end_transform)

    return Fitting(nps=nps, shape=pipe, ports=ports)


def make_threaded_pipe(nps: str, length_mm: float, thread_end: str = "both") -> Fitting:
    """
    Create a threaded pipe segment with defined ports.

    Coordinate system:
    - Pipe centerline is along Z-axis
    - Pipe body from Z=0 to Z=length
    - Flow direction: from start (Z=0) toward end (Z=length)

    Ports (using standard convention - Z points OUTWARD toward connecting fitting):
    - "start": At Z=0, Z-axis points -Z (toward fitting that connects at start)
    - "end": At Z=length, Z-axis points +Z (toward fitting that connects at end)

    For threaded connections, the pipe screws into the fitting socket.
    The port position accounts for thread engagement depth.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "1", "2")
        length_mm: Total length of pipe in mm
        thread_end: Which ends to thread ("both", "inlet", "outlet", "none")

    Returns:
        Fitting object with shape and ports
    """
    from typing import Literal, cast  # noqa: I001

    from pypegen.pipe_generator import make_threaded_pipe as make_threaded_pipe_shape

    # Cast thread_end to the expected Literal type
    thread_end_literal = cast(Literal["none", "inlet", "outlet", "both"], thread_end)

    # Create the threaded pipe shape
    pipe_shape = make_threaded_pipe_shape(
        nps=nps,
        length=length_mm,
        thread_end=thread_end_literal,
        include_threads=True,
    )

    ports = {}

    # For threaded pipe, the port position accounts for thread engagement
    # Port origin is where the pipe tip will end up after screwing in

    # Start port at Z=0
    # Z-axis points -Z (outward toward the fitting connecting at this end)
    # The port is at the pipe end (Z=0), the pipe body extends in +Z
    start_transform = rotation_matrix_x(180)  # Flip so Z points -Z
    ports["start"] = Port("start", start_transform)

    # End port at Z=length
    # Z-axis points +Z (outward toward the fitting connecting at this end)
    end_transform = translation_matrix(0, 0, length_mm)
    ports["end"] = Port("end", end_transform)

    return Fitting(nps=nps, shape=pipe_shape, ports=ports)


# =============================================================================
# MATING CALCULATION
# =============================================================================


def compute_mate_transform(port_a: Port, port_b: Port, fitting_a_transform: np.ndarray, gap: float = 0.0) -> np.ndarray:
    """
    Compute the world transform for fitting B such that port B mates with port A.

    Port A is on fitting A (already positioned in world).
    Port B is on fitting B (we're computing where to place it).

    The ports mate when:
    - Their Z-axes point toward each other (opposite directions)
    - Their origins are aligned (with optional gap between them)

    Args:
        port_a: Port on fitting A
        port_b: Port on fitting B
        fitting_a_transform: World transform of fitting A
        gap: Gap distance between ports (along connection axis)

    Returns:
        World transform for fitting B
    """
    # Port A's world transform
    port_a_world = fitting_a_transform @ port_a.transform

    # We need fitting B positioned such that port B faces port A
    # Port B's Z-axis should point opposite to port A's Z-axis

    # Get port A's world position and Z-axis
    pos_a = get_position(port_a_world)
    z_a = get_z_axis(port_a_world)

    # Port B should have its Z-axis pointing in -z_a direction
    # And be positioned at pos_a + gap * z_a (gap along port A's outward direction)

    # The mating position (where port B origin should be)
    mate_pos = point_along_direction(pos_a, z_a, gap)

    # Target direction for port B's Z-axis in world coords
    # Port B should face opposite to port A (they face each other)
    target_z = (-z_a[0], -z_a[1], -z_a[2])

    # Compute rotation that aligns world +Z with target_z
    # This rotation will be the rotation part of desired_port_b_world
    # (The Z-axis of the resulting transform should be target_z)

    # Dot product of (0,0,1) with target_z is just target_z[2]
    dot = target_z[2]

    if abs(dot - 1.0) < 1e-6:
        # target_z is +Z, no rotation needed
        R_desired = np.eye(4)
    elif abs(dot + 1.0) < 1e-6:
        # target_z is -Z, rotate 180° around X axis
        R_desired = rotation_matrix_x(180)
    else:
        # General case: rotation axis is (0,0,1) × target_z = (-target_z[1], target_z[0], 0)
        axis = (-target_z[1], target_z[0], 0)
        angle = math.degrees(math.acos(max(-1, min(1, dot))))
        R_desired = rotation_from_axis_angle(axis, angle)

    # Now we need to account for port B's offset from fitting B's origin
    # fitting_b_world @ port_b.transform = desired_port_b_world

    # desired_port_b_world has position = mate_pos, and rotation R_desired
    # (R_desired has its Z-axis pointing in target_z direction)
    # We need: fitting_b_world = desired_port_b_world @ inv(port_b.transform)

    # Construct desired port B world transform
    desired_port_b = translation_matrix(*mate_pos) @ R_desired

    # Fitting B world = desired_port_b @ inv(port_b.transform)
    port_b_inv = np.linalg.inv(port_b.transform)
    fitting_b_world = desired_port_b @ port_b_inv

    return fitting_b_world


# =============================================================================
# GASKET
# =============================================================================


def make_gasket(
    nps: str,
    gasket_type: str = "flat_ring",
    thickness_mm: float | None = None,
    pressure_class: int = 300,
) -> Fitting:
    """
    Create a gasket fitting with ports on both faces.

    The gasket sits between two mating flange faces. It has two ports:
    - "face_a": Top face at Z=+thickness/2, Z-axis points +Z
    - "face_b": Bottom face at Z=-thickness/2, Z-axis points -Z

    Coordinate system:
    - Gasket centered at origin
    - Lies in XY plane
    - Thickness extends from Z=-t/2 to Z=+t/2

    When mated between flanges:
    - Flange 1's face port mates to gasket's face_a
    - Gasket's face_b mates to Flange 2's face port

    Args:
        nps: Nominal pipe size (e.g., "2", "4", "6")
        gasket_type: "flat_ring" (ASME B16.21) or "spiral_wound" (ASME B16.20)
        thickness_mm: Override thickness in mm (uses standard if None)
        pressure_class: 150 or 300 (for flat_ring gaskets)

    Returns:
        Fitting object with shape and ports
    """
    if gasket_type == "flat_ring":
        from .gasket_flat_ring import ASME_B1621_CLASS150_FLAT, ASME_B1621_CLASS300_FLAT, make_flat_ring_gasket

        # Get dimensions for thickness
        if pressure_class == 150:
            dims_table = ASME_B1621_CLASS150_FLAT
        else:
            dims_table = ASME_B1621_CLASS300_FLAT

        dims = dims_table.get(nps)
        if dims is None:
            raise ValueError(f"No flat ring gasket dimensions for NPS {nps} Class {pressure_class}")

        thickness = thickness_mm if thickness_mm is not None else dims.thickness
        shape = make_flat_ring_gasket(nps, thickness_mm=thickness, pressure_class=pressure_class)
        description = "FLAT RING GASKET"

    elif gasket_type == "spiral_wound":
        from .gasket_spiral_wound import ASME_B1620_CLASS300_SPIRAL, make_spiral_wound_gasket

        dims = ASME_B1620_CLASS300_SPIRAL.get(nps)
        if dims is None:
            raise ValueError(f"No spiral wound gasket dimensions for NPS {nps}")

        thickness = thickness_mm if thickness_mm is not None else dims.compressed_thickness
        shape = make_spiral_wound_gasket(nps, thickness_mm=thickness)
        description = "SPIRAL WOUND GASKET"

    else:
        raise ValueError(f"Unknown gasket type: {gasket_type}. Use 'flat_ring' or 'spiral_wound'.")

    # Define ports on both faces
    ports = {}

    # Face A port: at +Z face (top), Z-axis points +Z
    face_a_transform = translation_matrix(0, 0, thickness / 2)
    ports["face_a"] = Port("face_a", face_a_transform)

    # Face B port: at -Z face (bottom), Z-axis points -Z
    # Rotate 180° around X to flip Z-axis
    face_b_transform = translation_matrix(0, 0, -thickness / 2) @ rotation_matrix_x(180)
    ports["face_b"] = Port("face_b", face_b_transform)

    # No weld attachment points for gaskets (they are bolted, not welded)
    attachment_points: dict[str, AttachmentPoint] = {}

    # Store gasket metadata
    fitting = Fitting(nps=nps, shape=shape, ports=ports, attachment_points=attachment_points)

    return fitting


def get_gasket_thickness(
    nps: str,
    gasket_type: str = "flat_ring",
    pressure_class: int = 300,
) -> float:
    """
    Get the standard thickness for a gasket.

    Args:
        nps: Nominal pipe size
        gasket_type: "flat_ring" or "spiral_wound"
        pressure_class: 150 or 300 (for flat_ring)

    Returns:
        Thickness in mm
    """
    if gasket_type == "flat_ring":
        from .gasket_flat_ring import ASME_B1621_CLASS150_FLAT, ASME_B1621_CLASS300_FLAT

        if pressure_class == 150:
            dims = ASME_B1621_CLASS150_FLAT.get(nps)
        else:
            dims = ASME_B1621_CLASS300_FLAT.get(nps)

        if dims is None:
            raise ValueError(f"No flat ring gasket dimensions for NPS {nps}")
        return dims.thickness

    elif gasket_type == "spiral_wound":
        from .gasket_spiral_wound import ASME_B1620_CLASS300_SPIRAL

        dims = ASME_B1620_CLASS300_SPIRAL.get(nps)
        if dims is None:
            raise ValueError(f"No spiral wound gasket dimensions for NPS {nps}")
        return dims.compressed_thickness

    else:
        raise ValueError(f"Unknown gasket type: {gasket_type}")


def mate_flanges_with_gasket(
    flange_a: Fitting,
    flange_a_transform: np.ndarray,
    flange_b: Fitting,
    gasket_type: str = "flat_ring",
    pressure_class: int = 300,
) -> tuple[np.ndarray, Fitting, np.ndarray]:
    """
    Mate two flanges face-to-face with automatic gasket insertion.

    This is a convenience function that:
    1. Creates a gasket of the appropriate type and size
    2. Positions the gasket against flange_a's face
    3. Positions flange_b against the other side of the gasket

    The gasket thickness becomes the effective gap between flange faces.

    Args:
        flange_a: First flange (already positioned in world)
        flange_a_transform: 4x4 world transform of flange_a
        flange_b: Second flange (to be positioned)
        gasket_type: "flat_ring" or "spiral_wound"
        pressure_class: 150 or 300 (for flat_ring gaskets)

    Returns:
        Tuple of:
        - flange_b_transform: 4x4 world transform for flange_b
        - gasket: The Gasket Fitting object
        - gasket_transform: 4x4 world transform for the gasket

    Example:
        flange_a = make_weld_neck_flange("2")
        flange_b = make_weld_neck_flange("2")

        flange_a_transform = identity_matrix()
        flange_b_transform, gasket, gasket_transform = mate_flanges_with_gasket(
            flange_a, flange_a_transform, flange_b, gasket_type="flat_ring"
        )

        # Now assemble the shapes
        parts = [
            apply_transform_to_shape(flange_a.shape, flange_a_transform),
            apply_transform_to_shape(gasket.shape, gasket_transform),
            apply_transform_to_shape(flange_b.shape, flange_b_transform),
        ]
        assembly = cq.Compound.makeCompound(parts)
    """
    # Use the NPS from flange_a (assume both flanges are same size)
    nps = flange_a.nps

    # Create the gasket
    gasket = make_gasket(nps, gasket_type=gasket_type, pressure_class=pressure_class)

    # Step 1: Position gasket's face_a to mate with flange_a's face port
    # Gasket sits directly against the flange face (no gap)
    gasket_transform = compute_mate_transform(
        port_a=flange_a.get_port("face"),
        port_b=gasket.get_port("face_a"),
        fitting_a_transform=flange_a_transform,
        gap=0,
    )

    # Step 2: Position flange_b's face port to mate with gasket's face_b
    # Flange_b sits directly against the other side of the gasket
    flange_b_transform = compute_mate_transform(
        port_a=gasket.get_port("face_b"),
        port_b=flange_b.get_port("face"),
        fitting_a_transform=gasket_transform,
        gap=0,
    )

    return (flange_b_transform, gasket, gasket_transform)


# =============================================================================
# TEST
# =============================================================================


def test_fittings():
    """Test the fitting creation and port definitions."""
    print("Testing Pipe Fittings with Coordinate Frames")
    print("=" * 50)

    # Create a flange
    flange = make_weld_neck_flange("2", units="mm")
    print("\nFlange NPS 2:")
    print(f"  Face port position: {flange.get_port('face').position}")
    print(f"  Face port direction: {flange.get_port('face').direction}")
    print(f"  Weld port position: {flange.get_port('weld').position}")
    print(f"  Weld port direction: {flange.get_port('weld').direction}")

    # Create an elbow
    elbow = make_socket_weld_elbow("2", units="mm")
    print("\nElbow NPS 2:")
    print(f"  Inlet port position: {elbow.get_port('inlet').position}")
    print(f"  Inlet port direction: {elbow.get_port('inlet').direction}")
    print(f"  Outlet port position: {elbow.get_port('outlet').position}")
    print(f"  Outlet port direction: {elbow.get_port('outlet').direction}")

    # Create a pipe
    pipe = make_pipe("2", 500)
    print("\nPipe NPS 2, 500mm:")
    print(f"  Start port position: {pipe.get_port('start').position}")
    print(f"  Start port direction: {pipe.get_port('start').direction}")
    print(f"  End port position: {pipe.get_port('end').position}")
    print(f"  End port direction: {pipe.get_port('end').direction}")

    # Export individual fittings
    if flange.shape:
        cq.exporters.export(flange.shape, "step/test_flange_ports.step")
    if elbow.shape:
        cq.exporters.export(elbow.shape, "step/test_elbow_ports.step")
    if pipe.shape:
        cq.exporters.export(pipe.shape, "step/test_pipe_ports.step")

    print("\nExported test fittings to step/")

    # Test mating: connect pipe to flange weld port
    print("\nTesting mating calculation:")

    flange_world = identity_matrix()  # Flange at origin
    gap = 1.6  # 1/16" = 1.6mm

    # Mate pipe's start port to flange's weld port
    pipe_world = compute_mate_transform(port_a=flange.get_port("weld"), port_b=pipe.get_port("start"), fitting_a_transform=flange_world, gap=gap)

    print(f"  Pipe world position: {get_position(pipe_world)}")
    # Pipe starts at flange weld port + gap along weld direction (-Z)
    # weld port at z=-hub_length, weld direction is -Z
    # pipe position = weld_pos + gap * (-Z) = (0, 0, -hub_length - gap)
    expected_z = flange.get_port("weld").position[2] - gap  # -76.2 - 1.6 = -77.8
    print(f"  Expected: near (0, 0, {expected_z:.1f})")

    # Create assembly
    flange_shape = flange.get_shape_at_transform(flange_world)
    pipe_shape = pipe.get_shape_at_transform(pipe_world)

    assembly = cq.Compound.makeCompound([flange_shape, pipe_shape])
    cq.exporters.export(assembly, "step/test_flange_pipe_assembly.step")
    print("\nExported test_flange_pipe_assembly.step")


if __name__ == "__main__":
    test_fittings()
