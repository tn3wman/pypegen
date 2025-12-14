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

from .socket_weld_elbow import ASME_B1611_ELBOW90_CLASS3000, add_socket_counterbores

# Import dimension data and shape creation function
from .weld_neck_flange import ASME_B165_CLASS300_WN, make_weld_neck_flange_class300

# =============================================================================
# TRANSFORMATION MATRIX UTILITIES
# =============================================================================

def identity_matrix() -> np.ndarray:
    """Return 4x4 identity matrix."""
    return np.eye(4)


def point_along_direction(
    point: tuple[float, float, float],
    direction: tuple[float, float, float],
    distance: float
) -> tuple[float, float, float]:
    """
    Compute a point offset from the given point along a direction.

    Args:
        point: Starting point (x, y, z)
        direction: Direction vector (dx, dy, dz)
        distance: Distance to move along the direction

    Returns:
        New point (x + distance*dx, y + distance*dy, z + distance*dz)
    """
    return (
        point[0] + distance * direction[0],
        point[1] + distance * direction[1],
        point[2] + distance * direction[2]
    )


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
    norm = math.sqrt(x*x + y*y + z*z)
    if norm < 1e-10:
        return np.eye(4)
    x, y, z = x/norm, y/norm, z/norm

    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c

    R = np.eye(4)
    R[0, 0] = t*x*x + c
    R[0, 1] = t*x*y - z*s
    R[0, 2] = t*x*z + y*s
    R[1, 0] = t*x*y + z*s
    R[1, 1] = t*y*y + c
    R[1, 2] = t*y*z - x*s
    R[2, 0] = t*x*z - y*s
    R[2, 1] = t*y*z + x*s
    R[2, 2] = t*z*z + c
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
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
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
    """Apply 4x4 transformation matrix to a CadQuery shape."""
    pos = get_position(T)
    R = get_rotation_3x3(T)

    # Calculate Euler angles from rotation matrix
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        rx = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        ry = math.degrees(math.atan2(-R[2, 0], sy))
        rz = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    else:
        rx = math.degrees(math.atan2(-R[1, 2], R[1, 1]))
        ry = math.degrees(math.atan2(-R[2, 0], sy))
        rz = 0

    # Apply rotations then translation
    result = shape
    if abs(rx) > 0.001:
        result = result.rotate((0, 0, 0), (1, 0, 0), rx)
    if abs(ry) > 0.001:
        result = result.rotate((0, 0, 0), (0, 1, 0), ry)
    if abs(rz) > 0.001:
        result = result.rotate((0, 0, 0), (0, 0, 1), rz)

    result = result.moved(cq.Location(cq.Vector(*pos)))

    return result


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
    normal: tuple[float, float, float]    # Outward direction from surface
    attachment_type: str = "weld"         # "weld", "bom", or "label"
    port_name: str | None = None       # Associated port name


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

    def get_attachment_point_world_position(
        self,
        point_name: str,
        fitting_transform: np.ndarray
    ) -> tuple[float, float, float]:
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

    def get_attachment_point_world_normal(
        self,
        point_name: str,
        fitting_transform: np.ndarray
    ) -> tuple[float, float, float]:
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

def make_weld_neck_flange(nps: str, units: str = "mm") -> Fitting:
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
    flange_shape = make_weld_neck_flange_class300(nps, units)

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
        ("east",   r_hub_weld,  0.0,          1.0,  0.0),   # +X
        ("west",  -r_hub_weld,  0.0,         -1.0,  0.0),   # -X
        ("north",  0.0,         r_hub_weld,   0.0,  1.0),   # +Y
        ("south",  0.0,        -r_hub_weld,   0.0, -1.0),   # -Y
    ]

    for suffix, x_off, y_off, nx, ny in weld_attachment_positions:
        name = f"weld_{suffix}"
        attachment_points[name] = AttachmentPoint(
            name=name,
            position=(x_off, y_off, -hub_length),
            normal=(nx, ny, 0.0),
            attachment_type="weld",
            port_name="weld"
        )

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
    C = dims.socket_wall_thickness
    D = dims.min_bore
    G = dims.body_wall_thickness
    J = dims.socket_depth

    r_bore = D / 2.0
    r_outer = r_bore + G
    r_socket_inner = B / 2.0
    r_socket_outer = r_socket_inner + C

    # Build elbow geometry
    # Two cylinders + sphere at origin

    # Leg 1: from (A+J, 0, 0) to origin along -X (inlet at +X end)
    leg1_outer = cq.Solid.makeCylinder(
        r_outer, A,
        cq.Vector(A, 0, 0),
        cq.Vector(-1, 0, 0)
    )
    leg2_outer = cq.Solid.makeCylinder(
        r_outer, A,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 1, 0)
    )
    sphere_outer = cq.Solid.makeSphere(
        r_outer,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=360
    )

    outer = leg1_outer.fuse(leg2_outer).fuse(sphere_outer)

    # Inner bore
    leg1_inner = cq.Solid.makeCylinder(
        r_bore, A,
        cq.Vector(A, 0, 0),
        cq.Vector(-1, 0, 0)
    )
    leg2_inner = cq.Solid.makeCylinder(
        r_bore, A,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 1, 0)
    )
    sphere_inner = cq.Solid.makeSphere(
        r_bore,
        cq.Vector(0, 0, 0),
        cq.Vector(0, 0, 1),
        angleDegrees1=-90,
        angleDegrees2=90,
        angleDegrees3=360
    )

    inner = leg1_inner.fuse(leg2_inner).fuse(sphere_inner)
    elbow = outer.cut(inner)

    # Add socket counterbores
    elbow = add_socket_counterbores(elbow, r_socket_outer, r_socket_inner, J)

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
        ("up",    0.0,             r_socket_inner, 0.0,  1.0),   # +Z
        ("down",  0.0,            -r_socket_inner, 0.0, -1.0),   # -Z
        ("north", r_socket_inner,  0.0,            1.0,  0.0),   # +Y
        ("south",-r_socket_inner,  0.0,           -1.0,  0.0),   # -Y
    ]

    for suffix, y_off, z_off, ny, nz in inlet_attachment_positions:
        name = f"inlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(
            name=name,
            position=(socket_opening_x, y_off, z_off),
            normal=(0.0, ny, nz),
            attachment_type="weld",
            port_name="inlet"
        )

    # Outlet socket opening is at (0, A+J, 0) - a circle in the XZ plane
    # Points at orthogonal positions around the socket ID (where pipe enters and fillet weld is applied)
    outlet_attachment_positions = [
        # (name suffix, X offset, Z offset, normal_x, normal_z)
        ("up",    0.0,             r_socket_inner, 0.0,  1.0),   # +Z
        ("down",  0.0,            -r_socket_inner, 0.0, -1.0),   # -Z
        ("east",  r_socket_inner,  0.0,            1.0,  0.0),   # +X
        ("west", -r_socket_inner,  0.0,           -1.0,  0.0),   # -X
    ]

    for suffix, x_off, z_off, nx, nz in outlet_attachment_positions:
        name = f"outlet_weld_{suffix}"
        attachment_points[name] = AttachmentPoint(
            name=name,
            position=(x_off, socket_opening_y, z_off),
            normal=(nx, 0.0, nz),
            attachment_type="weld",
            port_name="outlet"
        )

    return Fitting(nps=nps, shape=elbow, ports=ports, attachment_points=attachment_points)


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


# =============================================================================
# MATING CALCULATION
# =============================================================================

def compute_mate_transform(
    port_a: Port,
    port_b: Port,
    fitting_a_transform: np.ndarray,
    gap: float = 0.0
) -> np.ndarray:
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
    pipe_world = compute_mate_transform(
        port_a=flange.get_port("weld"),
        port_b=pipe.get_port("start"),
        fitting_a_transform=flange_world,
        gap=gap
    )

    print(f"  Pipe world position: {get_position(pipe_world)}")
    # Pipe starts at flange weld port + gap along weld direction (-Z)
    # weld port at z=-hub_length, weld direction is -Z
    # pipe position = weld_pos + gap * (-Z) = (0, 0, -hub_length - gap)
    expected_z = flange.get_port('weld').position[2] - gap  # -76.2 - 1.6 = -77.8
    print(f"  Expected: near (0, 0, {expected_z:.1f})")

    # Create assembly
    flange_shape = flange.get_shape_at_transform(flange_world)
    pipe_shape = pipe.get_shape_at_transform(pipe_world)

    assembly = cq.Compound.makeCompound([flange_shape, pipe_shape])
    cq.exporters.export(assembly, "step/test_flange_pipe_assembly.step")
    print("\nExported test_flange_pipe_assembly.step")


if __name__ == "__main__":
    test_fittings()
