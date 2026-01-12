#!/usr/bin/env python3
"""
Pipe Routing System with Proper Coordinate Frames

Routes pipe from a starting flange through a series of directions,
automatically placing fittings at direction changes and filling with pipe.

Uses 4x4 homogeneous transformation matrices for proper coordinate handling.

Conventions:
- Coordinate system: X=East, Y=North, Z=Up
- All internal calculations in mm
- 1/16" (1.6mm) expansion gap at socket weld connections
- Pipe starts 1/16" from flange weld end

Example usage:
    route = PipeRoute(
        nps="2",
        start_fitting="flange",
        moves=[
            ("east", "10 ft"),
            ("up", "6 ft"),
            ("north", "5 ft"),
        ]
    )
    route.build()
    route.export("my_pipe_route.step")

References:
- https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
"""

import math
import re
from dataclasses import dataclass, field

import cadquery as cq
import numpy as np

from .fittings.pipe_fittings import (
    PIPE_SPECS,
    Fitting,
    PipeSpec,
    apply_transform_to_shape,
    compute_mate_transform,
    get_position,
    get_z_axis,
    identity_matrix,
    make_pipe,
    make_socket_weld_elbow,
    make_threaded_elbow_90_fitting,
    make_threaded_pipe,
    make_weld_neck_flange,
    point_along_direction,
    rotation_from_axis_angle,
    rotation_matrix_x,
    rotation_matrix_y,
    translation_matrix,
)
from .fittings.socket_weld_elbow import ASME_B1611_ELBOW90_CLASS3000, SocketWeldElbowDims
from .fittings.threaded_elbow import (
    ASME_B1611_THREADED_ELBOW90_CLASS3000,
    NPT_THREAD_ENGAGEMENT,
    ThreadedElbowDims,
)
from .fittings.weld_neck_flange import ASME_B165_CLASS300_WN, WeldNeckFlangeDims

# Import ComponentRecord and WeldRecord for BOM and weld tracking
try:
    from .drawing_generator.bom import ComponentRecord, WeldRecord
    HAS_BOM = True
except ImportError:
    HAS_BOM = False
    ComponentRecord = None
    WeldRecord = None


# =============================================================================
# CONSTANTS
# =============================================================================

EXPANSION_GAP_MM = 1.6  # 1/16" expansion gap

# Direction vectors (X=East, Y=North, Z=Up)
DIRECTIONS = {
    "east": (1, 0, 0),
    "west": (-1, 0, 0),
    "north": (0, 1, 0),
    "south": (0, -1, 0),
    "up": (0, 0, 1),
    "down": (0, 0, -1),
    "+x": (1, 0, 0),
    "-x": (-1, 0, 0),
    "+y": (0, 1, 0),
    "-y": (0, -1, 0),
    "+z": (0, 0, 1),
    "-z": (0, 0, -1),
}


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def parse_distance(distance_str: str) -> float:
    """Parse a distance string and return value in mm."""
    distance_str = distance_str.strip().lower()
    match = re.match(r"([0-9.]+)\s*(ft|feet|foot|in|inch|inches|mm|m|meter|meters|cm)?", distance_str)
    if not match:
        raise ValueError(f"Cannot parse distance: {distance_str}")

    value = float(match.group(1))
    unit = match.group(2) or "mm"

    conversions = {
        "ft": 304.8, "feet": 304.8, "foot": 304.8,
        "in": 25.4, "inch": 25.4, "inches": 25.4,
        "mm": 1.0, "m": 1000.0, "meter": 1000.0, "meters": 1000.0, "cm": 10.0,
    }
    return value * conversions.get(unit, 1.0)


def normalize_direction(direction: str) -> tuple[float, float, float]:
    """Get unit vector for a direction name."""
    direction = direction.strip().lower()
    if direction not in DIRECTIONS:
        raise ValueError(f"Unknown direction: {direction}. Use: {list(DIRECTIONS.keys())}")
    return DIRECTIONS[direction]


def rotation_to_align_z_with_direction(target: tuple[float, float, float]) -> np.ndarray:
    """
    Create rotation matrix that aligns +Z axis with target direction.
    """
    tx, ty, tz = target

    # Handle the 6 axis-aligned cases
    if abs(tz - 1.0) < 1e-6:  # +Z
        return identity_matrix()
    elif abs(tz + 1.0) < 1e-6:  # -Z
        return rotation_matrix_x(180)
    elif abs(tx - 1.0) < 1e-6:  # +X
        return rotation_matrix_y(90)
    elif abs(tx + 1.0) < 1e-6:  # -X
        return rotation_matrix_y(-90)
    elif abs(ty - 1.0) < 1e-6:  # +Y
        return rotation_matrix_x(-90)
    elif abs(ty + 1.0) < 1e-6:  # -Y
        return rotation_matrix_x(90)
    else:
        # General case using axis-angle
        # Rotate from z_axis (0, 0, 1) to target direction
        dot = tz
        if abs(dot - 1.0) < 1e-6:
            return identity_matrix()
        elif abs(dot + 1.0) < 1e-6:
            return rotation_matrix_x(180)

        # Cross product for rotation axis
        axis = (
            -ty,  # z_axis cross target
            tx,
            0
        )
        angle = math.degrees(math.acos(max(-1, min(1, dot))))
        return rotation_from_axis_angle(axis, angle)


def compute_elbow_rotation(incoming_dir: tuple[float, float, float],
                           outgoing_dir: tuple[float, float, float]) -> np.ndarray:
    """
    Compute rotation matrix to orient elbow for pipe routing.

    Args:
        incoming_dir: Direction the incoming pipe is traveling (e.g., EAST if pipe goes east)
        outgoing_dir: Direction the outgoing pipe will travel (e.g., UP for vertical rise)

    The elbow's default configuration (before rotation):
    - Inlet port at +X end, Z-axis points +X (outward toward incoming pipe)
    - Outlet port at +Y end, Z-axis points +Y (outward toward outgoing pipe)

    For proper mating:
    - Inlet Z must OPPOSE the incoming pipe's end Z (they face each other)
    - Incoming pipe end Z points in incoming_dir (e.g., EAST)
    - So inlet Z must point OPPOSITE to incoming_dir (e.g., WEST)

    - Outlet Z must point toward outgoing pipe
    - This is the same as outgoing_dir

    Therefore we map:
    - Elbow +X (inlet Z direction) → -incoming_dir (WEST if pipe came from EAST)
    - Elbow +Y (outlet Z direction) → outgoing_dir (UP)
    """
    # Inlet should face OPPOSITE to pipe travel direction
    # (to face the incoming pipe which is traveling toward the elbow)
    inlet_face_dir = (-incoming_dir[0], -incoming_dir[1], -incoming_dir[2])

    # Outlet should face the outgoing direction
    # (toward where the next pipe will go)
    outlet_face_dir = outgoing_dir

    ix, iy, iz = inlet_face_dir
    ox, oy, oz = outlet_face_dir

    # The third axis is the cross product (right-hand rule)
    cx = iy * oz - iz * oy
    cy = iz * ox - ix * oz
    cz = ix * oy - iy * ox

    # Build rotation matrix with columns [inlet_face_dir, outlet_face_dir, cross]
    R = np.eye(4)
    R[0, 0], R[1, 0], R[2, 0] = ix, iy, iz  # First column (maps +X to inlet_face_dir)
    R[0, 1], R[1, 1], R[2, 1] = ox, oy, oz  # Second column (maps +Y to outlet_face_dir)
    R[0, 2], R[1, 2], R[2, 2] = cx, cy, cz  # Third column (maps +Z to cross)

    return R


# =============================================================================
# PIPE ROUTE
# =============================================================================

@dataclass
class PipeRoute:
    """
    Defines and builds a pipe route.

    Args:
        nps: Nominal pipe size (e.g., "2")
        start_fitting: Type of starting fitting ("flange" or "none")
        moves: List of (direction, distance) tuples
        track_components: Whether to track components for BOM generation
        connection_type: Type of pipe connections ("socket_weld" or "threaded")
    """
    nps: str
    start_fitting: str
    moves: list[tuple[str, str]]
    parts: list[cq.Shape] = field(default_factory=list)
    components: list = field(default_factory=list)  # List[ComponentRecord]
    welds: list = field(default_factory=list)  # List[WeldRecord]
    fittings: list = field(default_factory=list)  # List[(Fitting, world_transform)] for attachment point visualization
    debug: bool = False
    track_components: bool = True  # Enable BOM and weld tracking
    connection_type: str = "socket_weld"  # "socket_weld" or "threaded"

    # Set in __post_init__ - typed here for Pylance
    pipe_spec: PipeSpec = field(init=False)
    elbow_dims: SocketWeldElbowDims | ThreadedElbowDims = field(init=False)
    flange_dims: WeldNeckFlangeDims | None = field(init=False)
    thread_engagement: float = field(init=False)

    def __post_init__(self):
        pipe_spec = PIPE_SPECS.get(self.nps)
        if pipe_spec is None:
            raise ValueError(f"No pipe spec for NPS {self.nps}")
        self.pipe_spec = pipe_spec

        if self.connection_type == "threaded":
            # Get threaded elbow dimensions
            elbow_dims = ASME_B1611_THREADED_ELBOW90_CLASS3000.get(self.nps)
            if elbow_dims is None:
                raise ValueError(f"No threaded elbow dimensions for NPS {self.nps}")
            self.elbow_dims = elbow_dims

            # Get thread engagement
            thread_spec = NPT_THREAD_ENGAGEMENT.get(self.nps)
            if thread_spec is None:
                raise ValueError(f"No NPT thread spec for NPS {self.nps}")
            self.thread_engagement = thread_spec.engagement

            # No flange for threaded routes (unless specified)
            self.flange_dims = None
        else:
            # Socket weld
            elbow_dims = ASME_B1611_ELBOW90_CLASS3000.get(self.nps)
            if elbow_dims is None:
                raise ValueError(f"No elbow dimensions for NPS {self.nps}")
            self.elbow_dims = elbow_dims

            flange_dims = ASME_B165_CLASS300_WN.get(self.nps)
            if flange_dims is None:
                raise ValueError(f"No flange dimensions for NPS {self.nps}")
            self.flange_dims = flange_dims

            self.thread_engagement = 0.0

    def _add_component(
        self,
        component_type: str,
        description: str,
        schedule_class: str,
        shape: cq.Shape,
        world_transform: np.ndarray,
        length_mm: float | None = None,
        connected_directions: list[str] | None = None,
        pipe_direction: str | None = None
    ):
        """Add a component record for BOM tracking.

        Args:
            component_type: "flange", "elbow", or "pipe"
            description: Human-readable description
            schedule_class: Schedule or class (e.g., "Sch 40", "Class 300")
            shape: The CadQuery geometry
            world_transform: 4x4 transformation matrix
            length_mm: For pipes only, the length
            connected_directions: World directions where other components connect
                                  (e.g., ["east"] for a flange with pipe going east)
            pipe_direction: For pipes, the direction the pipe runs (e.g., "east")
        """
        if not self.track_components or not HAS_BOM or ComponentRecord is None:
            return

        item_number = len(self.components) + 1
        self.components.append(ComponentRecord(
            item_number=item_number,
            component_type=component_type,
            nps=self.nps,
            description=description,
            schedule_class=schedule_class,
            length_mm=length_mm,
            shape=shape,
            world_transform=world_transform.copy(),
            connected_directions=connected_directions or [],
            pipe_direction=pipe_direction
        ))

    def _add_weld(
        self,
        weld_type: str,
        description: str,
        position_3d: tuple[float, float, float],
        pipe_direction: str | None = None,
        avoid_direction: str | None = None,
        pipe_radius_mm: float | None = None
    ):
        """Add a weld record for weld numbering per ASME B31.3.

        Args:
            weld_type: "BW" (butt weld), "SW" (socket weld), or "FW" (fillet weld)
            description: Human-readable description (e.g., "FLANGE TO PIPE")
            position_3d: 3D world position of the weld joint (at centerline)
            pipe_direction: World direction of pipe at this weld (e.g., "east")
            avoid_direction: Direction to avoid for marker placement (where next pipe goes)
            pipe_radius_mm: Pipe outer radius for computing edge position
        """
        if not self.track_components or not HAS_BOM or WeldRecord is None:
            return

        weld_number = len(self.welds) + 1
        self.welds.append(WeldRecord(
            weld_number=weld_number,
            weld_type=weld_type,
            description=description,
            world_position_3d=position_3d,
            pipe_direction=pipe_direction,
            avoid_direction=avoid_direction,
            pipe_radius_mm=pipe_radius_mm
        ))

        if self.debug:
            print(f"  Weld W{weld_number} ({weld_type}): {description} at {position_3d}")

    def _select_weld_attachment_point(
        self,
        fitting: Fitting,
        port_name: str,
        pipe_direction: str,
        weld_type: str = "BW",
        avoid_direction: str | None = None
    ) -> str:
        """
        Select the best weld attachment point based on pipe direction and weld type.

        For butt welds (BW): Prefer north side (perpendicular to E-W pipes, front of iso view)
        For socket welds (SW): Prefer south/back side (to avoid crossing connected fittings)

        Args:
            fitting: The Fitting object with attachment points
            port_name: The port to find attachments for (e.g., "weld", "inlet", "outlet")
            pipe_direction: The direction the pipe runs at this weld
            weld_type: "BW" for butt weld, "SW" for socket weld
            avoid_direction: Direction to avoid (where next fitting is)

        Returns:
            Name of the best attachment point to use
        """
        # Get all weld attachment points for this port
        attachments = fitting.get_attachment_points_for_port(port_name)
        if not attachments:
            raise ValueError(f"No attachment points for port '{port_name}' on fitting")

        # Define preference order based on weld type and pipe direction
        # For isometric view: north/east are "front", south/west are "back"
        pipe_dir = pipe_direction.lower()

        if weld_type == "BW":
            # Butt welds: prefer front side (north for E-W pipes, east for N-S pipes, north for vertical)
            if pipe_dir in ("east", "west"):
                preference = ["north", "up", "south", "down"]
            elif pipe_dir in ("north", "south"):
                preference = ["east", "up", "west", "down"]
            else:  # up/down
                preference = ["north", "east", "south", "west"]
        else:
            # Socket welds: attachment point selection depends on pipe direction
            # For horizontal pipes (E-W): attach at "down", leader extends south
            # For horizontal pipes (N-S): attach at "down", leader extends west
            # For vertical pipes (up/down): attach at "west", leader extends east (right)
            if pipe_dir in ("east", "west"):
                preference = ["down", "south", "up", "north"]
            elif pipe_dir in ("north", "south"):
                preference = ["down", "west", "up", "east"]
            else:  # up/down - flow-aware: prefer attachment on "outgoing" side
                if pipe_dir == "up":
                    # Pipe flows upward, prefer "up" attachment (away from incoming)
                    preference = ["up", "west", "down", "east"]
                else:  # down
                    # Pipe flows downward, prefer "down" attachment (away from incoming)
                    preference = ["down", "west", "up", "east"]

        # If we have an avoid direction, move it to the end of preference
        # But don't avoid "up"/"down" for vertical pipes - those are preferred directions
        if avoid_direction:
            avoid_dir = avoid_direction.lower()
            # For vertical pipes, don't penalize the preferred flow direction
            is_vertical_pipe = pipe_dir in ("up", "down")
            is_flow_direction = avoid_dir in ("up", "down")
            if avoid_dir in preference and not (is_vertical_pipe and is_flow_direction):
                preference.remove(avoid_dir)
                preference.append(avoid_dir)

        # Find the first available attachment point in preference order
        attachment_names = {ap.name: ap for ap in attachments}

        for direction in preference:
            # Build possible attachment point names
            possible_names = [
                f"{port_name}_weld_{direction}",
                f"{port_name}_{direction}",
            ]
            for name in possible_names:
                if name in attachment_names:
                    return name

        # Fallback: return first available attachment
        return attachments[0].name

    def build(self) -> cq.Compound | None:
        """Build the complete pipe route."""
        self.parts = []
        self.components = []
        self.welds = []
        self.fittings = []  # Track fittings with their world transforms for attachment point visualization

        # Dispatch to appropriate build method based on connection type
        if self.connection_type == "threaded":
            return self._build_threaded()
        else:
            return self._build_socket_weld()

    def _build_socket_weld(self) -> cq.Compound | None:
        """Build a socket weld pipe route."""
        # Parse moves
        parsed_moves = []
        for direction, distance in self.moves:
            dir_vec = normalize_direction(direction)
            dist_mm = parse_distance(distance)
            parsed_moves.append((dir_vec, dist_mm, direction))

        if len(parsed_moves) == 0:
            raise ValueError("No moves specified")

        # Get elbow dimensions
        elbow_A = self.elbow_dims.center_to_end  # mm (center to socket opening)

        first_dir = parsed_moves[0][0]

        # =====================================================================
        # STEP 1: Place starting flange
        # =====================================================================
        flange = make_weld_neck_flange(self.nps, units="mm")

        # The flange's weld port should face in first_dir
        # Weld port default direction is -Z
        # We need to rotate so weld port faces first_dir

        # Rotation to align -Z with first_dir is same as aligning +Z with -first_dir
        neg_first = (-first_dir[0], -first_dir[1], -first_dir[2])
        flange_rotation = rotation_to_align_z_with_direction(neg_first)

        # Flange at origin with this rotation
        flange_transform = flange_rotation.copy()

        assert flange.shape is not None, "Flange shape is required"
        flange_shape = apply_transform_to_shape(flange.shape, flange_transform)
        self.parts.append(flange_shape)

        # Get the first pipe direction name for flange connection tracking
        first_dir_name = parsed_moves[0][2]  # Direction name like "east"

        # Track flange for BOM - flange connects to pipe in first_dir
        # NOTE: Pass local shape (flange.shape) not world shape (flange_shape)
        # because _add_component applies world_transform to get world position
        self._add_component(
            component_type="flange",
            description="WN FLANGE",
            schedule_class="Class 300",
            shape=flange.shape,
            world_transform=flange_transform,
            connected_directions=[first_dir_name]  # Pipe goes in this direction
        )

        # Track flange for attachment point visualization
        self.fittings.append((flange, flange_transform))

        # Get flange weld position using explicit attachment points
        # The flange has attachment points at orthogonal positions around the weld end
        # Select the best attachment point based on pipe direction (prefer perpendicular to pipe, north side)
        weld_attachment_name = self._select_weld_attachment_point(
            flange, "weld", first_dir_name, weld_type="BW"
        )
        weld_pos = flange.get_attachment_point_world_position(weld_attachment_name, flange_transform)
        weld_normal = flange.get_attachment_point_world_normal(weld_attachment_name, flange_transform)

        # Get pipe radius for smart marker placement
        pipe_radius = self.pipe_spec.od_mm / 2.0

        if self.debug:
            print(f"Flange weld attachment: {weld_attachment_name}")
            print(f"Flange weld world position: {weld_pos}")
            print(f"Flange weld world normal: {weld_normal}")

        # Track weld at flange neck (butt weld to pipe)
        # Position is now at the actual weld surface (pipe OD)
        self._add_weld(
            weld_type="BW",
            description="FLANGE TO PIPE",
            position_3d=weld_pos,
            pipe_direction=first_dir_name,
            pipe_radius_mm=pipe_radius
        )

        # =====================================================================
        # STEP 2: Track current connection point
        # =====================================================================
        # Current port is the flange's weld port
        weld_port = flange.get_port("weld")
        current_transform = flange_transform
        current_port = weld_port

        # Process each move
        for i, (dir_vec, dist_mm, dir_name) in enumerate(parsed_moves):
            is_last = (i == len(parsed_moves) - 1)

            if self.debug:
                print(f"\n--- Move {i+1}: {dir_name} {dist_mm:.1f}mm ---")

            # =====================================================================
            # STEP 3: Calculate pipe length (CENTERLINE-BASED)
            # =====================================================================
            # Distances are measured from reference points:
            # - First segment: flange face (raised face) to elbow center
            # - Middle segments: elbow center to elbow center
            # - Last segment: elbow center to pipe end
            is_first = (i == 0)
            INCH_TO_MM = 25.4

            if is_first:
                # First segment: from flange raised face to elbow center
                # Flange offset = hub_length + raised_face_height (RF to weld port)
                flange_offset = (self.flange_dims.hub_length + self.flange_dims.raised_face_height) * INCH_TO_MM
                start_offset = flange_offset
            else:
                # From previous elbow center (center_to_end = A dimension)
                start_offset = elbow_A

            if is_last:
                # Last segment: elbow center to pipe end
                end_offset = 0
                gap_count = 1  # Only gap at start (after previous fitting)
            else:
                # To next elbow center
                end_offset = elbow_A
                gap_count = 2  # Gap at both ends

            pipe_length = dist_mm - start_offset - end_offset - gap_count * EXPANSION_GAP_MM

            if pipe_length <= 0:
                raise ValueError(
                    f"Move {i+1} ({dir_name} {dist_mm:.1f}mm) is too short. "
                    f"Minimum: {start_offset + end_offset:.1f}mm"
                )

            if self.debug:
                print(f"  Pipe length: {pipe_length:.1f}mm")

            # =====================================================================
            # STEP 4: Create and place pipe
            # =====================================================================
            pipe = make_pipe(self.nps, pipe_length)

            # Mate pipe's start port to current port
            pipe_transform = compute_mate_transform(
                port_a=current_port,
                port_b=pipe.get_port("start"),
                fitting_a_transform=current_transform,
                gap=EXPANSION_GAP_MM
            )

            assert pipe.shape is not None, "Pipe shape is required"
            pipe_shape = apply_transform_to_shape(pipe.shape, pipe_transform)
            self.parts.append(pipe_shape)

            # Track pipe for BOM - pipe runs in the current direction
            # NOTE: Pass local shape (pipe.shape) not world shape (pipe_shape)
            # because _add_component applies world_transform to get world position
            self._add_component(
                component_type="pipe",
                description="PIPE",
                schedule_class="Sch 40",
                shape=pipe.shape,
                world_transform=pipe_transform,
                length_mm=pipe_length,
                pipe_direction=dir_name  # Direction the pipe runs (e.g., "east")
            )

            if self.debug:
                print(f"  Pipe placed at: {get_position(pipe_transform)}")

            # Update current port to pipe's end
            current_transform = pipe_transform
            current_port = pipe.get_port("end")

            # =====================================================================
            # STEP 5: Place elbow (if not last move)
            # =====================================================================
            if not is_last:
                next_dir = parsed_moves[i + 1][0]

                elbow = make_socket_weld_elbow(self.nps, units="mm")

                # Elbow orientation:
                # - Inlet faces OPPOSITE to current pipe direction (to receive pipe)
                # - Outlet faces toward next direction (to send pipe)
                # compute_elbow_rotation handles the negation of incoming direction
                elbow_rotation = compute_elbow_rotation(dir_vec, next_dir)

                inlet_port = elbow.get_port("inlet")
                outlet_port = elbow.get_port("outlet")

                # Get the pipe end port's world position and direction
                pipe_end_world = current_transform @ current_port.transform
                pipe_end_pos = get_position(pipe_end_world)
                pipe_end_dir = get_z_axis(pipe_end_world)

                # Mating position: where inlet port origin should be
                # Inlet port is at socket bottom, pipe tip should be gap away from it
                # So inlet_mate_pos = pipe_end + gap along pipe direction
                inlet_mate_pos = point_along_direction(
                    pipe_end_pos, pipe_end_dir, EXPANSION_GAP_MM
                )

                # Compute elbow center position
                # inlet_local_pos is at (A, 0, 0) in elbow local frame
                # After elbow_rotation, this maps to rotated_inlet_offset from elbow center
                inlet_local_pos = get_position(inlet_port.transform)
                rotated_inlet_offset = elbow_rotation[:3, :3] @ np.array(inlet_local_pos)

                # elbow_center + rotated_inlet_offset = inlet_mate_pos
                # Therefore: elbow_center = inlet_mate_pos - rotated_inlet_offset
                elbow_center = (
                    inlet_mate_pos[0] - rotated_inlet_offset[0],
                    inlet_mate_pos[1] - rotated_inlet_offset[1],
                    inlet_mate_pos[2] - rotated_inlet_offset[2],
                )

                # Final elbow transform = translation to center, then rotation
                final_elbow_transform = translation_matrix(*elbow_center) @ elbow_rotation

                assert elbow.shape is not None, "Elbow shape is required"
                elbow_shape = apply_transform_to_shape(elbow.shape, final_elbow_transform)
                self.parts.append(elbow_shape)

                # Get the next direction name for elbow connection tracking
                next_dir_name = parsed_moves[i + 1][2]

                # Track elbow for BOM - elbow connects pipes from dir_name and to next_dir_name
                # The elbow has pipe coming FROM the opposite of dir_name (pipe travels in dir_name)
                # and pipe going TO next_dir_name
                # NOTE: Pass local shape (elbow.shape) not world shape (elbow_shape)
                self._add_component(
                    component_type="elbow",
                    description="90° SW ELL",
                    schedule_class="Class 3000",
                    shape=elbow.shape,
                    world_transform=final_elbow_transform,
                    connected_directions=[dir_name, next_dir_name]  # Pipes in these directions
                )

                # Track elbow for attachment point visualization
                self.fittings.append((elbow, final_elbow_transform))

                # Get weld positions from elbow's explicit attachment points
                # Socket welds (fillet welds) are at the socket OPENING at the pipe OD
                # The attachment points are already at the correct position (socket opening, on OD)

                # Get pipe radius for reference
                pipe_radius = self.pipe_spec.od_mm / 2.0

                # Direction names for marker placement
                opposite_dir_name = {
                    "east": "west", "west": "east",
                    "north": "south", "south": "north",
                    "up": "down", "down": "up",
                }.get(dir_name, dir_name)

                # Select best attachment point for inlet weld (avoid next_dir_name)
                inlet_attachment_name = self._select_weld_attachment_point(
                    elbow, "inlet", dir_name, weld_type="SW", avoid_direction=next_dir_name
                )
                inlet_weld_pos = elbow.get_attachment_point_world_position(
                    inlet_attachment_name, final_elbow_transform
                )

                # Select best attachment point for outlet weld
                # For vertical pipes, don't use avoid_direction (we want "west" specifically)
                outlet_avoid = None if next_dir_name in ("up", "down") else opposite_dir_name
                outlet_attachment_name = self._select_weld_attachment_point(
                    elbow, "outlet", next_dir_name, weld_type="SW", avoid_direction=outlet_avoid
                )
                outlet_weld_pos = elbow.get_attachment_point_world_position(
                    outlet_attachment_name, final_elbow_transform
                )

                self._add_weld(
                    weld_type="SW",
                    description="PIPE TO ELBOW",
                    position_3d=inlet_weld_pos,
                    pipe_direction=dir_name,
                    avoid_direction=next_dir_name,
                    pipe_radius_mm=pipe_radius
                )
                self._add_weld(
                    weld_type="SW",
                    description="ELBOW TO PIPE",
                    position_3d=outlet_weld_pos,
                    pipe_direction=next_dir_name,
                    avoid_direction=opposite_dir_name,
                    pipe_radius_mm=pipe_radius
                )

                if self.debug:
                    print(f"  Elbow center at: {elbow_center}")
                    print(f"  Inlet weld attachment: {inlet_attachment_name} at {inlet_weld_pos}")
                    print(f"  Outlet weld attachment: {outlet_attachment_name} at {outlet_weld_pos}")

                # Update current port to elbow's outlet
                current_transform = final_elbow_transform
                current_port = outlet_port

        # Combine all parts
        if self.parts:
            return cq.Compound.makeCompound(self.parts)
        return None

    def _build_threaded(self) -> cq.Compound | None:
        """Build a threaded pipe route (NPT connections)."""
        # Parse moves
        parsed_moves = []
        for direction, distance in self.moves:
            dir_vec = normalize_direction(direction)
            dist_mm = parse_distance(distance)
            parsed_moves.append((dir_vec, dist_mm, direction))

        if len(parsed_moves) == 0:
            raise ValueError("No moves specified")

        # Get elbow dimensions
        elbow_A = self.elbow_dims.center_to_end  # mm (center to thread end)

        # For threaded routes, we start directly with the first pipe
        # Position tracking: current position and direction
        current_pos = np.array([0.0, 0.0, 0.0])

        if self.debug:
            print("Building THREADED pipe route")
            print(f"  Thread engagement: {self.thread_engagement:.1f}mm")

        # Track current connection point using identity transform initially
        current_transform = identity_matrix()
        # Initialize current_port - will be set after first pipe is placed
        current_port = None

        # Process each move
        for i, (dir_vec, dist_mm, dir_name) in enumerate(parsed_moves):
            is_last = (i == len(parsed_moves) - 1)
            is_first = (i == 0)

            if self.debug:
                print(f"\n--- Move {i+1}: {dir_name} {dist_mm:.1f}mm ---")

            # =====================================================================
            # Calculate pipe length for threaded connections (CENTERLINE-BASED)
            # =====================================================================
            # Distances are measured from reference points:
            # - First segment: pipe start to elbow center
            # - Middle segments: elbow center to elbow center
            # - Last segment: elbow center to pipe end
            # No expansion gaps for threaded (pipe screws directly into fittings)

            if is_first:
                if is_last:
                    # Only pipe, no fittings - distance is pipe length
                    pipe_length = dist_mm
                else:
                    # First segment: pipe start to elbow center
                    # dist_mm = pipe_length + elbow_A
                    pipe_length = dist_mm - elbow_A
            else:
                if is_last:
                    # Last segment: elbow center to pipe end
                    # dist_mm = elbow_A + pipe_length
                    pipe_length = dist_mm - elbow_A
                else:
                    # Middle segment: elbow center to elbow center
                    # dist_mm = elbow_A + pipe_length + elbow_A
                    pipe_length = dist_mm - 2 * elbow_A

            if pipe_length <= 0:
                min_length = dist_mm + 1  # Just for error message
                raise ValueError(
                    f"Move {i+1} ({dir_name} {dist_mm:.1f}mm) is too short for threaded fittings. "
                    f"Minimum: {min_length:.1f}mm"
                )

            if self.debug:
                print(f"  Pipe length: {pipe_length:.1f}mm")

            # =====================================================================
            # Create and place threaded pipe
            # =====================================================================
            pipe = make_threaded_pipe(self.nps, pipe_length, thread_end="both")

            if is_first:
                # First pipe: position at origin, oriented along direction
                # Pipe is along Z-axis by default, so we need to rotate to align with dir_vec
                pipe_rotation = rotation_to_align_z_with_direction(dir_vec)
                pipe_transform = pipe_rotation.copy()

                # Track starting position
                pipe_start_pos = current_pos.copy()
            else:
                # Pipe comes from previous elbow's outlet
                # Mate pipe's start port to current port
                assert current_port is not None, "current_port should be set after first pipe"
                pipe_transform = compute_mate_transform(
                    port_a=current_port,
                    port_b=pipe.get_port("start"),
                    fitting_a_transform=current_transform,
                    gap=0  # No gap for threaded connections
                )
                pipe_start_pos = get_position(pipe_transform)

            assert pipe.shape is not None, "Pipe shape is required"
            pipe_shape = apply_transform_to_shape(pipe.shape, pipe_transform)
            self.parts.append(pipe_shape)

            # Track pipe for BOM
            # NOTE: Pass local shape (pipe.shape) not world shape (pipe_shape)
            self._add_component(
                component_type="pipe",
                description="PIPE, NPT THREADED",
                schedule_class="Sch 40",
                shape=pipe.shape,
                world_transform=pipe_transform,
                length_mm=pipe_length,
                pipe_direction=dir_name
            )

            if self.debug:
                print(f"  Pipe placed at: {pipe_start_pos}")

            # Update current connection to pipe's end
            current_transform = pipe_transform
            current_port = pipe.get_port("end")

            # =====================================================================
            # Place elbow (if not last move)
            # =====================================================================
            if not is_last:
                next_dir = parsed_moves[i + 1][0]
                next_dir_name = parsed_moves[i + 1][2]

                elbow = make_threaded_elbow_90_fitting(self.nps, units="mm")

                # Elbow orientation:
                # - Inlet faces OPPOSITE to current pipe direction (to receive pipe)
                # - Outlet faces toward next direction (to send pipe)
                elbow_rotation = compute_elbow_rotation(dir_vec, next_dir)

                inlet_port = elbow.get_port("inlet")
                outlet_port = elbow.get_port("outlet")

                # Get the pipe end port's world position
                pipe_end_world = current_transform @ current_port.transform
                pipe_end_pos = get_position(pipe_end_world)

                # For threaded: pipe screws directly into elbow, no gap
                inlet_mate_pos = pipe_end_pos

                # Compute elbow center position
                inlet_local_pos = get_position(inlet_port.transform)
                rotated_inlet_offset = elbow_rotation[:3, :3] @ np.array(inlet_local_pos)

                elbow_center = (
                    inlet_mate_pos[0] - rotated_inlet_offset[0],
                    inlet_mate_pos[1] - rotated_inlet_offset[1],
                    inlet_mate_pos[2] - rotated_inlet_offset[2],
                )

                # Final elbow transform
                final_elbow_transform = translation_matrix(*elbow_center) @ elbow_rotation

                assert elbow.shape is not None, "Elbow shape is required"
                elbow_shape = apply_transform_to_shape(elbow.shape, final_elbow_transform)
                self.parts.append(elbow_shape)

                # Track elbow for BOM
                # NOTE: Pass local shape (elbow.shape) not world shape (elbow_shape)
                self._add_component(
                    component_type="elbow",
                    description="90° NPT THREADED ELL",
                    schedule_class="Class 3000",
                    shape=elbow.shape,
                    world_transform=final_elbow_transform,
                    connected_directions=[dir_name, next_dir_name]
                )

                # Track elbow for visualization
                self.fittings.append((elbow, final_elbow_transform))

                if self.debug:
                    print(f"  Elbow center at: {elbow_center}")

                # Update current port to elbow's outlet
                current_transform = final_elbow_transform
                current_port = outlet_port

        # Combine all parts
        if self.parts:
            return cq.Compound.makeCompound(self.parts)
        return None

    def export(self, filename: str):
        """Export the route to a STEP file."""
        if not self.parts:
            self.build()

        compound = cq.Compound.makeCompound(self.parts)
        cq.exporters.export(compound, filename)
        print(f"Exported {filename}")


# =============================================================================
# EXAMPLES
# =============================================================================

def example_simple_route():
    """Create a simple pipe route: east then up."""
    print("Pipe Router - Simple Route Example")
    print("=" * 50)

    route = PipeRoute(
        nps="2",
        start_fitting="flange",
        moves=[
            ("east", "500 mm"),
            ("up", "300 mm"),
        ],
        debug=True
    )

    route.build()
    route.export("step/pipe_route_simple.step")
    print("\nCreated simple route: flange -> east 500mm -> up 300mm")


def example_three_segment_route():
    """Create a route with three segments."""
    print("\nPipe Router - Three Segment Route")
    print("=" * 50)

    route = PipeRoute(
        nps="2",
        start_fitting="flange",
        moves=[
            ("east", "1000 mm"),
            ("up", "500 mm"),
            ("north", "800 mm"),
        ],
        debug=True
    )

    route.build()
    route.export("step/pipe_route_three_segments.step")
    print("\nCreated route: flange -> east 1m -> up 0.5m -> north 0.8m")


def example_imperial_units():
    """Create a route using imperial units."""
    print("\nPipe Router - Imperial Units Example")
    print("=" * 50)

    route = PipeRoute(
        nps="2",
        start_fitting="flange",
        moves=[
            ("east", "3 ft"),
            ("up", "2 ft"),
        ],
        debug=True
    )

    route.build()
    route.export("step/pipe_route_imperial.step")
    print("\nCreated route: flange -> east 3ft -> up 2ft")


if __name__ == "__main__":
    example_simple_route()
    example_three_segment_route()
    example_imperial_units()
    print("\nDone!")
