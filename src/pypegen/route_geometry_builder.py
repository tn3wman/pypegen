#!/usr/bin/env python3
"""
Route Geometry Builder for Branching Pipe Routes.

This module provides a recursive tree traversal algorithm to build 3D geometry
from a BranchingPipeRoute tree structure.

The builder:
1. Traverses the RouteNode tree depth-first
2. Builds geometry for each node (fittings, pipes)
3. Computes world transforms using port-based mating
4. Tracks components and welds for BOM generation
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import cadquery as cq
import numpy as np

from .fittings.butt_weld_elbow import ASME_B169_ELBOW90
from .fittings.butt_weld_elbow_45 import ASME_B169_ELBOW45
from .fittings.butt_weld_reducer import (
    ASME_B169_REDUCER,
    get_butt_weld_reducer_dims,
    validate_reducer,
)
from .fittings.butt_weld_tee import ASME_B169_TEE
from .fittings.pipe_fittings import (
    PIPE_SPECS,
    Fitting,
    Port,
    apply_transform_to_shape,
    compute_initial_up_vector,
    compute_up_vector_after_elbow,
    compute_up_vector_for_branch,
    create_port_markers_for_fitting,
    get_position,
    identity_matrix,
    make_butt_weld_elbow_fitting,
    make_butt_weld_elbow_45_fitting,
    make_butt_weld_reducer_fitting,
    make_butt_weld_tee_fitting,
    make_pipe,
    make_socket_weld_elbow,
    make_socket_weld_elbow_45,
    make_weld_neck_flange,
    rotation_matrix_x,
    rotation_matrix_y,
    translation_matrix,
)
from .fittings.socket_weld_cross import ASME_B1611_CROSS_CLASS3000, make_socket_weld_cross
from .fittings.socket_weld_elbow import ASME_B1611_ELBOW90_CLASS3000, ASME_B1611_ELBOW45_CLASS3000
from .fittings.socket_weld_tee import ASME_B1611_TEE_CLASS3000, make_socket_weld_tee
from .fittings.weld_neck_flange import ASME_B165_CLASS300_WN
from .pipe_router import (
    DIRECTIONS,
    EXPANSION_GAP_MM,
    normalize_direction,
    parse_distance,
    rotation_to_align_z_with_direction,
)
from .route_nodes import (
    CrossNode,
    ElbowNode,
    FlangeNode,
    PipeSegmentNode,
    ReducerNode,
    RouteNode,
    TeeNode,
    TerminalNode,
    WeldoletNode,
)

if TYPE_CHECKING:
    from .route_builder import BranchingPipeRoute


# =============================================================================
# COMPONENT/WELD TRACKING (compatible with existing BOM system)
# =============================================================================

try:
    from .drawing_generator.bom import ComponentRecord, WeldRecord

    HAS_BOM = True
except ImportError:
    HAS_BOM = False
    ComponentRecord = None  # type: ignore[misc, assignment]
    WeldRecord = None  # type: ignore[misc, assignment]


# =============================================================================
# DIRECTION UTILITIES
# =============================================================================


def compute_45_degree_turn_direction(
    incoming_direction: tuple[float, float, float],
    turn_plane_direction: tuple[float, float, float],
) -> tuple[float, float, float]:
    """
    Compute the actual output direction after a 45° turn.

    For a 45° elbow, we rotate 45° from the incoming direction toward the
    turn_plane_direction. The turn_plane_direction specifies the direction
    a 90° elbow would result in.

    This correctly handles cases where incoming and turn_plane are not
    perpendicular (e.g., chained 45° elbows).

    Args:
        incoming_direction: Direction pipe was traveling before the elbow
        turn_plane_direction: Cardinal direction specifying the turn plane
                              (what a 90° turn would result in)

    Returns:
        Unit vector of the actual output direction (45° from incoming)
    """
    import math

    inc = np.array(incoming_direction)
    turn = np.array(turn_plane_direction)

    # Normalize inputs
    inc = inc / np.linalg.norm(inc)
    turn = turn / np.linalg.norm(turn)

    # Find the component of turn that's perpendicular to incoming
    # This defines the direction we're turning toward
    turn_parallel = np.dot(turn, inc) * inc
    turn_perp = turn - turn_parallel

    if np.linalg.norm(turn_perp) < 1e-6:
        # turn is parallel to incoming - can't turn in that direction
        # This shouldn't happen for valid elbow configurations
        return tuple(incoming_direction)

    turn_perp = turn_perp / np.linalg.norm(turn_perp)

    # Rotate 45° from incoming toward turn_perp
    angle = math.radians(45)
    out_45 = math.cos(angle) * inc + math.sin(angle) * turn_perp

    return (float(out_45[0]), float(out_45[1]), float(out_45[2]))


def direction_name_to_vector(name: str) -> tuple[float, float, float]:
    """Convert a direction name to a unit vector."""
    return normalize_direction(name)


def offset_transform_by_gap(
    transform: np.ndarray,
    direction: tuple[float, float, float],
    gap: float = EXPANSION_GAP_MM,
) -> np.ndarray:
    """
    Offset a transform by the weld gap distance in the given direction.

    This creates space between mating ports for the weld gap.

    Args:
        transform: The 4x4 transformation matrix to offset
        direction: Unit vector direction to offset in
        gap: Distance to offset (default: EXPANSION_GAP_MM)

    Returns:
        New transform offset by gap in the given direction
    """
    offset = np.array([direction[0] * gap, direction[1] * gap, direction[2] * gap])
    position = get_position(transform)
    new_position = (position[0] + offset[0], position[1] + offset[1], position[2] + offset[2])

    # Create new transform with offset position but same rotation
    result = transform.copy()
    result[0, 3] = new_position[0]
    result[1, 3] = new_position[1]
    result[2, 3] = new_position[2]
    return result


def vector_to_direction_name(vec: tuple[float, float, float]) -> str:
    """Convert a unit vector to a direction name."""
    for name, dir_vec in DIRECTIONS.items():
        if abs(vec[0] - dir_vec[0]) < 0.01 and abs(vec[1] - dir_vec[1]) < 0.01 and abs(vec[2] - dir_vec[2]) < 0.01:
            return name
    return f"({vec[0]:.2f}, {vec[1]:.2f}, {vec[2]:.2f})"


def compute_perpendicular_directions(
    run_direction: tuple[float, float, float],
    reference_up: tuple[float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """
    Compute two perpendicular directions for cross branches.

    Args:
        run_direction: Direction of the main run
        reference_up: Reference "up" direction for orientation

    Returns:
        Tuple of (left_direction, right_direction)
    """
    run = np.array(run_direction)
    up = np.array(reference_up)

    # Left is perpendicular to both run and up
    left = np.cross(up, run)
    if np.linalg.norm(left) < 0.01:
        # run and up are parallel, use a different reference
        up = np.array([1, 0, 0]) if abs(run[0]) < 0.9 else np.array([0, 1, 0])
        left = np.cross(up, run)

    left = left / np.linalg.norm(left)
    right = -left

    return tuple(left), tuple(right)  # type: ignore[return-value]


# =============================================================================
# ROUTE GEOMETRY BUILDER
# =============================================================================


class RouteGeometryBuilder:
    """
    Builds 3D geometry from a BranchingPipeRoute tree.

    This class traverses the route tree depth-first, building geometry for
    each node and tracking components/welds for BOM generation.
    """

    def __init__(self, route: BranchingPipeRoute):
        """
        Initialize the geometry builder.

        Args:
            route: The BranchingPipeRoute to build geometry for
        """
        self.route = route
        self.nps = route.nps
        self.schedule = route.schedule

        # Get pipe spec
        self.pipe_spec = PIPE_SPECS.get(self.nps)
        if self.pipe_spec is None:
            raise ValueError(f"No pipe spec for NPS {self.nps}")

        # Get dimension tables
        self.elbow_dims = ASME_B1611_ELBOW90_CLASS3000.get(self.nps)
        self.elbow_45_dims = ASME_B1611_ELBOW45_CLASS3000.get(self.nps)
        self.butt_weld_elbow_dims = ASME_B169_ELBOW90.get(self.nps)
        self.butt_weld_elbow_45_dims = ASME_B169_ELBOW45.get(self.nps)
        self.flange_dims = ASME_B165_CLASS300_WN.get(self.nps)
        self.tee_dims = ASME_B1611_TEE_CLASS3000.get(self.nps)
        self.butt_weld_tee_dims = ASME_B169_TEE.get(self.nps)
        self.cross_dims = ASME_B1611_CROSS_CLASS3000.get(self.nps)

        # Component tracking
        self._item_counter = 0
        self._weld_counter = 0

    def _save_nps_state(self) -> dict:
        """Save the current NPS-related state for later restoration.

        This is needed when building branching fittings (tees, crosses) where
        each branch should start with the same NPS, regardless of whether
        a sibling branch contains a reducer that changes the NPS.
        """
        return {
            "nps": self.nps,
            "pipe_spec": self.pipe_spec,
            "elbow_dims": self.elbow_dims,
            "elbow_45_dims": self.elbow_45_dims,
            "butt_weld_elbow_dims": self.butt_weld_elbow_dims,
            "butt_weld_elbow_45_dims": self.butt_weld_elbow_45_dims,
            "flange_dims": self.flange_dims,
            "tee_dims": self.tee_dims,
            "butt_weld_tee_dims": self.butt_weld_tee_dims,
            "cross_dims": self.cross_dims,
        }

    def _restore_nps_state(self, state: dict) -> None:
        """Restore NPS-related state from a previously saved snapshot."""
        self.nps = state["nps"]
        self.pipe_spec = state["pipe_spec"]
        self.elbow_dims = state["elbow_dims"]
        self.elbow_45_dims = state["elbow_45_dims"]
        self.butt_weld_elbow_dims = state["butt_weld_elbow_dims"]
        self.butt_weld_elbow_45_dims = state["butt_weld_elbow_45_dims"]
        self.flange_dims = state["flange_dims"]
        self.tee_dims = state["tee_dims"]
        self.butt_weld_tee_dims = state["butt_weld_tee_dims"]
        self.cross_dims = state["cross_dims"]

    def _add_fitting_with_markers(
        self, fitting: Fitting, world_transform: np.ndarray
    ) -> None:
        """
        Add a fitting to the route and optionally add coordinate frame markers.

        Args:
            fitting: The fitting object with ports
            world_transform: World transform matrix for the fitting
        """
        self.route.fittings.append((fitting, world_transform))

        # Add coordinate frame markers if enabled
        if self.route.show_coordinate_frames:
            markers = create_port_markers_for_fitting(fitting, world_transform)
            self.route.parts.extend(markers)

    def build(self) -> None:
        """Build geometry for the entire route tree."""
        if self.route.root is None:
            raise ValueError("No root node defined")

        # Start with identity transform at origin
        initial_transform = identity_matrix()

        # Get initial direction from root node
        if isinstance(self.route.root, FlangeNode):
            initial_direction = direction_name_to_vector(self.route.root.direction)
        else:
            initial_direction = (1.0, 0.0, 0.0)  # Default: east

        # Compute initial up-vector based on direction
        initial_up = compute_initial_up_vector(initial_direction)

        # Recursive build starting from root
        self._build_node(
            node=self.route.root,
            incoming_transform=initial_transform,
            incoming_direction=initial_direction,
            up_vector=initial_up,
        )

    def _build_node(
        self,
        node: RouteNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
        prev_fitting_type: str = "pipe",
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """
        Build geometry for a single node and recursively build children.

        Args:
            node: The RouteNode to build
            incoming_transform: World transform for the inlet port
            incoming_direction: World direction the pipe was traveling
            up_vector: Current "up" reference direction (perpendicular to pipe)
            prev_fitting_type: Type of previous fitting for centerline calculations

        Returns:
            Dictionary mapping port names to (transform, direction, up_vector) tuples
            for each outlet port.
        """
        if isinstance(node, FlangeNode):
            return self._build_flange(node, incoming_transform, incoming_direction, up_vector)
        elif isinstance(node, PipeSegmentNode):
            return self._build_pipe(node, incoming_transform, incoming_direction, up_vector, prev_fitting_type)
        elif isinstance(node, ElbowNode):
            return self._build_elbow(node, incoming_transform, incoming_direction, up_vector)
        elif isinstance(node, TeeNode):
            return self._build_tee(node, incoming_transform, incoming_direction, up_vector)
        elif isinstance(node, CrossNode):
            return self._build_cross(node, incoming_transform, incoming_direction, up_vector)
        elif isinstance(node, WeldoletNode):
            return self._build_weldolet(node, incoming_transform, incoming_direction, up_vector)
        elif isinstance(node, ReducerNode):
            return self._build_reducer(node, incoming_transform, incoming_direction, up_vector)
        elif isinstance(node, TerminalNode):
            return self._build_terminal(node, incoming_transform, incoming_direction, up_vector)
        else:
            raise ValueError(f"Unknown node type: {type(node)}")

    # -------------------------------------------------------------------------
    # NODE BUILDERS
    # -------------------------------------------------------------------------

    def _build_flange(
        self,
        node: FlangeNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build a flange fitting."""
        _ = up_vector  # Initial up computed based on direction

        # Create flange with bolt hole orientation
        fitting = make_weld_neck_flange(
            node.nps, units="mm", bolt_hole_orientation=node.bolt_hole_orientation
        )

        # Determine if this is a starting flange (root) or ending flange
        is_root_flange = (node is self.route.root)

        if is_root_flange:
            # Starting flange: weld port faces in node.direction (pipe extends that way)
            direction = direction_name_to_vector(node.direction)
            # Orient flange so weld port faces in the specified direction
            # Weld port default direction is -Z, so rotate to align -Z with direction
            neg_direction = (-direction[0], -direction[1], -direction[2])
            flange_rotation = rotation_to_align_z_with_direction(neg_direction)
            # Apply to incoming transform
            flange_transform = incoming_transform @ flange_rotation
        else:
            # Ending flange: weld port should face OPPOSITE to incoming direction
            # (toward the pipe that's coming toward this flange)
            direction = incoming_direction  # Flange face points in pipe direction
            # Weld port should face opposite to incoming direction (toward the pipe)
            # Rotate to align -Z with opposite of incoming direction
            # (so weld port Z faces back toward the pipe)
            neg_weld_dir = (incoming_direction[0], incoming_direction[1], incoming_direction[2])
            flange_rotation = rotation_to_align_z_with_direction(neg_weld_dir)

            # The weld port is NOT at the flange origin - it's at local (0, 0, -hub_length)
            # We need to position the flange so the weld port is at the incoming position
            weld_port = fitting.get_port("weld")
            weld_local_pos = get_position(weld_port.transform)  # (0, 0, -hub_length)

            # After rotation, the weld port offset from flange center is:
            rotated_weld_offset = flange_rotation[:3, :3] @ np.array(weld_local_pos)

            # The flange center should be positioned so weld port ends up at incoming position
            # weld_world = flange_center + rotated_weld_offset
            # Therefore: flange_center = incoming_position - rotated_weld_offset
            incoming_position = get_position(incoming_transform)
            flange_center = (
                incoming_position[0] - rotated_weld_offset[0],
                incoming_position[1] - rotated_weld_offset[1],
                incoming_position[2] - rotated_weld_offset[2],
            )
            flange_transform = translation_matrix(*flange_center) @ flange_rotation

        # Add geometry
        if fitting.shape is not None:
            shape = apply_transform_to_shape(fitting.shape, flange_transform)
            self.route.parts.append(shape)

        # Get the direction name for tracking (node.direction for root, derived for ending)
        if is_root_flange:
            direction_name = node.direction
        else:
            direction_name = vector_to_direction_name(incoming_direction)

        # Track component
        self._add_component(
            comp_type="flange",
            description="WN FLANGE",
            schedule_class=f"Class {node.flange_class}",
            shape=fitting.shape,
            transform=flange_transform,
            connected_directions=[direction_name],
        )

        # Track fitting and add coordinate frame markers if enabled
        self._add_fitting_with_markers(fitting, flange_transform)

        # Track weld at hub (only for starting flange - ending flange has no outgoing pipe)
        if is_root_flange:
            self._add_weld(
                weld_type="BW",
                description="FLANGE TO PIPE",
                fitting=fitting,
                port_name="weld",
                transform=flange_transform,
                pipe_direction=direction_name,
            )

        # Store transform on node
        node._world_transform = flange_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transform with weld gap offset
        weld_port = fitting.get_port("weld")
        outlet_transform = flange_transform @ weld_port.transform
        # Add weld gap - offset in the flow direction
        outlet_transform = offset_transform_by_gap(outlet_transform, direction)

        # Compute up-vector for this direction
        outlet_up = compute_initial_up_vector(direction)

        outlets = {"weld": (outlet_transform, direction, outlet_up)}

        # Recursively build children - flange passes "flange" as prev_fitting_type
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec, up_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec, up_vec, prev_fitting_type="flange")

        return outlets

    def _build_pipe(
        self,
        node: PipeSegmentNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
        prev_fitting_type: str = "pipe",
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build a pipe segment with centerline-based length calculation."""
        # Parse user-specified centerline distance
        centerline_dist_mm = parse_distance(node.length)

        # =====================================================================
        # CENTERLINE-BASED PIPE LENGTH CALCULATION
        # =====================================================================
        # Distances are measured from reference points:
        # - From flange: flange face (raised face) to fitting center
        # - From elbow/tee/cross: fitting center to fitting center
        # - To terminal: fitting center to pipe end

        # Determine start offset based on previous fitting type
        # Note: prev_fitting_type format is "type_weldtype" (e.g., "elbow_bw", "tee_branch_bw")
        INCH_TO_MM = 25.4
        # Parse prev_fitting_type: handle "tee_branch_bw" as type="tee_branch", weld="bw"
        if prev_fitting_type.startswith("tee_branch"):
            prev_type = "tee_branch"
            prev_weld = prev_fitting_type.split("_")[-1] if "_" in prev_fitting_type else "sw"
        else:
            prev_parts = prev_fitting_type.split("_")
            prev_type = prev_parts[0]
            prev_weld = prev_parts[1] if len(prev_parts) > 1 else "sw"

        if prev_type == "flange" and self.flange_dims is not None:
            # First segment: from flange raised face to next fitting center
            start_offset = (self.flange_dims.hub_length + self.flange_dims.raised_face_height) * INCH_TO_MM
        elif prev_type == "elbow":
            # From elbow center - use appropriate dimensions based on weld type
            # Note: This is for 90° elbows. 45° elbows would have different dimensions
            # but this is the "previous" fitting, and we use the default (90°) dimensions
            if prev_weld == "bw" and self.butt_weld_elbow_dims is not None:
                start_offset = self.butt_weld_elbow_dims.center_to_end_lr
            elif self.elbow_dims is not None:
                start_offset = self.elbow_dims.center_to_end
            else:
                start_offset = 0.0
        elif prev_type == "elbow45":
            # From 45° elbow center
            # Note: For 45° elbows, the center-to-end dimension is NOT along the pipe axis
            # at the outlet. The distance from center to outlet along the pipe axis is
            # approximately 0, so we use 0 offset here. The center-to-end dimension B
            # is measured perpendicular to the pipe axis.
            start_offset = 0.0
        elif prev_type == "tee":
            # From tee center (run direction)
            if prev_weld == "bw" and self.butt_weld_tee_dims is not None:
                start_offset = self.butt_weld_tee_dims.center_to_end_run
            elif self.tee_dims is not None:
                start_offset = self.tee_dims.center_to_end_run
            else:
                start_offset = 0.0
        elif prev_type == "tee_branch":
            # From tee center (branch direction)
            if prev_weld == "bw" and self.butt_weld_tee_dims is not None:
                start_offset = self.butt_weld_tee_dims.center_to_end_branch
            elif self.tee_dims is not None:
                start_offset = self.tee_dims.center_to_end_branch
            else:
                start_offset = 0.0
        elif prev_type == "cross" and self.cross_dims is not None:
            # From cross center
            start_offset = self.cross_dims.center_to_end
        else:
            # Default: no offset (e.g., pipe to pipe)
            start_offset = 0.0

        # Determine end offset based on what comes next
        next_node = node.children.get("end")
        if isinstance(next_node, ElbowNode):
            # Use appropriate elbow dimensions based on weld type and angle
            if next_node.angle == 45:
                # 45° elbow: the center-to-end dimension is NOT along the pipe axis
                # at the inlet. The distance from center to inlet along the pipe axis
                # is approximately 0, so we use 0 offset. The center-to-end dimension B
                # is measured perpendicular to the pipe axis.
                end_offset = 0.0
            else:
                # 90° elbow (default)
                if next_node.weld_type == "bw" and self.butt_weld_elbow_dims is not None:
                    end_offset = self.butt_weld_elbow_dims.center_to_end_lr
                elif self.elbow_dims is not None:
                    end_offset = self.elbow_dims.center_to_end
                else:
                    end_offset = 0.0
            gap_count = 2  # Gap at both ends
        elif isinstance(next_node, TeeNode):
            # Use appropriate tee dimensions based on weld type
            if next_node.weld_type == "bw" and self.butt_weld_tee_dims is not None:
                end_offset = self.butt_weld_tee_dims.center_to_end_run
            elif self.tee_dims is not None:
                end_offset = self.tee_dims.center_to_end_run
            else:
                end_offset = 0.0
            gap_count = 2
        elif isinstance(next_node, CrossNode) and self.cross_dims is not None:
            end_offset = self.cross_dims.center_to_end
            gap_count = 2
        elif isinstance(next_node, FlangeNode) and self.flange_dims is not None:
            # Pipe ends at flange - centerline is measured to flange face (raised face)
            INCH_TO_MM = 25.4
            end_offset = (self.flange_dims.hub_length + self.flange_dims.raised_face_height) * INCH_TO_MM
            gap_count = 2  # Gap at both ends
        elif isinstance(next_node, TerminalNode) or next_node is None:
            # Terminal: pipe extends to end
            end_offset = 0.0
            gap_count = 1  # Only gap at start
        else:
            end_offset = 0.0
            gap_count = 1

        # Calculate actual pipe length
        pipe_length = centerline_dist_mm - start_offset - end_offset - gap_count * EXPANSION_GAP_MM

        if pipe_length <= 0:
            raise ValueError(
                f"Centerline distance {centerline_dist_mm:.1f}mm is too short for fittings. "
                f"Minimum: {start_offset + end_offset + gap_count * EXPANSION_GAP_MM:.1f}mm"
            )

        # Create pipe with calculated length
        fitting = make_pipe(self.nps, pipe_length)

        # Mate pipe's start port to incoming transform
        # The incoming_transform is already at the mating position
        # Pipe start port faces opposite to incoming direction
        pipe_transform = self._compute_fitting_transform_for_pipe(fitting, "start", incoming_transform, incoming_direction)

        # Add geometry
        if fitting.shape is not None:
            shape = apply_transform_to_shape(fitting.shape, pipe_transform)
            self.route.parts.append(shape)

        # Track component
        self._add_component(
            comp_type="pipe",
            description="PIPE",
            schedule_class=f"Sch {node.schedule}",
            shape=fitting.shape,
            transform=pipe_transform,
            length_mm=pipe_length,
            pipe_direction=vector_to_direction_name(incoming_direction),
        )

        # Store transform on node
        node._world_transform = pipe_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transform (at pipe end) with weld gap offset
        end_port = fitting.get_port("end")
        outlet_transform = pipe_transform @ end_port.transform
        # Add weld gap - offset in the flow direction
        outlet_transform = offset_transform_by_gap(outlet_transform, incoming_direction)

        # Pipe doesn't change up-vector (no rotation around centerline)
        outlets = {"end": (outlet_transform, incoming_direction, up_vector)}

        # Handle weldolets on this pipe
        for weldolet in node.weldolets:
            self._build_weldolet_on_pipe(weldolet, node, pipe_transform, incoming_direction, up_vector)

        # Recursively build children - pipe passes "pipe" as prev_fitting_type
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec, up_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec, up_vec, prev_fitting_type="pipe")

        return outlets

    def _build_elbow(
        self,
        node: ElbowNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build an elbow fitting (90° or 45°)."""
        # The turn_direction from the node is the cardinal direction of the turn plane
        # For 90° elbows, this IS the output direction
        # For 45° elbows, the output direction is 45° between incoming and turn_direction
        turn_plane_direction = direction_name_to_vector(node.turn_direction)
        elbow_angle = node.angle  # 90 or 45

        if elbow_angle == 45:
            # Compute actual 45° output direction
            actual_turn_direction = compute_45_degree_turn_direction(
                incoming_direction, turn_plane_direction
            )
        else:
            # 90° elbow: output direction is the turn plane direction
            actual_turn_direction = turn_plane_direction

        # Create elbow based on weld type and angle
        if node.weld_type == "bw":
            if elbow_angle == 45:
                fitting = make_butt_weld_elbow_45_fitting(self.nps, units="mm")
            else:
                fitting = make_butt_weld_elbow_fitting(self.nps, units="mm")
            # Butt weld elbow has different port orientations, use dedicated transform
            elbow_transform = self._compute_butt_weld_elbow_transform(
                fitting, incoming_transform, incoming_direction, actual_turn_direction, up_vector,
                elbow_angle=elbow_angle
            )
        else:
            if elbow_angle == 45:
                fitting = make_socket_weld_elbow_45(self.nps, units="mm")
            else:
                fitting = make_socket_weld_elbow(self.nps, units="mm")
            # Socket weld elbow uses standard transform computation
            elbow_transform = self._compute_elbow_transform(
                fitting, incoming_transform, incoming_direction, actual_turn_direction, up_vector,
                elbow_angle=elbow_angle
            )

        # Add geometry
        if fitting.shape is not None:
            shape = apply_transform_to_shape(fitting.shape, elbow_transform)
            self.route.parts.append(shape)

        # Track component
        angle_desc = f"{elbow_angle}DEG ELBOW"
        self._add_component(
            comp_type="elbow",
            description=angle_desc,
            schedule_class="Class 3000 SW" if node.weld_type == "sw" else "BW",
            shape=fitting.shape,
            transform=elbow_transform,
            connected_directions=[
                vector_to_direction_name(incoming_direction),
                node.turn_direction,  # Use the specified turn plane for tracking
            ],
        )

        # Track fitting and add coordinate frame markers if enabled
        self._add_fitting_with_markers(fitting, elbow_transform)

        # Track welds - inlet always, outlet only if there's a child connected
        weld_type_str = "SW" if node.weld_type == "sw" else "BW"
        incoming_dir_name = vector_to_direction_name(incoming_direction)
        turn_dir_name = node.turn_direction  # Use specified turn plane for description
        actual_turn_dir_name = vector_to_direction_name(actual_turn_direction)

        # Inlet weld: avoid direction is where the elbow goes (turn direction)
        self._add_weld(
            weld_type=weld_type_str,
            description="PIPE TO ELBOW",
            fitting=fitting,
            port_name="inlet",
            transform=elbow_transform,
            pipe_direction=incoming_dir_name,
            avoid_direction=turn_dir_name,
        )

        # Outlet weld: only add if there's a pipe/fitting connected to the outlet
        # (no weld needed if elbow is at the end of a run with no continuation)
        if "outlet" in node.children and node.children["outlet"] is not None:
            opposite_incoming = vector_to_direction_name(
                (-incoming_direction[0], -incoming_direction[1], -incoming_direction[2])
            )
            self._add_weld(
                weld_type=weld_type_str,
                description="ELBOW TO PIPE",
                fitting=fitting,
                port_name="outlet",
                transform=elbow_transform,
                pipe_direction=actual_turn_dir_name,  # Use actual output direction
                avoid_direction=opposite_incoming,
            )

        # Store transform on node
        node._world_transform = elbow_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transform with weld gap offset
        outlet_port = fitting.get_port("outlet")
        outlet_transform = elbow_transform @ outlet_port.transform
        # Add weld gap - offset in the actual turn direction
        outlet_transform = offset_transform_by_gap(outlet_transform, actual_turn_direction)

        # Compute new up-vector after the elbow turn
        outlet_up = compute_up_vector_after_elbow(incoming_direction, actual_turn_direction, up_vector)

        outlets = {"outlet": (outlet_transform, actual_turn_direction, outlet_up)}

        # Recursively build children - elbow passes "elbow_{weld_type}" or "elbow45_{weld_type}" as prev_fitting_type
        if node.angle == 45:
            elbow_fitting_type = f"elbow45_{node.weld_type}"
        else:
            elbow_fitting_type = f"elbow_{node.weld_type}"
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec, up_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec, up_vec, prev_fitting_type=elbow_fitting_type)

        return outlets

    def _build_tee(
        self,
        node: TeeNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build a tee fitting."""
        # Create tee based on weld type
        if node.weld_type == "bw":
            fitting = make_butt_weld_tee_fitting(self.nps, units="mm")
            tee_shape = fitting.shape
        else:
            # Create socket weld tee shape
            tee_shape = make_socket_weld_tee(self.nps)
            # Create a Fitting object for the tee
            fitting = self._create_tee_fitting(self.nps, tee_shape)

        if node.enter_via_branch:
            return self._build_tee_branch_entry(
                node, fitting, tee_shape, incoming_transform, incoming_direction, up_vector
            )
        else:
            return self._build_tee_standard(
                node, fitting, tee_shape, incoming_transform, incoming_direction, up_vector
            )

    def _build_tee_standard(
        self,
        node: TeeNode,
        fitting: Fitting,
        tee_shape,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build a tee in standard mode (incoming connects to inlet)."""
        branch_direction = direction_name_to_vector(node.branch_direction)

        # Compute tee transform with up-vector awareness
        tee_transform = self._compute_tee_transform(
            fitting, incoming_transform, incoming_direction, branch_direction, up_vector,
            weld_type=node.weld_type
        )

        # Add geometry
        shape = apply_transform_to_shape(tee_shape, tee_transform)
        self.route.parts.append(shape)

        # Track component
        self._add_component(
            comp_type="tee",
            description="TEE",
            schedule_class="Class 3000 SW" if node.weld_type == "sw" else "BW",
            shape=tee_shape,
            transform=tee_transform,
            connected_directions=[
                vector_to_direction_name(incoming_direction),
                vector_to_direction_name(incoming_direction),  # run continues same
                node.branch_direction,
            ],
        )

        # Track fitting and add coordinate frame markers if enabled
        self._add_fitting_with_markers(fitting, tee_transform)

        # Track welds at all three ports
        weld_type_str = "SW" if node.weld_type == "sw" else "BW"
        incoming_dir_name = vector_to_direction_name(incoming_direction)
        branch_dir_name = node.branch_direction

        # Inlet weld: avoid both run and branch directions
        self._add_weld(
            weld_type=weld_type_str,
            description="PIPE TO TEE",
            fitting=fitting,
            port_name="inlet",
            transform=tee_transform,
            pipe_direction=incoming_dir_name,
            avoid_direction=branch_dir_name,  # Avoid branch side
        )

        # Run weld: avoid incoming and branch directions
        opposite_incoming = vector_to_direction_name(
            (-incoming_direction[0], -incoming_direction[1], -incoming_direction[2])
        )
        self._add_weld(
            weld_type=weld_type_str,
            description="TEE TO PIPE",
            fitting=fitting,
            port_name="run",
            transform=tee_transform,
            pipe_direction=incoming_dir_name,  # Run continues same direction
            avoid_direction=opposite_incoming,  # Avoid inlet side
        )

        # Branch weld: avoid run direction
        self._add_weld(
            weld_type=weld_type_str,
            description="TEE BRANCH TO PIPE",
            fitting=fitting,
            port_name="branch",
            transform=tee_transform,
            pipe_direction=branch_dir_name,
            avoid_direction=incoming_dir_name,  # Avoid run direction
        )

        # Store transform on node
        node._world_transform = tee_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transforms with weld gap offsets
        run_port = fitting.get_port("run")
        branch_port = fitting.get_port("branch")

        run_outlet_transform = tee_transform @ run_port.transform
        branch_outlet_transform = tee_transform @ branch_port.transform
        # Add weld gaps - offset in the respective flow directions
        run_outlet_transform = offset_transform_by_gap(run_outlet_transform, incoming_direction)
        branch_outlet_transform = offset_transform_by_gap(branch_outlet_transform, branch_direction)

        # Compute up-vectors for each outlet
        # Run continues in same direction, up-vector unchanged
        run_up = up_vector
        # Branch goes in a different direction, compute new up-vector
        branch_up = compute_up_vector_for_branch(incoming_direction, branch_direction, up_vector)

        outlets = {
            "run": (run_outlet_transform, incoming_direction, run_up),
            "branch": (branch_outlet_transform, branch_direction, branch_up),
        }

        # Recursively build children - tee passes "tee_{weld_type}" or "tee_branch_{weld_type}"
        # Save NPS state before building branches - each branch should start with the same NPS,
        # regardless of whether a sibling branch contains a reducer that changes the NPS.
        saved_nps_state = self._save_nps_state()
        tee_run_type = f"tee_{node.weld_type}"
        tee_branch_type = f"tee_branch_{node.weld_type}"
        for port_name, child in node.children.items():
            if port_name in outlets:
                # Restore NPS state before building each branch
                self._restore_nps_state(saved_nps_state)
                transform, dir_vec, up_vec = outlets[port_name]
                # Use different fitting type for branch vs run
                fitting_type = tee_branch_type if port_name == "branch" else tee_run_type
                self._build_node(child, transform, dir_vec, up_vec, prev_fitting_type=fitting_type)

        return outlets

    def _build_tee_branch_entry(
        self,
        node: TeeNode,
        fitting: Fitting,
        tee_shape,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build a tee in branch entry mode (incoming connects to branch port)."""
        run_direction = direction_name_to_vector(node.run_direction)
        opposite_run = (-run_direction[0], -run_direction[1], -run_direction[2])

        # Compute tee transform for branch entry
        tee_transform = self._compute_tee_transform_branch_entry(
            fitting, incoming_transform, incoming_direction, run_direction, up_vector,
            weld_type=node.weld_type
        )

        # Add geometry
        shape = apply_transform_to_shape(tee_shape, tee_transform)
        self.route.parts.append(shape)

        # Track component
        run_dir_name = node.run_direction
        opposite_run_name = vector_to_direction_name(opposite_run)
        self._add_component(
            comp_type="tee",
            description="TEE",
            schedule_class="Class 3000 SW" if node.weld_type == "sw" else "BW",
            shape=tee_shape,
            transform=tee_transform,
            connected_directions=[
                vector_to_direction_name(incoming_direction),  # branch inlet
                run_dir_name,  # run_a
                opposite_run_name,  # run_b
            ],
        )

        # Track fitting and add coordinate frame markers if enabled
        self._add_fitting_with_markers(fitting, tee_transform)

        # Track welds at all three ports
        weld_type_str = "SW" if node.weld_type == "sw" else "BW"
        incoming_dir_name = vector_to_direction_name(incoming_direction)

        # Branch weld (inlet in this mode): avoid run directions
        self._add_weld(
            weld_type=weld_type_str,
            description="PIPE TO TEE",
            fitting=fitting,
            port_name="branch",
            transform=tee_transform,
            pipe_direction=incoming_dir_name,
            avoid_direction=run_dir_name,
        )

        # Inlet port weld (run_b outlet): avoid branch direction
        self._add_weld(
            weld_type=weld_type_str,
            description="TEE TO PIPE",
            fitting=fitting,
            port_name="inlet",
            transform=tee_transform,
            pipe_direction=opposite_run_name,
            avoid_direction=incoming_dir_name,
        )

        # Run port weld (run_a outlet): avoid branch direction
        self._add_weld(
            weld_type=weld_type_str,
            description="TEE TO PIPE",
            fitting=fitting,
            port_name="run",
            transform=tee_transform,
            pipe_direction=run_dir_name,
            avoid_direction=incoming_dir_name,
        )

        # Store transform on node
        node._world_transform = tee_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transforms with weld gap offsets
        # In branch entry mode: inlet port becomes run_b, run port becomes run_a
        inlet_port = fitting.get_port("inlet")
        run_port = fitting.get_port("run")

        # run_a goes in run_direction (from the "run" port)
        run_a_outlet_transform = tee_transform @ run_port.transform
        run_a_outlet_transform = offset_transform_by_gap(run_a_outlet_transform, run_direction)

        # run_b goes opposite to run_direction (from the "inlet" port)
        run_b_outlet_transform = tee_transform @ inlet_port.transform
        run_b_outlet_transform = offset_transform_by_gap(run_b_outlet_transform, opposite_run)

        # Compute up-vectors for each outlet
        # Both run outlets are perpendicular to incoming, compute new up-vectors
        run_a_up = compute_up_vector_for_branch(incoming_direction, run_direction, up_vector)
        run_b_up = compute_up_vector_for_branch(incoming_direction, opposite_run, up_vector)

        outlets = {
            "run_a": (run_a_outlet_transform, run_direction, run_a_up),
            "run_b": (run_b_outlet_transform, opposite_run, run_b_up),
        }

        # Recursively build children
        # Save NPS state before building branches - each branch should start with the same NPS.
        saved_nps_state = self._save_nps_state()
        tee_run_type = f"tee_{node.weld_type}"
        for port_name, child in node.children.items():
            if port_name in outlets:
                # Restore NPS state before building each branch
                self._restore_nps_state(saved_nps_state)
                transform, dir_vec, up_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec, up_vec, prev_fitting_type=tee_run_type)

        return outlets

    def _build_cross(
        self,
        node: CrossNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build a cross fitting."""
        orientation = direction_name_to_vector(node.orientation)

        # Create cross shape
        cross_shape = make_socket_weld_cross(self.nps)

        # Create a Fitting object for the cross
        fitting = self._create_cross_fitting(self.nps, cross_shape)

        # Compute perpendicular directions for branches
        left_dir, right_dir = compute_perpendicular_directions(incoming_direction, orientation)

        # Compute cross transform with up-vector awareness
        cross_transform = self._compute_cross_transform(
            fitting, incoming_transform, incoming_direction, left_dir, up_vector
        )

        # Add geometry
        shape = apply_transform_to_shape(cross_shape, cross_transform)
        self.route.parts.append(shape)

        # Track component
        self._add_component(
            comp_type="cross",
            description="CROSS",
            schedule_class="Class 3000 SW" if node.weld_type == "sw" else "BW",
            shape=cross_shape,
            transform=cross_transform,
            connected_directions=[
                vector_to_direction_name(incoming_direction),
                vector_to_direction_name(incoming_direction),  # run
                vector_to_direction_name(left_dir),
                vector_to_direction_name(right_dir),
            ],
        )

        # Track fitting and add coordinate frame markers if enabled
        self._add_fitting_with_markers(fitting, cross_transform)

        # Store transform on node
        node._world_transform = cross_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transforms with weld gap offsets
        run_port = fitting.get_port("run")
        left_port = fitting.get_port("branch_left")
        right_port = fitting.get_port("branch_right")

        run_outlet_transform = cross_transform @ run_port.transform
        left_outlet_transform = cross_transform @ left_port.transform
        right_outlet_transform = cross_transform @ right_port.transform
        # Add weld gaps - offset in the respective flow directions
        run_outlet_transform = offset_transform_by_gap(run_outlet_transform, incoming_direction)
        left_outlet_transform = offset_transform_by_gap(left_outlet_transform, left_dir)
        right_outlet_transform = offset_transform_by_gap(right_outlet_transform, right_dir)

        # Compute up-vectors for each outlet
        run_up = up_vector  # Run continues same
        left_up = compute_up_vector_for_branch(incoming_direction, left_dir, up_vector)
        right_up = compute_up_vector_for_branch(incoming_direction, right_dir, up_vector)

        outlets = {
            "run": (run_outlet_transform, incoming_direction, run_up),
            "branch_left": (left_outlet_transform, left_dir, left_up),
            "branch_right": (right_outlet_transform, right_dir, right_up),
        }

        # Recursively build children - cross passes "cross" as prev_fitting_type
        # Save NPS state before building branches - each branch should start with the same NPS.
        saved_nps_state = self._save_nps_state()
        for port_name, child in node.children.items():
            if port_name in outlets:
                # Restore NPS state before building each branch
                self._restore_nps_state(saved_nps_state)
                transform, dir_vec, up_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec, up_vec, prev_fitting_type="cross")

        return outlets

    def _build_weldolet(
        self,
        node: WeldoletNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build a standalone weldolet (not attached to a pipe segment)."""
        # This is typically called when a weldolet is not attached to a pipe
        # In practice, weldolets should be in the pipe's weldolets list
        branch_direction = direction_name_to_vector(node.branch_direction)

        # Compute up-vector for the branch
        branch_up = compute_up_vector_for_branch(incoming_direction, branch_direction, up_vector)

        outlets = {"branch": (incoming_transform, branch_direction, branch_up)}

        # Recursively build children - weldolet passes "weldolet" as prev_fitting_type
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec, up_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec, up_vec, prev_fitting_type="weldolet")

        return outlets

    def _build_weldolet_on_pipe(
        self,
        node: WeldoletNode,
        pipe_node: PipeSegmentNode,
        pipe_transform: np.ndarray,
        pipe_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> None:
        """Build a weldolet attached to a pipe segment."""
        _ = pipe_node  # Unused but kept for API compatibility
        # Get position along pipe
        position_mm = parse_distance(node.position_along_pipe)

        # Get branch direction
        branch_direction = direction_name_to_vector(node.branch_direction)

        # Calculate world position of weldolet center
        # Position is measured from pipe start along pipe direction
        dir_vec = np.array(pipe_direction)
        pipe_start = get_position(pipe_transform)

        weldolet_center = (
            pipe_start[0] + position_mm * dir_vec[0],
            pipe_start[1] + position_mm * dir_vec[1],
            pipe_start[2] + position_mm * dir_vec[2],
        )

        # Create transform for the weldolet outlet
        # Branch extends in branch_direction from weldolet_center
        outlet_rotation = rotation_to_align_z_with_direction(branch_direction)
        outlet_transform = translation_matrix(*weldolet_center) @ outlet_rotation

        # Track component (simplified - actual weldolet geometry TBD)
        self._add_component(
            comp_type="weldolet",
            description=f"{node.fitting_type.upper()}",
            schedule_class="",
            shape=None,
            transform=outlet_transform,
            connected_directions=[node.branch_direction],
        )

        # Store transform on node
        node._world_transform = outlet_transform
        node._built = True

        # Compute up-vector for the branch
        branch_up = compute_up_vector_for_branch(pipe_direction, branch_direction, up_vector)

        # Build children from the weldolet branch - passes "weldolet" as prev_fitting_type
        # Add weld gap offset in the branch direction
        outlet_with_gap = offset_transform_by_gap(outlet_transform, branch_direction)
        outlets = {"branch": (outlet_with_gap, branch_direction, branch_up)}
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec, up_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec, up_vec, prev_fitting_type="weldolet")

    def _build_reducer(
        self,
        node: ReducerNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build a concentric reducer fitting.

        The reducer changes pipe size from current NPS to target NPS.
        Supports both contraction (large to small) and expansion (small to large).
        Only butt weld reducers are supported (as is standard for reducers).
        """
        current_nps = self.nps
        target_nps = node.target_nps

        # Validate and determine direction (contraction or expansion)
        # This will raise a helpful error if the combination doesn't exist
        large_nps, small_nps, is_expansion = validate_reducer(current_nps, target_nps)

        # Get dimensions (for reference, mainly used in transform computation)
        reducer_dims = get_butt_weld_reducer_dims(large_nps, small_nps)

        # Create reducer fitting with appropriate port orientation
        # For expansion, ports are swapped (inlet at small end, outlet at large end)
        fitting = make_butt_weld_reducer_fitting(
            large_nps, small_nps, units="mm", reversed=is_expansion
        )

        # Compute reducer transform
        # The reducer's local coordinate has large end at X=0, small end at X=H
        # For contraction: align local +X with incoming_direction
        # For expansion: align local -X with incoming_direction (flip the reducer)
        reducer_transform = self._compute_reducer_transform(
            fitting, incoming_transform, incoming_direction, up_vector, is_expansion
        )

        # Add geometry
        if fitting.shape is not None:
            shape = apply_transform_to_shape(fitting.shape, reducer_transform)
            self.route.parts.append(shape)

        # Track component with dual NPS in description
        self._add_component(
            comp_type="reducer",
            description=f"CONC REDUCER {large_nps}\" x {small_nps}\"",
            schedule_class="BW",
            shape=fitting.shape,
            transform=reducer_transform,
            connected_directions=[
                vector_to_direction_name(incoming_direction),
                vector_to_direction_name(incoming_direction),  # Same direction, different size
            ],
        )

        # Track fitting and add coordinate frame markers if enabled
        self._add_fitting_with_markers(fitting, reducer_transform)

        # Track welds at both ends
        incoming_dir_name = vector_to_direction_name(incoming_direction)
        opposite_dir = (-incoming_direction[0], -incoming_direction[1], -incoming_direction[2])
        opposite_dir_name = vector_to_direction_name(opposite_dir)

        # Inlet weld (large end): avoid direction is opposite (where outlet is)
        self._add_weld(
            weld_type="BW",
            description="PIPE TO REDUCER",
            fitting=fitting,
            port_name="inlet",
            transform=reducer_transform,
            pipe_direction=incoming_dir_name,
            avoid_direction=opposite_dir_name,
        )

        # Outlet weld (small end): only add if there's a child connected
        if "outlet" in node.children and node.children["outlet"] is not None:
            self._add_weld(
                weld_type="BW",
                description="REDUCER TO PIPE",
                fitting=fitting,
                port_name="outlet",
                transform=reducer_transform,
                pipe_direction=incoming_dir_name,
                avoid_direction=incoming_dir_name,
            )

        # Store transform on node
        node._world_transform = reducer_transform
        node._fitting = fitting
        node._built = True

        # UPDATE NPS for downstream components
        # Always use target_nps (works for both contraction and expansion)
        self.nps = target_nps
        self.pipe_spec = PIPE_SPECS.get(target_nps)
        # Update dimension lookups for new size
        self._update_dimension_lookups_for_nps(target_nps)

        # Compute outlet transform with weld gap offset
        outlet_port = fitting.get_port("outlet")
        outlet_transform = reducer_transform @ outlet_port.transform
        # Add weld gap in the flow direction
        outlet_transform = offset_transform_by_gap(outlet_transform, incoming_direction)

        outlets = {"outlet": (outlet_transform, incoming_direction, up_vector)}

        # Recursively build children with NEW NPS
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec, up_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec, up_vec, prev_fitting_type="reducer")

        return outlets

    def _compute_reducer_transform(
        self,
        fitting: Fitting,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
        is_expansion: bool = False,
    ) -> np.ndarray:
        """
        Compute the world transform for a concentric reducer.

        The reducer's local coordinate has:
        - Large end at X=0
        - Small end at X=H
        - Centerline along +X

        For contraction: align local +X with flow direction
          - Large end (X=0) at inlet, small end (X=H) at outlet
        For expansion: align local -X with flow direction (flip geometry)
          - Small end (X=H) at inlet, large end (X=0) at outlet

        We position the inlet port at the incoming_transform position.
        """
        # For contraction: local +X = flow direction
        # For expansion: local +X = opposite of flow direction (to flip the reducer)
        if is_expansion:
            inc = np.array([-incoming_direction[0], -incoming_direction[1], -incoming_direction[2]], dtype=float)
        else:
            inc = np.array(incoming_direction, dtype=float)

        # Use provided up-vector for consistent orientation
        up = np.array(up_vector, dtype=float)

        # If incoming direction is nearly parallel to up, use a different reference
        if abs(np.dot(inc, up)) > 0.99:
            # Use a perpendicular reference
            up = np.array([0.0, 1.0, 0.0]) if abs(inc[1]) < 0.99 else np.array([1.0, 0.0, 0.0])

        # Build rotation matrix: X = incoming_direction, Z from up-vector
        # Y = Z × X (to complete right-hand system)
        # Z = X × Y (orthonormalize)
        y_vec = np.cross(up, inc)
        y_norm = np.linalg.norm(y_vec)
        if y_norm > 1e-6:
            y_vec = y_vec / y_norm
        else:
            # Fallback if somehow parallel
            y_vec = np.array([0.0, 1.0, 0.0])

        z_vec = np.cross(inc, y_vec)
        z_vec = z_vec / np.linalg.norm(z_vec)

        # Build rotation matrix
        rotation = np.eye(4)
        rotation[:3, 0] = inc      # X-axis = flow direction
        rotation[:3, 1] = y_vec    # Y-axis
        rotation[:3, 2] = z_vec    # Z-axis (should be close to up)

        # Get the inlet port's local position
        # For normal mode: inlet at (0, 0, 0)
        # For reversed mode: inlet at (H, 0, 0)
        inlet_port = fitting.get_port("inlet")
        inlet_local_pos = get_position(inlet_port.transform)

        # World position where inlet should be
        inlet_world_pos = get_position(incoming_transform)

        # The fitting's local origin needs to be placed such that after rotation,
        # the inlet port ends up at inlet_world_pos
        # local_inlet_in_world = rotation @ local_inlet_pos
        # So origin_in_world = inlet_world_pos - rotation @ local_inlet_pos
        rotated_inlet_offset = rotation[:3, :3] @ inlet_local_pos
        origin_world_pos = inlet_world_pos - rotated_inlet_offset

        reducer_transform = translation_matrix(*origin_world_pos) @ rotation

        return reducer_transform

    def _update_dimension_lookups_for_nps(self, new_nps: str) -> None:
        """Update dimension table references after a reducer changes the NPS."""
        # Update socket weld dimension lookups
        self.elbow_dims = ASME_B1611_ELBOW90_CLASS3000.get(new_nps)
        self.elbow_45_dims = ASME_B1611_ELBOW45_CLASS3000.get(new_nps)
        self.tee_dims = ASME_B1611_TEE_CLASS3000.get(new_nps)
        self.cross_dims = ASME_B1611_CROSS_CLASS3000.get(new_nps)

        # Update butt weld dimension lookups
        self.butt_weld_elbow_dims = ASME_B169_ELBOW90.get(new_nps)
        self.butt_weld_elbow_45_dims = ASME_B169_ELBOW45.get(new_nps)
        self.butt_weld_tee_dims = ASME_B169_TEE.get(new_nps)

        # Update flange dimensions
        self.flange_dims = ASME_B165_CLASS300_WN.get(new_nps)

    def _build_terminal(
        self,
        node: TerminalNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float], tuple[float, float, float]]]:
        """Build a terminal node (cap, blind flange, or open end)."""
        _ = up_vector  # Unused but kept for API consistency
        node._world_transform = incoming_transform
        node._built = True

        if node.end_type == "cap":
            self._add_component(
                comp_type="cap",
                description="CAP",
                schedule_class="",
                shape=None,
                transform=incoming_transform,
                connected_directions=[vector_to_direction_name(incoming_direction)],
            )
        elif node.end_type == "blind_flange":
            self._add_component(
                comp_type="blind_flange",
                description="BLIND FLANGE",
                schedule_class=f"Class {self.route.flange_class}",
                shape=None,
                transform=incoming_transform,
                connected_directions=[vector_to_direction_name(incoming_direction)],
            )

        return {}  # No outlets

    # -------------------------------------------------------------------------
    # FITTING CREATION HELPERS
    # -------------------------------------------------------------------------

    def _create_tee_fitting(self, nps: str, shape: cq.Shape) -> Fitting:
        """Create a Fitting object for a tee with proper ports."""
        dims = self.tee_dims
        if dims is None:
            raise ValueError(f"No tee dimensions for NPS {nps}")

        C = dims.center_to_end_run
        M = dims.center_to_end_branch

        # Create ports for the tee
        # Port convention: Z-axis points OUTWARD toward where connecting pipe body extends
        #
        # Inlet: at -X side of tee, Z points -X (toward incoming pipe body)
        inlet_transform = translation_matrix(-C, 0, 0) @ rotation_matrix_y(-90)

        # Run: at +X side of tee, Z points +X (toward outgoing pipe body)
        run_transform = translation_matrix(C, 0, 0) @ rotation_matrix_y(90)

        # Branch: at +Y side of tee, Z points +Y (toward branch pipe body)
        branch_transform = translation_matrix(0, M, 0) @ rotation_matrix_x(-90)

        ports = {
            "inlet": Port("inlet", inlet_transform),
            "run": Port("run", run_transform),
            "branch": Port("branch", branch_transform),
        }

        return Fitting(nps=nps, shape=shape, ports=ports)

    def _create_cross_fitting(self, nps: str, shape: cq.Shape) -> Fitting:
        """Create a Fitting object for a cross with proper ports."""
        dims = self.cross_dims
        if dims is None:
            raise ValueError(f"No cross dimensions for NPS {nps}")

        C = dims.center_to_end

        # Create ports for the cross
        # Port convention: Z-axis points OUTWARD toward where connecting pipe body extends
        #
        # Inlet: at -X side of cross, Z points -X (toward incoming pipe body)
        inlet_transform = translation_matrix(-C, 0, 0) @ rotation_matrix_y(-90)

        # Run: at +X side of cross, Z points +X (toward outgoing pipe body)
        run_transform = translation_matrix(C, 0, 0) @ rotation_matrix_y(90)

        # Branch Left: at +Y side of cross, Z points +Y (toward left branch pipe body)
        left_transform = translation_matrix(0, C, 0) @ rotation_matrix_x(-90)

        # Branch Right: at -Y side of cross, Z points -Y (toward right branch pipe body)
        right_transform = translation_matrix(0, -C, 0) @ rotation_matrix_x(90)

        ports = {
            "inlet": Port("inlet", inlet_transform),
            "run": Port("run", run_transform),
            "branch_left": Port("branch_left", left_transform),
            "branch_right": Port("branch_right", right_transform),
        }

        return Fitting(nps=nps, shape=shape, ports=ports)

    # -------------------------------------------------------------------------
    # TRANSFORM COMPUTATION HELPERS
    # -------------------------------------------------------------------------

    def _compute_fitting_transform_for_pipe(
        self,
        fitting: Fitting,
        inlet_port_name: str,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
    ) -> np.ndarray:
        """Compute transform for a pipe segment."""
        # Mark unused parameters (kept for API consistency)
        _, _ = fitting, inlet_port_name

        # Pipe is created along Z axis, start at Z=0, end at Z=length
        # Start port faces -Z (toward previous fitting)
        # End port faces +Z (toward next fitting)
        #
        # incoming_direction is the direction the pipe should EXTEND
        # So we need +Z to point in incoming_direction
        rotation = rotation_to_align_z_with_direction(incoming_direction)

        # Position at incoming_transform origin
        position = get_position(incoming_transform)
        pipe_transform = translation_matrix(*position) @ rotation

        return pipe_transform

    def _compute_elbow_transform(
        self,
        fitting: Fitting,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        turn_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
        elbow_angle: int = 90,
    ) -> np.ndarray:
        """
        Compute transform for an elbow with up-vector awareness.

        The elbow's Z-axis (normal to the plane of the elbow) should be
        consistent with the up-vector when possible.

        Args:
            fitting: The elbow fitting
            incoming_transform: Transform at the inlet position
            incoming_direction: Direction the pipe was traveling before the elbow
            turn_direction: Direction the pipe will travel after the elbow
            up_vector: Reference up direction
            elbow_angle: Elbow angle (90 or 45 degrees)
        """
        _ = fitting  # Unused but kept for API consistency

        # Elbow default orientation: inlet from +X, outlet toward +Y (for 90°)
        # For 45° elbow: inlet from +X, outlet toward 45° between +X and +Y
        # The elbow lies in the XY plane, with Z perpendicular to it

        # Compute the elbow's Z-axis as the cross product of inlet and outlet directions
        inc = np.array(incoming_direction)
        out = np.array(turn_direction)
        up = np.array(up_vector)

        # The plane of the elbow contains both pipe directions
        # Z-axis is perpendicular to this plane
        z_vec = np.cross(inc, out)
        z_norm = np.linalg.norm(z_vec)

        if z_norm < 1e-6:
            # Directions are parallel (shouldn't happen for elbows)
            # Fall back to up-vector as Z
            z_vec = up
        else:
            z_vec = z_vec / z_norm
            # Ensure Z is in the "up" hemisphere when possible
            if np.dot(z_vec, up) < 0:
                z_vec = -z_vec

        # The inlet faces opposite to incoming direction (toward the pipe)
        inlet_face = (-inc[0], -inc[1], -inc[2])
        # The outlet faces the turn direction (toward the next pipe)
        outlet_face = out

        # Build rotation matrix:
        # X column: inlet direction (where inlet Z points)
        # Y column: outlet direction (where outlet Z points)
        # Z column: perpendicular to both (normal to elbow plane)
        rotation = np.eye(4)
        rotation[:3, 0] = inlet_face
        rotation[:3, 1] = outlet_face
        rotation[:3, 2] = z_vec

        # Position elbow so inlet port is at incoming_transform
        # Get the appropriate elbow dimensions based on angle
        if elbow_angle == 45:
            dims = self.elbow_45_dims
            if dims is None:
                raise ValueError(f"No 45° elbow dimensions for NPS {self.nps}")
        else:
            dims = self.elbow_dims
            if dims is None:
                raise ValueError(f"No elbow dimensions for NPS {self.nps}")

        A = dims.center_to_end  # Center to socket end

        # Elbow center is A distance along incoming direction from inlet position
        inlet_pos = get_position(incoming_transform)
        center_pos = (
            inlet_pos[0] + A * incoming_direction[0],
            inlet_pos[1] + A * incoming_direction[1],
            inlet_pos[2] + A * incoming_direction[2],
        )

        elbow_transform = translation_matrix(*center_pos) @ rotation

        return elbow_transform

    def _compute_butt_weld_elbow_transform(
        self,
        fitting: Fitting,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        turn_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
        elbow_angle: int = 90,
    ) -> np.ndarray:
        """
        Compute transform for a butt weld elbow.

        Butt weld elbow geometry:
        - Arc centered at origin in the XY plane
        - Inlet at (bend_radius, 0, 0) with tangent +Y
        - For 90°: outlet at (0, bend_radius, 0) with tangent -X
        - For 45°: outlet at (R*cos45, R*sin45, 0) with tangent (-sin45, cos45, 0)

        Port orientations (Z points toward connecting pipe body):
        - Inlet port Z = -Y (toward incoming pipe body)
        - Outlet port Z = tangent direction at outlet

        For world transform, we need to rotate so that:
        - Local +Y (inlet tangent/flow) maps to world incoming_direction
        - Local XY plane (bend plane) contains both incoming and turn directions

        Args:
            fitting: The elbow fitting
            incoming_transform: Transform at the inlet position
            incoming_direction: Direction the pipe was traveling before the elbow
            turn_direction: Direction the pipe will travel after the elbow (for 45°,
                           this is the actual 45° output direction)
            up_vector: Reference up direction
            elbow_angle: Elbow angle (90 or 45 degrees)
        """
        _ = fitting  # Unused but kept for API consistency

        inc = np.array(incoming_direction)
        out = np.array(turn_direction)
        up = np.array(up_vector)

        # The elbow lies in the XY plane locally:
        # - Local +Y is the inlet tangent (flow direction into elbow)
        # - Local +X points toward the bend center from the inlet
        #   (for CCW arc from +X toward +Y, the bend center is at origin,
        #    so at inlet position (R,0,0), the center is in -X direction)
        #
        # We want:
        # - Local +Y → world incoming_direction
        # - Local bend plane (XY) → world plane containing incoming and turn directions
        #
        # The elbow bends from +X toward +Y. In world coordinates, this corresponds
        # to bending from the turn_direction side toward incoming_direction side.
        #
        # For a 90° elbow: turn_direction is perpendicular to incoming_direction
        # For a 45° elbow: turn_direction is 45° from incoming_direction

        # Build rotation matrix:
        # col1 (where local Y goes) = incoming_direction (inlet tangent)
        col1 = inc / np.linalg.norm(inc)

        # col0 (where local X goes) should be perpendicular to col1 and in the bend plane
        # The bend plane contains both incoming and turn directions
        # Local +X points "toward the turn side" from inlet perspective
        #
        # For 90° elbow: local -X maps to turn_direction
        # For 45° elbow: local outlet tangent (-sin45, cos45, 0) maps to turn_direction
        #
        # To find col0: it should be in the plane of inc and out, perpendicular to inc
        # Project out onto the plane perpendicular to inc, then normalize
        out_parallel = np.dot(out, col1) * col1
        out_perp = out - out_parallel  # Component of out perpendicular to inc

        if np.linalg.norm(out_perp) < 1e-6:
            # inc and out are parallel (shouldn't happen for valid elbow)
            # Use up vector as fallback
            out_perp = up - np.dot(up, col1) * col1

        out_perp = out_perp / np.linalg.norm(out_perp)

        # Local +X should point opposite to the turn direction component perpendicular to incoming
        # (because the arc goes from +X toward +Y, so outlet is "toward +Y" from center)
        # For CCW arc: bend center is at -X from inlet, outlet is at angle θ
        # When θ=90°, outlet tangent is -X. When θ=45°, outlet tangent is at 135° from +X.
        col0 = -out_perp

        # col2 is perpendicular to both (right-hand rule)
        z_vec = np.cross(col0, col1)
        z_norm = np.linalg.norm(z_vec)

        if z_norm < 1e-6:
            z_vec = up
        else:
            z_vec = z_vec / z_norm

        rotation = np.eye(4)
        rotation[:3, 0] = col0
        rotation[:3, 1] = col1
        rotation[:3, 2] = z_vec

        # Get butt weld elbow dimensions based on angle
        # Note: For positioning, we need the inlet offset from elbow center,
        # which is the bend radius (NOT the center-to-end dimension for 45° elbows)
        if elbow_angle == 45:
            dims = self.butt_weld_elbow_45_dims
            if dims is None:
                raise ValueError(f"No butt weld 45° elbow dimensions for NPS {self.nps}")
            # For 45° elbow: inlet is at local (bend_radius, 0, 0)
            # bend_radius = B / tan(22.5°) where B = center_to_end
            import math
            B = dims.center_to_end
            inlet_offset = B / math.tan(math.radians(22.5))  # This is the bend radius
        else:
            dims = self.butt_weld_elbow_dims
            if dims is None:
                raise ValueError(f"No butt weld elbow dimensions for NPS {self.nps}")
            # For 90° elbow: inlet at local (A, 0, 0) where A = center_to_end_lr = bend radius
            inlet_offset = dims.center_to_end_lr

        # Position elbow so inlet is at incoming_transform position
        # Butt weld elbow: inlet at local (inlet_offset, 0, 0)
        # After rotation R, inlet offset from center = inlet_offset * col0
        # So: inlet_world = center + inlet_offset * col0
        # We want: inlet_world = inlet_pos
        # Therefore: center = inlet_pos - inlet_offset * col0
        inlet_pos = get_position(incoming_transform)

        center_pos = (
            inlet_pos[0] - inlet_offset * col0[0],
            inlet_pos[1] - inlet_offset * col0[1],
            inlet_pos[2] - inlet_offset * col0[2],
        )

        elbow_transform = translation_matrix(*center_pos) @ rotation

        return elbow_transform

    def _compute_tee_transform(
        self,
        fitting: Fitting,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        branch_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
        weld_type: str = "sw",
    ) -> np.ndarray:
        """
        Compute transform for a tee with up-vector awareness.

        The tee's Z-axis (perpendicular to both run and branch) should be
        consistent with the up-vector to ensure proper orientation.
        """
        _ = fitting  # Unused but kept for API consistency

        # Tee default: inlet from -X, run to +X, branch to +Y
        # The tee body lies in the XY plane

        # Use correct dimensions based on weld type
        if weld_type == "bw":
            dims = self.butt_weld_tee_dims
            if dims is None:
                raise ValueError(f"No butt weld tee dimensions for NPS {self.nps}")
        else:
            dims = self.tee_dims
            if dims is None:
                raise ValueError(f"No tee dimensions for NPS {self.nps}")

        C = dims.center_to_end_run

        # Build rotation matrix
        # Run direction (X) is same as incoming
        run_vec = np.array(incoming_direction)
        branch_vec = np.array(branch_direction)
        up = np.array(up_vector)

        # Compute Z-axis as cross product of run and branch
        z_vec = np.cross(run_vec, branch_vec)
        z_norm = np.linalg.norm(z_vec)

        if z_norm < 1e-6:
            # Run and branch are parallel (shouldn't happen for a proper tee)
            z_vec = up
        else:
            z_vec = z_vec / z_norm
            # KEY FIX: Ensure Z is in the "up" hemisphere when possible
            # This prevents the tee from being upside-down
            if np.dot(z_vec, up) < 0:
                z_vec = -z_vec

        # Build rotation matrix [run | branch | z]
        rotation = np.eye(4)
        rotation[:3, 0] = run_vec
        rotation[:3, 1] = branch_vec
        rotation[:3, 2] = z_vec

        # Position tee center
        inlet_pos = get_position(incoming_transform)
        center_pos = (
            inlet_pos[0] + C * incoming_direction[0],
            inlet_pos[1] + C * incoming_direction[1],
            inlet_pos[2] + C * incoming_direction[2],
        )

        tee_transform = translation_matrix(*center_pos) @ rotation

        return tee_transform

    def _compute_tee_transform_branch_entry(
        self,
        fitting: Fitting,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        run_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
        weld_type: str = "sw",
    ) -> np.ndarray:
        """
        Compute transform for a tee in branch entry mode.

        In this mode, the incoming pipe connects to the branch port (local +Y),
        and the run (local ±X) becomes the two outlets.

        The tee's local geometry:
        - inlet port at -X (becomes run_b outlet)
        - run port at +X (becomes run_a outlet)
        - branch port at +Y (receives incoming pipe)
        """
        _ = fitting  # Unused but kept for API consistency

        # Use correct dimensions based on weld type
        if weld_type == "bw":
            dims = self.butt_weld_tee_dims
            if dims is None:
                raise ValueError(f"No butt weld tee dimensions for NPS {self.nps}")
        else:
            dims = self.tee_dims
            if dims is None:
                raise ValueError(f"No tee dimensions for NPS {self.nps}")

        M = dims.center_to_end_branch  # Distance from center to branch port

        # Build rotation matrix
        # Local +X (run direction) maps to world run_direction
        # Local +Y (branch out) maps to world -incoming_direction (port faces out, flow comes in)
        run_vec = np.array(run_direction)
        branch_out_vec = np.array((-incoming_direction[0], -incoming_direction[1], -incoming_direction[2]))
        up = np.array(up_vector)

        # Compute Z-axis as cross product of run and branch_out
        z_vec = np.cross(run_vec, branch_out_vec)
        z_norm = np.linalg.norm(z_vec)

        if z_norm < 1e-6:
            # Run and branch are parallel (shouldn't happen for a proper tee)
            z_vec = up
        else:
            z_vec = z_vec / z_norm
            # Ensure Z is in the "up" hemisphere when possible
            if np.dot(z_vec, up) < 0:
                z_vec = -z_vec

        # Build rotation matrix [run | branch_out | z]
        rotation = np.eye(4)
        rotation[:3, 0] = run_vec
        rotation[:3, 1] = branch_out_vec
        rotation[:3, 2] = z_vec

        # Position tee center - offset from branch port position along incoming direction
        branch_port_pos = get_position(incoming_transform)
        center_pos = (
            branch_port_pos[0] + M * incoming_direction[0],
            branch_port_pos[1] + M * incoming_direction[1],
            branch_port_pos[2] + M * incoming_direction[2],
        )

        tee_transform = translation_matrix(*center_pos) @ rotation

        return tee_transform

    def _compute_cross_transform(
        self,
        fitting: Fitting,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        left_direction: tuple[float, float, float],
        up_vector: tuple[float, float, float],
    ) -> np.ndarray:
        """
        Compute transform for a cross with up-vector awareness.

        Similar to tee but with four ports instead of three.
        """
        _ = fitting  # Unused but kept for API consistency

        dims = self.cross_dims
        if dims is None:
            raise ValueError(f"No cross dimensions for NPS {self.nps}")

        C = dims.center_to_end

        # Build rotation matrix similar to tee
        run_vec = np.array(incoming_direction)
        left_vec = np.array(left_direction)
        up = np.array(up_vector)

        z_vec = np.cross(run_vec, left_vec)
        z_norm = np.linalg.norm(z_vec)

        if z_norm < 1e-6:
            z_vec = up
        else:
            z_vec = z_vec / z_norm
            # Ensure Z is in the "up" hemisphere
            if np.dot(z_vec, up) < 0:
                z_vec = -z_vec

        rotation = np.eye(4)
        rotation[:3, 0] = run_vec
        rotation[:3, 1] = left_vec
        rotation[:3, 2] = z_vec

        inlet_pos = get_position(incoming_transform)
        center_pos = (
            inlet_pos[0] + C * incoming_direction[0],
            inlet_pos[1] + C * incoming_direction[1],
            inlet_pos[2] + C * incoming_direction[2],
        )

        cross_transform = translation_matrix(*center_pos) @ rotation

        return cross_transform

    # -------------------------------------------------------------------------
    # COMPONENT/WELD TRACKING
    # -------------------------------------------------------------------------

    def _add_component(
        self,
        comp_type: str,
        description: str,
        schedule_class: str,
        shape: cq.Shape | None,
        transform: np.ndarray,
        **kwargs,
    ) -> None:
        """Track a component for BOM."""
        if not HAS_BOM or ComponentRecord is None:
            return

        self._item_counter += 1
        self.route.components.append(
            ComponentRecord(
                item_number=self._item_counter,
                component_type=comp_type,
                nps=self.nps,
                description=description,
                schedule_class=schedule_class,
                shape=shape,
                world_transform=transform.copy(),
                length_mm=kwargs.get("length_mm"),
                connected_directions=kwargs.get("connected_directions", []),
                pipe_direction=kwargs.get("pipe_direction"),
            )
        )

    def _select_weld_attachment_point(
        self,
        fitting: Fitting,
        port_name: str,
        pipe_direction: str,
        weld_type: str = "BW",
        avoid_direction: str | None = None,
        world_transform: np.ndarray | None = None,
    ) -> str:
        """
        Select the best weld attachment point based on world-space direction.

        For vertical pipes, selects the attachment point that actually faces
        the desired world direction (up/down), not just by local name.

        Args:
            fitting: The Fitting object with attachment points
            port_name: The port to find attachments for (e.g., "weld", "inlet", "outlet")
            pipe_direction: The direction the pipe runs at this weld
            weld_type: "BW" for butt weld, "SW" for socket weld
            avoid_direction: Direction to avoid (where next fitting is)
            world_transform: 4x4 transformation matrix to world coordinates

        Returns:
            Name of the best attachment point to use
        """
        # Get all weld attachment points for this port
        attachments = fitting.get_attachment_points_for_port(port_name)
        if not attachments:
            # Fallback to port position if no attachment points
            return ""

        pipe_dir = pipe_direction.lower()

        # For vertical pipes with world_transform, select by actual world direction
        if pipe_dir in ("up", "down") and world_transform is not None:
            # For vertical pipes, attachment points are perpendicular to the pipe axis.
            # We want to place markers on the OPPOSITE side from connected fittings
            # (avoid_direction) to prevent markers from crossing geometry.
            #
            # Direction mapping for world coordinates:
            direction_to_world = {
                "east": np.array([1.0, 0.0, 0.0]),
                "west": np.array([-1.0, 0.0, 0.0]),
                "north": np.array([0.0, 1.0, 0.0]),
                "south": np.array([0.0, -1.0, 0.0]),
                "up": np.array([0.0, 0.0, 1.0]),
                "down": np.array([0.0, 0.0, -1.0]),
            }

            # Define pipe axis direction first (needed for cross product)
            if pipe_dir == "up":
                pipe_axis_dir = np.array([0.0, 0.0, 1.0])
            else:
                pipe_axis_dir = np.array([0.0, 0.0, -1.0])

            # Determine preferred world direction for attachment:
            # For vertical pipes with horizontal branches, we want a direction that is:
            # 1. Perpendicular to the pipe axis (so marker is on the side)
            # 2. Perpendicular to the avoid direction (so marker doesn't cross branch pipes)
            # This is achieved by cross product: -(pipe_axis × avoid_dir)
            if avoid_direction and avoid_direction.lower() in direction_to_world:
                avoid_world_dir = direction_to_world[avoid_direction.lower()]
                # Cross product gives perpendicular direction
                # Negate to get the "away" side (right-hand rule)
                preferred_world_dir = -np.cross(pipe_axis_dir, avoid_world_dir)
                # Normalize in case of numerical issues
                norm = np.linalg.norm(preferred_world_dir)
                if norm > 0.1:
                    preferred_world_dir = preferred_world_dir / norm
                else:
                    # Fallback if cross product is near zero (avoid_dir along pipe axis)
                    preferred_world_dir = np.array([0.0, -1.0, 0.0])
            else:
                # Default: prefer south for visibility in isometric view
                preferred_world_dir = np.array([0.0, -1.0, 0.0])

            best_ap = None
            best_score = -2.0

            rotation_matrix = world_transform[:3, :3]

            for ap in attachments:
                # Transform local normal to world space
                local_normal = np.array(ap.normal)
                world_normal = rotation_matrix @ local_normal
                norm = np.linalg.norm(world_normal)
                if norm > 0:
                    world_normal = world_normal / norm

                # Score based on:
                # 1. Alignment with pipe axis (for pipe-end attachments) - highest priority
                # 2. Alignment with preferred direction (opposite of avoid) - for perpendicular
                pipe_axis_dot = np.dot(world_normal, pipe_axis_dir)
                preferred_dot = np.dot(world_normal, preferred_world_dir)

                if abs(pipe_axis_dot) > 0.7:
                    # Attachment faces along pipe axis - prioritize
                    score = pipe_axis_dot
                else:
                    # Perpendicular attachment - use preferred direction
                    score = preferred_dot * 0.9

                if score > best_score:
                    best_score = score
                    best_ap = ap

            if best_ap is not None and best_score > 0.3:
                return best_ap.name

        # Fall back to name-based selection for horizontal pipes or if world transform unavailable
        # Define preference order based on pipe direction and isometric visibility
        if pipe_dir in ("east", "west"):
            # E-W pipes: prefer down, then south (both visible in iso)
            preference = ["down", "south", "up", "north"]
        elif pipe_dir in ("north", "south"):
            # N-S pipes: prefer down, then west (visible in iso)
            preference = ["down", "west", "up", "east"]
        else:  # up/down (vertical pipes) - fallback if world transform didn't work
            if pipe_dir == "up":
                preference = ["up", "south", "west", "north", "east", "down"]
            else:
                preference = ["down", "south", "west", "north", "east", "up"]

        # Special case for branch ports (like tee branch going east)
        if port_name == "branch" and pipe_dir == "east":
            preference = ["east", "down", "south", "west", "north", "up"]

        # If we have an avoid direction, move it to the end of preference
        if avoid_direction:
            avoid_dir = avoid_direction.lower()
            is_vertical_pipe = pipe_dir in ("up", "down")
            is_flow_direction = avoid_dir in ("up", "down")
            if avoid_dir in preference and not (is_vertical_pipe and is_flow_direction):
                preference.remove(avoid_dir)
                preference.append(avoid_dir)

        # Find the first available attachment point in preference order
        attachment_names = {ap.name: ap for ap in attachments}

        for direction in preference:
            possible_names = [
                f"{port_name}_weld_{direction}",
                f"{port_name}_{direction}",
            ]
            for name in possible_names:
                if name in attachment_names:
                    return name

        # Fallback: return first available attachment
        return attachments[0].name

    def _add_weld(
        self,
        weld_type: str,
        description: str,
        fitting: Fitting,
        port_name: str,
        transform: np.ndarray,
        pipe_direction: str,
        avoid_direction: str | None = None,
    ) -> None:
        """Track a weld for drawing annotation using attachment points."""
        if not HAS_BOM or WeldRecord is None:
            return

        self._weld_counter += 1

        # Select the best attachment point for this weld
        attachment_name = self._select_weld_attachment_point(
            fitting, port_name, pipe_direction, weld_type, avoid_direction, transform
        )

        # Extract attachment direction from attachment name (e.g., "inlet_weld_down" -> "down")
        attachment_direction: str | None = None
        if attachment_name:
            # Parse the direction from the attachment name
            # Format is typically "{port}_weld_{direction}" or "{port}_{direction}"
            parts = attachment_name.split("_")
            if len(parts) >= 2:
                # Last part is the direction (up, down, north, south, east, west)
                direction = parts[-1].lower()
                if direction in ("up", "down", "north", "south", "east", "west"):
                    attachment_direction = direction

        # Get weld position from attachment point (preferred) or port (fallback)
        if attachment_name:
            weld_pos = fitting.get_attachment_point_world_position(attachment_name, transform)
        else:
            # Fallback to port position if no attachment points
            port = fitting.get_port(port_name)
            port_world = transform @ port.transform
            weld_pos = get_position(port_world)

        pipe_radius = self.pipe_spec.od_mm / 2.0 if self.pipe_spec else 30.0

        self.route.welds.append(
            WeldRecord(
                weld_number=self._weld_counter,
                weld_type=weld_type,
                description=description,
                world_position_3d=weld_pos,
                pipe_direction=pipe_direction,
                avoid_direction=avoid_direction,
                attachment_direction=attachment_direction,
                pipe_radius_mm=pipe_radius,
            )
        )
