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

from .fittings.pipe_fittings import (
    PIPE_SPECS,
    Fitting,
    Port,
    apply_transform_to_shape,
    get_position,
    identity_matrix,
    make_pipe,
    make_socket_weld_elbow,
    make_weld_neck_flange,
    rotation_matrix_x,
    rotation_matrix_y,
    translation_matrix,
)
from .fittings.socket_weld_cross import ASME_B1611_CROSS_CLASS3000, make_socket_weld_cross
from .fittings.socket_weld_elbow import ASME_B1611_ELBOW90_CLASS3000
from .fittings.socket_weld_tee import ASME_B1611_TEE_CLASS3000, make_socket_weld_tee
from .fittings.weld_neck_flange import ASME_B165_CLASS300_WN
from .pipe_router import (
    DIRECTIONS,
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


def direction_name_to_vector(name: str) -> tuple[float, float, float]:
    """Convert a direction name to a unit vector."""
    return normalize_direction(name)


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
        self.flange_dims = ASME_B165_CLASS300_WN.get(self.nps)
        self.tee_dims = ASME_B1611_TEE_CLASS3000.get(self.nps)
        self.cross_dims = ASME_B1611_CROSS_CLASS3000.get(self.nps)

        # Component tracking
        self._item_counter = 0
        self._weld_counter = 0

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

        # Recursive build starting from root
        self._build_node(
            node=self.route.root,
            incoming_transform=initial_transform,
            incoming_direction=initial_direction,
        )

    def _build_node(
        self,
        node: RouteNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float]]]:
        """
        Build geometry for a single node and recursively build children.

        Args:
            node: The RouteNode to build
            incoming_transform: World transform for the inlet port
            incoming_direction: World direction the pipe was traveling

        Returns:
            Dictionary mapping port names to (transform, direction) tuples
            for each outlet port.
        """
        if isinstance(node, FlangeNode):
            return self._build_flange(node, incoming_transform)
        elif isinstance(node, PipeSegmentNode):
            return self._build_pipe(node, incoming_transform, incoming_direction)
        elif isinstance(node, ElbowNode):
            return self._build_elbow(node, incoming_transform, incoming_direction)
        elif isinstance(node, TeeNode):
            return self._build_tee(node, incoming_transform, incoming_direction)
        elif isinstance(node, CrossNode):
            return self._build_cross(node, incoming_transform, incoming_direction)
        elif isinstance(node, WeldoletNode):
            return self._build_weldolet(node, incoming_transform, incoming_direction)
        elif isinstance(node, ReducerNode):
            return self._build_reducer(node, incoming_transform, incoming_direction)
        elif isinstance(node, TerminalNode):
            return self._build_terminal(node, incoming_transform, incoming_direction)
        else:
            raise ValueError(f"Unknown node type: {type(node)}")

    # -------------------------------------------------------------------------
    # NODE BUILDERS
    # -------------------------------------------------------------------------

    def _build_flange(
        self,
        node: FlangeNode,
        incoming_transform: np.ndarray,
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float]]]:
        """Build a flange fitting."""
        direction = direction_name_to_vector(node.direction)

        # Create flange
        fitting = make_weld_neck_flange(node.nps, units="mm")

        # Orient flange so weld port faces in the specified direction
        # Weld port default direction is -Z, so rotate to align -Z with direction
        neg_direction = (-direction[0], -direction[1], -direction[2])
        flange_rotation = rotation_to_align_z_with_direction(neg_direction)

        # Apply to incoming transform
        flange_transform = incoming_transform @ flange_rotation

        # Add geometry
        if fitting.shape is not None:
            shape = apply_transform_to_shape(fitting.shape, flange_transform)
            self.route.parts.append(shape)

        # Track component
        self._add_component(
            comp_type="flange",
            description="WN FLANGE",
            schedule_class=f"Class {node.flange_class}",
            shape=fitting.shape,
            transform=flange_transform,
            connected_directions=[node.direction],
        )

        # Track fitting
        self.route.fittings.append((fitting, flange_transform))

        # Track weld at hub
        self._add_weld(
            weld_type="BW",
            description="FLANGE TO PIPE",
            fitting=fitting,
            port_name="weld",
            transform=flange_transform,
            pipe_direction=node.direction,
        )

        # Store transform on node
        node._world_transform = flange_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transform
        weld_port = fitting.get_port("weld")
        outlet_transform = flange_transform @ weld_port.transform

        outlets = {"weld": (outlet_transform, direction)}

        # Recursively build children
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec)

        return outlets

    def _build_pipe(
        self,
        node: PipeSegmentNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float]]]:
        """Build a pipe segment."""
        # Parse length
        length_mm = parse_distance(node.length)

        # Create pipe
        fitting = make_pipe(self.nps, length_mm)

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
            length_mm=length_mm,
            pipe_direction=vector_to_direction_name(incoming_direction),
        )

        # Store transform on node
        node._world_transform = pipe_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transform (at pipe end)
        end_port = fitting.get_port("end")
        outlet_transform = pipe_transform @ end_port.transform

        outlets = {"end": (outlet_transform, incoming_direction)}

        # Handle weldolets on this pipe
        for weldolet in node.weldolets:
            self._build_weldolet_on_pipe(weldolet, node, pipe_transform, incoming_direction)

        # Recursively build children
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec)

        return outlets

    def _build_elbow(
        self,
        node: ElbowNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float]]]:
        """Build an elbow fitting."""
        turn_direction = direction_name_to_vector(node.turn_direction)

        # Create elbow
        fitting = make_socket_weld_elbow(self.nps, units="mm")

        # Compute elbow rotation
        elbow_transform = self._compute_elbow_transform(fitting, incoming_transform, incoming_direction, turn_direction)

        # Add geometry
        if fitting.shape is not None:
            shape = apply_transform_to_shape(fitting.shape, elbow_transform)
            self.route.parts.append(shape)

        # Track component
        self._add_component(
            comp_type="elbow",
            description="90DEG ELBOW",
            schedule_class="Class 3000 SW" if node.weld_type == "sw" else "BW",
            shape=fitting.shape,
            transform=elbow_transform,
            connected_directions=[
                vector_to_direction_name(incoming_direction),
                node.turn_direction,
            ],
        )

        # Track fitting
        self.route.fittings.append((fitting, elbow_transform))

        # Track welds
        self._add_weld(
            weld_type="SW" if node.weld_type == "sw" else "BW",
            description="PIPE TO ELBOW",
            fitting=fitting,
            port_name="inlet",
            transform=elbow_transform,
            pipe_direction=vector_to_direction_name(incoming_direction),
        )

        # Store transform on node
        node._world_transform = elbow_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transform
        outlet_port = fitting.get_port("outlet")
        outlet_transform = elbow_transform @ outlet_port.transform

        outlets = {"outlet": (outlet_transform, turn_direction)}

        # Recursively build children
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec)

        return outlets

    def _build_tee(
        self,
        node: TeeNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float]]]:
        """Build a tee fitting."""
        branch_direction = direction_name_to_vector(node.branch_direction)

        # Create tee shape
        tee_shape = make_socket_weld_tee(self.nps)

        # Create a Fitting object for the tee
        fitting = self._create_tee_fitting(self.nps, tee_shape)

        # Compute tee transform
        tee_transform = self._compute_tee_transform(fitting, incoming_transform, incoming_direction, branch_direction)

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

        # Track fitting
        self.route.fittings.append((fitting, tee_transform))

        # Store transform on node
        node._world_transform = tee_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transforms
        run_port = fitting.get_port("run")
        branch_port = fitting.get_port("branch")

        run_outlet_transform = tee_transform @ run_port.transform
        branch_outlet_transform = tee_transform @ branch_port.transform

        outlets = {
            "run": (run_outlet_transform, incoming_direction),
            "branch": (branch_outlet_transform, branch_direction),
        }

        # Recursively build children
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec)

        return outlets

    def _build_cross(
        self,
        node: CrossNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float]]]:
        """Build a cross fitting."""
        orientation = direction_name_to_vector(node.orientation)

        # Create cross shape
        cross_shape = make_socket_weld_cross(self.nps)

        # Create a Fitting object for the cross
        fitting = self._create_cross_fitting(self.nps, cross_shape)

        # Compute perpendicular directions for branches
        left_dir, right_dir = compute_perpendicular_directions(incoming_direction, orientation)

        # Compute cross transform
        cross_transform = self._compute_cross_transform(fitting, incoming_transform, incoming_direction, left_dir)

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

        # Track fitting
        self.route.fittings.append((fitting, cross_transform))

        # Store transform on node
        node._world_transform = cross_transform
        node._fitting = fitting
        node._built = True

        # Compute outlet transforms
        run_port = fitting.get_port("run")
        left_port = fitting.get_port("branch_left")
        right_port = fitting.get_port("branch_right")

        outlets = {
            "run": (cross_transform @ run_port.transform, incoming_direction),
            "branch_left": (cross_transform @ left_port.transform, left_dir),
            "branch_right": (cross_transform @ right_port.transform, right_dir),
        }

        # Recursively build children
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec)

        return outlets

    def _build_weldolet(
        self,
        node: WeldoletNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float]]]:
        """Build a standalone weldolet (not attached to a pipe segment)."""
        # This is typically called when a weldolet is not attached to a pipe
        # In practice, weldolets should be in the pipe's weldolets list
        branch_direction = direction_name_to_vector(node.branch_direction)

        outlets = {"branch": (incoming_transform, branch_direction)}

        # Recursively build children
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec)

        return outlets

    def _build_weldolet_on_pipe(
        self,
        node: WeldoletNode,
        pipe_node: PipeSegmentNode,
        pipe_transform: np.ndarray,
        pipe_direction: tuple[float, float, float],
    ) -> None:
        """Build a weldolet attached to a pipe segment."""
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

        # Build children from the weldolet branch
        outlets = {"branch": (outlet_transform, branch_direction)}
        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec)

    def _build_reducer(
        self,
        node: ReducerNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float]]]:
        """Build a reducer (stub - geometry TBD)."""
        # For now, just pass through
        node._world_transform = incoming_transform
        node._built = True

        outlets = {"outlet": (incoming_transform, incoming_direction)}

        for port_name, child in node.children.items():
            if port_name in outlets:
                transform, dir_vec = outlets[port_name]
                self._build_node(child, transform, dir_vec)

        return outlets

    def _build_terminal(
        self,
        node: TerminalNode,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
    ) -> dict[str, tuple[np.ndarray, tuple[float, float, float]]]:
        """Build a terminal node (cap, blind flange, or open end)."""
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
        # Inlet: from -X toward center, Z points +X (toward incoming pipe)
        inlet_transform = translation_matrix(-C, 0, 0) @ rotation_matrix_y(-90)

        # Run: from center toward +X, Z points +X (toward outgoing pipe)
        run_transform = translation_matrix(C, 0, 0) @ rotation_matrix_y(90)

        # Branch: from center toward +Y, Z points +Y (toward branch pipe)
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
        # Inlet: from -X toward center
        inlet_transform = translation_matrix(-C, 0, 0) @ rotation_matrix_y(-90)

        # Run: from center toward +X
        run_transform = translation_matrix(C, 0, 0) @ rotation_matrix_y(90)

        # Branch Left: from center toward +Y
        left_transform = translation_matrix(0, C, 0) @ rotation_matrix_x(-90)

        # Branch Right: from center toward -Y
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
        # The pipe's start port should align with incoming_transform
        # and face opposite to incoming_direction

        # Pipe is created along Z axis, start at Z=0, end at Z=length
        # Start port faces -Z, end port faces +Z

        # Rotate pipe so start port faces opposite to incoming direction
        # (start port should face the fitting we're connecting from)
        neg_dir = (-incoming_direction[0], -incoming_direction[1], -incoming_direction[2])
        rotation = rotation_to_align_z_with_direction(neg_dir)

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
    ) -> np.ndarray:
        """Compute transform for an elbow."""
        # Elbow default orientation: inlet from +X, outlet toward +Y
        # Need to rotate so inlet faces incoming pipe and outlet faces turn_direction

        # Import the compute_elbow_rotation function
        from .pipe_router import compute_elbow_rotation

        elbow_rotation = compute_elbow_rotation(incoming_direction, turn_direction)

        # Position elbow so inlet port is at incoming_transform
        dims = self.elbow_dims
        if dims is None:
            raise ValueError(f"No elbow dimensions for NPS {self.nps}")

        A = dims.center_to_end  # Center to socket end

        # Elbow center is A distance back from inlet port along incoming direction
        inlet_pos = get_position(incoming_transform)

        # After rotation, inlet is along the negative of incoming_direction from center
        # So center = inlet_pos + A * incoming_direction
        center_pos = (
            inlet_pos[0] + A * incoming_direction[0],
            inlet_pos[1] + A * incoming_direction[1],
            inlet_pos[2] + A * incoming_direction[2],
        )

        elbow_transform = translation_matrix(*center_pos) @ elbow_rotation

        return elbow_transform

    def _compute_tee_transform(
        self,
        fitting: Fitting,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        branch_direction: tuple[float, float, float],
    ) -> np.ndarray:
        """Compute transform for a tee."""
        # Tee default: inlet from -X, run to +X, branch to +Y
        # Need to rotate so:
        # - Inlet faces opposite to incoming_direction
        # - Branch faces branch_direction

        dims = self.tee_dims
        if dims is None:
            raise ValueError(f"No tee dimensions for NPS {self.nps}")

        C = dims.center_to_end_run

        # Build rotation matrix
        # Run direction is same as incoming
        run_vec = np.array(incoming_direction)
        branch_vec = np.array(branch_direction)

        # Compute the rotation matrix that aligns X with run and Y with branch
        # Third axis is cross product
        z_vec = np.cross(run_vec, branch_vec)
        z_vec = z_vec / np.linalg.norm(z_vec)

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

    def _compute_cross_transform(
        self,
        fitting: Fitting,
        incoming_transform: np.ndarray,
        incoming_direction: tuple[float, float, float],
        left_direction: tuple[float, float, float],
    ) -> np.ndarray:
        """Compute transform for a cross."""
        dims = self.cross_dims
        if dims is None:
            raise ValueError(f"No cross dimensions for NPS {self.nps}")

        C = dims.center_to_end

        # Build rotation matrix similar to tee
        run_vec = np.array(incoming_direction)
        left_vec = np.array(left_direction)
        z_vec = np.cross(run_vec, left_vec)
        z_vec = z_vec / np.linalg.norm(z_vec)

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

    def _add_weld(
        self,
        weld_type: str,
        description: str,
        fitting: Fitting,
        port_name: str,
        transform: np.ndarray,
        pipe_direction: str,
    ) -> None:
        """Track a weld for drawing annotation."""
        if not HAS_BOM or WeldRecord is None:
            return

        self._weld_counter += 1

        # Get weld position from fitting port
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
                pipe_radius_mm=pipe_radius,
            )
        )
