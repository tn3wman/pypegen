#!/usr/bin/env python3
"""
Fluent Builder API for Branching Pipe Routes

This module provides a fluent, chainable API for constructing pipe routes
with support for branching at tees, crosses, and mid-pipe weldolets.

The builder uses context managers for branch definition, providing clean
Python idiom with automatic scope handling.

Example - Simple Route with Tee:
    builder = RouteBuilder(nps="2")
    builder.start_with_flange("east")
    builder.add_pipe("10 ft")

    with builder.add_tee(branch_direction="north") as tee:
        tee.run().add_pipe("5 ft").add_cap()
        tee.branch().add_pipe("3 ft").add_elbow("up").add_pipe("2 ft").add_cap()

    route = builder.build()

Example - Nested Branches (3+ Levels):
    with builder.add_tee("north") as tee1:
        with tee1.run().add_pipe("5 ft").add_tee("up") as tee2:
            tee2.run().add_pipe("3 ft").add_cap()
            tee2.branch().add_pipe("2 ft").add_cap()
        tee1.branch().add_pipe("4 ft").add_cap()

Example - Composable Patterns:
    def create_drain_leg(builder: BranchBuilder) -> None:
        builder.add_pipe("2 ft").add_elbow("down").add_pipe("1 ft").add_cap()

    with builder.add_tee("down") as tee:
        tee.run().add_pipe("10 ft").add_cap()
        create_drain_leg(tee.branch())  # Reusable pattern
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, overload

from .fittings.weld_neck_flange import BoltHoleOrientation
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
    WeldType,
)

if TYPE_CHECKING:
    import cadquery as cq
    import numpy as np

    from .drawing_generator.bom import ComponentRecord, WeldRecord
    from .fittings.pipe_fittings import Fitting


# =============================================================================
# BRANCHING PIPE ROUTE (RESULT OF BUILD)
# =============================================================================


@dataclass
class BranchingPipeRoute:
    """
    Result of building a branching pipe route.

    Contains the tree structure, geometry, and tracking data for
    components and welds.

    Attributes:
        nps: Nominal pipe size (e.g., "2")
        schedule: Pipe schedule (e.g., "40", "80")
        flange_class: Flange pressure class (e.g., 150, 300)
        material: Material description (e.g., "316 Stainless Steel")
        root: Root node of the route tree
        parts: List of CadQuery shapes (geometry)
        components: List of ComponentRecord for BOM
        welds: List of WeldRecord for weld annotation
        fittings: List of (Fitting, world_transform) tuples
    """

    nps: str = "2"
    schedule: str = "40"
    flange_class: int = 300
    material: str = "316 Stainless Steel"

    root: RouteNode | None = None

    # Build results (populated after build())
    parts: list[cq.Shape] = field(default_factory=list)
    components: list[ComponentRecord] = field(default_factory=list)
    welds: list[WeldRecord] = field(default_factory=list)
    fittings: list[tuple[Fitting, np.ndarray]] = field(default_factory=list)

    # Debug options
    show_coordinate_frames: bool = False  # Add coordinate frame markers at ports

    # Private state
    _built: bool = field(default=False, repr=False)

    def export(self, filename: str) -> None:
        """Export the route geometry to a STEP file."""
        if not self._built:
            raise RuntimeError("Route has not been built yet. Call build() first.")
        if not self.parts:
            raise RuntimeError("No geometry to export.")

        import cadquery as cq

        # Combine all parts into a single compound
        compound = cq.Compound.makeCompound(self.parts)
        cq.exporters.export(compound, filename)


# =============================================================================
# BRANCH BUILDER (FOR SINGLE BRANCH PATHS)
# =============================================================================


class BranchBuilder:
    """
    Builder for a single branch path.

    Provides the same fluent interface as RouteBuilder but operates
    on a specific branch of a junction fitting (tee, cross, weldolet).

    This class is returned by context methods like tee.run() or tee.branch()
    and allows method chaining within each branch.
    """

    def __init__(
        self,
        parent_builder: RouteBuilder,
        junction_node: RouteNode,
        port_name: str,
    ):
        self._parent = parent_builder
        self._junction_node = junction_node
        self._port_name = port_name
        self._current_node: RouteNode = junction_node
        self._current_port: str = port_name
        self._terminated: bool = False

    @property
    def nps(self) -> str:
        """Current nominal pipe size."""
        return self._parent._nps

    @property
    def schedule(self) -> str:
        """Current pipe schedule."""
        return self._parent._schedule

    def add_pipe(self, length: str, name: str | None = None) -> BranchBuilder:
        """
        Add a pipe segment to this branch.

        Args:
            length: Pipe length (e.g., "5 ft", "1500 mm")
            name: Optional name for this segment

        Returns:
            Self for method chaining
        """
        if self._terminated:
            raise RuntimeError("Cannot add to a terminated branch")

        node = PipeSegmentNode(
            length=length,
            schedule=self._parent._schedule,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._current_node = node
        self._current_port = "end"
        return self

    def add_elbow(
        self,
        turn_to: str,
        weld_type: WeldType = "sw",
        name: str | None = None,
    ) -> BranchBuilder:
        """
        Add an elbow turning toward the specified direction.

        Args:
            turn_to: Direction after the elbow (e.g., "up", "north")
            weld_type: "sw" for socket weld, "bw" for butt weld
            name: Optional name for this elbow

        Returns:
            Self for method chaining
        """
        if self._terminated:
            raise RuntimeError("Cannot add to a terminated branch")

        node = ElbowNode(
            weld_type=weld_type,
            turn_direction=turn_to,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._current_node = node
        self._current_port = "outlet"
        return self

    def add_tee(
        self,
        branch_direction: str | None = None,
        weld_type: WeldType = "sw",
        name: str | None = None,
        *,
        enter_via_branch: bool = False,
        run_direction: str | None = None,
    ) -> TeeContext:
        """
        Add a tee fitting for nested branching.

        Standard mode (enter_via_branch=False):
            Incoming pipe connects to tee inlet, continues through the run,
            with a side branch at 90 degrees.

        Branch entry mode (enter_via_branch=True):
            Incoming pipe connects to the branch port. Both ends of the run
            become outlets (run_a and run_b).

        Args:
            branch_direction: World direction of the branch outlet (standard mode)
            weld_type: "sw" for socket weld, "bw" for butt weld
            name: Optional name for this tee
            enter_via_branch: If True, connect incoming pipe to branch port
            run_direction: World direction of the run (branch entry mode only)

        Returns:
            TeeContext for defining the outlet paths
        """
        if self._terminated:
            raise RuntimeError("Cannot add to a terminated branch")

        if enter_via_branch:
            if run_direction is None:
                raise ValueError("run_direction is required when enter_via_branch=True")
            node = TeeNode(
                weld_type=weld_type,
                enter_via_branch=True,
                run_direction=run_direction,
                name=name,
            )
        else:
            if branch_direction is None:
                raise ValueError("branch_direction is required when enter_via_branch=False")
            node = TeeNode(
                weld_type=weld_type,
                branch_direction=branch_direction,
                name=name,
            )
        self._current_node.add_child(self._current_port, node)
        self._terminated = True  # Branch continues via TeeContext
        return TeeContext(self._parent, node)

    def add_cross(
        self,
        orientation: str = "up",
        weld_type: WeldType = "sw",
        name: str | None = None,
    ) -> CrossContext:
        """
        Add a cross fitting for nested branching.

        Args:
            orientation: Defines the plane of perpendicular branches
            weld_type: "sw" for socket weld, "bw" for butt weld
            name: Optional name for this cross

        Returns:
            CrossContext for defining the run, left, and right branches
        """
        if self._terminated:
            raise RuntimeError("Cannot add to a terminated branch")

        node = CrossNode(
            weld_type=weld_type,
            orientation=orientation,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._terminated = True
        return CrossContext(self._parent, node)

    def add_reducer(
        self,
        target_nps: str,
        reducer_type: Literal["concentric", "eccentric"] = "concentric",
        weld_type: WeldType = "bw",
        name: str | None = None,
    ) -> BranchBuilder:
        """
        Add a reducer to change pipe size.

        Args:
            target_nps: Target nominal pipe size after reducer
            reducer_type: "concentric" or "eccentric"
            weld_type: Weld type (typically butt weld for reducers)
            name: Optional name

        Returns:
            Self for method chaining
        """
        if self._terminated:
            raise RuntimeError("Cannot add to a terminated branch")

        node = ReducerNode(
            target_nps=target_nps,
            reducer_type=reducer_type,
            weld_type=weld_type,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._current_node = node
        self._current_port = "outlet"
        # Update parent builder's NPS for subsequent segments
        self._parent._nps = target_nps
        return self

    def add_cap(self, name: str | None = None) -> None:
        """
        Terminate this branch with a cap.

        Args:
            name: Optional name for the cap
        """
        if self._terminated:
            raise RuntimeError("Cannot add to a terminated branch")

        node = TerminalNode(end_type="cap", name=name)
        self._current_node.add_child(self._current_port, node)
        self._terminated = True

    def add_blind_flange(self, name: str | None = None) -> None:
        """
        Terminate this branch with a blind flange.

        Args:
            name: Optional name for the blind flange
        """
        if self._terminated:
            raise RuntimeError("Cannot add to a terminated branch")

        node = TerminalNode(end_type="blind_flange", name=name)
        self._current_node.add_child(self._current_port, node)
        self._terminated = True

    def add_open_end(self) -> None:
        """Leave this branch as an open end (for future connection)."""
        if self._terminated:
            raise RuntimeError("Cannot add to a terminated branch")

        node = TerminalNode(end_type="open")
        self._current_node.add_child(self._current_port, node)
        self._terminated = True

    def add_flange(
        self,
        direction: str = "east",
        flange_class: int | None = None,
        bolt_hole_orientation: BoltHoleOrientation = "single_hole",
        name: str | None = None,
    ) -> None:
        """
        Terminate this branch with a weld neck flange.

        Args:
            direction: Orientation direction (for the flange face)
            flange_class: Pressure class (defaults to route's flange_class)
            bolt_hole_orientation: Bolt hole orientation relative to the pipe direction.
                - "single_hole": One bolt hole lies on the reference plane
                - "two_hole": Two bolt holes symmetric about the reference plane
            name: Optional name
        """
        if self._terminated:
            raise RuntimeError("Cannot add to a terminated branch")

        # Use route's default if not specified
        if flange_class is None:
            flange_class = self._parent._flange_class

        node = FlangeNode(
            nps=self._parent._nps,
            flange_class=flange_class,
            direction=direction,
            bolt_hole_orientation=bolt_hole_orientation,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._terminated = True


# =============================================================================
# CONTEXT MANAGERS FOR JUNCTION FITTINGS
# =============================================================================


class TeeContext:
    """
    Context manager for defining tee branches.

    Standard mode usage:
        with builder.add_tee("north") as tee:
            tee.run().add_pipe("5 ft").add_cap()
            tee.branch().add_pipe("3 ft").add_cap()

    Branch entry mode usage:
        with builder.add_tee(enter_via_branch=True, run_direction="east") as tee:
            tee.run_a().add_pipe("5 ft").add_cap()  # Goes east
            tee.run_b().add_pipe("5 ft").add_cap()  # Goes west
    """

    def __init__(self, builder: RouteBuilder, tee_node: TeeNode):
        self._builder = builder
        self._tee_node = tee_node
        self._defined_branches: set[str] = set()
        self._enter_via_branch = tee_node.enter_via_branch

    def __enter__(self) -> TeeContext:
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        # Warn about undefined branches (they'll be open ends)
        if self._enter_via_branch:
            required = {"run_a", "run_b"}
        else:
            required = {"run", "branch"}
        missing = required - self._defined_branches
        if missing and exc_type is None:  # Only warn if no exception
            warnings.warn(
                f"Tee branches not defined: {missing}. They will be treated as open ends.",
                stacklevel=2,
            )
            # Add open end nodes for undefined branches
            for port in missing:
                self._tee_node.add_child(port, TerminalNode(end_type="open"))

    def run(self) -> BranchBuilder:
        """
        Define the run-through branch (continues straight through the tee).
        Only available in standard mode (enter_via_branch=False).

        Returns:
            BranchBuilder for defining this branch's path
        """
        if self._enter_via_branch:
            raise RuntimeError("Use run_a() and run_b() when enter_via_branch=True")
        if "run" in self._defined_branches:
            raise RuntimeError("Tee run branch already defined")
        self._defined_branches.add("run")
        return BranchBuilder(self._builder, self._tee_node, "run")

    def branch(self) -> BranchBuilder:
        """
        Define the side branch (90 degrees from the run).
        Only available in standard mode (enter_via_branch=False).

        Returns:
            BranchBuilder for defining this branch's path
        """
        if self._enter_via_branch:
            raise RuntimeError("Use run_a() and run_b() when enter_via_branch=True")
        if "branch" in self._defined_branches:
            raise RuntimeError("Tee side branch already defined")
        self._defined_branches.add("branch")
        return BranchBuilder(self._builder, self._tee_node, "branch")

    def run_a(self) -> BranchBuilder:
        """
        Define the first run outlet (in the run_direction).
        Only available in branch entry mode (enter_via_branch=True).

        Returns:
            BranchBuilder for defining this branch's path
        """
        if not self._enter_via_branch:
            raise RuntimeError("Use run() and branch() when enter_via_branch=False")
        if "run_a" in self._defined_branches:
            raise RuntimeError("Tee run_a branch already defined")
        self._defined_branches.add("run_a")
        return BranchBuilder(self._builder, self._tee_node, "run_a")

    def run_b(self) -> BranchBuilder:
        """
        Define the second run outlet (opposite of run_direction).
        Only available in branch entry mode (enter_via_branch=True).

        Returns:
            BranchBuilder for defining this branch's path
        """
        if not self._enter_via_branch:
            raise RuntimeError("Use run() and branch() when enter_via_branch=False")
        if "run_b" in self._defined_branches:
            raise RuntimeError("Tee run_b branch already defined")
        self._defined_branches.add("run_b")
        return BranchBuilder(self._builder, self._tee_node, "run_b")


class CrossContext:
    """
    Context manager for defining cross branches.

    Usage:
        with builder.add_cross(orientation="up") as cross:
            cross.run().add_pipe("5 ft").add_cap()
            cross.branch_left().add_pipe("3 ft").add_cap()
            cross.branch_right().add_pipe("3 ft").add_cap()
    """

    def __init__(self, builder: RouteBuilder, cross_node: CrossNode):
        self._builder = builder
        self._cross_node = cross_node
        self._defined_branches: set[str] = set()

    def __enter__(self) -> CrossContext:
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        required = {"run", "branch_left", "branch_right"}
        missing = required - self._defined_branches
        if missing and exc_type is None:
            warnings.warn(
                f"Cross branches not defined: {missing}. They will be treated as open ends.",
                stacklevel=2,
            )
            for port in missing:
                self._cross_node.add_child(port, TerminalNode(end_type="open"))

    def run(self) -> BranchBuilder:
        """
        Define the run-through branch (continues straight).

        Returns:
            BranchBuilder for defining this branch's path
        """
        if "run" in self._defined_branches:
            raise RuntimeError("Cross run branch already defined")
        self._defined_branches.add("run")
        return BranchBuilder(self._builder, self._cross_node, "run")

    def branch_left(self) -> BranchBuilder:
        """
        Define the left branch (90 degrees left of run direction).

        Returns:
            BranchBuilder for defining this branch's path
        """
        if "branch_left" in self._defined_branches:
            raise RuntimeError("Cross left branch already defined")
        self._defined_branches.add("branch_left")
        return BranchBuilder(self._builder, self._cross_node, "branch_left")

    def branch_right(self) -> BranchBuilder:
        """
        Define the right branch (90 degrees right of run direction).

        Returns:
            BranchBuilder for defining this branch's path
        """
        if "branch_right" in self._defined_branches:
            raise RuntimeError("Cross right branch already defined")
        self._defined_branches.add("branch_right")
        return BranchBuilder(self._builder, self._cross_node, "branch_right")


class WeldoletContext:
    """
    Context manager for defining a weldolet branch.

    Usage:
        builder.add_pipe("20 ft")
        with builder.add_weldolet("up", position="10 ft") as weldolet:
            weldolet.add_pipe("5 ft").add_cap()
    """

    def __init__(
        self,
        builder: RouteBuilder,
        weldolet_node: WeldoletNode,
        parent_pipe: PipeSegmentNode,
    ):
        self._builder = builder
        self._weldolet_node = weldolet_node
        self._parent_pipe = parent_pipe
        self._branch_builder: BranchBuilder | None = None

    def __enter__(self) -> BranchBuilder:
        self._branch_builder = BranchBuilder(self._builder, self._weldolet_node, "branch")
        return self._branch_builder

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        # Check if branch was defined
        if exc_type is None and "branch" not in self._weldolet_node.children:
            warnings.warn(
                "Weldolet branch not defined. It will be an open end.",
                stacklevel=2,
            )
            self._weldolet_node.add_child("branch", TerminalNode(end_type="open"))


# =============================================================================
# MAIN ROUTE BUILDER
# =============================================================================


class RouteBuilder:
    """
    Fluent builder for constructing branching pipe routes.

    Supports method chaining and context managers for branch definition.
    Configuration can be set via constructor or chainable setter methods.

    Example:
        # Constructor defaults
        builder = RouteBuilder(nps="2", schedule="40", flange_class=300)

        # Or chainable setters
        builder = RouteBuilder(nps="2").schedule("80").material("Carbon Steel")

        # Build the route
        builder.start_with_flange("east")
        builder.add_pipe("10 ft")
        route = builder.build()
    """

    def __init__(
        self,
        nps: str = "2",
        schedule: str = "40",
        flange_class: int = 300,
        material: str = "316 Stainless Steel",
        show_coordinate_frames: bool = False,
    ):
        """
        Initialize the route builder.

        Args:
            nps: Nominal pipe size (e.g., "2", "1.5", "4")
            schedule: Pipe schedule (e.g., "40", "80", "160")
            flange_class: Flange pressure class (e.g., 150, 300, 600)
            material: Material description
            show_coordinate_frames: If True, add debug coordinate frame markers at ports
        """
        self._initial_nps = nps  # Store original for route building
        self._nps = nps  # Current NPS (changes after reducers)
        self._schedule = schedule
        self._flange_class = flange_class
        self._material = material
        self._show_coordinate_frames = show_coordinate_frames

        self._root: RouteNode | None = None
        self._current_node: RouteNode | None = None
        self._current_port: str | None = None

    # -------------------------------------------------------------------------
    # Configuration Setters (Chainable)
    # -------------------------------------------------------------------------

    def set_nps(self, nps: str) -> RouteBuilder:
        """
        Set the nominal pipe size.

        Args:
            nps: Pipe size (e.g., "2", "1.5", "4")

        Returns:
            Self for method chaining
        """
        self._initial_nps = nps
        self._nps = nps
        return self

    def set_schedule(self, schedule: str) -> RouteBuilder:
        """
        Set the pipe schedule.

        Args:
            schedule: Schedule (e.g., "40", "80", "160")

        Returns:
            Self for method chaining
        """
        self._schedule = schedule
        return self

    def set_flange_class(self, flange_class: int) -> RouteBuilder:
        """
        Set the flange pressure class.

        Args:
            flange_class: Class (e.g., 150, 300, 600)

        Returns:
            Self for method chaining
        """
        self._flange_class = flange_class
        return self

    def set_material(self, material: str) -> RouteBuilder:
        """
        Set the material description.

        Args:
            material: Material (e.g., "316 Stainless Steel")

        Returns:
            Self for method chaining
        """
        self._material = material
        return self

    def set_show_coordinate_frames(self, show: bool = True) -> RouteBuilder:
        """
        Enable/disable coordinate frame markers for debugging.

        When enabled, adds X (red), Y (green), Z (blue) arrow markers
        at each fitting port in the exported STEP file.

        Args:
            show: Whether to show coordinate frames

        Returns:
            Self for method chaining
        """
        self._show_coordinate_frames = show
        return self

    # -------------------------------------------------------------------------
    # Route Building Methods
    # -------------------------------------------------------------------------

    def start_with_flange(
        self,
        direction: str = "east",
        flange_class: int | None = None,
        bolt_hole_orientation: BoltHoleOrientation = "single_hole",
        name: str | None = None,
    ) -> RouteBuilder:
        """
        Begin the route with a weld-neck flange.

        The flange is oriented so pipe extends in the specified direction.

        Args:
            direction: Direction pipe extends from flange (e.g., "east", "up")
            flange_class: Override default flange class
            bolt_hole_orientation: Bolt hole orientation relative to the pipe direction.
                - "single_hole": One bolt hole lies on the reference plane
                - "two_hole": Two bolt holes symmetric about the reference plane
            name: Optional name for this flange

        Returns:
            Self for method chaining
        """
        if self._root is not None:
            raise RuntimeError("Route already has a starting point")

        if flange_class is None:
            flange_class = self._flange_class

        self._root = FlangeNode(
            nps=self._nps,
            flange_class=flange_class,
            direction=direction,
            bolt_hole_orientation=bolt_hole_orientation,
            name=name,
        )
        self._current_node = self._root
        self._current_port = "weld"
        return self

    def add_pipe(self, length: str, name: str | None = None) -> RouteBuilder:
        """
        Add a pipe segment at the current position.

        Args:
            length: Pipe length (e.g., "10 ft", "3048 mm")
            name: Optional name for this segment

        Returns:
            Self for method chaining
        """
        self._ensure_started()
        assert self._current_node is not None and self._current_port is not None

        node = PipeSegmentNode(
            length=length,
            schedule=self._schedule,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._current_node = node
        self._current_port = "end"
        return self

    def add_elbow(
        self,
        turn_to: str,
        weld_type: WeldType = "sw",
        name: str | None = None,
    ) -> RouteBuilder:
        """
        Add an elbow turning toward the specified direction.

        Args:
            turn_to: Direction after the elbow (e.g., "up", "north")
            weld_type: "sw" for socket weld, "bw" for butt weld
            name: Optional name for this elbow

        Returns:
            Self for method chaining
        """
        self._ensure_started()
        assert self._current_node is not None and self._current_port is not None

        node = ElbowNode(
            weld_type=weld_type,
            turn_direction=turn_to,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._current_node = node
        self._current_port = "outlet"
        return self

    @overload
    def add_tee(
        self,
        branch_direction: str,
        weld_type: Literal["sw"] = ...,
        name: str | None = ...,
    ) -> TeeContext: ...

    @overload
    def add_tee(
        self,
        branch_direction: str,
        weld_type: Literal["bw"],
        name: str | None = ...,
    ) -> TeeContext: ...

    def add_tee(
        self,
        branch_direction: str,
        weld_type: WeldType = "sw",
        name: str | None = None,
    ) -> TeeContext:
        """
        Add a tee fitting. Returns a context for defining branches.

        Usage:
            with builder.add_tee("north") as tee:
                tee.run().add_pipe("5 ft").add_cap()
                tee.branch().add_pipe("3 ft").add_cap()

        Args:
            branch_direction: World direction of the branch outlet
            weld_type: "sw" for socket weld, "bw" for butt weld
            name: Optional name for this tee

        Returns:
            TeeContext for defining run and branch paths
        """
        self._ensure_started()
        assert self._current_node is not None and self._current_port is not None

        node = TeeNode(
            weld_type=weld_type,
            branch_direction=branch_direction,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        # Reset current position since branches take over
        self._current_node = None
        self._current_port = None
        return TeeContext(self, node)

    @overload
    def add_cross(
        self,
        orientation: str = ...,
        weld_type: Literal["sw"] = ...,
        name: str | None = ...,
    ) -> CrossContext: ...

    @overload
    def add_cross(
        self,
        orientation: str = ...,
        weld_type: Literal["bw"] = ...,
        name: str | None = ...,
    ) -> CrossContext: ...

    def add_cross(
        self,
        orientation: str = "up",
        weld_type: WeldType = "sw",
        name: str | None = None,
    ) -> CrossContext:
        """
        Add a cross fitting. Returns a context for defining branches.

        Usage:
            with builder.add_cross(orientation="up") as cross:
                cross.run().add_pipe("5 ft").add_cap()
                cross.branch_left().add_pipe("3 ft").add_cap()
                cross.branch_right().add_pipe("3 ft").add_cap()

        Args:
            orientation: Defines the plane of perpendicular branches
            weld_type: "sw" for socket weld, "bw" for butt weld
            name: Optional name for this cross

        Returns:
            CrossContext for defining run, left, and right branches
        """
        self._ensure_started()
        assert self._current_node is not None and self._current_port is not None

        node = CrossNode(
            weld_type=weld_type,
            orientation=orientation,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._current_node = None
        self._current_port = None
        return CrossContext(self, node)

    def add_weldolet(
        self,
        branch_direction: str,
        position: str,
        fitting_type: Literal["weldolet", "sockolet", "threadolet"] = "sockolet",
        branch_nps: str | None = None,
        name: str | None = None,
    ) -> WeldoletContext:
        """
        Add a weldolet/sockolet at a position along the previous pipe.

        Note: This must be called immediately after add_pipe().

        Usage:
            builder.add_pipe("20 ft")
            with builder.add_weldolet("up", position="10 ft") as weldolet:
                weldolet.add_pipe("5 ft").add_cap()

        Args:
            branch_direction: Direction the branch extends
            position: Position along the pipe (e.g., "10 ft")
            fitting_type: Type of branch fitting
            branch_nps: NPS of branch (defaults to main pipe NPS)
            name: Optional name

        Returns:
            WeldoletContext for defining the branch
        """
        # Weldolets attach to the most recent pipe segment
        if not isinstance(self._current_node, PipeSegmentNode):
            raise ValueError(f"Weldolets can only be added immediately after a pipe segment. Current node is: {type(self._current_node).__name__}")

        parent_pipe = self._current_node
        assert isinstance(parent_pipe, PipeSegmentNode)

        node = WeldoletNode(
            branch_direction=branch_direction,
            position_along_pipe=position,
            fitting_type=fitting_type,
            branch_nps=branch_nps,
            name=name,
        )
        # Weldolets attach to the pipe segment's weldolets list
        parent_pipe.weldolets.append(node)

        return WeldoletContext(self, node, parent_pipe)

    def add_cap(self, name: str | None = None) -> RouteBuilder:
        """
        Terminate the current path with a cap.

        Args:
            name: Optional name for the cap

        Returns:
            Self for method chaining
        """
        self._ensure_started()
        assert self._current_node is not None and self._current_port is not None

        node = TerminalNode(end_type="cap", name=name)
        self._current_node.add_child(self._current_port, node)
        self._current_node = None
        self._current_port = None
        return self

    def add_blind_flange(self, name: str | None = None) -> RouteBuilder:
        """
        Terminate the current path with a blind flange.

        Args:
            name: Optional name for the blind flange

        Returns:
            Self for method chaining
        """
        self._ensure_started()
        assert self._current_node is not None and self._current_port is not None

        node = TerminalNode(end_type="blind_flange", name=name)
        self._current_node.add_child(self._current_port, node)
        self._current_node = None
        self._current_port = None
        return self

    def add_flange(
        self,
        direction: str = "east",
        flange_class: int | None = None,
        bolt_hole_orientation: BoltHoleOrientation = "single_hole",
        name: str | None = None,
    ) -> RouteBuilder:
        """
        Add a weld neck flange at the current position.

        This terminates the current path with a flange.

        Args:
            direction: Orientation direction for the flange face
            flange_class: Override default flange class
            bolt_hole_orientation: Bolt hole orientation relative to the pipe direction.
                - "single_hole": One bolt hole lies on the reference plane
                - "two_hole": Two bolt holes symmetric about the reference plane
            name: Optional name

        Returns:
            Self for method chaining
        """
        self._ensure_started()
        assert self._current_node is not None and self._current_port is not None

        if flange_class is None:
            flange_class = self._flange_class

        node = FlangeNode(
            nps=self._nps,
            flange_class=flange_class,
            direction=direction,
            bolt_hole_orientation=bolt_hole_orientation,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._current_node = None
        self._current_port = None
        return self

    def add_reducer(
        self,
        target_nps: str,
        reducer_type: Literal["concentric", "eccentric"] = "concentric",
        weld_type: WeldType = "bw",
        name: str | None = None,
    ) -> RouteBuilder:
        """
        Add a reducer to change pipe size.

        Args:
            target_nps: Target nominal pipe size after reducer
            reducer_type: "concentric" or "eccentric"
            weld_type: Weld type (typically butt weld for reducers)
            name: Optional name

        Returns:
            Self for method chaining
        """
        self._ensure_started()
        assert self._current_node is not None and self._current_port is not None

        node = ReducerNode(
            target_nps=target_nps,
            reducer_type=reducer_type,
            weld_type=weld_type,
            name=name,
        )
        self._current_node.add_child(self._current_port, node)
        self._current_node = node
        self._current_port = "outlet"
        # Update NPS for subsequent segments
        self._nps = target_nps
        return self

    # -------------------------------------------------------------------------
    # Build
    # -------------------------------------------------------------------------

    def build(self) -> BranchingPipeRoute:
        """
        Build the complete route geometry.

        Returns:
            BranchingPipeRoute with geometry and tracking data

        Raises:
            ValueError: If no route has been defined
        """
        if self._root is None:
            raise ValueError("No route defined. Call start_with_flange() first.")

        # Create the route container (use initial NPS, not current after reducers)
        route = BranchingPipeRoute(
            nps=self._initial_nps,
            schedule=self._schedule,
            flange_class=self._flange_class,
            material=self._material,
            root=self._root,
            show_coordinate_frames=self._show_coordinate_frames,
        )

        # Import and use the geometry builder
        from .route_geometry_builder import RouteGeometryBuilder

        builder = RouteGeometryBuilder(route)
        builder.build()

        route._built = True
        return route

    # -------------------------------------------------------------------------
    # Private Helpers
    # -------------------------------------------------------------------------

    def _ensure_started(self) -> None:
        """Ensure the route has been started."""
        if self._root is None:
            raise RuntimeError("Route has not been started. Call start_with_flange() first.")
        if self._current_node is None or self._current_port is None:
            raise RuntimeError("No current attachment point. The previous fitting may have been a terminus (cap, flange) or a junction (tee, cross) that requires using the context manager.")
