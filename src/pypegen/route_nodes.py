#!/usr/bin/env python3
"""
Route Node Hierarchy for Branching Pipe Routes

This module defines a tree-based data model for pipe routes that support
branching at tees, crosses, and mid-pipe weldolets.

Each RouteNode represents a fitting or pipe segment in the route tree.
Nodes connect via named outlet ports (e.g., "run", "branch" for a tee).

Example:
    FlangeNode (outlet: "weld")
        └── PipeSegmentNode (outlet: "end")
                └── TeeNode (outlets: "run", "branch")
                        ├── run: PipeSegmentNode -> TerminalNode
                        └── branch: ElbowNode -> PipeSegmentNode -> TerminalNode
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, TypeAlias

import numpy as np

if TYPE_CHECKING:
    from .fittings.pipe_fittings import Fitting


# =============================================================================
# TYPE ALIASES FOR PORT NAMES
# =============================================================================

TeePort: TypeAlias = Literal["run", "branch"]
CrossPort: TypeAlias = Literal["run", "branch_left", "branch_right"]
WeldoletPort: TypeAlias = Literal["branch"]

# Direction literals for type hints
Direction: TypeAlias = Literal["east", "west", "north", "south", "up", "down", "+x", "-x", "+y", "-y", "+z", "-z"]

# Weld types
WeldType: TypeAlias = Literal["sw", "bw"]  # socket weld, butt weld

# Terminal types
TerminalType: TypeAlias = Literal["cap", "blind_flange", "open"]


# =============================================================================
# BASE CLASS
# =============================================================================


@dataclass
class RouteNode(ABC):
    """
    Abstract base class for all nodes in the route tree.

    Each node represents a fitting or pipe segment. Nodes connect to child
    nodes via named outlet ports. During build(), geometry is created and
    world transforms are computed.

    Attributes:
        name: Optional human-readable name for this node
        children: Child nodes keyed by outlet port name
        _world_transform: 4x4 matrix set during build (None until built)
        _fitting: Fitting object set during build (None until built)
        _built: True after geometry has been built
    """

    name: str | None = None
    children: dict[str, RouteNode] = field(default_factory=dict)

    # Set during build phase
    _world_transform: np.ndarray | None = field(default=None, repr=False)
    _fitting: Fitting | None = field(default=None, repr=False)
    _built: bool = field(default=False, repr=False)

    @abstractmethod
    def get_outlet_ports(self) -> list[str]:
        """Return names of ports available for child connections."""

    def add_child(self, port_name: str, node: RouteNode) -> RouteNode:
        """
        Attach a child node to the specified outlet port.

        Args:
            port_name: Name of the outlet port (e.g., "run", "branch")
            node: Child RouteNode to attach

        Returns:
            The attached child node (for chaining)

        Raises:
            ValueError: If port_name is not a valid outlet or already has a child
        """
        available = self.get_outlet_ports()
        if port_name not in available:
            raise ValueError(f"Port '{port_name}' not available on {self.__class__.__name__}. Valid ports: {available}")
        if port_name in self.children:
            raise ValueError(f"Port '{port_name}' already has a child attached on {self.__class__.__name__}")
        self.children[port_name] = node
        return node

    def get_child(self, port_name: str) -> RouteNode | None:
        """Get the child node at the specified port, or None if empty."""
        return self.children.get(port_name)

    @property
    def world_transform(self) -> np.ndarray:
        """Get the world transform (raises if not built yet)."""
        if self._world_transform is None:
            raise RuntimeError(f"{self.__class__.__name__} has not been built yet. Call route.build() first.")
        return self._world_transform

    @property
    def fitting(self) -> Fitting:
        """Get the fitting object (raises if not built yet)."""
        if self._fitting is None:
            raise RuntimeError(f"{self.__class__.__name__} has not been built yet. Call route.build() first.")
        return self._fitting


# =============================================================================
# CONCRETE NODE TYPES
# =============================================================================


@dataclass
class FlangeNode(RouteNode):
    """
    Starting flange node (weld neck flange).

    The flange is oriented so its weld port faces the specified direction,
    allowing pipe to extend in that direction.

    Outlets:
        - "weld": Connection point for pipe extending from flange hub
    """

    nps: str = "2"
    flange_class: int = 300
    direction: str = "east"  # Direction the pipe will go from this flange

    def get_outlet_ports(self) -> list[str]:
        return ["weld"]


@dataclass
class PipeSegmentNode(RouteNode):
    """
    Straight pipe segment.

    Outlets:
        - "end": Connection point at the far end of the pipe

    Weldolets can be attached via the weldolets list.
    """

    length: str = "10 ft"  # Supports parse_distance() format
    schedule: str = "40"

    # Mid-pipe branch fittings (weldolets/sockolets)
    weldolets: list[WeldoletNode] = field(default_factory=list)

    def get_outlet_ports(self) -> list[str]:
        return ["end"]


@dataclass
class ElbowNode(RouteNode):
    """
    90-degree elbow fitting.

    Outlets:
        - "outlet": Connection point after the turn
    """

    weld_type: WeldType = "sw"
    turn_direction: str = "up"  # World direction after the elbow

    def get_outlet_ports(self) -> list[str]:
        return ["outlet"]


@dataclass
class TeeNode(RouteNode):
    """
    Tee fitting - 3 ports total (1 inlet, 2 outlets).

    The inlet receives pipe from the current direction. The run continues
    straight through, and the branch exits at 90 degrees.

    Outlets:
        - "run": Continues straight through the tee
        - "branch": 90-degree outlet in branch_direction
    """

    weld_type: WeldType = "sw"
    branch_direction: str = "north"  # World direction of the branch port

    def get_outlet_ports(self) -> list[str]:
        return ["run", "branch"]


@dataclass
class CrossNode(RouteNode):
    """
    Cross fitting - 4 ports total (1 inlet, 3 outlets).

    The inlet receives pipe from the current direction. The run continues
    straight through, and two branches exit at 90 degrees left and right.

    Outlets:
        - "run": Continues straight through the cross
        - "branch_left": 90-degree outlet to the left
        - "branch_right": 90-degree outlet to the right
    """

    weld_type: WeldType = "sw"
    orientation: str = "up"  # Defines the plane of the cross (perpendicular branches)

    def get_outlet_ports(self) -> list[str]:
        return ["run", "branch_left", "branch_right"]


@dataclass
class WeldoletNode(RouteNode):
    """
    Mid-pipe branch fitting (weldolet/sockolet/threadolet).

    This node attaches to a PipeSegmentNode at a specific position along
    the pipe, creating a branch perpendicular to the run.

    Outlets:
        - "branch": Connection point for the branch pipe
    """

    branch_direction: str = "up"
    position_along_pipe: str = "5 ft"  # Distance from pipe start
    fitting_type: Literal["weldolet", "sockolet", "threadolet"] = "sockolet"
    branch_nps: str | None = None  # If None, uses parent pipe NPS

    def get_outlet_ports(self) -> list[str]:
        return ["branch"]


@dataclass
class ReducerNode(RouteNode):
    """
    Concentric or eccentric reducer.

    Changes pipe size from the current NPS to the specified target NPS.

    Outlets:
        - "outlet": Connection point at the reduced end
    """

    target_nps: str = "1.5"
    reducer_type: Literal["concentric", "eccentric"] = "concentric"
    weld_type: WeldType = "bw"

    def get_outlet_ports(self) -> list[str]:
        return ["outlet"]


@dataclass
class TerminalNode(RouteNode):
    """
    End point of a pipe run (cap, blind flange, or open end).

    Outlets: None (this is a terminus)
    """

    end_type: TerminalType = "open"

    def get_outlet_ports(self) -> list[str]:
        return []  # No outlets - this is a terminus


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================


def count_nodes(node: RouteNode) -> int:
    """Count total nodes in the tree rooted at the given node."""
    count = 1
    for child in node.children.values():
        count += count_nodes(child)
    return count


def iter_nodes(node: RouteNode) -> list[RouteNode]:
    """Iterate over all nodes in the tree (depth-first)."""
    nodes = [node]
    for child in node.children.values():
        nodes.extend(iter_nodes(child))
    return nodes


def find_node_by_name(node: RouteNode, name: str) -> RouteNode | None:
    """Find a node by name in the tree."""
    if node.name == name:
        return node
    for child in node.children.values():
        found = find_node_by_name(child, name)
        if found is not None:
            return found
    return None
