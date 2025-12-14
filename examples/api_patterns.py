#!/usr/bin/env python3
"""
Example: Two API Patterns for Branching Routes

This example demonstrates both API patterns:
1. Context Manager Pattern - inline branch definition with `with` statements
2. Separate Route Objects Pattern - reusable helper functions

Both patterns can be combined for maximum flexibility.
"""

from pathlib import Path

from pypegen.route_builder import BranchBuilder, RouteBuilder

# Output directory for STEP files
OUTPUT_DIR = Path(__file__).parent / "output" / "step"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


# =============================================================================
# PATTERN 1: Context Manager (Inline Definition)
# =============================================================================

def context_manager_example():
    """
    Context Manager Pattern - define branches inline with `with` statements.

    Good for:
    - One-off routes
    - Visual nesting that matches physical structure
    - Quick prototyping
    """
    print("=" * 60)
    print("PATTERN 1: Context Manager (Inline)")
    print("=" * 60)

    builder = RouteBuilder(nps="2")
    builder.start_with_flange("east")
    builder.add_pipe("10 ft")

    # Branches defined inline within context manager
    with builder.add_tee("north") as tee:

        # Run branch - continues east
        run = tee.run()
        run.add_pipe("5 ft")
        run.add_elbow("up")
        run.add_pipe("3 ft")
        run.add_cap()

        # Side branch - goes north
        branch = tee.branch()
        branch.add_pipe("4 ft")
        branch.add_elbow("west")
        branch.add_pipe("2 ft")
        branch.add_cap()

    route = builder.build()
    print(f"Parts: {len(route.parts)}, Components: {len(route.components)}")
    route.export(str(OUTPUT_DIR / "pattern1_context_manager.step"))
    print(f"Exported: {OUTPUT_DIR / 'pattern1_context_manager.step'}\n")
    return route


# =============================================================================
# PATTERN 2: Separate Route Objects (Reusable Functions)
# =============================================================================

# Define reusable branch patterns as functions
def drain_leg(branch: BranchBuilder) -> None:
    """
    Standard drain leg pattern.

    Goes down 2ft, turns, drops 1ft to a cap.
    """
    branch.add_pipe("2 ft")
    branch.add_elbow("down")
    branch.add_pipe("1 ft")
    branch.add_cap()


def vent_riser(branch: BranchBuilder) -> None:
    """
    Standard vent riser pattern.

    Goes up with an elbow, rises 5ft to atmosphere.
    """
    branch.add_pipe("1 ft")
    branch.add_elbow("up")
    branch.add_pipe("5 ft")
    branch.add_cap()


def instrument_connection(branch: BranchBuilder) -> None:
    """
    Standard instrument tap pattern.

    Short stub with a flange for instrument mounting.
    """
    branch.add_pipe("6 in")
    branch.add_flange("up")


def bypass_loop(branch: BranchBuilder, length: str = "3 ft") -> None:
    """
    Bypass loop pattern with configurable length.

    U-shaped bypass around equipment.
    """
    branch.add_pipe("1 ft")
    branch.add_elbow("up")
    branch.add_pipe(length)
    branch.add_elbow("east")
    branch.add_pipe("1 ft")
    branch.add_cap()


def separate_route_objects_example():
    """
    Separate Route Objects Pattern - use reusable helper functions.

    Good for:
    - Standardized patterns used across many routes
    - Testing patterns in isolation
    - Enforcing design standards
    """
    print("=" * 60)
    print("PATTERN 2: Separate Route Objects (Reusable)")
    print("=" * 60)

    builder = RouteBuilder(nps="2")
    builder.start_with_flange("east")
    builder.add_pipe("15 ft")

    # First tee - use drain_leg pattern
    with builder.add_tee("down") as tee1:
        tee1.run().add_pipe("10 ft")  # Continue to next tee
        drain_leg(tee1.branch())  # Reusable pattern!

    # Can't continue after tee context, so build what we have
    route = builder.build()
    print(f"Parts: {len(route.parts)}, Components: {len(route.components)}")
    route.export(str(OUTPUT_DIR / "pattern2_separate_objects.step"))
    print(f"Exported: {OUTPUT_DIR / 'pattern2_separate_objects.step'}\n")

    # Another route using different patterns
    builder2 = RouteBuilder(nps="2")
    builder2.start_with_flange("east")
    builder2.add_pipe("10 ft")

    with builder2.add_tee("up") as tee:
        tee.run().add_pipe("5 ft").add_cap()
        vent_riser(tee.branch())  # Same function, different route!

    route2 = builder2.build()
    print(f"Parts: {len(route2.parts)}, Components: {len(route2.components)}")
    route2.export(str(OUTPUT_DIR / "pattern2_vent_riser.step"))
    print(f"Exported: {OUTPUT_DIR / 'pattern2_vent_riser.step'}\n")

    return route, route2


# =============================================================================
# PATTERN 3: Combination (Best of Both)
# =============================================================================

def combined_pattern_example():
    """
    Combined Pattern - context managers for structure, functions for reuse.

    Good for:
    - Complex routes with both unique and standard sections
    - Maximum flexibility
    - Production use
    """
    print("=" * 60)
    print("PATTERN 3: Combined (Context + Reusable)")
    print("=" * 60)

    builder = RouteBuilder(nps="2")
    builder.start_with_flange("east")
    builder.add_pipe("20 ft")

    # Outer tee - mix inline and reusable
    with builder.add_tee("north") as main_tee:

        # Run continues with nested structure (inline)
        with main_tee.run().add_pipe("10 ft").add_tee("down") as inner_tee:
            inner_tee.run().add_pipe("5 ft").add_cap()
            drain_leg(inner_tee.branch())  # Reusable pattern

        # North branch has instrument tap (reusable)
        instrument_connection(main_tee.branch())

    route = builder.build()
    print(f"Parts: {len(route.parts)}, Components: {len(route.components)}")
    route.export(str(OUTPUT_DIR / "pattern3_combined.step"))
    print(f"Exported: {OUTPUT_DIR / 'pattern3_combined.step'}\n")
    return route


# =============================================================================
# BONUS: Cross Fitting with Multiple Patterns
# =============================================================================

def cross_with_patterns_example():
    """
    Cross fitting using different patterns on each branch.
    """
    print("=" * 60)
    print("BONUS: Cross with Multiple Patterns")
    print("=" * 60)

    builder = RouteBuilder(nps="2")
    builder.start_with_flange("east")
    builder.add_pipe("10 ft")

    with builder.add_cross(orientation="up") as cross:
        # Run - inline definition
        run = cross.run()
        run.add_pipe("8 ft")
        run.add_cap()

        # Left branch - reusable vent
        vent_riser(cross.branch_left())

        # Right branch - reusable drain
        drain_leg(cross.branch_right())

    route = builder.build()
    print(f"Parts: {len(route.parts)}, Components: {len(route.components)}")
    route.export(str(OUTPUT_DIR / "pattern_cross.step"))
    print(f"Exported: {OUTPUT_DIR / 'pattern_cross.step'}\n")
    return route


# =============================================================================
# MAIN
# =============================================================================

def main():
    print("\n" + "=" * 60)
    print("API PATTERNS DEMONSTRATION")
    print("=" * 60 + "\n")

    context_manager_example()
    separate_route_objects_example()
    combined_pattern_example()
    cross_with_patterns_example()

    print("=" * 60)
    print("All examples completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
