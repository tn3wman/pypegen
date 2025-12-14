#!/usr/bin/env python3
"""
Example: Branching Pipe Route with Tees and Crosses

This example demonstrates the new RouteBuilder API with support for
branching at tees, crosses, and mid-pipe weldolets.
"""

from pathlib import Path

from pypegen.route_builder import BranchBuilder, RouteBuilder

# Output directory for STEP files
OUTPUT_DIR = Path(__file__).parent / "output" / "step"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def simple_tee_example():
    """Simple route with a single tee."""
    print("Building simple tee example...")

    builder = RouteBuilder(nps="2", schedule="40", flange_class=300)
    builder.start_with_flange("east")
    builder.add_pipe("10 ft")

    # Add a tee with branches defined in a context manager
    with builder.add_tee(branch_direction="north") as tee:
        # Run continues straight (east)
        tee.run().add_pipe("5 ft").add_cap()

        # Branch goes north
        tee.branch().add_pipe("3 ft").add_elbow("up").add_pipe("2 ft").add_cap()

    route = builder.build()
    print(f"  Parts: {len(route.parts)}")
    print(f"  Components: {len(route.components)}")
    route.export(str(OUTPUT_DIR / "simple_tee.step"))
    print(f"  Exported to {OUTPUT_DIR / 'simple_tee.step'}")
    return route


def nested_tee_example():
    """Route with nested tees (3+ levels deep)."""
    print("Building nested tee example...")

    builder = RouteBuilder(nps="2")
    builder.start_with_flange("east")
    builder.add_pipe("10 ft")

    # Level 1 tee
    with builder.add_tee(branch_direction="north") as tee1:
        # Level 2 tee on the run
        with tee1.run().add_pipe("5 ft").add_tee(branch_direction="up") as tee2:
            # Level 3 - end the run
            tee2.run().add_pipe("3 ft").add_cap()

            # Level 3 tee on the branch
            with tee2.branch().add_pipe("2 ft").add_tee(branch_direction="west") as tee3:
                # Level 4 - end both branches
                tee3.run().add_pipe("2 ft").add_cap()
                tee3.branch().add_pipe("1 ft").add_cap()

        # Level 2 - north branch
        tee1.branch().add_pipe("4 ft").add_cap()

    route = builder.build()
    print(f"  Parts: {len(route.parts)}")
    print(f"  Components: {len(route.components)}")
    route.export(str(OUTPUT_DIR / "nested_tees.step"))
    print(f"  Exported to {OUTPUT_DIR / 'nested_tees.step'}")
    return route


def cross_example():
    """Route with a cross fitting."""
    print("Building cross example...")

    builder = RouteBuilder(nps="2")
    builder.start_with_flange("east")
    builder.add_pipe("10 ft")

    # Add a cross with three outlet branches
    with builder.add_cross(orientation="up") as cross:
        # Run continues east
        cross.run().add_pipe("5 ft").add_cap()

        # Left branch (north)
        cross.branch_left().add_pipe("3 ft").add_elbow("up").add_pipe("2 ft").add_cap()

        # Right branch (south)
        cross.branch_right().add_pipe("3 ft").add_cap()

    route = builder.build()
    print(f"  Parts: {len(route.parts)}")
    print(f"  Components: {len(route.components)}")
    route.export(str(OUTPUT_DIR / "cross_fitting.step"))
    print(f"  Exported to {OUTPUT_DIR / 'cross_fitting.step'}")
    return route


def weldolet_example():
    """Route with a mid-pipe weldolet branch."""
    print("Building weldolet example...")

    builder = RouteBuilder(nps="4", schedule="40")
    builder.start_with_flange("east")
    builder.add_pipe("20 ft")

    # Add a 2" weldolet at 10 ft along the pipe
    with builder.add_weldolet(
        branch_direction="up",
        position="10 ft",
        fitting_type="sockolet",
        branch_nps="2",
    ) as weldolet:
        weldolet.add_pipe("5 ft").add_cap()

    # Continue the main run and cap it
    builder.add_cap()

    route = builder.build()
    print(f"  Parts: {len(route.parts)}")
    print(f"  Components: {len(route.components)}")
    route.export(str(OUTPUT_DIR / "weldolet_branch.step"))
    print(f"  Exported to {OUTPUT_DIR / 'weldolet_branch.step'}")
    return route


def composable_pattern_example():
    """Demonstrate composable/reusable route patterns."""
    print("Building composable pattern example...")

    def create_drain_leg(branch: "BranchBuilder") -> None:
        """Reusable drain leg pattern."""
        branch.add_pipe("2 ft").add_elbow("down").add_pipe("1 ft").add_cap()

    def create_vent_riser(branch: "BranchBuilder") -> None:
        """Reusable vent riser pattern."""
        branch.add_pipe("3 ft").add_elbow("up").add_pipe("5 ft").add_cap()

    builder = RouteBuilder(nps="2")
    builder.start_with_flange("east")
    builder.add_pipe("15 ft")

    # Use composable patterns
    with builder.add_tee(branch_direction="down") as tee1:
        tee1.run().add_pipe("10 ft")

        # Note: After add_pipe we can't continue chaining after add_tee
        # because add_tee takes over flow control
        create_drain_leg(tee1.branch())

    # Continue with another tee
    builder2 = RouteBuilder(nps="2")
    builder2.start_with_flange("east")
    builder2.add_pipe("15 ft")

    with builder2.add_tee(branch_direction="up") as tee2:
        tee2.run().add_pipe("5 ft").add_cap()
        create_vent_riser(tee2.branch())

    route = builder2.build()
    print(f"  Parts: {len(route.parts)}")
    print(f"  Components: {len(route.components)}")
    route.export(str(OUTPUT_DIR / "composable_pattern.step"))
    print(f"  Exported to {OUTPUT_DIR / 'composable_pattern.step'}")
    return route


def chainable_config_example():
    """Demonstrate chainable configuration setters."""
    print("Building chainable config example...")

    # Constructor defaults + chainable setters
    builder = (
        RouteBuilder(nps="2")
        .set_schedule("80")
        .set_flange_class(600)
        .set_material("Carbon Steel")
    )

    builder.start_with_flange("east")
    builder.add_pipe("10 ft")
    builder.add_elbow("up")
    builder.add_pipe("5 ft")
    builder.add_cap()

    route = builder.build()
    print(f"  Parts: {len(route.parts)}")
    print(f"  Material: {route.material}")
    print(f"  Schedule: {route.schedule}")
    route.export(str(OUTPUT_DIR / "chainable_config.step"))
    print(f"  Exported to {OUTPUT_DIR / 'chainable_config.step'}")
    return route


def main():
    """Run all examples."""
    print("=" * 60)
    print("Branching Pipe Route Examples")
    print("=" * 60)
    print()

    try:
        simple_tee_example()
        print()

        nested_tee_example()
        print()

        cross_example()
        print()

        weldolet_example()
        print()

        composable_pattern_example()
        print()

        chainable_config_example()
        print()

        print("=" * 60)
        print("All examples completed successfully!")
        print("=" * 60)

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
