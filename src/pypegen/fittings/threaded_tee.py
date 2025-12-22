"""
ASME B16.11 NPT Threaded Tee Generator using CadQuery.

This module creates NPT threaded equal tees per ASME B16.11 dimensions.
The tee has three ports: inlet (run-in), run (run-out), and branch.

Sources:
- https://ferrobend.com/dimensions/ansi-asme/pipe-fitting/b16.11-threaded-class-3000-tee/
"""

from __future__ import annotations

from dataclasses import dataclass

import cadquery as cq

from pypegen.threads.npt_female import get_npt_spec, make_npt_internal_thread
from .threaded_elbow import NPT_THREAD_ENGAGEMENT, PIPE_OD

# =============================================================================
# DATA STRUCTURES
# =============================================================================


@dataclass
class ThreadedTeeDims:
    """ASME B16.11 Threaded Tee dimensions (all in mm).

    For an equal tee, the run and branch dimensions are the same.
    """

    center_to_end: float  # A - center to thread end (same for run and branch)
    band_od: float  # H - outside diameter of band/body
    wall_thickness: float  # G - minimum wall thickness
    min_thread_length: float  # B - minimum length of perfect thread


# ASME B16.11 Class 3000 Threaded Equal Tee dimension table
ASME_B1611_THREADED_TEE_CLASS3000: dict[str, ThreadedTeeDims] = {
    "1/8": ThreadedTeeDims(
        center_to_end=21.0,
        band_od=22.0,
        wall_thickness=3.18,
        min_thread_length=6.4,
    ),
    "1/4": ThreadedTeeDims(
        center_to_end=25.0,
        band_od=25.0,
        wall_thickness=3.30,
        min_thread_length=8.1,
    ),
    "3/8": ThreadedTeeDims(
        center_to_end=28.0,
        band_od=33.0,
        wall_thickness=3.51,
        min_thread_length=9.1,
    ),
    "1/2": ThreadedTeeDims(
        center_to_end=33.0,
        band_od=38.0,
        wall_thickness=4.09,
        min_thread_length=10.9,
    ),
    "3/4": ThreadedTeeDims(
        center_to_end=38.0,
        band_od=46.0,
        wall_thickness=4.32,
        min_thread_length=12.7,
    ),
    "1": ThreadedTeeDims(
        center_to_end=44.0,
        band_od=56.0,
        wall_thickness=4.98,
        min_thread_length=14.7,
    ),
    "1-1/4": ThreadedTeeDims(
        center_to_end=51.0,
        band_od=62.0,
        wall_thickness=5.28,
        min_thread_length=17.0,
    ),
    "1-1/2": ThreadedTeeDims(
        center_to_end=60.0,
        band_od=75.0,
        wall_thickness=5.56,
        min_thread_length=17.8,
    ),
    "2": ThreadedTeeDims(
        center_to_end=64.0,
        band_od=84.0,
        wall_thickness=7.14,
        min_thread_length=19.0,
    ),
    "2-1/2": ThreadedTeeDims(
        center_to_end=83.0,
        band_od=102.0,
        wall_thickness=7.65,
        min_thread_length=23.6,
    ),
    "3": ThreadedTeeDims(
        center_to_end=95.0,
        band_od=121.0,
        wall_thickness=8.84,
        min_thread_length=25.9,
    ),
    "4": ThreadedTeeDims(
        center_to_end=114.0,
        band_od=152.0,
        wall_thickness=11.18,
        min_thread_length=27.7,
    ),
}


# =============================================================================
# TEE GEOMETRY
# =============================================================================


def make_threaded_tee(
    nps: str,
    pressure_class: int = 3000,
    include_threads: bool = False,
) -> cq.Shape:
    """
    Create an ASME B16.11 NPT threaded equal tee.

    The tee is oriented with:
    - Inlet (run-in) from -X direction toward origin
    - Run (run-out) from origin toward +X direction
    - Branch from origin toward +Y direction

    The center of the tee body is at the origin.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        pressure_class: 3000 only currently implemented
        include_threads: If True, add actual NPT internal thread geometry

    Returns:
        CadQuery Shape of the tee
    """
    if pressure_class != 3000:
        raise ValueError(f"Pressure class {pressure_class} not implemented")

    dims = ASME_B1611_THREADED_TEE_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.11 Class {pressure_class} threaded tee dimensions for NPS {nps}")

    pipe_od = PIPE_OD.get(nps)
    if pipe_od is None:
        raise ValueError(f"No pipe OD for NPS {nps}")

    # Get dimensions
    A = dims.center_to_end  # Center to end (run and branch)

    # Radii
    r_bore = pipe_od / 2.0  # Bore accepts pipe OD
    r_outer = dims.band_od / 2.0  # Outer body radius

    # Build the tee body
    # The tee consists of:
    # 1. A horizontal cylinder (run) from -X to +X
    # 2. A vertical cylinder (branch) from origin to +Y

    # Run cylinder (along X axis)
    run_cylinder = cq.Solid.makeCylinder(
        r_outer,
        A * 2,  # Full length from -X to +X
        cq.Vector(-A, 0, 0),  # Start at -A
        cq.Vector(1, 0, 0),  # Along +X
    )

    # Branch cylinder (along Y axis)
    branch_cylinder = cq.Solid.makeCylinder(
        r_outer,
        A,  # Length from origin to +Y
        cq.Vector(0, 0, 0),  # Start at origin
        cq.Vector(0, 1, 0),  # Along +Y
    )

    # Union the cylinders to form the tee body
    tee_body = run_cylinder.fuse(branch_cylinder)

    # Cut the flow bore through all three directions
    # Run bore (X axis) - extra length to ensure full cut
    run_bore = cq.Solid.makeCylinder(
        r_bore,
        A * 2 + 10,
        cq.Vector(-A - 5, 0, 0),
        cq.Vector(1, 0, 0),
    )

    # Branch bore (Y axis)
    branch_bore = cq.Solid.makeCylinder(
        r_bore,
        A + 5,
        cq.Vector(0, -2, 0),  # Start slightly inside to ensure intersection cut
        cq.Vector(0, 1, 0),
    )

    # Cut the bores
    tee_shape = tee_body.cut(run_bore).cut(branch_bore)

    # Add internal threads if requested
    if include_threads:
        npt_spec = get_npt_spec(nps)
        thread_length = min(dims.min_thread_length, npt_spec.L2)

        # Create internal thread (generated along +Z axis, represents volume to CUT)
        # Thread starts at Z=0 (smaller opening) and expands toward Z=thread_length (larger opening)
        # When cutting, the smaller opening should be at the fitting face
        thread = make_npt_internal_thread(nps, thread_length=thread_length)

        # Inlet (run-in): Thread at -X end
        # Face at x=-A, thread extends toward center (x=0)
        # Ry(-90°): (x,y,z) -> (z, y, -x), so (0,0,0)->(0,0,0), (0,0,L)->(L,0,0)
        # After translate (-A,0,0): smaller at (-A,0,0), larger at (-A+L,0,0) toward center ✓
        thread_inlet = thread.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))  # +Z to +X
        thread_inlet = thread_inlet.moved(cq.Location(cq.Vector(-A, 0, 0)))  # Position at inlet face
        tee_shape = tee_shape.cut(thread_inlet)

        # Run (run-out): Thread at +X end
        # Face at x=A, thread extends toward center (x=0)
        # Ry(+90°): (x,y,z) -> (-z, y, x), so (0,0,0)->(0,0,0), (0,0,L)->(-L,0,0)
        # After translate (A,0,0): smaller at (A,0,0), larger at (A-L,0,0) toward center ✓
        thread_run = thread.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))  # +Z to -X
        thread_run = thread_run.moved(cq.Location(cq.Vector(A, 0, 0)))  # Position at run-out face
        tee_shape = tee_shape.cut(thread_run)

        # Branch: Thread at +Y end
        # Face at y=A, thread extends toward center (y=0)
        # Rx(+90°): (x,y,z) -> (x, -z, y), so (0,0,0)->(0,0,0), (0,0,L)->(0,-L,0)
        # After translate (0,A,0): smaller at (0,A,0), larger at (0,A-L,0) toward center ✓
        thread_branch = thread.moved(cq.Location(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90))  # +Z to -Y
        thread_branch = thread_branch.moved(cq.Location(cq.Vector(0, A, 0)))  # Position at branch face
        tee_shape = tee_shape.cut(thread_branch)

    return tee_shape


def get_threaded_tee_dims(nps: str, pressure_class: int = 3000) -> ThreadedTeeDims:
    """
    Get the dimensions for a threaded tee.

    Args:
        nps: Nominal pipe size (e.g., "1/2", "2", "4")
        pressure_class: 3000 only currently implemented

    Returns:
        ThreadedTeeDims with all dimensions

    Raises:
        ValueError: If NPS or pressure class is not supported
    """
    if pressure_class != 3000:
        raise ValueError(f"Pressure class {pressure_class} not implemented")

    dims = ASME_B1611_THREADED_TEE_CLASS3000.get(nps)
    if dims is None:
        raise ValueError(f"No ASME B16.11 Class {pressure_class} dimensions for NPS {nps}")

    return dims


def get_npt_thread_engagement(nps: str) -> float:
    """Get NPT thread engagement depth (hand + wrench makeup) in mm."""
    spec = NPT_THREAD_ENGAGEMENT.get(nps)
    if spec is None:
        raise ValueError(f"No NPT thread spec for NPS {nps}")
    return spec.engagement


# =============================================================================
# EXPORT UTILITIES
# =============================================================================


def export_to_step(shape: cq.Shape, filename: str):
    """Export a shape to STEP file."""
    cq.exporters.export(shape, filename)


# =============================================================================
# EXAMPLES
# =============================================================================


def example_threaded_tee():
    """Create a single 2" threaded tee."""
    tee = make_threaded_tee("2")
    export_to_step(tee, "threaded_tee_2in.step")
    print("Exported threaded_tee_2in.step")

    dims = ASME_B1611_THREADED_TEE_CLASS3000["2"]
    print(f"  Center-to-end: {dims.center_to_end} mm")
    print(f"  Band OD: {dims.band_od} mm")


if __name__ == "__main__":
    print("ASME B16.11 NPT Threaded Tee Generator")
    print("=" * 40)
    example_threaded_tee()
    print("\nDone!")
