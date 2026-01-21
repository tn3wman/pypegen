#!/usr/bin/env python3
"""
Demonstration: Creating Sloped Pipes Using Elbow Roll Angles

This example shows how to achieve sloped pipes (like drainage with 1/8" per foot fall)
by rotating elbows about the pipe axis.

Key Concepts:
1. A 45° elbow changes direction by 45° in a plane
2. Rotating the elbow about the incoming pipe axis (roll) changes which plane the turn is in
3. Two 45° elbows with proper roll angles can achieve any compound direction change

Example: Going West with a slight downward slope
- Pure west = (−1, 0, 0)
- West + 1" drop over 60" = direction vector (-60, 0, -1) normalized

The roll angle needed for an elbow = atan2(vertical_component, horizontal_component)
"""

import math

import cadquery as cq
import numpy as np

from src.pypegen.fittings.butt_weld_elbow_45 import ASME_B169_ELBOW45, make_butt_weld_elbow_45
from src.pypegen.fittings.butt_weld_elbow import ASME_B169_ELBOW90, make_butt_weld_elbow_90
from src.pypegen.fittings.pipe_fittings import (
    PIPE_SPECS,
    make_pipe,
    rotation_from_axis_angle,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
    translation_matrix,
    apply_transform_to_shape,
    identity_matrix,
)

INCH = 25.4  # mm per inch


def calculate_slope_angle(run_distance: float, drop_distance: float) -> float:
    """
    Calculate the slope angle in degrees.
    
    Args:
        run_distance: Horizontal run distance
        drop_distance: Vertical drop distance
        
    Returns:
        Slope angle in degrees
    """
    return math.degrees(math.atan2(drop_distance, run_distance))


def calculate_roll_for_slope(slope_angle_deg: float, elbow_angle_deg: float = 45.0) -> float:
    """
    Calculate the roll angle needed to achieve a desired slope using a given elbow.
    
    For small slopes, the relationship is approximately:
    slope_angle ≈ elbow_angle × sin(roll_angle)
    
    Therefore:
    roll_angle = arcsin(slope_angle / elbow_angle)  [for small angles, in radians]
    
    Args:
        slope_angle_deg: Desired slope angle in degrees
        elbow_angle_deg: Elbow bend angle in degrees (45 or 90)
        
    Returns:
        Roll angle in degrees
    """
    # For the exact calculation:
    # The vertical component = sin(elbow_angle) × sin(roll_angle)
    # For slope_angle = arctan(drop/run), and a 45° elbow:
    # sin(roll) = tan(slope) / tan(elbow)
    
    slope_rad = math.radians(slope_angle_deg)
    elbow_rad = math.radians(elbow_angle_deg)
    
    # sin(roll) = sin(slope) / sin(elbow)  [more accurate for larger angles]
    sin_roll = math.sin(slope_rad) / math.sin(elbow_rad)
    
    if abs(sin_roll) > 1.0:
        raise ValueError(f"Slope {slope_angle_deg}° cannot be achieved with {elbow_angle_deg}° elbow")
    
    return math.degrees(math.asin(sin_roll))


def create_rolled_elbow_45(nps: str, roll_angle_deg: float = 0.0) -> cq.Shape:
    """
    Create a 45° elbow with a roll angle applied.
    
    The roll rotates the elbow about the inlet pipe axis (the +X axis in elbow's
    local coordinate system).
    
    Args:
        nps: Nominal pipe size
        roll_angle_deg: Roll angle in degrees (rotation about inlet axis)
        
    Returns:
        CadQuery Shape of the rolled elbow
    """
    elbow = make_butt_weld_elbow_45(nps)
    
    if abs(roll_angle_deg) < 0.001:
        return elbow
    
    # The elbow's inlet is along +X axis
    # Roll rotation is about the +X axis
    roll_transform = rotation_matrix_x(roll_angle_deg)
    
    return apply_transform_to_shape(elbow, roll_transform)


def demo_slope_calculation():
    """Demonstrate slope angle calculations."""
    print("=" * 60)
    print("SLOPE ANGLE CALCULATIONS")
    print("=" * 60)
    
    # Common drainage slopes
    slopes = [
        ("1/8\" per foot", 1/8, 12),
        ("1/4\" per foot", 1/4, 12),
        ("1\" per 60\"", 1, 60),
        ("2\" per 10 ft", 2, 120),
    ]
    
    for name, drop, run in slopes:
        angle = calculate_slope_angle(run, drop)
        print(f"\n{name}:")
        print(f"  Slope angle: {angle:.4f}°")
        
        # Calculate roll angle needed for 45° elbow
        try:
            roll_45 = calculate_roll_for_slope(angle, 45.0)
            print(f"  Roll for 45° elbow: {roll_45:.4f}°")
        except ValueError as e:
            print(f"  45° elbow: {e}")


def demo_visual_comparison():
    """Create visual comparison of rolled vs non-rolled elbows."""
    print("\n" + "=" * 60)
    print("VISUAL COMPARISON: Creating STEP files")
    print("=" * 60)
    
    nps = "4"
    dims = ASME_B169_ELBOW45[nps]
    pipe_spec = PIPE_SPECS[nps]
    
    parts = []
    
    # Create a pipe feeding into each elbow for reference
    pipe_length = 200  # mm
    
    # === Configuration 1: Standard 45° elbow (no roll) ===
    # Turn is purely in the XY plane
    elbow1 = make_butt_weld_elbow_45(nps)
    
    # Add inlet pipe - make_pipe returns a Fitting, get its shape
    inlet_pipe_fitting = make_pipe(nps, pipe_length)
    inlet_pipe = inlet_pipe_fitting.shape
    # Position pipe so it feeds into elbow from +X direction
    inlet_transform = translation_matrix(dims.center_to_end + pipe_length/2, 0, 0)
    inlet_pipe_shape = apply_transform_to_shape(inlet_pipe, inlet_transform)
    
    parts.append(elbow1)
    parts.append(inlet_pipe_shape)
    
    # === Configuration 2: 45° elbow with 30° roll ===
    # This tilts the turn plane, adding a vertical component
    roll_angle = 30.0  # degrees
    elbow2 = create_rolled_elbow_45(nps, roll_angle)
    
    # Offset this assembly to the side for comparison
    offset = 300  # mm
    offset_transform = translation_matrix(0, offset, 0)
    
    elbow2_moved = apply_transform_to_shape(elbow2, offset_transform)
    inlet_pipe2 = apply_transform_to_shape(inlet_pipe_shape, offset_transform)
    
    parts.append(elbow2_moved)
    parts.append(inlet_pipe2)
    
    # === Configuration 3: 45° elbow with calculated roll for 1" drop over 60" ===
    slope_angle = calculate_slope_angle(60, 1)  # 60" run, 1" drop
    roll_for_slope = calculate_roll_for_slope(slope_angle, 45.0)
    
    print(f"\nFor 1\" drop over 60\" run:")
    print(f"  Slope angle: {slope_angle:.4f}°")
    print(f"  Required roll: {roll_for_slope:.4f}°")
    
    elbow3 = create_rolled_elbow_45(nps, roll_for_slope)
    offset_transform3 = translation_matrix(0, offset * 2, 0)
    
    elbow3_moved = apply_transform_to_shape(elbow3, offset_transform3)
    inlet_pipe3 = apply_transform_to_shape(inlet_pipe_shape, offset_transform3)
    
    parts.append(elbow3_moved)
    parts.append(inlet_pipe3)
    
    # Export
    compound = cq.Compound.makeCompound(parts)
    output_path = "examples/output/sloped_pipe_elbows_demo.step"
    cq.exporters.export(compound, output_path)
    print(f"\nExported to: {output_path}")
    print("\nIn the STEP file:")
    print("  - Front (Y=0): Standard 45° elbow - turns in XY plane")
    print(f"  - Middle (Y={offset}): 45° elbow with 30° roll - turns diagonally")
    print(f"  - Back (Y={offset*2}): 45° elbow with {roll_for_slope:.2f}° roll - creates 1:60 slope")


def demo_rolling_offset():
    """
    Demonstrate a complete rolling offset: two 45° elbows creating an offset path.
    
    A rolling offset uses two elbows to move the pipe both horizontally and vertically
    while maintaining the same final direction.
    """
    print("\n" + "=" * 60)
    print("ROLLING OFFSET DEMONSTRATION")
    print("=" * 60)
    
    nps = "4"
    dims = ASME_B169_ELBOW45[nps]
    pipe_spec = PIPE_SPECS[nps]
    
    # Calculate the offset geometry
    # For a 45° elbow, the outlet direction is at 45° from inlet
    # With roll, we can control the plane of that turn
    
    # Example: Create an offset that goes:
    # - 24" of travel between elbows
    # - Net horizontal offset (perpendicular to flow)
    # - Net vertical drop
    
    travel_distance = 24 * INCH  # mm - pipe between the two elbows
    roll_angle = 30.0  # degrees - controls the ratio of horizontal to vertical offset
    
    # Calculate the offset
    # After first 45° elbow with roll:
    # - Forward component (along original direction): cos(45°) = 0.707
    # - Lateral component (perpendicular): sin(45°) × cos(roll)
    # - Vertical component: sin(45°) × sin(roll)
    
    forward = travel_distance * math.cos(math.radians(45))
    lateral = travel_distance * math.sin(math.radians(45)) * math.cos(math.radians(roll_angle))
    vertical = travel_distance * math.sin(math.radians(45)) * math.sin(math.radians(roll_angle))
    
    print(f"\nRolling offset with {travel_distance/INCH:.1f}\" travel and {roll_angle}° roll:")
    print(f"  Forward advance: {forward/INCH:.2f}\"")
    print(f"  Lateral offset: {lateral/INCH:.2f}\"")
    print(f"  Vertical drop: {vertical/INCH:.2f}\"")
    
    # Build the geometry
    parts = []
    
    # Start with inlet pipe going in +X direction
    inlet_length = 150  # mm
    inlet_pipe_fitting = make_pipe(nps, inlet_length)
    inlet_pipe = inlet_pipe_fitting.shape
    inlet_transform = translation_matrix(inlet_length/2, 0, 0)
    parts.append(apply_transform_to_shape(inlet_pipe, inlet_transform))
    
    # First 45° elbow with roll
    elbow1 = create_rolled_elbow_45(nps, roll_angle)
    # Position at end of inlet pipe
    elbow1_transform = translation_matrix(inlet_length + dims.center_to_end, 0, 0) @ rotation_matrix_z(180)
    parts.append(apply_transform_to_shape(elbow1, elbow1_transform))
    
    # Calculate outlet position and direction of first elbow
    # The elbow turns 45° in a plane defined by the roll angle
    elbow_outlet_dir = np.array([
        math.cos(math.radians(45)),
        math.sin(math.radians(45)) * math.cos(math.radians(roll_angle)),
        -math.sin(math.radians(45)) * math.sin(math.radians(roll_angle))
    ])
    
    # Travel pipe between elbows
    travel_pipe_fitting = make_pipe(nps, travel_distance)
    travel_pipe = travel_pipe_fitting.shape
    # Position this pipe along the outlet direction
    travel_start = np.array([inlet_length + dims.center_to_end, 0, 0])
    travel_center = travel_start + elbow_outlet_dir * (dims.center_to_end + travel_distance/2)
    
    # Rotation to align pipe with travel direction
    # We need to rotate +Z (pipe default axis) to align with elbow_outlet_dir
    # For simplicity, let's just use the computed offset position
    travel_end = travel_start + elbow_outlet_dir * (2 * dims.center_to_end + travel_distance)
    
    print(f"\n  Elbow 1 outlet direction: ({elbow_outlet_dir[0]:.3f}, {elbow_outlet_dir[1]:.3f}, {elbow_outlet_dir[2]:.3f})")
    print(f"  Travel pipe ends at: ({travel_end[0]/INCH:.2f}\", {travel_end[1]/INCH:.2f}\", {travel_end[2]/INCH:.2f}\")")
    
    # Export what we have
    compound = cq.Compound.makeCompound(parts)
    output_path = "examples/output/rolling_offset_demo.step"
    cq.exporters.export(compound, output_path)
    print(f"\nExported partial demo to: {output_path}")
    print("\nNote: A complete rolling offset builder would need proper port mating")
    print("to chain the second elbow with opposite roll to return to original direction.")


def main():
    """Run all demonstrations."""
    print("SLOPED PIPE DEMONSTRATION")
    print("Using elbow roll angles to achieve slopes")
    print("=" * 60)
    
    demo_slope_calculation()
    demo_visual_comparison()
    demo_rolling_offset()
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print("""
To achieve sloped pipes in pypegen:

1. For small slopes (like drainage 1/8" per foot):
   - Calculate roll angle: roll = arcsin(tan(slope) / tan(elbow_angle))
   - Apply roll rotation to the elbow about its inlet axis
   
2. For the current routing system:
   - Add a 'roll' parameter to elbow fitting functions
   - Modify compute_elbow_rotation() to accept a roll angle
   - Update RouteNode to store roll information

3. Practical limits:
   - 45° elbow can achieve slopes up to 45° (0.7% slope = 0.4° to 100% = 45°)
   - For very small slopes (<1°), the roll angle is also very small (<1.4°)
   - For larger slopes, consider using two elbows in a rolling offset
""")


if __name__ == "__main__":
    main()
