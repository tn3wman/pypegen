#!/usr/bin/env python3
"""Debug script for step-by-step internal NPT thread creation on elbows.

This script implements the CORRECT approach for internal threads:
1. Create elbow body
2. Cut tapered bore (smallest at opening, largest inside)
3. Apply chamfer to bore ID edge using BRepFilletAPI_MakeChamfer
4. Create internal thread CUTTER (V-grooves positioned INSIDE bore)
5. SUBTRACT cutter from elbow to CUT threads into wall

This mirrors the external pipe thread process:
- External: tapered pipe → chamfer OD edge → cut threads IN
- Internal: elbow body → bore hole → chamfer ID edge → cut threads OUT
"""

import math

import cadquery as cq
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt

# NPT constants
NPT_TAPER_RATIO = 1.0 / 16.0  # 1:16 taper (0.75" per foot)


def get_npt_spec(nps: str) -> dict:
    """Get NPT thread specification for a given nominal pipe size."""
    # Simplified specs for 1/2" NPT
    specs = {
        "1/2": {
            "od": 21.336,  # Pipe OD in mm (0.840")
            "tpi": 14,  # Threads per inch
            "pitch_mm": 25.4 / 14,  # 1.814 mm
            "L2": 13.556,  # Effective thread length in mm
        }
    }
    spec = specs[nps]
    spec["thread_depth"] = 0.8 * spec["pitch_mm"]  # Per ASME B1.20.1
    return spec


def make_elbow_body(nps: str, center_to_end: float = 33.0) -> cq.Shape:
    """Create basic 90° elbow body (no threads).

    The elbow is positioned with:
    - Leg 1 along +X axis (opening at X=center_to_end)
    - Leg 2 along +Z axis (opening at Z=center_to_end)
    - Bend center at origin
    """
    spec = get_npt_spec(nps)

    # Use fitting OD (slightly larger than pipe OD for socket)
    fitting_od = spec["od"] + 4.0  # Wall thickness ~ 2mm each side
    fitting_or = fitting_od / 2.0

    # Create torus section for the bend
    bend_radius = center_to_end - fitting_or

    # Build elbow as torus section + cylindrical legs
    elbow = (
        cq.Workplane("XY")
        .center(0, 0)
        .circle(fitting_or)
        .revolve(90, (bend_radius, 0, 0), (0, 0, 1))
    )

    # Add leg 1 (along +X)
    leg1 = (
        cq.Workplane("YZ")
        .center(0, 0)
        .circle(fitting_or)
        .extrude(center_to_end - bend_radius)
        .translate((bend_radius, 0, 0))
    )

    # Add leg 2 (along +Z)
    leg2 = (
        cq.Workplane("XY")
        .center(0, 0)
        .circle(fitting_or)
        .extrude(center_to_end - bend_radius)
        .translate((0, 0, bend_radius))
    )

    elbow = elbow.union(leg1).union(leg2)
    return elbow.val()


def make_tapered_bore(
    nps: str,
    thread_length: float,
    bore_at_face: float,
) -> cq.Shape:
    """Create tapered cylinder for cutting bore.

    The bore is SMALLEST at Z=0 (fitting face) and LARGEST at Z=thread_length.
    This matches NPT taper direction for internal threads.
    """
    taper_delta = thread_length * NPT_TAPER_RATIO / 2.0
    bore_at_end = bore_at_face + 2 * taper_delta

    # Create tapered cylinder using loft
    bore = (
        cq.Workplane("XY")
        .circle(bore_at_face)
        .workplane(offset=thread_length)
        .circle(bore_at_end)
        .loft()
    )
    return bore.val()


def apply_bore_chamfer(
    shape: cq.Shape,
    bore_edge_x: float,
    radial_depth: float,
    axial_depth: float,
) -> cq.Shape:
    """Apply chamfer to ID edge of bore using BRepFilletAPI_MakeChamfer.

    Args:
        shape: The solid with a bore
        bore_edge_x: X position of the bore opening
        radial_depth: How deep the chamfer cuts radially
        axial_depth: How deep the chamfer cuts axially
    """
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
    from OCP.TopExp import TopExp

    solid = shape.wrapped

    # Build edge-face map
    edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(solid, TopAbs_EDGE, TopAbs_FACE, edge_face_map)

    # Find the bore ID edge at the opening
    bore_edge = None
    bore_face = None

    explorer = TopExp_Explorer(solid, TopAbs_EDGE)
    while explorer.More():
        edge = TopoDS.Edge_s(explorer.Current())
        edge_shape = cq.Edge(edge)
        start = edge_shape.startPoint()
        end = edge_shape.endPoint()

        # Check if edge is at bore opening (X ≈ bore_edge_x)
        if abs(start.x - bore_edge_x) < 0.5 and abs(end.x - bore_edge_x) < 0.5:
            # Calculate radius in YZ plane (bore is along X axis)
            radius = math.sqrt(start.y**2 + start.z**2)
            if radius > 5.0:  # Bore edge, not a tiny feature
                # Get adjacent faces
                adjacent_faces = edge_face_map.FindFromKey(edge)
                if not adjacent_faces.IsEmpty():
                    f1 = TopoDS.Face_s(adjacent_faces.First())
                    fc1 = cq.Face(f1).Center()

                    # The bore face is the conical surface (center X not at bore_edge_x)
                    if abs(fc1.x - bore_edge_x) > 0.5:
                        bore_face = f1
                    elif adjacent_faces.Size() > 1:
                        f2 = TopoDS.Face_s(adjacent_faces.Last())
                        fc2 = cq.Face(f2).Center()
                        if abs(fc2.x - bore_edge_x) > 0.5:
                            bore_face = f2

                    if bore_face is not None:
                        bore_edge = edge
                        print(f"  Found bore ID edge at X={start.x:.2f}, radius={radius:.2f}")
                        break
        explorer.Next()

    if bore_edge is None or bore_face is None:
        print("  WARNING: No bore ID edge found!")
        return shape

    # Apply chamfer to bore edge
    chamfer_maker = BRepFilletAPI_MakeChamfer(solid)
    # For internal bore: radial depth cuts into wall, axial depth cuts into fitting
    chamfer_maker.Add(axial_depth, radial_depth, bore_edge, bore_face)
    chamfer_maker.Build()

    if chamfer_maker.IsDone():
        return cq.Shape(chamfer_maker.Shape())
    else:
        print("  WARNING: Chamfer operation failed!")
        return shape


def make_internal_thread_cutter(
    nps: str,
    thread_length: float,
) -> cq.Shape:
    """Create helical V-groove cutting tool for internal NPT threads.

    The cutter is positioned INSIDE the bore and cuts OUTWARD into the
    fitting wall. This is the INVERSE of the external cutter.

    Geometry:
    - V-groove is WIDE at bore surface (minor_r) - where it enters the wall
    - V-groove is NARROW at thread root (major_r) - deepest into wall

    External cutter (for comparison):
    - V-groove is WIDE at pipe surface (outer_r) - where it enters the pipe
    - V-groove is NARROW at root (root_r) - deepest into pipe

    Orientation:
    - Axis along +Z
    - Z=0 is the fitting face (SMALLEST diameter)
    - Z=thread_length is into the fitting (LARGEST diameter)
    """
    spec = get_npt_spec(nps)
    pitch = spec["pitch_mm"]
    thread_depth = spec["thread_depth"]

    # For internal threads at Z=0 (fitting face - SMALLEST diameter)
    # minor_r = bore surface (where external pipe crests contact)
    # major_r = minor_r + thread_depth (thread roots - deepest into wall)
    minor_radius_at_start = spec["od"] / 2.0

    # Thread profile dimensions (per ASME B1.20.1)
    flat_width = 0.038 * pitch  # Flat at crest/root
    half_flat = flat_width / 2
    flank_axial_span = thread_depth * math.tan(math.radians(30))  # 60° included angle

    # Calculate number of thread turns
    num_turns = thread_length / pitch

    # Create thread groove solid by lofting cross-sections
    num_sections = max(int(num_turns * 36), 36)  # At least 36 sections

    loft = BRepOffsetAPI_ThruSections(True, False)  # solid=True, ruled=False

    for i in range(num_sections + 1):
        t = i / num_sections
        z = t * thread_length
        angle = t * num_turns * 2 * math.pi  # Angle around helix

        # Radius at this Z position (NPT taper - increases with Z for internal)
        taper_offset = z * NPT_TAPER_RATIO / 2.0
        minor_r = minor_radius_at_start + taper_offset  # Bore surface
        major_r = minor_r + thread_depth  # Thread root (into wall)

        # Extend cutter slightly inside bore for clean cuts
        inner_r = minor_r - 0.3

        # Profile center position (in XY plane at angle)
        cx = math.cos(angle)
        cy = math.sin(angle)

        # Create V-groove profile points (4 corners of trapezoid)
        # For INTERNAL threads cutting OUTWARD:
        # - WIDE span at inner_r (bore surface, where groove enters wall)
        # - NARROW span at major_r (thread root, deepest into wall)

        # Inner edge (at bore surface) - WIDE span
        z_inner_top = z - half_flat - flank_axial_span
        z_inner_bot = z + half_flat + flank_axial_span

        # Outer edge (at thread root) - NARROW span (just the flat)
        z_outer_top = z - half_flat
        z_outer_bot = z + half_flat

        pts = [
            gp_Pnt(cx * inner_r, cy * inner_r, z_inner_top),  # Inner top (wide)
            gp_Pnt(cx * inner_r, cy * inner_r, z_inner_bot),  # Inner bot (wide)
            gp_Pnt(cx * major_r, cy * major_r, z_outer_bot),  # Outer bot (narrow)
            gp_Pnt(cx * major_r, cy * major_r, z_outer_top),  # Outer top (narrow)
        ]

        # Build wire from points
        wire_builder = BRepBuilderAPI_MakeWire()
        for j in range(4):
            edge = BRepBuilderAPI_MakeEdge(pts[j], pts[(j + 1) % 4]).Edge()
            wire_builder.Add(edge)

        wire = wire_builder.Wire()
        loft.AddWire(wire)

    loft.Build()
    if not loft.IsDone():
        raise ValueError("Failed to create internal thread cutter loft")

    return cq.Shape(loft.Shape())


def main():
    """Step-by-step debug of internal NPT thread creation."""
    nps = "1/2"
    spec = get_npt_spec(nps)
    thread_length = spec["L2"]  # ~13.56 mm
    center_to_end = 33.0  # Elbow dimension

    print(f"NPT {nps} Thread Parameters:")
    print(f"  OD: {spec['od']:.3f} mm")
    print(f"  Pitch: {spec['pitch_mm']:.3f} mm")
    print(f"  Thread depth: {spec['thread_depth']:.3f} mm")
    print(f"  Thread length: {thread_length:.3f} mm")
    print()

    # Step 1: Create elbow body
    print("Step 1: Creating elbow body...")
    elbow = make_elbow_body(nps, center_to_end)
    vol1 = elbow.Volume()
    print(f"  Elbow volume: {vol1:.2f} mm³")
    cq.exporters.export(cq.Workplane().add(elbow), "debug_step1_elbow_body.step")
    print("  Exported: debug_step1_elbow_body.step")
    print()

    # Step 2: Cut tapered bore into leg 1 (+X direction)
    print("Step 2: Cutting tapered bore into leg 1...")

    # Bore at minor (apex) radius - where external pipe crests contact
    bore_radius = spec["od"] / 2.0  # Same as external pipe major radius
    print(f"  Bore radius at face: {bore_radius:.3f} mm")

    # Create bore along X axis (into leg 1)
    # The bore opening is at X=center_to_end, extending toward origin
    bore = make_tapered_bore(nps, thread_length, bore_radius)

    # Position bore: rotate to align with X axis, translate to leg 1 end
    bore_positioned = bore.rotate((0, 0, 0), (0, 1, 0), -90)  # Z axis -> X axis
    bore_positioned = bore_positioned.translate((center_to_end, 0, 0))

    # Export bore for inspection
    cq.exporters.export(cq.Workplane().add(bore_positioned), "debug_step2_bore_leg1.step")
    print("  Exported: debug_step2_bore_leg1.step")

    # Cut bore from elbow
    elbow_with_bore = elbow.cut(bore_positioned)
    vol2 = elbow_with_bore.Volume()
    print(f"  Volume after bore: {vol2:.2f} mm³ (removed {vol1 - vol2:.2f} mm³)")
    cq.exporters.export(cq.Workplane().add(elbow_with_bore), "debug_step2_elbow_with_bore.step")
    print("  Exported: debug_step2_elbow_with_bore.step")
    print()

    # Step 3: Apply chamfer to bore ID edge
    print("Step 3: Applying chamfer to bore ID edge...")
    radial_depth = spec["thread_depth"]  # Same as thread depth
    axial_depth = radial_depth * math.tan(math.radians(30))  # 60° from vertical
    print(f"  Radial depth: {radial_depth:.3f} mm")
    print(f"  Axial depth: {axial_depth:.3f} mm")

    elbow_with_chamfer = apply_bore_chamfer(
        elbow_with_bore,
        bore_edge_x=center_to_end,  # Bore opening at X=33
        radial_depth=radial_depth,
        axial_depth=axial_depth,
    )
    vol3 = elbow_with_chamfer.Volume()
    print(f"  Volume after chamfer: {vol3:.2f} mm³ (removed {vol2 - vol3:.2f} mm³)")
    cq.exporters.export(cq.Workplane().add(elbow_with_chamfer), "debug_step3_elbow_with_chamfer.step")
    print("  Exported: debug_step3_elbow_with_chamfer.step")
    print()

    # Step 4: Create internal thread cutter
    print("Step 4: Creating internal thread cutter...")
    thread_cutter = make_internal_thread_cutter(nps, thread_length)
    cutter_vol = thread_cutter.Volume()
    print(f"  Thread cutter volume: {cutter_vol:.2f} mm³")

    # Export raw cutter (at origin, along Z)
    cq.exporters.export(cq.Workplane().add(thread_cutter), "debug_step4_thread_cutter_raw.step")
    print("  Exported: debug_step4_thread_cutter_raw.step")

    # Position cutter for leg 1 (+X axis)
    # Rotate to align Z axis with X axis, translate to bore opening
    cutter_positioned = thread_cutter.rotate((0, 0, 0), (0, 1, 0), -90)
    cutter_positioned = cutter_positioned.translate((center_to_end, 0, 0))

    cq.exporters.export(cq.Workplane().add(cutter_positioned), "debug_step4_thread_cutter_leg1.step")
    print("  Exported: debug_step4_thread_cutter_leg1.step")
    print()

    # Step 5: Cut threads from elbow
    print("Step 5: Cutting threads from elbow...")

    # Use OCP boolean directly for better error messages
    cut_op = BRepAlgoAPI_Cut(elbow_with_chamfer.wrapped, cutter_positioned.wrapped)
    cut_op.Build()

    if cut_op.IsDone():
        elbow_threaded = cq.Shape(cut_op.Shape())
        vol5 = elbow_threaded.Volume()
        print(f"  Volume after thread cutting: {vol5:.2f} mm³ (removed {vol3 - vol5:.2f} mm³)")
        cq.exporters.export(cq.Workplane().add(elbow_threaded), "debug_step5_elbow_threaded.step")
        print("  Exported: debug_step5_elbow_threaded.step")
    else:
        print("  ERROR: Thread cutting boolean failed!")
        # Export the shapes for debugging
        combined = cq.Workplane().add(elbow_with_chamfer).add(cutter_positioned)
        cq.exporters.export(combined, "debug_step5_failed_inputs.step")
        print("  Exported inputs to: debug_step5_failed_inputs.step")

    print()
    print("Done! Review the STEP files to verify geometry.")


if __name__ == "__main__":
    main()
