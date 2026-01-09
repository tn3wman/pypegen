# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Development Commands

```bash
# Install dependencies (use virtual environment)
pip install -e ".[dev]"

# Run all tests
pytest

# Run a single test file
pytest tests/test_drawing_simple.py

# Run tests with verbose output (default)
pytest -v

# Lint and format
ruff check src/
ruff format src/

# Run the example to generate drawings
python examples/pipe_bom_simple.py

# Update golden test file (when intentional changes are made)
python tests/test_drawing_simple.py --update-golden
```

## Architecture Overview

pypegen is a parametric piping fabrication drawing generator using CadQuery for 3D geometry and custom SVG rendering for 2D fabrication drawings.

### Core Modules

**`pipe_router.py`** - High-level pipe routing system
- `PipeRoute` class: Defines pipe runs with a starting fitting and a list of directional moves
- Uses 4x4 homogeneous transformation matrices for proper coordinate handling
- Automatically places fittings (flanges, elbows) at direction changes
- Tracks components and welds for BOM generation
- Coordinate convention: X=East, Y=North, Z=Up; all internal calculations in mm

**`pipe_generator.py`** - Lower-level pipe and fitting geometry generators
- Contains `PipeSpec`, `FittingSpec`, `PortDef` dataclasses for parametric definitions
- Creates ASME-compliant socket weld fittings (B16.11 elbows, couplings)
- Provides route building utilities (`build_segments`, `make_pipe_solid`)

**`fittings/pipe_fittings.py`** - Port-based mating system
- `Fitting` base class with ports and attachment points
- `Port` class: Connection points with 4x4 transformation matrices (Z-axis points outward)
- `AttachmentPoint` class: Surface locations for weld symbols and BOM balloons
- `compute_mate_transform()`: Calculates world transform to mate two ports
- Mating rule: Port Z-axes must oppose each other (they face each other)

**`fittings/socket_weld_elbow.py` & `weld_neck_flange.py`** - ASME dimension tables
- Contains actual ASME B16.11 and B16.5 dimension data
- Dimensions stored in mm

**`drawing_generator/drawing.py`** - Main drawing generator
- `PipingDrawing` class: Generates complete fabrication drawings
- Renders 3D geometry through CadQuery's SVG exporter
- Applies isometric projection (standard piping iso view from direction (1,1,1))
- Overlays BOM table, balloon leaders, and weld markers
- `CoordinateMapper`: Maps 3D world coordinates to 2D SVG coordinates

**`drawing_generator/bom.py`** - Bill of Materials and annotations
- `ComponentRecord`, `WeldRecord`: Track components and welds during routing
- `BOMTable`: Renders the material list table
- `TopologyBalloonPlacer`: Places balloon leaders based on pipe topology

### Key Design Patterns

1. **Port-based mating**: Fittings connect via ports with defined coordinate frames. The `compute_mate_transform()` function handles proper alignment.

2. **Component tracking**: During `PipeRoute.build()`, components and welds are recorded with their world transforms for later annotation placement.

3. **SVG coordinate pipeline**: 3D points → isometric projection → CadQuery SVG transform → fit scaling → final screen coordinates. The `CoordinateMapper` encapsulates this chain.

4. **Attachment points vs ports**: Ports are for mating geometry; attachment points are for visual annotation placement (on outer surfaces).

5. **Weld symbol placement**: Weld markers use attachment points (not ports) for proper positioning:
   - Each fitting has attachment points named `{port_name}_weld_{direction}` (e.g., `inlet_weld_north`)
   - `_select_weld_attachment_point()` chooses the best point based on pipe direction and weld type
   - Butt welds prefer the front/visible side (north for E-W pipes in isometric view)
   - Socket welds prefer the back/bottom to avoid crossing connected geometry
   - `avoid_direction` parameter prevents placing markers where connected pipes go

### Butt Weld vs Socket Weld Elbow Transforms

The two elbow types have different local geometries requiring different transform calculations:

**Socket Weld Elbow** (`_compute_elbow_transform`):
- Local geometry: L-shaped body with axis-aligned faces
- Inlet port Z = +X, Outlet port Z = +Y
- Center calculation: `center = inlet_pos + A * incoming_direction`

**Butt Weld Elbow** (`_compute_butt_weld_elbow_transform`):
- Local geometry: Arc centered at origin from (A, 0, 0) to (0, A, 0)
- Arc tangent at inlet is +Y (flow direction), at outlet is -X
- Inlet port Z = -Y (opposite of flow), Outlet port Z = -X
- **Critical**: The rotation matrix must map:
  - Local +Y (inlet tangent) → world `incoming_direction`
  - Local -X (outlet tangent) → world `turn_direction`
- This gives: `col0 = -turn_direction`, `col1 = +incoming_direction`
- Center calculation: `center = inlet_pos + A * turn_direction`

**Common mistake**: Using `col1 = -incoming_direction` causes the arc to sweep in the wrong direction (toward the incoming pipe instead of away from it).

### Units and Constants

- Internal units: millimeters (mm)
- User-facing: Accepts "3 ft", "500 mm", etc. via `parse_distance()`
- Expansion gap: 1.6mm (1/16") per ASME for socket weld connections
- Drawing output: ISO A4 landscape (297mm × 210mm)
