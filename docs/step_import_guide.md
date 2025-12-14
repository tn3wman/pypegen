# Importing STEP Files as Fittings

This guide explains how to import manufacturer-provided STEP files (valves, mass flow controllers, sensors, etc.) into pypegen as fittings that can be used in pipe routing.

## Overview

pypegen can detect named coordinate systems embedded in STEP files to automatically identify port locations and orientations. The workflow is:

1. Open the STEP file in your CAD software (Autodesk Inventor)
2. Add coordinate systems at each port location with specific names
3. Export as STEP
4. Use pypegen's CLI tool to generate a configuration file
5. Load the fitting in your pypegen scripts

## Adding Coordinate Systems in Autodesk Inventor

### Step 1: Open the STEP File

1. Launch Autodesk Inventor
2. File → Open
3. Select the manufacturer's STEP file
4. The model will open as a derived part

### Step 2: Create a User Coordinate System (UCS) at Each Port

For each connection point on the fitting:

1. Go to **3D Model** tab → **Work Features** panel → **User Coordinate System**
2. Position the UCS origin at the **center of the port opening** (where a pipe would connect)
3. Orient the axes:
   - **Z-axis** (blue): Points **outward** from the fitting, toward where the connecting pipe extends. This is the most important axis.
   - **X-axis** (red): Points **"up"** (typically world +Z direction). This defines the rotational alignment when fittings mate.
   - **Y-axis** (green): Completes the right-hand coordinate system (automatically determined by X and Z).
4. Click to place the UCS

```
Example: Horizontal port on a valve body

        Z (blue) → points outward toward pipe
        ↑
        │
        ●───→ X (red) points up (world +Z)
       ╱
      ↓
     Y (green) computed automatically
```

**Tip:** If you're unsure about X-axis orientation, just make sure Z points outward correctly. pypegen can compute a reasonable X-axis automatically if needed.

### Step 3: Rename the Coordinate Systems

In the Model Browser (left panel):

1. Expand the model tree to find the new UCS entries
2. Right-click each UCS → **Properties** (or slow double-click to rename)
3. Rename using this convention:

| Name | Purpose |
|------|---------|
| `PORT_INLET` | Inlet connection port |
| `PORT_OUTLET` | Outlet connection port |
| `PORT_BRANCH` | Branch connection (for tees, etc.) |
| `ATTACH_BODY` | Attachment point for BOM balloon |
| `ATTACH_LABEL` | Attachment point for labels |
| `ORIGIN` | Optional: redefine fitting origin |

You can use any name after the prefix (e.g., `PORT_LEFT`, `PORT_RIGHT`, `PORT_TOP`).

### Step 4: Export as STEP

1. File → **Save As** → **Save Copy As**
2. Change file type to **STEP Files (*.stp, *.step)**
3. Click **Options**
4. Ensure **"Export sketch and work geometry"** is checked (this preserves the UCS entities)
5. Click **OK** and save the file

## Using the CLI Tool

After exporting the STEP file with markers:

```bash
# Analyze the STEP file and generate configuration
pypegen-fitting analyze valve.step --nps 2 -o valve.yaml

# Or with more options
pypegen-fitting analyze valve.step \
    --nps 2 \
    --id "ball_valve_2in" \
    --description "2-inch Ball Valve" \
    -o valve.yaml
```

The tool will:
1. Scan the STEP file for `PORT_*`, `ATTACH_*`, and `ORIGIN` coordinate systems
2. Extract their positions and orientations
3. Generate a YAML configuration file

### If No Markers Are Found

If you can't add markers in your CAD software, use interactive mode:

```bash
pypegen-fitting create valve.step --nps 2 -o valve.yaml
```

This will prompt you to manually enter port positions and orientations.

### Validating Configuration

To check if a configuration file is valid:

```bash
pypegen-fitting validate valve.yaml
```

## Configuration File Format

The generated YAML file looks like this:

```yaml
version: '1.0'
settings: {}
fittings:
  - id: ball_valve_2in
    step_file: valve.step
    nps: '2'
    description: 2-inch Ball Valve
    ports:
      - name: inlet
        position: [0.0, 0.0, -50.0]
        orientation:
          z_direction: [0.0, 0.0, -1.0]
          x_direction: [1.0, 0.0, 0.0]
      - name: outlet
        position: [0.0, 0.0, 50.0]
        orientation:
          z_direction: [0.0, 0.0, 1.0]
          x_direction: [1.0, 0.0, 0.0]
    attachment_points:
      - name: body
        position: [25.0, 0.0, 0.0]
        normal: [1.0, 0.0, 0.0]
        attachment_type: bom
```

You can edit this file manually to adjust positions or add attachment points.

## Loading Fittings in Python

```python
from pypegen.fittings.step_import import FittingLoader

# Load all fittings from a config file
loader = FittingLoader()
fittings = loader.load_library("valve.yaml")

# Access a specific fitting
valve = fittings["ball_valve_2in"]

# Use in pipe routing (example)
# route.add_fitting(valve)
```

## Port Orientation Convention

The Z-axis of each port must point **outward** from the fitting:

```
    Fitting Body
    ┌─────────────┐
    │             │
←── │  PORT_INLET │ PORT_OUTLET ──→
Z   │             │              Z
    └─────────────┘
```

When two fittings mate, their port Z-axes will be **opposite** (facing each other).

## Troubleshooting

### No markers detected

- Ensure "Export sketch and work geometry" is enabled in STEP export options
- Verify the UCS names start with `PORT_`, `ATTACH_`, or are exactly `ORIGIN`
- Check that names are uppercase (the detector is case-insensitive, but some CAD exports may alter case)

### Wrong port orientation

- The Z-axis should point toward where the connecting pipe extends
- Use Inventor's UCS tool to visualize and adjust the axis directions
- You can manually edit the `z_direction` in the YAML file

### Units mismatch

- Add `units: inch` or `units: mm` to the fitting config if auto-detection fails
- pypegen uses millimeters internally; inch values are converted automatically

## Other CAD Software

The marker detection relies on named `AXIS2_PLACEMENT_3D` entities in the STEP file. Other CAD packages that support this:

- **SolidWorks**: Reference Geometry → Coordinate System (name appears in STEP)
- **FreeCAD**: Datum → Local Coordinate System
- **Onshape**: Mate Connectors (export as named entities)
- **Fusion 360**: Limited support; may need to use manual configuration

Consult your CAD software's documentation for exporting named coordinate systems to STEP format.
