# pypegen

A Python library for generating parametric piping assemblies and fabrication drawings.

pypegen creates 3D pipe models and 2D fabrication drawings from simple route definitions. It uses [CadQuery](https://cadquery.readthedocs.io/) for geometry generation and produces industry-standard isometric drawings with bill of materials and weld tracking.

## Features

- **Parametric pipe routing** - Define pipe runs with directional moves (east, north, up, etc.)
- **ASME-compliant fittings** - Socket weld elbows (B16.11), weld neck flanges (B16.5)
- **Automatic fitting placement** - Elbows inserted at direction changes
- **3D STEP export** - Compatible with any CAD software
- **Fabrication drawings** - Isometric views with BOM tables and weld markers
- **Component tracking** - Bill of materials and weld numbering per ASME B31.3

## Installation

```bash
pip install pypegen
```

Or install from source:

```bash
git clone https://github.com/tn3wman/pypegen.git
cd pypegen
pip install -e ".[dev]"
```

## Quick Start

```python
from pypegen.pipe_router import PipeRoute
from pypegen.drawing_generator.drawing import PipingDrawing

# Define a pipe route
route = PipeRoute(
    nps="2",                    # 2" nominal pipe size
    start_fitting="flange",     # Start with a weld neck flange
    moves=[
        ("east", "3 ft"),       # Run 3 feet east
        ("up", "2 ft"),         # Turn up and run 2 feet
    ],
    track_components=True,      # Enable BOM tracking
)

# Build the geometry
route.build()

# Export 3D model
route.export("my_pipe.step")

# Create fabrication drawing
combined_shape = route.parts[0]
for part in route.parts[1:]:
    combined_shape = combined_shape.fuse(part)

drawing = PipingDrawing(
    shape=combined_shape,
    title='2" PROCESS LINE',
    drawing_number="SPOOL-001",
    material="CS A106 GR.B",
    components=route.components,
    welds=route.welds,
    fittings=route.fittings,
    show_bom=True,
    show_welds=True,
)

drawing.generate()
drawing.export_svg("my_pipe.svg")
drawing.export_pdf("my_pipe.pdf")
```

## Supported Pipe Sizes

NPS 1/2" through 4" (Schedule 40)

## Supported Fittings

| Fitting | Standard | Classes |
|---------|----------|---------|
| Socket Weld 90° Elbow | ASME B16.11 | 3000 |
| Weld Neck Flange | ASME B16.5 | 300 |

## Coordinate System

- **X** = East/West
- **Y** = North/South
- **Z** = Up/Down

All internal calculations use millimeters. User input accepts various units:

```python
("east", "3 ft")      # feet
("up", "500 mm")      # millimeters
("north", "2 m")      # meters
("west", "24 in")     # inches
```

## Output Formats

- **STEP** - 3D CAD geometry
- **SVG** - Vector fabrication drawing
- **PDF** - Print-ready fabrication drawing

## License

Apache-2.0
