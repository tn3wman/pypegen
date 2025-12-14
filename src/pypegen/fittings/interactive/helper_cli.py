"""
CLI helper tool for pypegen fitting import.

This module provides a command-line interface for working with STEP files:
- analyze: Detect markers and show STEP file information
- create: Create a fitting configuration file
- validate: Validate an existing configuration file

Usage:
    pypegen-fitting analyze valve.step
    pypegen-fitting create valve.step --nps 2 -o valve.yaml
    pypegen-fitting validate valve.yaml
"""

from pathlib import Path

import click

from ..step_import.config_schema import FittingLibraryConfig
from ..step_import.marker_detector import StepMarkerDetector


@click.group()
@click.version_option(version="0.1.0")
def cli():
    """pypegen fitting helper - import STEP files as fittings."""
    pass


@cli.command()
@click.argument("step_file", type=click.Path(exists=True, path_type=Path))
@click.option(
    "--output", "-o",
    type=click.Path(path_type=Path),
    help="Output YAML file path. If not specified, prints to stdout.",
)
@click.option(
    "--nps",
    default="2",
    help="Nominal pipe size for the fitting (default: 2).",
)
@click.option(
    "--id", "fitting_id",
    default=None,
    help="Fitting ID (default: derived from filename).",
)
@click.option(
    "--description", "-d",
    default="",
    help="Description of the fitting.",
)
def analyze(
    step_file: Path,
    output: Path | None,
    nps: str,
    fitting_id: str | None,
    description: str,
):
    """
    Analyze a STEP file and detect port markers.

    This command scans the STEP file for coordinate systems named with the
    pypegen convention (PORT_*, ATTACH_*, ORIGIN) and generates a fitting
    configuration.

    \b
    Expected marker naming in your CAD software:
      PORT_INLET   - Connection port named "inlet"
      PORT_OUTLET  - Connection port named "outlet"
      ATTACH_BODY  - Attachment point for BOM balloon
      ORIGIN       - Optional origin alignment

    Example:
        pypegen-fitting analyze valve.step --nps 2 -o valve.yaml
    """
    detector = StepMarkerDetector()

    # Analyze the file
    click.echo(f"\nAnalyzing: {step_file}")
    click.echo("-" * 50)

    analysis = detector.analyze(step_file)

    # Show results
    click.echo(f"Markers found: {analysis['markers_found']}")
    click.echo()

    if analysis["ports"]:
        click.echo("Ports detected:")
        for port in analysis["ports"]:
            click.echo(f"  {port['name']}:")
            click.echo(f"    Position:    {_format_tuple(port['position'])}")
            click.echo(f"    Z-direction: {_format_tuple(port['z_direction'])}")
    else:
        click.echo("No ports detected.")
        click.echo("  Tip: Add coordinate systems in your CAD software with names")
        click.echo("  like PORT_INLET, PORT_OUTLET, etc.")

    if analysis["attachment_points"]:
        click.echo("\nAttachment points detected:")
        for ap in analysis["attachment_points"]:
            click.echo(f"  {ap['name']}:")
            click.echo(f"    Position: {_format_tuple(ap['position'])}")

    if analysis["origin"]:
        click.echo(f"\nOrigin: {_format_tuple(analysis['origin'])}")

    # Generate config if markers were found
    if analysis["markers_found"] > 0:
        if fitting_id is None:
            fitting_id = step_file.stem.replace(" ", "_").lower()

        config = detector.generate_library_config(
            step_path=step_file,
            fitting_id=fitting_id,
            nps=nps,
            description=description,
        )

        if output:
            config.to_yaml(output)
            click.echo(f"\nConfiguration saved to: {output}")
        else:
            click.echo("\n" + "-" * 50)
            click.echo("Generated YAML configuration:")
            click.echo("-" * 50)
            import yaml
            click.echo(yaml.dump(config._to_dict(), default_flow_style=False, sort_keys=False))
    elif output:
        click.echo("\nNo markers found - cannot generate configuration.")
        click.echo("Use 'pypegen-fitting create' for interactive port definition.")


@cli.command()
@click.argument("step_file", type=click.Path(exists=True, path_type=Path))
@click.option(
    "--output", "-o",
    type=click.Path(path_type=Path),
    required=True,
    help="Output YAML file path.",
)
@click.option(
    "--nps",
    required=True,
    help="Nominal pipe size for the fitting.",
)
@click.option(
    "--id", "fitting_id",
    default=None,
    help="Fitting ID (default: derived from filename).",
)
@click.option(
    "--description", "-d",
    default="",
    help="Description of the fitting.",
)
def create(
    step_file: Path,
    output: Path,
    nps: str,
    fitting_id: str | None,
    description: str,
):
    """
    Create a fitting configuration interactively.

    This command guides you through defining ports for a STEP file that
    doesn't have CAD markers. You'll be prompted to enter port positions
    and orientations manually.

    Example:
        pypegen-fitting create valve.step --nps 2 -o valve.yaml
    """
    detector = StepMarkerDetector()

    # First check for markers
    markers = detector.detect_markers(step_file)
    if markers:
        click.echo(f"Found {len(markers)} markers in the STEP file.")
        if click.confirm("Use detected markers instead of manual entry?"):
            if fitting_id is None:
                fitting_id = step_file.stem.replace(" ", "_").lower()
            config = detector.generate_library_config(
                step_path=step_file,
                fitting_id=fitting_id,
                nps=nps,
                description=description,
            )
            config.to_yaml(output)
            click.echo(f"Configuration saved to: {output}")
            return

    # Interactive port definition
    click.echo(f"\nCreating configuration for: {step_file}")
    click.echo("-" * 50)

    if fitting_id is None:
        fitting_id = click.prompt(
            "Fitting ID",
            default=step_file.stem.replace(" ", "_").lower(),
        )

    # Collect ports
    ports = []
    click.echo("\nDefine ports (enter 'done' for name to finish):")
    while True:
        port = _prompt_for_port()
        if port is None:
            break
        ports.append(port)

    if not ports:
        click.echo("Warning: No ports defined. The fitting won't be usable for routing.")

    # Collect attachment points (optional)
    attachment_points = []
    if click.confirm("\nDefine attachment points for annotations?", default=False):
        click.echo("Define attachment points (enter 'done' for name to finish):")
        while True:
            ap = _prompt_for_attachment_point()
            if ap is None:
                break
            attachment_points.append(ap)

    # Build config
    from ..step_import.config_schema import (
        AttachmentPointConfig,
        FittingConfig,
        OrientationConfig,
        PortConfig,
    )

    port_configs = [
        PortConfig(
            name=p["name"],
            position=p["position"],
            orientation=OrientationConfig(z_direction=p["z_direction"]),
        )
        for p in ports
    ]

    ap_configs = [
        AttachmentPointConfig(
            name=ap["name"],
            position=ap["position"],
            normal=ap["normal"],
            attachment_type="bom",
        )
        for ap in attachment_points
    ]

    fitting_config = FittingConfig(
        id=fitting_id,
        step_file=step_file.name,
        nps=nps,
        description=description,
        ports=port_configs,
        attachment_points=ap_configs,
    )

    library_config = FittingLibraryConfig(fittings=[fitting_config])
    library_config.to_yaml(output)

    click.echo(f"\nConfiguration saved to: {output}")
    click.echo(f"Ports defined: {len(ports)}")
    click.echo(f"Attachment points defined: {len(attachment_points)}")


@cli.command()
@click.argument("config_file", type=click.Path(exists=True, path_type=Path))
def validate(config_file: Path):
    """
    Validate a fitting configuration file.

    Checks that the configuration is valid and all referenced STEP files exist.

    Example:
        pypegen-fitting validate my_fittings.yaml
    """
    click.echo(f"\nValidating: {config_file}")
    click.echo("-" * 50)

    try:
        config = FittingLibraryConfig.from_yaml(config_file)
    except Exception as e:
        click.echo(f"Error loading config: {e}", err=True)
        raise SystemExit(1) from None

    errors = []
    warnings = []

    for fitting in config.fittings:
        # Check STEP file exists
        step_path = config_file.parent / fitting.step_file
        if not step_path.exists():
            errors.append(f"[{fitting.id}] STEP file not found: {fitting.step_file}")

        # Check ports
        if not fitting.ports:
            warnings.append(f"[{fitting.id}] No ports defined")
        else:
            for port in fitting.ports:
                if not port.orientation.z_direction and not port.orientation.euler:
                    errors.append(
                        f"[{fitting.id}] Port '{port.name}' has no orientation"
                    )

        # Check attachment points
        for ap in fitting.attachment_points:
            norm = ap.normal
            magnitude = (norm[0]**2 + norm[1]**2 + norm[2]**2) ** 0.5
            if abs(magnitude - 1.0) > 0.01:
                warnings.append(
                    f"[{fitting.id}] Attachment '{ap.name}' normal is not normalized"
                )

    # Report results
    if errors:
        click.echo("\nErrors:")
        for e in errors:
            click.echo(f"  - {e}")

    if warnings:
        click.echo("\nWarnings:")
        for w in warnings:
            click.echo(f"  - {w}")

    if not errors and not warnings:
        click.echo("Configuration is valid.")
        click.echo(f"  Fittings: {len(config.fittings)}")
        for f in config.fittings:
            click.echo(f"    - {f.id}: {len(f.ports)} ports, {len(f.attachment_points)} attachment points")

    if errors:
        raise SystemExit(1)


def _format_tuple(t: tuple | list) -> str:
    """Format a tuple for display."""
    return f"({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})"


def _prompt_for_port() -> dict | None:
    """Interactively prompt for a port definition."""
    name = click.prompt("  Port name", default="done")
    if name.lower() == "done":
        return None

    click.echo(f"  Enter position for '{name}' (X, Y, Z in model units):")
    x = click.prompt("    X", type=float)
    y = click.prompt("    Y", type=float)
    z = click.prompt("    Z", type=float)

    click.echo(f"  Enter Z-direction (outward normal) for '{name}':")
    zx = click.prompt("    Z-dir X", type=float, default=0.0)
    zy = click.prompt("    Z-dir Y", type=float, default=0.0)
    zz = click.prompt("    Z-dir Z", type=float, default=1.0)

    return {
        "name": name,
        "position": (x, y, z),
        "z_direction": (zx, zy, zz),
    }


def _prompt_for_attachment_point() -> dict | None:
    """Interactively prompt for an attachment point definition."""
    name = click.prompt("  Attachment name", default="done")
    if name.lower() == "done":
        return None

    click.echo(f"  Enter position for '{name}' (X, Y, Z in model units):")
    x = click.prompt("    X", type=float)
    y = click.prompt("    Y", type=float)
    z = click.prompt("    Z", type=float)

    click.echo(f"  Enter outward normal for '{name}':")
    nx = click.prompt("    Normal X", type=float, default=1.0)
    ny = click.prompt("    Normal Y", type=float, default=0.0)
    nz = click.prompt("    Normal Z", type=float, default=0.0)

    return {
        "name": name,
        "position": (x, y, z),
        "normal": (nx, ny, nz),
    }


if __name__ == "__main__":
    cli()
