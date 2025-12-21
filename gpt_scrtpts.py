"""
Example script demonstrating NPT thread generation using the die-cut approach.

This script creates a pipe with NPT external threads on both ends.
"""

from __future__ import annotations

import sys
from pathlib import Path

import cadquery as cq

# Allow running from repo root without installing the package.
ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if SRC.exists():
    sys.path.insert(0, str(SRC))

from pypegen.fittings.npt_thread import get_npt_spec, make_npt_external_thread
from pypegen.pipe_generator import INCH, PIPE_SPECS

# -----------------------------
# Parameters (inches)
# -----------------------------
NPS = "1"
SCHEDULE = "40"
PIPE_LEN_IN = 6.0
OUTPUT_STEP = "pipe_1in_sch40_npt_both_ends.step"
OUTPUT_STL = "pipe_1in_sch40_npt_both_ends.stl"


def _pipe_spec_key(nps: str, schedule: str) -> tuple[str, str]:
    nps_key = nps if nps.endswith("in") else f"{nps}in"
    return (nps_key, schedule)


def build_npt_threaded_pipe(
    nps: str,
    schedule: str,
    pipe_len_in: float,
) -> cq.Shape:
    """Build a pipe with external NPT threads on both ends (mm units internally).

    Uses the new die-cut approach for NPT thread generation.
    """
    spec = get_npt_spec(nps)
    pipe_spec = PIPE_SPECS[_pipe_spec_key(nps, schedule)]

    pipe_len = pipe_len_in * INCH
    thread_len = spec.L2

    if pipe_len <= 2.0 * thread_len:
        raise ValueError("pipe_len_in too short for two threaded ends.")

    # Create inlet threaded section using die-cut approach
    inlet_thread = make_npt_external_thread(
        nps,
        thread_length=thread_len,
        pipe_id=pipe_spec.id,
        end_finishes=("chamfer", "fade"),
        runout_turns=0.25,
    )

    # Create outlet threaded section (flipped)
    outlet_thread = make_npt_external_thread(
        nps,
        thread_length=thread_len,
        pipe_id=pipe_spec.id,
        end_finishes=("chamfer", "fade"),
        runout_turns=0.25,
    )
    outlet_thread = outlet_thread.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
    outlet_thread = outlet_thread.translate(cq.Vector(0, 0, pipe_len))

    # Middle straight pipe section
    middle_len = pipe_len - 2.0 * thread_len
    middle_outer = cq.Solid.makeCylinder(
        pipe_spec.od / 2.0,
        middle_len,
        cq.Vector(0, 0, thread_len),
        cq.Vector(0, 0, 1),
    )
    middle_inner = cq.Solid.makeCylinder(
        pipe_spec.id / 2.0,
        middle_len,
        cq.Vector(0, 0, thread_len),
        cq.Vector(0, 0, 1),
    )
    middle = middle_outer.cut(middle_inner)

    # Fuse all sections
    pipe = inlet_thread.fuse(middle)
    pipe = pipe.fuse(outlet_thread)
    return pipe


if __name__ == "__main__":
    print(f"Creating {NPS}\" NPT threaded pipe...")
    model = build_npt_threaded_pipe(
        nps=NPS,
        schedule=SCHEDULE,
        pipe_len_in=PIPE_LEN_IN,
    )

    print(f"Exporting to {OUTPUT_STEP}...")
    cq.exporters.export(model, OUTPUT_STEP)

    print(f"Exporting to {OUTPUT_STL}...")
    cq.exporters.export(model, OUTPUT_STL)

    print("Done!")

    # If using cq-editor / ocp_vscode:
    # show_object(model, name="1in_sch40_NPT_pipe")
