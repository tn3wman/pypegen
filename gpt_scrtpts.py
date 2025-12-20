from __future__ import annotations

import sys
from pathlib import Path

import cadquery as cq
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse

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
THREAD_LEN_IN = None  # None = use ASME B1.20.1 L2 for NPS
OUTPUT_STEP = "pipe_1in_sch40_npt_both_ends.step"


def _pipe_spec_key(nps: str, schedule: str) -> tuple[str, str]:
    nps_key = nps if nps.endswith("in") else f"{nps}in"
    return (nps_key, schedule)


def build_npt_threaded_pipe(
    nps: str,
    schedule: str,
    pipe_len_in: float,
    thread_len_in: float | None = None,
) -> cq.Shape:
    """Build a pipe with external NPT threads on both ends (mm units internally)."""

    def _fuse(a: cq.Shape, b: cq.Shape) -> cq.Shape:
        fused = BRepAlgoAPI_Fuse(a.wrapped, b.wrapped)
        fused.SetFuzzyValue(1e-3)
        fused.Build()
        if not fused.IsDone():
            return cq.Compound.makeCompound([a, b])
        return cq.Shape(fused.Shape())
    spec = get_npt_spec(nps)
    pipe_spec = PIPE_SPECS[_pipe_spec_key(nps, schedule)]

    pipe_len = pipe_len_in * INCH
    thread_len = spec.L2 if thread_len_in is None else thread_len_in * INCH

    if pipe_len <= 2.0 * thread_len:
        raise ValueError("pipe_len_in too short for two threaded ends.")

    # Thread section: Z=0 is the pipe end (small diameter), Z=thread_len is larger.
    thread_section = make_npt_external_thread(
        nps,
        thread_length=thread_len,
        pipe_id=pipe_spec.id,
        end_finishes=("raw", "pipe_junction"),
        blend_radius=pipe_spec.od / 2.0,
        runout_turns=2.0,
        method="die_cut",
    )

    inlet_thread = thread_section
    outlet_thread = thread_section.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
    outlet_thread = outlet_thread.translate(cq.Vector(0, 0, pipe_len))

    # Middle straight pipe section.
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

    fused = _fuse(inlet_thread, middle)
    fused = _fuse(fused, outlet_thread)
    return fused


if __name__ == "__main__":
    model = build_npt_threaded_pipe(
        nps=NPS,
        schedule=SCHEDULE,
        pipe_len_in=PIPE_LEN_IN,
        thread_len_in=THREAD_LEN_IN,
    )

    cq.exporters.export(model, OUTPUT_STEP)

    # If using cq-editor / ocp_vscode:
    # show_object(model, name="1in_sch40_NPT_pipe")
