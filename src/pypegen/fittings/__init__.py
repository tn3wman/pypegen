"""
Pipe Fittings Module

Provides parametric pipe fittings with port-based mating system.
"""

# Core fitting classes and utilities
from .pipe_fittings import (
    # Base classes
    AttachmentPoint,
    Fitting,
    Port,
    # Transformation utilities
    apply_transform_to_shape,
    compute_mate_transform,
    get_position,
    get_x_axis,
    get_y_axis,
    get_z_axis,
    identity_matrix,
    rotation_from_axis_angle,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
    translation_matrix,
    # Fitting factories
    make_butt_weld_elbow_fitting,
    make_butt_weld_elbow_45_fitting,
    make_butt_weld_reducer_fitting,
    make_butt_weld_tee_fitting,
    make_pipe,
    make_socket_weld_elbow,
    make_socket_weld_elbow_45,
    make_threaded_elbow_45_fitting,
    make_threaded_elbow_90_fitting,
    make_threaded_pipe,
    make_threaded_tee_fitting,
    make_weld_neck_flange,
    # Gasket functions
    get_gasket_thickness,
    make_gasket,
    mate_flanges_with_gasket,
)

# Butt weld reducer dimension data and utilities
from .butt_weld_reducer import (
    ASME_B169_REDUCER,
    ButtWeldReducerDims,
    compare_nps,
    get_available_reducers_from,
    get_butt_weld_reducer_dims,
    make_butt_weld_reducer,
    nps_to_float,
    validate_reducer,
)

# Butt weld 45° elbow dimension data and utilities
from .butt_weld_elbow_45 import (
    ASME_B169_ELBOW45,
    ButtWeldElbow45Dims,
    make_butt_weld_elbow_45,
    make_solid_elbow_45,
)

# Gasket dimension data
from .gasket_flat_ring import (
    ASME_B1621_CLASS150_FLAT,
    ASME_B1621_CLASS300_FLAT,
    FlatRingGasketDims,
    make_flat_ring_gasket,
)
from .gasket_spiral_wound import (
    ASME_B1620_CLASS300_SPIRAL,
    SpiralWoundGasketDims,
    make_spiral_wound_gasket,
)

# Flange dimension data and types
from .weld_neck_flange import ASME_B165_CLASS300_WN, BoltHoleOrientation, WeldNeckFlangeDims

__all__ = [
    # Base classes
    "AttachmentPoint",
    "Fitting",
    "Port",
    # Transformation utilities
    "apply_transform_to_shape",
    "compute_mate_transform",
    "get_position",
    "get_x_axis",
    "get_y_axis",
    "get_z_axis",
    "identity_matrix",
    "rotation_from_axis_angle",
    "rotation_matrix_x",
    "rotation_matrix_y",
    "rotation_matrix_z",
    "translation_matrix",
    # Fitting factories
    "make_butt_weld_elbow_fitting",
    "make_butt_weld_elbow_45_fitting",
    "make_butt_weld_reducer_fitting",
    "make_butt_weld_tee_fitting",
    "make_pipe",
    "make_socket_weld_elbow",
    "make_socket_weld_elbow_45",
    "make_threaded_elbow_45_fitting",
    "make_threaded_elbow_90_fitting",
    "make_threaded_pipe",
    "make_threaded_tee_fitting",
    "make_weld_neck_flange",
    # Gasket functions
    "get_gasket_thickness",
    "make_gasket",
    "mate_flanges_with_gasket",
    # Gasket dimension data
    "ASME_B1621_CLASS150_FLAT",
    "ASME_B1621_CLASS300_FLAT",
    "ASME_B1620_CLASS300_SPIRAL",
    "FlatRingGasketDims",
    "SpiralWoundGasketDims",
    "make_flat_ring_gasket",
    "make_spiral_wound_gasket",
    # Flange dimension data and types
    "ASME_B165_CLASS300_WN",
    "BoltHoleOrientation",
    "WeldNeckFlangeDims",
    # Butt weld reducer dimension data and utilities
    "ASME_B169_REDUCER",
    "ButtWeldReducerDims",
    "compare_nps",
    "get_available_reducers_from",
    "get_butt_weld_reducer_dims",
    "make_butt_weld_reducer",
    "nps_to_float",
    "validate_reducer",
    # Butt weld 45° elbow dimension data and utilities
    "ASME_B169_ELBOW45",
    "ButtWeldElbow45Dims",
    "make_butt_weld_elbow_45",
    "make_solid_elbow_45",
]
