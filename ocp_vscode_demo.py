# %%

# The markers "# %%" separate code blocks for execution (cells) 
# Press shift-enter to exectute a cell and move to next cell
# Press ctrl-enter to exectute a cell and keep cursor at the position
# For more details, see https://marketplace.visualstudio.com/items?itemName=ms-toolsai.jupyter

# %%

import cadquery as cq
from ocp_vscode import show_object

import cadquery as cq

from ocp_vscode import *
set_port(3939)

set_defaults(reset_camera=False, helper_scale=5)

shape = cq.importers.importStep("pipe_1in_sch40_npt_both_ends.step")
show_object(shape, name="pipe", port=3939)

