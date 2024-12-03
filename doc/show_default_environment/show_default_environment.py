import sys
sys.path.append('build/')
import parametric_motion_planner_module as pmp

sys.path.append('post-process/')
from visualization_helpers import show_environment, set_env_plot_limits

import json
import matplotlib.pyplot as plt

env = pmp.Environment()
env_json = json.loads(env.ToJson())
show_environment(env_json)
set_env_plot_limits(env_json)
plt.show()