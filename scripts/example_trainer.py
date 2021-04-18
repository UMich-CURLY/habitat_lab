import os
import random
import sys

import git
import numpy as np
from gym import spaces

# %matplotlib inline
from matplotlib import pyplot as plt

# %cd "/content/habitat-lab"


if "google.colab" in sys.modules:
    # This tells imageio to use the system FFMPEG that has hardware acceleration.
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"
repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# %cd $dir_path

from PIL import Image

import habitat
from habitat.core.logging import logger
from habitat.core.registry import registry
from habitat.sims.habitat_simulator.actions import HabitatSimActions
from habitat.tasks.nav.nav import NavigationTask
from habitat_baselines.common.baseline_registry import baseline_registry
from habitat_baselines.config.default import get_config as get_baselines_config

if __name__ == "__main__":
    config = get_baselines_config(
        "./habitat_baselines/config/pointnav/ppo_pointnav.yaml"
    )

# %%
# set random seeds
if __name__ == "__main__":
    seed = "42"  # @param {type:"string"}
    # steps_in_thousands = "10"  # @param {type:"string"}

    config.defrost()
    config.TASK_CONFIG.SEED = int(seed)
    # config.TOTAL_NUM_STEPS = int(steps_in_thousands)
    config.LOG_INTERVAL = 1
    config.freeze()

    random.seed(config.TASK_CONFIG.SEED)
    np.random.seed(config.TASK_CONFIG.SEED)

# %%
if __name__ == "__main__":
    trainer_init = baseline_registry.get_trainer(config.TRAINER_NAME)
    trainer = trainer_init(config)
    # trainer.train()
    trainer.eval()


# %%
# @markdown (double click to see the code)

# example tensorboard visualization
# for more details refer to [link](https://github.com/facebookresearch/habitat-lab/tree/master/habitat_baselines#additional-utilities).

try:
    from IPython import display

    with open("./res/img/tensorboard_video_demo.gif", "rb") as f:
        display.display(display.Image(data=f.read(), format="png"))
except ImportError:
    pass