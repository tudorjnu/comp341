from dm_control import viewer
import numpy as np
from mujoco_py import load_model_from_path, MjSim, MjViewer
from dm_control import mujoco

env = mujoco.Physics.from_xml_path("assets/hello.xml")
action_spec = env.action_spec()


def random_policy(time_step):
    del time_step  # Unused.
    return np.random.uniform(low=action_spec.minimum,
                             high=action_spec.maximum,
                             size=action_spec.shape)


viewer.launch(env, policy=random_policy)

# model = load_model_from_path("assets/hello.xml")
# sim = MjSim(model)

# viewer = MjViewer(sim)

# for _ in range(2000):
# viewer.render()
# sim.step()
