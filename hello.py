from mujoco_py import load_model_from_path, MjSim, MjViewer


model = load_model_from_path("assets/hello.xml")
sim = MjSim(model)

viewer = MjViewer(sim)

for _ in range(2000):
    viewer.render()
    sim.step()
