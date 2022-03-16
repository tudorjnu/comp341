import mujoco

model = mujoco.MjModel.from_xml_path("./xmls/tosser.xml")
data = mujoco.MjData(model)
max_width = 256
max_height = 256

ctx = mujoco.GLContext(max_width, max_height)
ctx.make_current()
while data.time < 1:
    mujoco.mj_step(model, data)
    print(data.geom_xpos)
