import mujoco
import mujoco.viewer
import time

model_name = "1_robot_model"  # Change for the model you are interested in
model_path = f"../models/{model_name}.xml"

# Load the model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

VIEWER_TIME = 5  # sec

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()

    while viewer.is_running() and time.time() - start < VIEWER_TIME:
        step_start = time.time()

        mujoco.mj_step(model, data)

        viewer.sync()

        # Control the simulation step time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
