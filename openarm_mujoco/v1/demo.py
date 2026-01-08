import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path('/home/ldp/openarm_ws/openarm_mujoco/v1/scene.xml')
data   = mujoco.MjData(model)

# 获取执行器索引
aid = model.actuator('position').id

with mujoco.viewer.launch_passive(model, data) as viewer:
    step = 0
    while viewer.is_running():
        # 简单正弦控制
        data.ctrl[aid] = 1.5 * mujoco.mju_sin(0.002 * step)
        mujoco.mj_step(model, data)

        print(f'step={step:4d}  pos={data.qpos[0]:+.2f}')
        viewer.sync()
        step += 1
        time.sleep(0.002)   # 放慢动画，方便观察