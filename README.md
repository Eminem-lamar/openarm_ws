# OpenArm 双臂仿真与手眼标定项目

本项目实现了双臂机器人的动力学仿真、运动规划以及感知层的手眼标定全流程。

---

## 一、 成员分工：成员 2（标定与视觉检测工程师）

**核心职责：聚焦“感知层面”的技术闭环**

- **手眼标定实现**：采用“眼在手外”方案，基于 Tsai-Lenz 算法建立坐标变换。针对双臂 TF 树中断问题进行了专项修复，确保了基座坐标溯源的准确性。
- **视觉检测与定位**：处理深度图像与点云，基于颜色识别实现对目标香蕉的实时检测，并将位姿变换至机器人基座坐标系。

---

## 二、 核心代码设计说明

- **virtual_camera.py**：将 MuJoCo 渲染流转换为标准 ROS 2 图像话题。
- **banana_detector.py**：基于 HSV 色彩滤波提取物体位姿，结合标定矩阵完成坐标系溯源。

---

## 三、 运行指南（四终端流程）

> **注意**：执行以下命令前，请务必确保在每个终端都运行了 \`source install/setup.bash\`。

### 1. 终端 1：启动仿真核心 (MuJoCo + MoveIt)
\`\`\`bash
cd ~/openarm_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch openarm_bringup openarm_bimanual_mujoco_moveit.launch.py \\
robot_controller:=/home/ros/openarm_ws/install/openarm_bringup/share/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml
\`\`\`

### 2. 终端 2：启动虚拟相机接口
\`\`\`bash
source /home/ros/openarm_ws/install/setup.bash
ros2 run openarm_vision virtual_camera
\`\`\`

### 3. 终端 3：启动视觉识别大脑
\`\`\`bash
cd /home/ros/openarm_ws
colcon build --packages-select openarm_vision
source install/setup.bash
ros2 run openarm_vision banana_detector
\`\`\`

### 4. 终端 4：数据监控与验证 (TF Echo)
\`\`\`bash
source /home/ros/openarm_ws/install/setup.bash
export ROS_SIM_TIME=true
ros2 run tf2_ros tf2_echo openarm_body_link0 banana_target
\`\`\`

---

## 四、 实验数据结果 (成员 2 核心产出)

### 4.1 标定矩阵 (物理真值溯源)
\`\`\`text
[[-5.3720e-04,  7.0266e-01,  7.1152e-01,  0.0000e+00],
 [ 1.0000e+00,  1.3085e-03, -5.3720e-04, -4.0000e-02],
 [-1.3085e-03,  7.1151e-01, -7.0266e-01,  6.2000e-01],
 [ 0.0000e+00,  0.0000e+00,  0.0000e+00,  1.0000e+00]]
\`\`\`

### 4.2 定位精度实测
- **平移 (Translation)**: \`[0.433, -0.011, 0.205]\`
- **旋转 (Quaternion)**: \`[0.0, 0.0, 0.0, 1.0]\`

---

## 五、 场景自定义
修改物体位置：编辑 \`src/openarm_mujoco/v1/scene_with_table.xml\`，修改 \`body name="banana"\` 的 \`pos\` 属性。
