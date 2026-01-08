# 一、环境搭建
## OpenArm MuJoCo + MoveIt 联合仿真说明

本工作区基于 OpenArm 官方 ROS 2 包和 MuJoCo 仿真，提供：

- 使用 **mujoco_ros2_control** 做动力学仿真和 ros2_control 控制
- 使用 **MoveIt** 在 RViz 中进行规划
- 使用自定义 **MuJoCo Viewer** 窗口同步显示机械臂姿态

下面说明需要安装的依赖、编译命令，以及如何一条命令同时起 RViz 和 MuJoCo 窗口，并实现两者同步运动。

---

## 1. 依赖环境

### 1.1 ROS 2

- 建议版本：ROS 2 Humble（Ubuntu 22.04）或兼容版本
- 已安装常见工具：`colcon`, `rosdep`, `rviz2`, `MoveIt 2`

### 1.2 MuJoCo 与相关 ROS 包

组件：

- 自行安装MuJoCo 3.4.0
- 在ros2 的humble环境自行安装ompl和filiters
- 仓库已存在的包的来源
    - `mujoco_ros2`https://github.com/Woolfrey/mujoco_ros2
    - `mujoco_ros2_control`https://github.com/moveit/mujoco_ros2_control
    - OpenArm ROS 2 栈：`openarm_ros2`（含 `openarm_bringup`、`openarm_bimanual_moveit_config` 等）https://github.com/enactic/openarm_ros2
    - 本仓库中的 MuJoCo 资源：`openarm_mujoco`（MJCF 场景文件）https://github.com/enactic/openarm_mujoco
    - 本仓库中的 Viewer：`openarm_mujoco_viewer`
	- 环境资源`openarm_description`https://github.com/enactic/openarm_description
- 已实现资源：香蕉，桌子，相机（MuJoCo 场景）
	- 香蕉：MJCF 模型 [openarm_mujoco/v1/011_banana.xml](openarm_mujoco/v1/011_banana.xml)，对应网格位于 [openarm_mujoco/v1/meshes/011_banana](openarm_mujoco/v1/meshes/011_banana)
	- 桌子：带桌子的整体场景文件 [openarm_mujoco/v1/scene_with_table.xml](openarm_mujoco/v1/scene_with_table.xml)
	- 相机：深度相机网格位于 [openarm_mujoco/v1/meshes/visual/cam](openarm_mujoco/v1/meshes/visual/cam)，并在带相机的双臂场景中使用，例如 [openarm_mujoco/v1/openarm_bimanual_cam.xml](openarm_mujoco/v1/openarm_bimanual_cam.xml)
- RViz 场景配置：
	- 基础双臂显示： [src/openarm_description/rviz/bimanual.rviz](src/openarm_description/rviz/bimanual.rviz)
	- 单臂显示： [src/openarm_description/rviz/arm_only.rviz](src/openarm_description/rviz/arm_only.rviz)
	- Bringup 使用的双臂 RViz： [src/openarm_ros2/openarm_bringup/rviz/bimanual.rviz](src/openarm_ros2/openarm_bringup/rviz/bimanual.rviz)
	- MoveIt 规划专用 RViz 配置： [src/openarm_ros2/openarm_bimanual_moveit_config/config/moveit.rviz](src/openarm_ros2/openarm_bimanual_moveit_config/config/moveit.rviz)
	- 以上 RViz 配置中使用的主要 xacro/URDF：
		- 机器人总体描述（单臂/双臂）： [src/openarm_description/urdf/robot/v10.urdf.xacro](src/openarm_description/urdf/robot/v10.urdf.xacro)
		- 机械臂宏定义： [src/openarm_description/urdf/arm/openarm_macro.xacro](src/openarm_description/urdf/arm/openarm_macro.xacro)
		- 机身/底座： [src/openarm_description/urdf/body/openarm_body.xacro](src/openarm_description/urdf/body/openarm_body.xacro)
		- 末端执行器（手爪）： [src/openarm_description/urdf/ee/openarm_hand.xacro](src/openarm_description/urdf/ee/openarm_hand.xacro)
---

## 2. 编译工作区

在工作区根目录（本文件所在路径）执行：

```bash
cd ~/openarm_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --packages-ignore openarm_hardware

source install/setup.bash
```

说明：

- 忽略 `openarm_hardware`，仅使用 MuJoCo 仿真硬件接口
- 每次修改代码后建议重新执行 `colcon build` 与 `source install/setup.bash`

---

## 3. 一条 launch 启动：MuJoCo + MoveIt + RViz + MuJoCo Viewer

本工作区在包 `openarm_bringup` 中提供了一个统一的 launch 文件：

- 文件路径：`src/openarm_ros2/openarm_bringup/launch/openarm_bimanual_mujoco_moveit.launch.py`

它会自动启动：

- MuJoCo 后端仿真：`mujoco_ros2_control`（双臂 + 地面场景）
- `ros2_control` 控制器：
	- `joint_state_broadcaster`
	- `left_joint_trajectory_controller` / `right_joint_trajectory_controller`
	- `left_gripper_controller` / `right_gripper_controller`
- `robot_state_publisher`（发布 TF 和 /joint_states 供可视化）
- MoveIt：`move_group`
- MoveIt RViz：带交互规划界面的 RViz2
- MuJoCo Viewer：`openarm_mujoco_viewer/openarm_bimanual_viewer`（独立 MuJoCo 窗口）

### 3.1 启动命令

在终端中执行（建议使用一个新 shell）：

```bash
cd ~/openarm_ws
source install/setup.bash

ros2 launch openarm_bringup openarm_bimanual_mujoco_moveit.launch.py
```

启动完成后：

- 会弹出 **RViz2** 窗口（带 MoveIt 插件），用于规划和执行
- 会弹出 **MuJoCo** 3D 窗口，同步显示双臂仿真状态

两者都基于同一套 `/joint_states` 与 `/controller_manager` 控制器，因此规划执行后，RViz 和 MuJoCo 中的机械臂会**同步运动**。

---

## 4. 在 RViz 中使用 MoveIt 控制机械臂

1. 确认统一 launch 已正常启动（终端无严重报错）
2. 在 RViz 中：
	 - 打开 MotionPlanning 面板
	 - 选择规划组（如 `left_arm`、`right_arm` 或双臂组）
	 - 在交互 Marker 或 joint 控件上设定目标位姿/关节角
	 - 点击 **Plan and Execute**
3. 执行成功后：
	 - RViz 内机器人模型按规划轨迹运动
	 - MuJoCo 窗口中的机械臂同步跟随运动

---

## 5. MuJoCo Viewer 操作说明（视角 & 速度）

MuJoCo Viewer 节点代码位于：

- `src/openarm_mujoco_viewer/openarm_mujoco_viewer/openarm_bimanual_viewer.py`

其行为特性：

- 仅根据 `/joint_states` 做前向运动学，不额外积分物理（与 RViz 速度保持一致）
- 相机交互：
	- 滚轮：缩放
	- 左键拖动：旋转视角
	- 中键拖动：平移
	- 右键拖动：前后移动（类似缩放）

如需单独启动 MuJoCo Viewer（例如已通过其它 launch 起好 Mujoco + MoveIt）：

```bash
cd ~/openarm_ws
source install/setup.bash

ros2 run openarm_mujoco_viewer openarm_bimanual_viewer \
	--ros-args -p scene_path:=/home/zjc/openarm_ws/openarm_mujoco/v1/scene.xml
```

---
