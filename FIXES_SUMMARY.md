# OpenArm 抓取系统修复说明

## 修复的问题

### 1. IK求解器超时问题 ✅
**文件**: `src/openarm_ros2/openarm_bimanual_moveit_config/config/kinematics.yaml`

**问题**: IK求解器超时设置为5毫秒，太短导致无法计算逆运动学

**修复**:
```yaml
# 修改前:
kinematics_solver_timeout: 0.0050000000000000001  # 5ms

# 修改后:
kinematics_solver_timeout: 0.05  # 50ms
```

### 2. 夹爪控制器配置错误 ✅
**文件**: `src/openarm_ros2/openarm_bimanual_moveit_config/config/ros2_controllers.yaml`

**问题**: `GripperActionController` 需要使用 `joint` 参数，但配置中使用了 `joints`（复数）

**修复**:
```yaml
# 修改前:
left_gripper_controller:
  ros__parameters:
    joints:  # 错误
      - openarm_left_finger_joint1

# 修改后:
left_gripper_controller:
  ros__parameters:
    joint: openarm_left_finger_joint1  # 正确
```

### 3. 缺少MoveIt规划配置 ✅
**新文件**: `src/openarm_ros2/openarm_bimanual_moveit_config/config/ompl_planning.yaml`

**问题**: 缺少OMPL规划器配置，使用默认参数可能导致规划失败

**修复**: 创建了完整的OMPL配置，包含：
- RRTConnect规划器（默认）
- RRT和RRTstar作为备选
- 合理的longest_valid_segment_fraction (0.005)
- 适当的projection_evaluator

### 4. MoveIt规划参数优化 ✅
**文件**: `src/openarm_grasp_planner/openarm_grasp_planner/grasp_planner.py`

**修改**:
```python
# 增加规划时间: 10秒 → 20秒
goal_msg.request.allowed_planning_time = 20.0

# 增加尝试次数: 5次 → 10次
goal_msg.request.num_planning_attempts = 10

# 明确指定规划器
goal_msg.request.planner_id = "RRTConnect"

# 增加位置容差: 2cm → 5cm
box_constraint.dimensions = [0.05, 0.05, 0.05]

# 增加姿态容差: 0.2弧度 → 0.5弧度
orientation_constraint.absolute_x_axis_tolerance = 0.5

# 增加等待时间: 15秒 → 30秒
wait_timeout = 30.0
result_timeout = 30.0
```

### 5. 启动时序优化 ✅
**文件**: `src/openarm_grasp_planner/launch/full_integration.launch.py`

**问题**: move_group启动15秒后就开始规划，可能未完全初始化

**修复**:
```python
# 视觉和抓取规划模块启动延迟: 15秒 → 20秒
TimerAction(period=20.0, actions=[vision_node, grasp_planner_node])

# 夹爪控制器和全流程验证启动延迟: 12秒 → 20秒
TimerAction(period=20.0, actions=[gripper_controller_node, full_verification_node])
```

---

## 使用说明

### 1. 重新编译项目
```bash
cd /Users/waychen/openarm_ws
colcon build --packages-select openarm_ros2 openarm_grasp_planner --symlink-install
source install/setup.bash
```

### 2. 启动完整系统
```bash
# 一键启动所有模块
ros2 launch openarm_grasp_planner full_integration.launch.py
```

### 3. 使用调试脚本
```bash
# 在另一个终端运行调试脚本
./debug_grasp_system.sh
```

调试脚本会检查：
- 控制器状态
- Action服务器是否就绪
- 关键Topics是否存在
- 节点是否运行
- TF树结构
- MoveIt规划参数

### 4. 分步启动（用于调试）

如果需要分步启动以定位问题：

```bash
# Terminal 1: 启动基础环境
ros2 launch openarm_bringup openarm_bimanual_mujoco_moveit.launch.py

# 等待20秒后...

# Terminal 2: 启动虚拟相机
ros2 run openarm_vision virtual_camera

# Terminal 3: 启动视觉检测
ros2 run openarm_vision banana_detector

# Terminal 4: 启动抓取规划
ros2 run openarm_grasp_planner grasp_planner

# Terminal 5: 启动夹爪控制
ros2 run openarm_grasp_planner gripper_controller

# Terminal 6: 启动全流程验证
ros2 run openarm_grasp_planner full_verification
```

---

## 期望的修复效果

### MoveIt规划
- ✅ IK求解器不再超时
- ✅ 规划成功率提高（更多尝试时间、更宽松的容差）
- ✅ 使用RRTConnect规划器，快速收敛
- ✅ 足够的等待时间让MoveIt完全初始化

### 夹爪控制
- ✅ 夹爪Action服务器正确配置
- ✅ 服务调用能成功控制夹爪开合
- ✅ 夹爪状态正确读取和显示

### 系统稳定性
- ✅ 所有模块按正确顺序启动
- ✅ 足够的初始化时间
- ✅ 减少超时和连接失败

---

## 故障排查

### 如果MoveIt仍然超时

1. **检查MoveIt节点**:
```bash
ros2 node list | grep move_group
```

2. **检查规划参数**:
```bash
ros2 param get /move_group allowed_planning_time
ros2 param get /move_group num_planning_attempts
```

3. **手动测试规划**:
```bash
# 在RViz中手动规划，看是否能成功
```

4. **检查目标位置**:
```bash
# 查看banana_target的位置是否在机械臂工作空间内
ros2 run tf2_ros tf2_echo openarm_body_link0 banana_target
```

### 如果夹爪不工作

1. **检查控制器状态**:
```bash
ros2 control list_controllers
```
应该看到 `left_gripper_controller` 和 `right_gripper_controller` 在状态 "active"

2. **检查Action服务器**:
```bash
ros2 action list | grep gripper
ros2 action info /left_gripper_controller/gripper_cmd
```

3. **手动测试夹爪**:
```bash
# 手动调用服务
ros2 service call /control_gripper std_srvs/srv/SetBool "{data: true}"  # 闭合
ros2 service call /control_gripper std_srvs/srv/SetBool "{data: false}" # 打开
```

4. **查看夹爪关节状态**:
```bash
ros2 topic echo /joint_states | grep openarm_left_finger_joint1
```

---

## 技术说明

### IK超时的影响
IK求解器需要在给定时间内计算从末端位姿到关节角度的映射。如果超时太短（如5ms），MoveIt会立即放弃，认为目标不可达。50ms是合理的时间范围。

### 夹爪控制器的joint vs joints
`GripperActionController` 是特殊的控制器，它只控制一个关节（通常是mimic joint）。因此它需要 `joint` 参数，而不是 `joints`。这是ros2_control的设计要求。

### MoveIt规划器选择
- **RRTConnect**: 双向RRT，通常最快找到解
- **RRT**: 单向RRT，适合简单场景
- **RRTstar**: 渐进式优化，路径质量更好但较慢

### 容差的重要性
- **位置容差**: 允许末端到达目标的误差范围
- **姿态容差**: 允许末端姿态的误差范围
- 过严的容差会导致规划失败，过宽会导致精度下降

---

## 后续优化建议

1. **使用笛卡尔规划器** (PILZ):
```yaml
# 在ompl_planning.yaml中添加
left_arm:
  default_planner_config: LIN  # 直线运动
```

2. **启用碰撞检测优化**:
```python
# 在grasp_planner.py中
goal_msg.request.workspace_parameters.header.frame_id = 'openarm_body_link0'
goal_msg.request.workspace_parameters.min_corner.x = -0.5
goal_msg.request.workspace_parameters.min_corner.y = -0.5
goal_msg.request.workspace_parameters.min_corner.z = 0.0
goal_msg.request.workspace_parameters.max_corner.x = 0.5
goal_msg.request.workspace_parameters.max_corner.y = 0.5
goal_msg.request.workspace_parameters.max_corner.z = 1.0
```

3. **添加路径简化**:
```python
# 在规划成功后简化路径
goal_msg.request.max_velocity_scaling_factor = 0.5
goal_msg.request.max_acceleration_scaling_factor = 0.5
```
