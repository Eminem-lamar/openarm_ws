#!/bin/bash

# OpenArm Grasp System Debug and Test Script
# 用于诊断MoveIt规划和夹爪控制问题

echo "============================================"
echo "OpenArm 抓取系统调试脚本"
echo "============================================"
echo ""

# 检查ROS 2是否运行
if ! ros2 node list &>/dev/null; then
    echo "❌ 错误: ROS 2 未运行"
    echo "   请先启动系统: ros2 launch openarm_grasp_planner full_integration.launch.py"
    exit 1
fi

echo "✅ ROS 2 正在运行"
echo ""

# 1. 检查控制器状态
echo "============================================"
echo "1. 检查控制器状态"
echo "============================================"
ros2 control list_controllers
echo ""

# 2. 检查Action服务器
echo "============================================"
echo "2. 检查Action服务器"
echo "============================================"

echo "→ 轨迹控制器 Actions:"
ros2 action list | grep -i trajectory || echo "   未找到轨迹控制器"

echo ""
echo "→ 夹爪控制器 Actions:"
ros2 action list | grep -i gripper || echo "   未找到夹爪控制器"

echo ""
echo "→ MoveIt Actions:"
ros2 action list | grep -i move || echo "   未找到MoveIt Action"

echo ""

# 3. 检查Topic
echo "============================================"
echo "3. 检查关键Topics"
echo "============================================"

echo "→ 关节状态:"
ros2 topic list | grep -i joint_state || echo "   未找到/joint_states"

echo ""
echo "→ TF相关:"
ros2 topic list | grep -i tf || echo "   未找到TF话题"

echo ""

# 4. 检查节点
echo "============================================"
echo "4. 检查关键节点"
echo "============================================"

echo "→ 控制器管理器:"
ros2 node list | grep controller_manager || echo "   未找到controller_manager"

echo ""
echo "→ MoveIt:"
ros2 node list | grep move_group || echo "   未找到move_group节点"

echo ""
echo "→ 视觉模块:"
ros2 node list | grep -E "(virtual_camera|banana_detector)" || echo "   未找到视觉节点"

echo ""
echo "→ 抓取规划:"
ros2 node list | grep grasp_planner || echo "   未找到抓取规划节点"

echo ""
echo "→ 夹爪控制:"
ros2 node list | grep gripper_controller || echo "   未找到夹爪控制器"

echo ""

# 5. 检查TF树
echo "============================================"
echo "5. 检查TF树中的关键帧"
echo "============================================"
ros2 run tf2_tools view_frames 2>/dev/null &
TF_PID=$!
sleep 3
kill $TF_PID 2>/dev/null
echo "→ 检查重要frames:"
timeout 2 ros2 run tf2_ros tf2_echo openarm_body_link0 openarm_left_link7 2>&1 | head -5
echo ""

# 6. 测试夹爪控制器
echo "============================================"
echo "6. 测试夹爪控制"
echo "============================================"

# 检查夹爪控制器是否就绪
echo "→ 检查夹爪Action服务器状态:"
if ros2 action info /left_gripper_controller/gripper_cmd &>/dev/null; then
    echo "   ✅ 左夹爪Action服务器就绪"
    echo "   Action信息:"
    ros2 action info /left_gripper_controller/gripper_cmd
else
    echo "   ❌ 左夹爪Action服务器未就绪"
fi

echo ""

# 7. 测试MoveIt规划
echo "============================================"
echo "7. 检查MoveIt规划参数"
echo "============================================"

echo "→ MoveIt参数:"
ros2 param get /move_group allowed_planning_time 2>/dev/null || echo "   未获取到规划时间"
ros2 param get /move_group num_planning_attempts 2>/dev/null || echo "   未获取到尝试次数"
ros2 param get /move_group planner_id 2>/dev/null || echo "   未获取到规划器ID"

echo ""

# 8. 实时监控
echo "============================================"
echo "8. 实时监控 (Ctrl+C 退出)"
echo "============================================"

echo "→ 关节状态 (2秒更新一次):"
while true; do
    echo ""
    echo "--- $(date +%H:%M:%S) ---"
    ros2 topic echo /joint_states --once 2>/dev/null | grep -A 1 "name:" | head -8
    sleep 2
done
