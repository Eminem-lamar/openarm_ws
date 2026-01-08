from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # 构建 MoveIt 配置（由 setup_assistant 生成的包提供）
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()

    # 原始的 move_group LaunchDescription（不带 use_sim_time）
    inner_ld = generate_move_group_launch(moveit_config)

    # 对外暴露一个 use_sim_time 的 launch 参数，默认 false
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    # 在 move_group 节点启动后，通过 ros2 param 设置 use_sim_time
    set_use_sim_time = ExecuteProcess(
        cmd=[
            "ros2",
            "param",
            "set",
            "/move_group",
            "use_sim_time",
            LaunchConfiguration("use_sim_time"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim_time")),
    )

    # 留一点时间给 move_group 启动，否则参数设置会失败
    delayed_set_use_sim_time = TimerAction(period=5.0, actions=[set_use_sim_time])

    # 组合成一个新的 LaunchDescription：先加参数声明，再加原来的所有动作，最后加定时设置
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    for entity in inner_ld.entities:
        ld.add_action(entity)
    ld.add_action(delayed_set_use_sim_time)

    return ld
