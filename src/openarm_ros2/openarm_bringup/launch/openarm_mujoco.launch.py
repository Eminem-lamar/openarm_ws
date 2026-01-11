import os
import xacro

from launch import LaunchDescription
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def generate_launch_description():
    # --- Generate robot_description (URDF) from xacro with Mujoco backend enabled ---
    description_package_path = get_package_share_directory("openarm_description")
    xacro_file = os.path.join(description_package_path, "urdf", "robot", "v10.urdf.xacro")

    doc = xacro.process_file(
        xacro_file,
        mappings={
            "arm_type": "v10",
            "bimanual": "false",
            "use_fake_hardware": "false",
            "ros2_control": "true",
            "can_interface": "can0",
            "arm_prefix": "",
            "use_mujoco_hardware": "true",
        },
    )
    robot_description = {"robot_description": doc.toprettyxml(indent="  ")}

    use_sim_time = {"use_sim_time": True}

    # Controllers configuration: reuse existing v10 controllers from openarm_bringup
    bringup_share = get_package_share_directory("openarm_bringup")
    controllers_file_path = os.path.join(
        bringup_share,
        "config",
        "v10_controllers",
        "openarm_v10_controllers.yaml",
    )

    # 使用动态路径查找MuJoCo模型
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.abspath(os.path.join(current_file_dir, '../../../../..'))
    mujoco_model_path = os.path.join(workspace_root, "openarm_mujoco", "v1", "openarm.xml")
    
    # Verify file exists
    if not os.path.exists(mujoco_model_path):
        raise FileNotFoundError(
            f"MuJoCo model file not found: {mujoco_model_path}\n"
            f"Please ensure the openarm_mujoco directory is in the workspace root."
        )

    node_mujoco_ros2_control = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            controllers_file_path,
            use_sim_time,
            {"mujoco_model_path": mujoco_model_path},
            {"enable_rendering": False},
        ],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            use_sim_time,
            robot_description,
        ],
    )

    # 使用controller_manager的spawner节点加载控制器（更健壮的方式）
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    # 添加夹爪控制器加载
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    rviz_config_file = os.path.join(
        description_package_path,
        "rviz",
        "arm_only.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[use_sim_time],
        arguments=["-d", rviz_config_file],
    )

    # 使用TimerAction确保控制器按顺序加载
    return LaunchDescription(
        [
            node_mujoco_ros2_control,
            node_robot_state_publisher,
            rviz_node,
            TimerAction(period=2.0, actions=[jsb_spawner]),
            TimerAction(period=3.0, actions=[arm_controller_spawner]),
            TimerAction(period=3.5, actions=[gripper_controller_spawner]),
        ]
    )