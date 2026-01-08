import os
import xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

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

    # MuJoCo model path for OpenArm
    mujoco_model_path = "/home/zjc/openarm_ws/openarm_mujoco/v1/openarm.xml"

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

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_robot_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
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

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node_mujoco_ros2_control,
                    on_start=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_robot_controller],
                )
            ),
            node_mujoco_ros2_control,
            node_robot_state_publisher,
            rviz_node,
        ]
    )
