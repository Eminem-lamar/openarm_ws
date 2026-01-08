import os
import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def generate_launch_description():
    """Unified launch: MuJoCo ros2_control + MoveIt + RViz + MuJoCo viewer."""

    # === 1. Robot description (bimanual, MuJoCo backend) ===
    description_package_path = get_package_share_directory("openarm_description")
    xacro_file = os.path.join(description_package_path, "urdf", "robot", "v10.urdf.xacro")

    doc = xacro.process_file(
        xacro_file,
        mappings={
            "arm_type": "v10",
            "body_type": "v10",
            "bimanual": "true",
            "hand": "true",
            "ee_type": "openarm_hand",
            "use_fake_hardware": "false",
            "ros2_control": "true",
            "use_mujoco_hardware": "true",
            "left_can_interface": "can1",
            "right_can_interface": "can0",
            "left_arm_prefix": "left_",
            "right_arm_prefix": "right_",
        },
    )
    robot_description = {"robot_description": doc.toprettyxml(indent="  ")}

    # Use simulated time consistently across all nodes so that MoveIt,
    # ros2_control (inside mujoco_ros2_control) and RViz share the same
    # time base driven by the simulation's /clock.
    use_sim_time = {"use_sim_time": True}

    bringup_share = get_package_share_directory("openarm_bringup")
    # Use the dedicated bimanual ros2_control config so that controller
    # names (left/right_joint_trajectory_controller, grippers, etc.)
    # match the MoveIt configuration in
    # openarm_bimanual_moveit_config/config/moveit_controllers.yaml.
    controllers_file_path = os.path.join(
        bringup_share,
        "config",
        "v10_controllers",
        "openarm_v10_bimanual_controllers.yaml",
    )

    # === 2. MuJoCo ros2_control backend (no internal rendering) ===
    mujoco_model_path = "/home/zjc/openarm_ws/openarm_mujoco/v1/scene.xml"

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
        parameters=[use_sim_time, robot_description],
    )

    # Use controller_manager "spawner" nodes to bring up controllers,
    # same pattern as other openarm launch files. This is more robust
    # than shelling out to "ros2 control" from within a launch file.
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

    arm_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_joint_trajectory_controller",
            "right_joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    gripper_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_gripper_controller",
            "right_gripper_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    # === 3. MoveIt: move_group + RViz (bimanual config) ===
    moveit_pkg_share = get_package_share_directory("openarm_bimanual_moveit_config")

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg_share, "launch", "move_group.launch.py")
        ),
        # Propagate use_sim_time into MoveIt nodes if the launch file
        # exposes it as a launch argument (standard in MoveIt configs).
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg_share, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Ensure MoveIt move_group node also uses simulated time. Although we
    # pass use_sim_time via launch arguments above, in some versions of
    # MoveIt this parameter may not be wired through, so we force it here
    # once the node is up.
    set_move_group_use_sim_time = ExecuteProcess(
        cmd=[
            "ros2",
            "param",
            "set",
            "/move_group",
            "use_sim_time",
            "true",
        ],
        output="screen",
    )

    # === 4. Optional MuJoCo GUI viewer (subscribes to /joint_states) ===
    mujoco_viewer_node = Node(
        package="openarm_mujoco_viewer",
        executable="openarm_bimanual_viewer",
        output="screen",
        parameters=[{"scene_path": mujoco_model_path}],
    )

    # === 5. Compose launch description with simple timing for controllers ===
    # Give controller_manager inside mujoco_ros2_control a short time to
    # start before spawning controllers.
    delayed_jsb = TimerAction(period=2.0, actions=[jsb_spawner])
    delayed_arms = TimerAction(period=3.0, actions=[arm_controllers_spawner])
    delayed_grippers = TimerAction(period=3.5, actions=[gripper_controllers_spawner])
    delayed_set_move_group_time = TimerAction(period=5.0, actions=[set_move_group_use_sim_time])

    return LaunchDescription(
        [
            node_mujoco_ros2_control,
            node_robot_state_publisher,
            delayed_jsb,
            delayed_arms,
            delayed_grippers,
            delayed_set_move_group_time,
            move_group_launch,
            moveit_rviz_launch,
            mujoco_viewer_node,
        ]
    )
