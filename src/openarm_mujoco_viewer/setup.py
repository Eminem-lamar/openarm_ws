from setuptools import setup

package_name = "openarm_mujoco_viewer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openarm_user",
    maintainer_email="user@example.com",
    description="OpenArm bimanual MuJoCo viewer synchronized with ROS2 joint_states.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "openarm_bimanual_viewer = openarm_mujoco_viewer.openarm_bimanual_viewer:main",
        ],
    },
)
