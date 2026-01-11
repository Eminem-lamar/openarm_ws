from setuptools import setup
import os

package_name = 'openarm_grasp_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加launch目录
        (os.path.join('share', package_name, 'launch'), 
         [os.path.join('launch', 'full_integration.launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='抓取规划与夹爪控制模块',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_planner = openarm_grasp_planner.grasp_planner:main',
            'gripper_controller = openarm_grasp_planner.gripper_controller:main',
            'full_verification = openarm_grasp_planner.full_verification:main',
        ],
    },
)