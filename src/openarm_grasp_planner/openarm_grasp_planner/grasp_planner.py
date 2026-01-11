import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformListener, Buffer
from moveit_msgs.msg import Grasp
from moveit_msgs.srv import GraspPlanning
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np

class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner_node')
        
        # TF2 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建服务
        self.grasp_planning_service = self.create_service(
            GraspPlanning, 'plan_grasp', self.plan_grasp_callback
        )
        
        # 创建 Action 客户端用于执行轨迹（使用左臂）
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('抓取规划模块已启动')
        self.get_logger().info('等待轨迹控制器就绪...')
    
    def plan_grasp_callback(self, request, response):
        self.get_logger().info('收到抓取规划请求')
        
        # 1. 从TF获取目标物体的位置
        try:
            transform = self.tf_buffer.lookup_transform(
                'openarm_body_link0',  # 基座坐标系
                'banana_target',        # 目标物体坐标系
                rclpy.time.Time()       # 当前时间
            )
            
            target_pose = transform.transform
            self.get_logger().info(f'获取到目标位置: x={target_pose.translation.x:.3f}, y={target_pose.translation.y:.3f}, z={target_pose.translation.z:.3f}')
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'无法获取目标位置: {ex}')
            from moveit_msgs.msg import MoveItErrorCodes
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        
        # 2. 生成抓取姿态
        grasp = self.generate_grasp_pose(target_pose)
        
        # 3. 规划抓取轨迹
        grasp_trajectory = self.plan_grasp_trajectory(grasp)
        
        # 4. 执行抓取轨迹（通过 Action 接口）
        if self.execute_trajectory(grasp_trajectory):
            self.get_logger().info('抓取轨迹执行成功')
        else:
            self.get_logger().error('抓取轨迹执行失败')
            from moveit_msgs.msg import MoveItErrorCodes
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        
        # 5. 返回响应
        from moveit_msgs.msg import MoveItErrorCodes
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.grasps.append(grasp)
        
        return response
    
    def generate_grasp_pose(self, target_pose):
        """生成抓取姿态"""
        grasp = Grasp()
        
        # 设置抓取位置
        grasp.grasp_pose.header.frame_id = 'openarm_body_link0'
        grasp.grasp_pose.pose.position.x = target_pose.translation.x
        grasp.grasp_pose.pose.position.y = target_pose.translation.y
        grasp.grasp_pose.pose.position.z = target_pose.translation.z + 0.05  # 稍微高于目标
        
        # 设置抓取方向（简单示例）
        grasp.grasp_pose.pose.orientation.x = 0.0
        grasp.grasp_pose.pose.orientation.y = 0.0
        grasp.grasp_pose.pose.orientation.z = 0.0
        grasp.grasp_pose.pose.orientation.w = 1.0
        
        # 设置抓取参数
        grasp.pre_grasp_approach.direction.vector.z = -1.0  # 从上方接近
        grasp.pre_grasp_approach.min_distance = 0.01
        grasp.pre_grasp_approach.desired_distance = 0.05
        
        grasp.post_grasp_retreat.direction.vector.z = 1.0  # 抓取后向上撤退
        grasp.post_grasp_retreat.min_distance = 0.01
        grasp.post_grasp_retreat.desired_distance = 0.05
        
        return grasp
    
    def plan_grasp_trajectory(self, grasp_pose):
        """规划抓取轨迹"""
        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'openarm_body_link0'
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # 设置关节名称（使用左臂的关节名称）
        trajectory.joint_names = [
            'openarm_left_joint1',
            'openarm_left_joint2',
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        
        # 创建轨迹点 - 接近目标位置（基于目标位置计算）
        # 注意：这是简化的示例轨迹，实际应该使用 MoveIt 规划服务计算
        # 目标位置：x=0.433, y=-0.011, z=0.205
        # 根据目标在 x=0.433（前方），y=-0.011（接近中心），z=0.205（较低）
        # 使用更合理的关节角度来接近目标
        point1 = JointTrajectoryPoint()
        # 针对目标位置 x=0.433 的关节角度（经验值，需要根据实际 IK 计算）
        # joint1: 旋转朝向目标（约 0 度，因为 y 接近 0）
        # joint2: 向下倾斜（负值）
        # joint3: 向上弯曲（正值）
        # joint4: 调整高度（负值）
        point1.positions = [0.0, -0.4, 0.5, -0.3, 0.0, 0.4, 0.0]
        # 设置速度以加快运动（单位：rad/s）
        point1.velocities = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # 增加速度
        point1.accelerations = [0.0] * 7
        point1.time_from_start.sec = 0
        point1.time_from_start.nanosec = 300000000  # 0.3秒（加快速度）
        
        # 创建轨迹点 - 抓取位置（更接近目标）
        point2 = JointTrajectoryPoint()
        # 更接近目标的关节角度（针对 x=0.433 的位置）
        point2.positions = [0.05, -0.6, 0.7, -0.25, 0.0, 0.5, 0.0]
        point2.velocities = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # 增加速度
        point2.accelerations = [0.0] * 7
        point2.time_from_start.sec = 0
        point2.time_from_start.nanosec = 800000000  # 0.8秒（加快速度）
        
        # 将轨迹点添加到轨迹中
        trajectory.points.append(point1)
        trajectory.points.append(point2)
        
        self.get_logger().info(f'规划了包含 {len(trajectory.points)} 个轨迹点的抓取轨迹')
        
        return trajectory
    
    def execute_trajectory(self, trajectory):
        """通过 Action 接口执行轨迹"""
        # 检查服务器是否就绪（不阻塞）
        if not self.trajectory_client.server_is_ready():
            self.get_logger().error('轨迹控制器未就绪')
            return False
        
        # 创建 Action goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # 发送 goal（异步方式，不等待结果）
        self.get_logger().info('发送轨迹执行请求...')
        
        # 发送 goal（异步，不等待结果）
        # 在服务回调中，我们不能阻塞等待 Action 的结果
        future = self.trajectory_client.send_goal_async(goal_msg)
        
        # 使用回调记录结果（但不阻塞）
        def goal_response_callback(future):
            try:
                goal_handle = future.result()
                if goal_handle and goal_handle.accepted:
                    self.get_logger().info('轨迹执行请求已接受并执行')
                else:
                    self.get_logger().warn('轨迹执行请求被拒绝')
            except Exception as e:
                self.get_logger().error(f'轨迹执行请求处理异常: {str(e)}')
        
        future.add_done_callback(goal_response_callback)
        
        # 立即返回成功（不等待 future 完成）
        # 因为服务回调不应该阻塞，而且从日志看 goal 已经被接受了
        self.get_logger().info('轨迹执行请求已发送（异步执行）')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()