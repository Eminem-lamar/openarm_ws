import rclpy
from rclpy.node import Node 
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import GripperCommand
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
import threading

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')
        
        # 夹爪状态参数（根据 SRDF/URDF 定义：0.0=闭合，0.060=最大张开）
        # 注意：关节位置为单指移动距离。指间距 = 2 * 关节位置。
        self.gripper_open_position = 0.060  # 完全打开（间距 0.12m）
        self.gripper_closed_position = 0.0  # 完全闭合

        # 修改：修正抓取位置计算
        # 香蕉直径约 0.04m -> 单指接触点为 0.02m
        # 设置目标为 0.015m 以产生 "Virtual Spring" 挤压力 (0.02 - 0.015 = 5mm 挤压量)
        # 此前 0.045 太大，会导致夹爪张开 0.09m，无法夹住香蕉
        self.gripper_grasp_position = 0.015
        self.gripper_max_effort = 80.0  # 略微增加最大力以确保握持稳固
        
        # 分步夹紧参数
        self.grasp_steps = 3  # 改为3步，减少等待时间
        self.grasp_step_delay = 0.8 # 缩短每步间隔（0.8秒足够物理稳定），提高响应速度
        self.grasp_in_progress = False
        self.grasp_step_lock = threading.Lock()
        
        # 创建Action客户端（双指驱动由底层 ROS Control 自动处理，此处只要发给 Controller 即可）
        self.left_gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/left_gripper_controller/gripper_cmd'
        )
        
        self.right_gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/right_gripper_controller/gripper_cmd'
        )
        
        self.callback_group = ReentrantCallbackGroup()
        self.gripper_service = self.create_service(
            SetBool, 
            'control_gripper', 
            self.control_gripper_callback,
            callback_group=self.callback_group
        )
        
        # 订阅关节状态（读取 joint1 即可代表夹爪开合程度，因为 joint2 由控制器同步）
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        self.current_left_gripper_position = None
        self.current_right_gripper_position = None
        self.get_logger().info('夹爪控制模块已启动（参数已适配 4cm 香蕉抓取）')
    def joint_state_callback(self, msg):
        """获取夹爪当前位置"""
        try:
            index = msg.name.index('openarm_left_finger_joint1')
            self.current_left_gripper_position = msg.position[index]
        except ValueError:
            pass
            
        try:
            index = msg.name.index('openarm_right_finger_joint1')
            self.current_right_gripper_position = msg.position[index]
        except ValueError:
            pass
    
    def control_gripper_callback(self, request, response):
        """处理夹爪控制服务请求"""
        action = "抓取" if request.data else "打开"
        self.get_logger().info(f'收到夹爪控制请求: {action}')
        
        if request.data:
            # 抓取逻辑
            with self.grasp_step_lock:
                if self.grasp_in_progress:
                    response.success = False
                    response.message = '分步夹紧正在进行中'
                    return response
                self.grasp_in_progress = True
            
            threading.Thread(
                target=self.grasp_gradually_async,
                args=(self.gripper_grasp_position,),  # 使用修正后的 0.015
                daemon=True
            ).start()
            
            response.success = True
            response.message = '分步夹紧已启动'
        else:
            # 打开逻辑
            with self.grasp_step_lock:
                self.grasp_in_progress = False
            
            # 打开左夹爪（示例中仅控制左手，如需双手可复制逻辑）
            left_result = self.send_gripper_command(self.left_gripper_client, self.gripper_open_position, "左")

            if left_result:
                response.success = True
                response.message = '左夹爪已打开'
            else:
                response.success = False
                response.message = '左夹爪打开失败'
        return response
    
    def grasp_gradually_async(self, target_position):
        """异步分步夹紧"""
        try:
            current = self.current_left_gripper_position
            if current is None:
                current = self.gripper_open_position
                self.get_logger().warn('位置未知，假设从全开开始')
            
            # 确保目标更小（闭合动作）
            if target_position > current:
                self.get_logger().warn('目标位置必须小于当前位置(闭合动作)')
                # 直接执行一次
                self.send_gripper_command(self.left_gripper_client, target_position, "左")
                return
                
            diff = current - target_position
            step_size = diff / self.grasp_steps
            self.get_logger().info(f'执行分步抓取: {current:.3f} -> {target_position:.3f}')
                
            for step in range(1, self.grasp_steps + 1):
                with self.grasp_step_lock:
                    if not self.grasp_in_progress:
                        self.get_logger().info('抓取取消')
                        return
    
                cmd_pos = current - (step_size * step)
                self.send_gripper_command(self.left_gripper_client, cmd_pos, "左")

                if step < self.grasp_steps:
                    import time
                    time.sleep(self.grasp_step_delay)
            
            self.get_logger().info('抓取序列完成')
        except Exception as e:
            self.get_logger().error(f'抓取异常: {e}')
        finally:
            with self.grasp_step_lock:
                self.grasp_in_progress = False
    
    def send_gripper_command(self, client, position, arm_side):
        """发送 Action 目标"""
        if not client.server_is_ready():
            self.get_logger().error(f'{arm_side}夹爪控制器未连接')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.gripper_max_effort

        future = client.send_goal_async(goal)

        # 定义轻量级回调，不阻塞
        def check_accept(fut):
            try:
                if fut.result().accepted:
                    self.get_logger().debug(f'{arm_side}指令发送成功: {position:.3f}')
            except:
                pass

        future.add_done_callback(check_accept)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()