import rclpy
from rclpy.node import Node  
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import GripperCommand
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
import threading
import time

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')
        
        # 夹爪状态参数
        self.gripper_open_position = 0.060  
        self.gripper_closed_position = 0.0  
        # 修正后的抓取目标位置
        self.gripper_grasp_position = 0.015  
        self.gripper_max_effort = 80.0  
        
        # 分步夹紧参数
        self.grasp_steps = 3  
        self.grasp_step_delay = 0.8 
        self.grasp_in_progress = False
        self.grasp_step_lock = threading.Lock()
        
        # 创建Action客户端
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
        
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        self.current_left_gripper_position = None
        self.current_right_gripper_position = None
        
        self.get_logger().info('夹爪控制模块已启动')
        
        # 修改点1：启动一个后台线程来预热连接，避免第一次调用服务时还要现连
        threading.Thread(target=self._wait_for_controllers, daemon=True).start()

    def _wait_for_controllers(self):
        """后台等待控制器上线，仅作日志提示"""
        self.get_logger().info('正在连接夹爪控制器...')
        
        # 循环等待直到连接成功
        while not self.left_gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f'左夹爪控制器未就绪... (等待中: {self.left_gripper_client._action_name})')
        self.get_logger().info('左夹爪控制器连接成功')

        while not self.right_gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f'右夹爪控制器未就绪... (等待中: {self.right_gripper_client._action_name})')
        self.get_logger().info('右夹爪控制器连接成功')

    def joint_state_callback(self, msg):
        """获取夹爪当前位置"""
        try:
            if 'openarm_left_finger_joint1' in msg.name:
                index = msg.name.index('openarm_left_finger_joint1')
                self.current_left_gripper_position = msg.position[index]
        except ValueError:
            pass
            
        try:
            if 'openarm_right_finger_joint1' in msg.name:
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
                args=(self.gripper_grasp_position,), 
                daemon=True
            ).start()
            
            response.success = True
            response.message = '分步夹紧已启动'
        else:
            # 打开逻辑
            with self.grasp_step_lock:
                self.grasp_in_progress = False
            
            # 打开左夹爪
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
            
            if target_position > current: # 如果目标比当前大，说明是要张开（这里逻辑主要是为了闭合）
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
                success = self.send_gripper_command(self.left_gripper_client, cmd_pos, "左")
                
                if not success:
                    self.get_logger().error('分步抓取指令发送失败，中止')
                    break

                if step < self.grasp_steps:
                    time.sleep(self.grasp_step_delay)
            
            self.get_logger().info('抓取序列完成')
            
        except Exception as e:
            self.get_logger().error(f'抓取异常: {e}')
        finally:
            with self.grasp_step_lock:
                self.grasp_in_progress = False
    
    def send_gripper_command(self, client, position, arm_side):
        """发送 Action 目标"""
        # 修改：无限等待直到连接成功
        if not client.server_is_ready():
            self.get_logger().info(f'等待{arm_side}夹爪控制器连接...')
            while not client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(f'{arm_side}夹爪控制器未连接，继续等待...')

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.gripper_max_effort

        future = client.send_goal_async(goal)
        
        def check_accept(fut):
            try:
                result = fut.result()
                if result.accepted:
                    self.get_logger().debug(f'{arm_side}指令发送成功: {position:.3f}')
                else:
                    self.get_logger().warn(f'{arm_side}指令被控制器拒绝')
            except Exception as e:
                self.get_logger().error(f'指令回调异常: {e}')
                
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