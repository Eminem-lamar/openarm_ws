import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R  # 必须导入 R

class VisionEngineer(Node):
    def __init__(self):
        super().__init__('vision_engineer_node')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # ==========================================
        # 核心修改：基于物理真值的标定矩阵构建
        # ==========================================
        
        # 1. 填入 tf2_echo 查到的绝对物理坐标 (Base -> Camera Link)
        real_trans = [0.040, 0.000, 0.620]
        real_quat  = [0.654, 0.654, -0.268, 0.269]

        # 2. 构建【底座 -> 相机外壳】的物理变换矩阵
        r_link = R.from_quat(real_quat)
        T_base_link = np.eye(4)
        T_base_link[:3, :3] = r_link.as_matrix()
        T_base_link[:3, 3] = real_trans

        # 3. 构建【相机外壳 -> 光学传感器】的固有变换
        r_optical = R.from_euler('xyz', [-np.pi/2, 0, -np.pi/2])
        T_link_optical = np.eye(4)
        T_link_optical[:3, :3] = r_optical.as_matrix()

        # ==========================================
        # 4. 【新增】轴向修正矩阵 (Axis Correction)
        # ==========================================
        # 现象：当前输出 X≈0, Y≈0.43。说明相机存在90度的偏航角。
        # 对策：绕 Z 轴旋转 -90 度，将 Y 轴数据转回 X 轴。
        r_fix = R.from_euler('z', -90, degrees=True)
        T_fix = np.eye(4)
        T_fix[:3, :3] = r_fix.as_matrix()

        # 5. 级联计算最终标定矩阵
        # 【关键修改】：把 T_fix 放到最左边！
        # 含义：先修正基座方向，再叠加物理变换
        self.T_base_cam = T_fix @ T_base_link @ T_link_optical

        self.get_logger().info(f"已加载物理真值标定矩阵 (左乘修正):\n{self.T_base_cam}")
        # ==========================================

        # 下面订阅代码保持不变...
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        
        self.K = None
        self.depth_img = None
        self.get_logger().info('视觉大脑已启动，已加载物理溯源标定矩阵。')
        self.get_logger().info(f"最终标定矩阵 T_base_cam:\n{self.T_base_cam}")

    def info_cb(self, msg):
        # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.K = msg.k 

    def depth_cb(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def color_cb(self, msg):
        if self.K is None or self.depth_img is None:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 识别黄色香蕉
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                u = int(M["m10"] / M["m00"])
                v = int(M["m01"] / M["m00"])
                
                # 获取深度值 (Z)
                z = float(self.depth_img[v, u])
                if z > 0:
                    self.process_localization(u, v, z)
        else:
            self.get_logger().info('搜索目标中...', throttle_duration_sec=3.0)

    def process_localization(self, u, v, z):
        # 像素 -> 相机空间
        fx, cx = self.K[0], self.K[2]
        fy, cy = self.K[4], self.K[5]
        
        xc = (u - cx) * z / fx
        yc = (v - cy) * z / fy
        zc = z
        
        # 核心转换：P_base = T_base_cam * P_cam
        P_cam = np.array([xc, yc, zc, 1.0])
        P_base = self.T_base_cam @ P_cam
        
        # 发布 TF 坐标给成员 3 使用
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'openarm_body_link0'
        t.child_frame_id = 'banana_target'
        
        t.transform.translation.x = P_base[0]
        t.transform.translation.y = P_base[1]
        t.transform.translation.z = P_base[2]
        t.transform.rotation.w = 1.0 
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'定位成功! 基座坐标: x={P_base[0]:.3f}, y={P_base[1]:.3f}, z={P_base[2]:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionEngineer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
