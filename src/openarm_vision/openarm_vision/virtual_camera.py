import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from cv_bridge import CvBridge

class VirtualCamera(Node):
    def __init__(self):
        super().__init__('virtual_camera')
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        # 1. 发布彩色图 (带黄色方块)
        img = np.zeros((480, 640, 3), np.uint8)
        img[200:300, 300:400] = [0, 255, 255] # 画个黄香蕉
        self.color_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8', header=None))
        
        # 2. 发布伪深度图 (假设物体在 0.6 米处)
        depth_img = np.full((480, 640), 0.6, dtype=np.float32)
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_img, '32FC1'))
        
        # 3. 发布相机内参 (必须有这个，大脑才能算 3D 坐标)
        info = CameraInfo()
        info.header.stamp = now
        info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0] # 虚拟内参
        self.info_pub.publish(info)
        self.get_logger().info('全套传感器数据（彩色+深度+内参）已发布')
def main(args=None):
    rclpy.init(args=args)
    node = VirtualCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
