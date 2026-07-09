import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        print(cv2.__version__)
    
    def image_callback(self, msg):
        #convert ros2 img to opencv img
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #display image
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()