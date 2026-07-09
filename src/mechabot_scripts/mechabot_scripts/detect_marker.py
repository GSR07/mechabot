import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class QRDetectionNode(Node):
    def __init__(self):
        super().__init__("qr_detector_node")
        self.bridge = CvBridge()
        self.qr_decoder = cv2.QRCodeDetector()
        self.width = 640
        self.height = 480
        self.sub_camera = self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 10)
        
    def camera_callback(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        
        #detect QR code
        data, points, _=self.qr_decoder.detectAndDecode(frame)
        
        #if qr code found
        if points is not None:
            points = points[0].astype(int)
            
            #draw bounding box
            cv2.polylines(frame,[points], True, (0,255,0), 2)
            
            #get centre
            centre_x = int(points[:,0].mean())
            centre_y = int(points[:,1].mean())
            
            #draw centre circle on QR
            cv2.circle(frame, (centre_x, centre_y), 5, (255,0,0), -1)
            
            #draw centre circle on camera window
            cv2.circle(frame, (int(self.width/2), centre_y),6,(0, 255, 255), 2)
            
            #show QR data
            if data:
                cv2.putText(frame, f'QR: {data}', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                print(f"QR Code Detected: {data} at ({centre_x}, {centre_y})")
                
        cv2.imshow("QR Detection", frame)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = QRDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()