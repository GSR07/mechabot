import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.subscription = self.create_subscription(Imu, '/imu/out', self.imu_callback, 10)
    
    def imu_callback(self, msg):
        #extract quaternion
        x=msg.orientation.x
        y=msg.orientation.y
        z=msg.orientation.z
        w=msg.orientation.w
        
        #convert to yaw
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        yaw_deg = math.degrees(yaw)
        
        #get yaw rate from angular velocity
        yaw_rate = msg.angular_velocity.z
        yaw_rate_deg = math.degrees(yaw_rate)
        
        print(f"Yaw: {yaw_deg:.2f}° | Yaw Rate: {yaw_rate_deg:.2f}°/s")
        
def main(args=None):
    rclpy.init(args=args)
    node = ImuReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()