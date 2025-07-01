import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Imu()
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.angular_velocity.x = 0.01
        msg.angular_velocity.y = 0.01
        msg.angular_velocity.z = 0.02
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = -9.81
        self.publisher_.publish(msg)
        self.get_logger().info('Published IMU Data')

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
