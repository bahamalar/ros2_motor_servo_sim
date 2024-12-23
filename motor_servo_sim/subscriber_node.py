import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 #node'lar arasinda iletisim icin

class MotorServoSubscriber(Node):
    def __init__(self):
        super().__init__("motor_servo_subscriber")
        self.motor_sub = self.create_subscription(Float32, 'motor_speed', self.motor_callback, 10)
        self.servo_sub = self.create_subscription(Float32, 'servo_angle', self.servo_callback, 10)
    
    def motor_callback(self, msg):
        self.get_logger().info(f'Received motor speed: {msg.data}')
    
    def servo_callback(self, msg):
        self.get_logger().info(f'Received servo angle: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    node = MotorServoSubscriber()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()