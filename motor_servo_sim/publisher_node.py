import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 #node'lar arasinda iletisim icin

class MotorServoPublisher(Node):
    def __init__(self):
        super().__init__('motor_servo_publisher')
        self.motor_pub = self.create_publisher(Float32, 'motor_speed', 10) #olusturulan topic
        self.servo_pub = self.create_publisher(Float32, 'servo_angle', 10) #olusturulan topic
        self.timer = self.create_timer(1.0, self.timer_callback) #1 saniyede bir yayın yap
    
    def timer_callback(self):
        motor_speed = 100.0  # 100 RPM
        servo_angle = 45.0  # Derece

        self.motor_pub.publish(Float32(data=motor_speed))
        self.servo_pub.publish(Float32(data=servo_angle))
        self.get_logger().info(f'Published motor speed: {motor_speed}, Servo angle: {servo_angle}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorServoPublisher()

    rclpy.spin(node)
    rclpy.destroy_node(node)
    rclpy.shutdown()
    