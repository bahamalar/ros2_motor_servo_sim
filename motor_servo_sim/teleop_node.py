import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.motor_publisher = self.create_publisher(Float32, 'motor_speed', 10)
        self.servo_publisher = self.create_publisher(Float32, 'servo_angle', 10)
        
        self.motor_speed = 0.0  # Başlangıç motor hızı
        self.servo_angle = 90.0  # Başlangıç servo açısı (ortalama)

        # Motor hız ve servo açı sınırları
        self.motor_speed_min = -100.0  # Motor minimum hızı
        self.motor_speed_max = 100.0   # Motor maksimum hızı
        self.servo_angle_min = 0.0    # Servo minimum açısı
        self.servo_angle_max = 180.0  # Servo maksimum açısı

    def get_key(self):
        """Klavye girişlerini almak için terminal modunu düzenler."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def limit_value(self, value, min_value, max_value):
        """Değerleri belirtilen sınırda tutar."""
        return max(min(value, max_value), min_value)
    

    def run(self):
        self.get_logger().info("Motor ve Servo Kontrolü Başladı")
        self.get_logger().info("W/S: Motor Hızını Artır/Azalt, A/D: Servo Açısını Sola/Sağa Döndür, Q: Çıkış")
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':  # Motor hızını artır
                self.motor_speed += 10.0
            elif key == 's':  # Motor hızını azalt
                self.motor_speed -= 10.0
            elif key == 'a':  # Servo açısını sola çevir
                self.servo_angle -= 5.0
            elif key == 'd':  # Servo açısını sağa çevir
                self.servo_angle += 5.0
            elif key == 'q':  # Çıkış
                self.get_logger().info("Kontrol sonlandırılıyor...")
                break

            # Hız ve açı sınırlarını uygula
            self.motor_speed = self.limit_value(self.motor_speed, self.motor_speed_min, self.motor_speed_max)
            self.servo_angle = self.limit_value(self.servo_angle, self.servo_angle_min, self.servo_angle_max)

            # Mesajları yayınla
            motor_msg = Float32()
            motor_msg.data = self.motor_speed
            self.motor_publisher.publish(motor_msg)
            self.get_logger().info(f"Motor Hızı: {self.motor_speed}")

            servo_msg = Float32()
            servo_msg.data = self.servo_angle
            self.servo_publisher.publish(servo_msg)
            self.get_logger().info(f"Servo Açısı: {self.servo_angle}")

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()
