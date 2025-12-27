import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.sub = self.create_subscription(
            String,
            '/servo_cmd',
            self.callback_servo,
            10
        )

        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Arduino connect.")
        except:
            self.get_logger().error("Impossible de trouver Arduino sur /dev/ttyUSB0")
            exit(1)

    def callback_servo(self, msg):
        self.get_logger().info(f"angle reu : {msg.data}")

   
        try:
            joint, angle = msg.data.split(":")
            joint = int(joint.strip())
            angle = int(angle.strip())
        except:
            self.get_logger().error("Format invalide, attendu ex: 2:120")
            return

       
        if joint == 2:
            if angle > 110:
                angle = 100

        if joint == 3:
            if angle < 140:
                angle = 140

        
        cmd = f"{joint}:{angle}\n"
        self.arduino.write(cmd.encode())

       
        try:
            if self.arduino.in_waiting > 0:
                response = self.arduino.readline().decode('utf-8').strip()
                if response:
                    self.get_logger().info(f"Rnse Arduino: {response}")
        except Exception as e:
            self.get_logger().error(f"Erreur lecture Arduino: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
