#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class ServoBridge(Node):
    def __init__(self):
        super().__init__("servo_controller")

        # ROS topics
        self.cmd_sub = self.create_subscription(String, "/servo_cmd", self.on_cmd, 10)
        self.ack_pub = self.create_publisher(String, "/servo_ack", 10)

        # SERIAL config (match your working python test)
        self.port = "/dev/ttyUSB0"
        self.baud = 9600

        # Open serial
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        self.get_logger().info("Arduino connect.")

        # Poll serial frequently (non-blocking style)
        self.timer = self.create_timer(0.02, self.read_serial)

    def on_cmd(self, msg: String):
        cmd = msg.data.strip()
        if not cmd:
            return
        self.ser.write((cmd + "\n").encode("utf-8"))
        self.get_logger().info(f"angle reu : {cmd}")

    def read_serial(self):
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    return
                # Publish ACK/raw response
                self.ack_pub.publish(String(data=line))
                self.get_logger().info(f"arduino -> {line}")
        except Exception as e:
            self.get_logger().error(f"serial read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ServoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
