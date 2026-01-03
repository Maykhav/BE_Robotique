import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/ball_pose', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Exemple : la balle se déplace en cercle autour de la base
        msg.pose.position.x = 0.1 * math.cos(self.angle)
        msg.pose.position.y = 0.1 * math.sin(self.angle)
        msg.pose.position.z = 0.02
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.angle += 0.05
        self.get_logger().info(f'Balle à x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
