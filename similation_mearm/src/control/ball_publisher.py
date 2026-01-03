import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')

        # Publishers / Subscribers
        self.pose_pub = self.create_publisher(Pose, '/ball_pose', 10)
        self.subscription = self.create_subscription(Bool, '/grip_state', self.grip_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Ball state
        self.ball_pose = Pose()
        self.ball_pose.position.x = 0.1
        self.ball_pose.position.y = 0.0
        self.ball_pose.position.z = 0.05
        self.attached = False  # si la balle est "prise"

        self.get_logger().info(f'Balle initialisée à ({self.ball_pose.position.x}, {self.ball_pose.position.y}, {self.ball_pose.position.z})')

    def grip_callback(self, msg):
        """Réception de l’état de la pince"""
        self.attached = msg.data
        if self.attached:
            self.get_logger().info("🎯 La balle est maintenant attachée à la pince du robot !")
        else:
            self.get_logger().info("⚪ La balle est libérée.")

    def timer_callback(self):
        if self.attached:
            # On simule que la balle suit la pince (par exemple position fixe proche du bout du bras)
            # Dans un vrai système, on lirait la position de la pince.
            self.ball_pose.position.x = 0.15
            self.ball_pose.position.y = 0.0
            self.ball_pose.position.z = 0.12
        self.pose_pub.publish(self.ball_pose)

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
