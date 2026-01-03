#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class MarkerBallPublisher(Node):
    def __init__(self):
        super().__init__('marker_ball_publisher')
        self.publisher_ = self.create_publisher(Marker, 'ball_marker', 10)
        self.subscription = self.create_subscription(Bool, '/grip_state', self.grip_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Ball marker publisher started ')
        self.attached = False
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ball"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position de la balle
        marker.pose.position.x = 0.15
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.01

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # Taille et couleur
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0  # Alpha = opacité (1.0 = opaque)
        self.publisher_.publish(marker)

    def grip_callback(self, msg):
        """Réception de l’état de la pince"""
        self.attached = msg.data
        if self.attached:
            self.get_logger().info("La balle est maintenant attachée à la pince du robot !")
        else:
            self.get_logger().info(" La balle est libérée.")

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ball"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
       # Position de la balle
        marker.pose.position.x = 0.15
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.001

        # Taille et couleur
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0  # Alpha = opacité (1.0 = opaque)
        if self.attached:
            # On simule que la balle suit la pince (par exemple position fixe proche du bout du bras)
            # Dans un vrai système, on lirait la position de la pince.
           # Position de la balle
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.15
            marker.pose.position.z = 0.02
            

        
        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerBallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
