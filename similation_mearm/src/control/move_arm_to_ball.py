#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import math

l2 = 14
l1= 10
l3 = 1
q0=0.0
q1 =0.0
q2 =0.0
x=0.0
y=0.0
class MoveArmToBall(Node):
    def __init__(self):
        super().__init__('move_arm_to_ball')
        self.get_logger().info(' MoveArmToBall: prêt à bouger !')

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pose_pub = self.create_publisher(Pose, '/pose', 10)
        self.grip_pub = self.create_publisher(Bool, '/grip_state', 10)
        self.publisher_ = self.create_publisher(Marker, 'ball_marker', 10)

        # Subscription
        self.subscription = self.create_subscription(Pose, '/ball_pose', self.ball_callback, 10)

        # Séquences de mouvement
        self.initial_joints = [0.0, 0.0, 0.0, 0.0]
        self.target_joints = [-0.9, 0.0,0.4, 0.670]
        
        self.current_positions = [0.076, -0.908, 1.315, 0.755]

        
        self.current_joint_index = 0
        self.phase = 'init' 
        self.ball_pose = None
        
        self.x=1.15 
        self.y =0.0
        self.z = 0.2 
        self.i=0
        
       

        # Timer (0.5 s)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def ball_callback(self, msg):
        self.ball_pose = msg

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [f'joint_{i}' for i in range(len(self.current_positions))]
        joint_state.position = self.current_positions
        self.joint_pub.publish(joint_state)

    def timer_callback(self):
        if self.ball_pose is None:
            self.get_logger().info("En attente de la position de la balle...")
            return

        if self.phase == 'init':
            self.move_sequence(self.current_positions, next_phase='target')

        elif self.phase == 'target':
            #self.grip_pub.publish(Bool(data=True))
            self.move_sequence(self.target_joints, next_phase='done')

        elif self.phase == 'done':
            self.get_logger().info(" Séquence terminée ! Publication de la position finale.")
            pose_msg = Pose()
            pose_msg.position.x = self.ball_pose.position.x
            pose_msg.position.y = self.ball_pose.position.y
            pose_msg.position.z = self.ball_pose.position.z
            self.pose_pub.publish(pose_msg)
            self.timer.cancel()

    def move_sequence(self, target_positions, next_phase):
        
        if self.current_joint_index < len(target_positions):
            idx = self.current_joint_index
            self.current_positions[idx] = target_positions[idx]
            self.publish_joint_state()

            self.get_logger().info(
                f" {self.phase.upper()} | joint_{idx} → {target_positions[idx]:.3f} rad | "
                f"Positions: {[f'{p:.2f}' for p in self.current_positions]}"
            )
            if self.phase == 'target':
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "ball"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                #orientation 
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
                marker.color.a = 1.0  
                q0 = target_positions[0]
                q1 = target_positions[1]
                q2 = target_positions[2]
                x = (l2 * math.cos(q1) + l3 * math.cos(q1 + q2)) * math.cos(q0)
                y = (l2 * math.cos(q1) + l3 * math.cos(q1 + q2)) * math.sin(q0)
                z = l1 + l2 * math.sin(q1) + l3 * math.sin(q1 + q2)
                marker.pose.position.x = x / 100.0  
                marker.pose.position.y = y / 100.0
                marker.pose.position.z = z / 100.0   
                self.publisher_.publish(marker) 

            self.current_joint_index += 1
            self.x +=1
            
            
        else:
            self.get_logger().info(f" Phase {self.phase.upper()} terminée.")
            self.current_joint_index = 0
            self.phase = next_phase

def main(args=None):
    rclpy.init(args=args)
    node = MoveArmToBall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt demandé par l’utilisateur')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
