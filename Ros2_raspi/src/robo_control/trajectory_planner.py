#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # =============================
        # Paramètres mécaniques réels
        # =============================
        self.L1 = 9.0     # cm
        self.L2 = 12.0    # cm
        self.BASE_OFFSET = 6.0  # cm

        # Home position (servo angles)
        self.home_angles = [90.0, 60.0, 150.0, 90.0]

        # Drop position
        self.drop_x = 10.0
        self.drop_y = 5.0

        # File de commandes
        self.command_queue = deque()

        # Publisher vers les servos
        self.servo_pub = self.create_publisher(String, '/servo_cmd', 10)

        # Abonnement position balle
        self.ball_sub = self.create_subscription(
            Point,
            '/ball_pose',
            self.ball_callback,
            10
        )

        # Envoi périodique des commandes
        self.send_period = 5  # secondes
        self.timer = self.create_timer(self.send_period, self.send_next_command)

        self.get_logger().info("TrajectoryPlanner démarré.")

    # =====================================================
    # Cinématique inverse 2D (plan)
    # =====================================================
    def ik_2d(self, x, y):
        # Correction offset base
        x -= self.BASE_OFFSET

        L1, L2 = self.L1, self.L2
        d2 = x * x + y * y
        d = math.sqrt(d2)

        # Saturation workspace
        if d > (L1 + L2):
            scale = (L1 + L2) / d
            x *= scale
            y *= scale
            d2 = x * x + y * y

        # Angle coude
        c2 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        c2 = max(-1.0, min(1.0, c2))
        s2 = math.sqrt(1 - c2 * c2)

        theta2 = math.atan2(s2, c2)

        # Angle épaule
        k1 = L1 + L2 * c2
        k2 = L2 * s2
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        # Orientation poignet
        theta3 = -(theta1 + theta2)

        return theta1, theta2, theta3

    # =====================================================
    # Conversion math → servo (RESPECT CONTRAINTES)
    # =====================================================
    def math_to_servo_angles(self, t1, t2, t3):
        # Épaule
        s1 = 90.0 - self.rad_to_deg(t1)
        s1 = max(0.0, min(180.0, s1))

        # Coude (sens inverse, plage [30,100])
        s2 = 90.0 + self.rad_to_deg(t2)
        s2 = max(30.0, min(100.0, s2))

        # Poignet (plage [130,180])
        s3 = 180.0 + self.rad_to_deg(t3)
        s3 = max(130.0, min(180.0, s3))

        return s1, s2, s3

    def rad_to_deg(self, r):
        return r * 180.0 / math.pi

    # =====================================================
    # Génération commandes servo
    # =====================================================
    def make_command_for_pose(self, angles):
        cmds = []
        for i, a in enumerate(angles, start=1):
            cmds.append(f"{i}:{int(a)}")
        return cmds

    # =====================================================
    # Callback balle détectée
    # =====================================================
    def ball_callback(self, msg: Point):
        x, y = msg.x, msg.y
        self.get_logger().info(f"Balle détectée : x={x:.2f}, y={y:.2f}")

        # HOME
        home_cmds = self.make_command_for_pose(self.home_angles)

        # PICK
        t1, t2, t3 = self.ik_2d(x, y)
        s1, s2, s3 = self.math_to_servo_angles(t1, t2, t3)
        pick_cmds = self.make_command_for_pose([s1, s2, s3, 90.0])

        # Fermer pince
        close_gripper = ["4:30"]

        # DROP
        t1d, t2d, t3d = self.ik_2d(self.drop_x, self.drop_y)
        sd1, sd2, sd3 = self.math_to_servo_angles(t1d, t2d, t3d)
        drop_cmds = self.make_command_for_pose([sd1, sd2, sd3, 30.0])

        # Ouvrir pince
        open_gripper = ["4:90"]

        # Retour HOME
        return_home = self.make_command_for_pose(self.home_angles)

        # Charger la file
        self.command_queue.clear()
        for seq in [
            home_cmds,
            pick_cmds,
            close_gripper,
            drop_cmds,
            open_gripper,
            return_home,
        ]:
            for c in seq:
                self.command_queue.append(c)

        self.get_logger().info(
            f"Séquence générée ({len(self.command_queue)} commandes)."
        )

    # =====================================================
    # Envoi périodique
    # =====================================================
    def send_next_command(self):
        if not self.command_queue:
            return

        cmd = self.command_queue.popleft()
        msg = String()
        msg.data = cmd
        self.servo_pub.publish(msg)
        self.get_logger().info(f"Commande envoyée : {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
