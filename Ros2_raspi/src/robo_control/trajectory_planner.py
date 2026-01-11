#!/usr/bin/env python3
import math
import time
import re
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def rad2deg(r):
    return r * 180.0 / math.pi


# Accept "s3:110" or "(s3:110)"
ACK_RE = re.compile(r"\(?s(?P<sid>\d+):(?P<ang>\d+)\)?")


class TrajectoryPlanner(Node):
    """
    - ONE servo per message: "id:angle"
    - Send next command ONLY after receiving /servo_ack from Arduino
    - Enforce minimum dt_min = 2s between sends (optional safety)
    """

    def __init__(self):
        super().__init__("trajectory_planner_ack")

        # =============================
        # Geometry (cm)
        # =============================
        self.L1 = 7.0
        self.L2 = 11.0
        self.L3 = 13.0
        self.BASE_X_OFFSET = 0.0

        # =============================
        # Servo limits
        # =============================
        self.SHOULDER_MIN = 130.0
        self.SHOULDER_MAX = 180.0
        self.ELBOW_MIN = 30.0
        self.ELBOW_MAX = 100.0

        # =============================
        # Home / gripper
        # =============================
        self.base_home = 100.0
        self.shoulder_home = 140.0
        self.elbow_home = 90.0
        self.gripper_open = 90.0
        self.gripper_close = 160.0

        self.home_open = [self.base_home, self.shoulder_home, self.elbow_home, self.gripper_open]
        self.home_hold = [self.base_home, self.shoulder_home, self.elbow_home, self.gripper_close]

        # Drop target
        self.drop_x = 10.0
        self.drop_y = 5.0

        # Approach
        self.approach_dy = 3.0

        # Rotate base +90
        self.base_place = clamp(self.base_home + 90.0, 0.0, 180.0)

        # =============================
        # ACK / timing
        # =============================
        self.dt_min = 2.0                 # minimum time between 2 sends
        self.last_send_time = 0.0
        self.waiting_ack = False
        self.current_cmd = None           # last sent cmd e.g. "3:110"

        # Queue of single-servo commands
        self.command_queue = deque()

        # For IK branch scoring
        self.current_pose_for_scoring = self.home_open[:]

        # =============================
        # ROS interfaces
        # =============================
        self.servo_pub = self.create_publisher(String, "/servo_cmd", 10)
        self.ack_sub = self.create_subscription(String, "/servo_ack", self.on_ack, 10)
        self.ball_sub = self.create_subscription(Point, "/ball_pose", self.ball_callback, 10)

        # Tick timer: re-try sending when allowed (dt_min) and not waiting ack
        self.tick = self.create_timer(0.05, self.try_send_next)

        # =============================
        # Calibration
        # =============================
        self.calib_x = 13.0
        self.calib_y = 2.0
        self.calib_shoulder_servo = 170.0
        self.calib_elbow_servo = 60.0
        self.a2, self.b3 = self.compute_servo_offsets_from_calibration()

        self.get_logger().info(
            f"Started ACK planner. dt_min={self.dt_min}s. a2={self.a2:.2f}, b3={self.b3:.2f}"
        )

    # =====================================================
    # IK candidates
    # =====================================================
    def ik_candidates(self, x_base, y_base):
        xs = x_base - self.BASE_X_OFFSET
        ys = y_base - self.L1

        d2 = xs * xs + ys * ys
        d = math.sqrt(d2)

        dmax = self.L2 + self.L3
        if d > dmax:
            scale = dmax / d
            xs *= scale
            ys *= scale
            d2 = xs * xs + ys * ys

        cos_phi = (d2 - self.L2 * self.L2 - self.L3 * self.L3) / (2.0 * self.L2 * self.L3)
        cos_phi = clamp(cos_phi, -1.0, 1.0)
        phi = math.acos(cos_phi)
        phi_deg = rad2deg(phi)

        alpha = math.atan2(ys, xs)
        beta = math.atan2(self.L3 * math.sin(phi), self.L2 + self.L3 * math.cos(phi))

        theta_sh_1 = rad2deg(alpha - beta)
        theta_sh_2 = rad2deg(alpha + beta)

        return (theta_sh_1, phi_deg), (theta_sh_2, phi_deg)

    # =====================================================
    # Mapping math -> servos
    # =====================================================
    def math_to_servos(self, theta_sh_deg, phi_deg):
        shoulder = self.a2 - theta_sh_deg
        elbow = (180.0 - phi_deg) + self.b3
        shoulder = clamp(shoulder, self.SHOULDER_MIN, self.SHOULDER_MAX)
        elbow = clamp(elbow, self.ELBOW_MIN, self.ELBOW_MAX)
        return shoulder, elbow

    # =====================================================
    # Choose best IK branch
    # =====================================================
    def solve_servos_for_xy(self, x, y):
        c1, c2 = self.ik_candidates(x, y)
        s1 = self.math_to_servos(*c1)
        s2 = self.math_to_servos(*c2)

        cur_sh = self.current_pose_for_scoring[1]
        cur_el = self.current_pose_for_scoring[2]

        def score(sol):
            sh, el = sol
            penalty = 0.0
            if sh in (self.SHOULDER_MIN, self.SHOULDER_MAX):
                penalty += 50.0
            if el in (self.ELBOW_MIN, self.ELBOW_MAX):
                penalty += 50.0
            return abs(sh - cur_sh) + abs(el - cur_el) + penalty

        return s2 if score(s2) < score(s1) else s1

    # =====================================================
    # Calibration offsets
    # =====================================================
    def compute_servo_offsets_from_calibration(self):
        c1, c2 = self.ik_candidates(self.calib_x, self.calib_y)
        for (theta_sh_deg, phi_deg) in [c1, c2]:
            b3 = self.calib_elbow_servo - (180.0 - phi_deg)
            a2 = self.calib_shoulder_servo + theta_sh_deg
            if 150.0 <= a2 <= 240.0:
                return a2, b3

        theta_sh_deg, phi_deg = c1
        b3 = self.calib_elbow_servo - (180.0 - phi_deg)
        a2 = self.calib_shoulder_servo + theta_sh_deg
        return a2, b3

    # =====================================================
    # Queue helpers
    # =====================================================
    def enqueue_pose_as_single_commands(self, pose4):
        for sid, ang in enumerate(pose4, start=1):
            self.command_queue.append(f"{sid}:{int(round(ang))}")
        self.current_pose_for_scoring = pose4[:]

    # =====================================================
    # Build sequence on ball detection
    # =====================================================
    def ball_callback(self, msg: Point):
        x, y = msg.x, msg.y
        self.get_logger().info(f"Ball detected: x={x:.2f}, y={y:.2f}")

        # If executing already, ignore
        if self.command_queue or self.waiting_ack:
            self.get_logger().warn("Busy executing. Ignoring new ball pose.")
            return

        self.current_pose_for_scoring = self.home_open[:]

        poses = []
        poses.append(self.home_open[:])

        sh, el = self.solve_servos_for_xy(x, y + self.approach_dy)
        poses.append([self.base_home, sh, el, self.gripper_open])

        sh, el = self.solve_servos_for_xy(x, y)
        poses.append([self.base_home, sh, el, self.gripper_open])

        poses.append([self.base_home, sh, el, self.gripper_close])

        sh, el = self.solve_servos_for_xy(x, y + self.approach_dy)
        poses.append([self.base_home, sh, el, self.gripper_close])

        poses.append(self.home_hold[:])

        poses.append([self.base_place, self.shoulder_home, self.elbow_home, self.gripper_close])

        shd, eld = self.solve_servos_for_xy(self.drop_x, self.drop_y)
        poses.append([self.base_place, shd, eld, self.gripper_close])

        poses.append([self.base_place, shd, eld, self.gripper_open])

        poses.append([self.base_place, self.shoulder_home, self.elbow_home, self.gripper_open])

        poses.append(self.home_open[:])

        self.command_queue.clear()
        self.current_pose_for_scoring = self.home_open[:]
        for p in poses:
            self.enqueue_pose_as_single_commands(p)

        self.get_logger().info(
            f"Queued {len(poses)} poses = {len(self.command_queue)} single-joint commands. "
            f"Will execute with ACK + dt_min={self.dt_min}s."
        )
        # First command will be sent by tick timer

    # =====================================================
    # SEND: only when not waiting ACK + dt_min passed
    # =====================================================
    def try_send_next(self):
        if self.waiting_ack:
            return
        if not self.command_queue:
            return

        now = time.time()
        if now - self.last_send_time < self.dt_min:
            return

        cmd = self.command_queue.popleft()  # "id:angle"
        self.current_cmd = cmd
        self.waiting_ack = True
        self.last_send_time = now

        self.servo_pub.publish(String(data=cmd))
        self.get_logger().info(f"Sent: {cmd} (waiting ack)")

    # =====================================================
    # ACK callback from servo_node (Arduino)
    # Accept "s3:110" or "(s3:110)"
    # IMPORTANT: Arduino may clamp angle, so we match joint id.
    # =====================================================
    def on_ack(self, msg: String):
        txt = msg.data.strip()
        m = ACK_RE.search(txt)
        if not m:
            self.get_logger().warn(f"Bad ACK format: {txt}")
            return

        sid = int(m.group("sid"))
        ang = int(m.group("ang"))
        ack_cmd = f"{sid}:{ang}"

        if not self.waiting_ack:
            self.get_logger().warn(f"ACK received but not waiting: {txt}")
            return

        sent_sid = int(self.current_cmd.split(":")[0])

        # Accept if same joint id (angle may differ due to Arduino clamp)
        if sid != sent_sid:
            self.get_logger().warn(f"ACK joint mismatch: got {ack_cmd}, expected joint {sent_sid}")
            return

        self.get_logger().info(f"ACK OK: {txt} (sent was {self.current_cmd})")
        self.waiting_ack = False
        self.current_cmd = None
        # next send will happen via tick timer


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
