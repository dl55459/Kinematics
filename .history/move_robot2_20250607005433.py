#!/usr/bin/env python3
"""
PRAROB client node
– FK/IK helper merged with trajectory publisher
– interactive terminal input for now
– future: accept geometry_msgs/Point on /desired_point
"""

import math
import random
import rclpy
from rclpy.node         import Node
from rclpy.duration     import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg   import Point

# ╭─────────────────────────  FK / IK ─────────────────────────╮
DEG = 180.0 / math.pi
EPS = 1e-4

def fk(q1, q3, L1, L2, PZ):
    px = L1 * math.cos(q1) + L2 * math.cos(q1 - q3)
    py = L1 * math.sin(q1) + L2 * math.sin(q1 - q3)
    return px, py, PZ

def ik(px, py, L1, L2):
    r2 = px * px + py * py
    cos_q3 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if abs(cos_q3) > 1.0 + 1e-9:
        return []                # unreachable
    cos_q3 = max(-1.0, min(1.0, cos_q3))
    sols = []
    for q3 in (math.acos(cos_q3), -math.acos(cos_q3)):
        q1 = math.atan2(py, px) + math.atan2(L2 * math.sin(q3),
                                             L1 + L2 * math.cos(q3))
        sols.append((q1, q3))
    return sols
# ╰─────────────────────────────────────────────────────────────╯


class PrarobClientNode(Node):
    def __init__(self):
        super().__init__('prarob_client_node')

        # ---- parameters (ask once) -----------------------------
        self.L1 = float(input("L1  [mm] (default 190): ") or 190.0)
        self.L2 = float(input("L2  [mm] (default 189): ") or 189.0)
        self.PZ = float(input("Pz  [mm] (default -6.5): ") or -6.5)
        print()

        # ---- publishers / subscribers --------------------------
        self.pub_traj = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        # future trajectory-planner listener
        self.create_subscription(Point,
                                 '/desired_point',
                                 self.point_callback,
                                 10)

        self.get_logger().info("PRAROB client ready.  "
                               "Type 'Px Py' in mm (empty line quits):")
        self.user_input_loop()

    # -----------------------------------------------------------
    def user_input_loop(self):
        while rclpy.ok():
            try:
                line = input("Px Py > ").strip()
            except EOFError:
                break
            if not line:
                break
            try:
                px, py = map(float, line.split())
            except ValueError:
                print("  ! please enter two numbers")
                continue

            sols = ik(px, py, self.L1, self.L2)
            if not sols:
                print("  Point unreachable for q2 = 0")
                continue

            # choose first (elbow-down) solution
            q1, q3 = sols[0]
            self.publish_joint_goal(q1, 0.0, q3)

    # -----------------------------------------------------------
    def publish_joint_goal(self, q1, q2, q3):
        jt                = JointTrajectory()
        jt.joint_names    = ['joint1', 'joint2', 'joint3']
        pt                = JointTrajectoryPoint()
        pt.positions      = [q1, q2, q3]
        pt.time_from_start = Duration(seconds=6.0).to_msg()
        jt.points.append(pt)
        self.pub_traj.publish(jt)
        print(f"  → sent [rad]: {q1:+.3f}, {q2:+.3f}, {q3:+.3f}")

    # -----------------------------------------------------------
    def point_callback(self, msg: Point):
        # TODO: replace terminal input once planner publishes here
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PrarobClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()