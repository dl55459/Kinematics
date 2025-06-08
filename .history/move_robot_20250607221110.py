#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
import sympy as sp
import math
from math import atan2, cos, sin, pi

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray  # For receiving path arrays

# ... [Keep all your existing functions: direct(), inverse(), DEG2RAD(), mm2m(), activePos()] ...

class prarobClientNode(Node):
    def __init__(self):
        super().__init__('prarob_client_node')

        # Define publisher
        self.robot_goal_publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Define subscriber for path updates
        self.path_subscriber = self.create_subscription(
            Float64MultiArray,
            '/path_coordinates',  # Topic name for receiving paths
            self.path_callback,
            10
        )
        self.get_logger().info("Waiting for path coordinates on /path_coordinates topic...")

        # Initialize variables
        self.current_path = None
        self.DIRECT_MAT = direct(L1z, L3x, L3z, LEx, LEz)
        self.phiD, self.alphaD, self.thetaD = activePos(False)  # Start in idle position

    def path_callback(self, msg):
        """Callback for receiving new path coordinates"""
        try:
            # Convert Float64MultiArray to numpy array and reshape
            raw_data = np.array(msg.data)
            self.current_path = raw_data.reshape(-1, 2)  # Reshape to Nx2 array
            self.get_logger().info(f"Received new path: {self.current_path}")

            # Convert from mm to meters
            self.current_path = mm2m(self.current_path)

            # Execute the new path
            self.execute_path()

        except Exception as e:
            self.get_logger().error(f"Error processing path: {str(e)}")

    def execute_path(self):
        """Execute the current path"""
        if self.current_path is None:
            self.get_logger().warn("No path available to execute")
            return

        for point in self.current_path:
            x, y = point[0], point[1]

            # Workspace checking
            if not (xLowerWS <= x <= xUpperWS) or not (yLowerWS <= y <= yUpperWS):
                self.get_logger().warn(f"Point {x, y} outside workspace - skipping")
                continue

            if (xLowerB <= x <= xUpperB) and (yLowerB <= y <= yUpperB):
                self.get_logger().warn(f"Point {x, y} in inaccessible area - skipping")
                continue

            # Calculate and execute movement
            phiD, thetaD = inverse(self.DIRECT_MAT, LM1M3, LM3EE, x, y)
            self.move_robot([phiD, 0, thetaD], 5)

            # Log the angles
            self.get_logger().info(f"Moving to {x, y} with angles: phi={phiD:.2f}, theta={thetaD:.2f}")

            # Wait for movement to complete (6 seconds as in your original code)
            self.get_clock().sleep_for(Duration(seconds=6.0))

    def move_robot(self, q, durationS):
        """Publish joint trajectory (same as your original)"""
        goal_trajectory = JointTrajectory()
        goal_trajectory.joint_names = ['Brot', 'pitch', 'EErot']

        goal_point = JointTrajectoryPoint()
        goal_point.positions = q
        goal_point.time_from_start = Duration(durationS).to_msg()

        goal_trajectory.points.append(goal_point)
        self.robot_goal_publisher_.publish(goal_trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = prarobClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()