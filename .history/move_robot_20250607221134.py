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

    # xi(xi-1)
    # yi(yi-1)
    # zi(zi-1)

    # [xi(xi-1), xi(yi-1), xi(zi-1)]
    # [yi(xi-1), yi(yi-1), yi(zi-1)]
    # [zi(xi-1), zi(yi-1), zi(zi-1)]

    # L1z z-component of distance between base and motor 1
    # L3x x-component of distance between motor 1 and motor 3
    # L3z z-component of distance between motor 1 and motor 3
    # LEx x-component of distance between motor 3 and end effector
    # LEz z-component of distance between motor 3 and end effector

    # phi    angle of motor 1
    # theta  angle of motor 3

"""
^                                        8888b.  88 88""Yb 888888  dP""b8 888888
^                                         8I  Yb 88 88__dP 88__   dP   `"   88
^                                         8I  dY 88 88"Yb  88""   Yb        88
^                                        8888Y"  88 88  Yb 888888  YboodP   88
"""
# Function for calculating transformation matrix from base to end effector
def direct(L1z, L3x, L3z, LEx, LEz):
    phi, theta = sp.symbols("phi theta", real = True) # Angles

    # Translation matrix from base to motor 1
    BM1_T = sp.Matrix([
        [0],
        [0],
        [L1z] # 41.9
    ])

    # Rotation matrix from base to motor 1
    BM1_R = sp.Matrix([
        [sp.cos(phi), -sp.sin(phi), 0],
        [sp.sin(phi),  sp.cos(phi), 0],
        [          0,            0, 1]
    ])

    # Translation matrix from motor 1 to motor3
    M1M3_T = sp.Matrix([
        [L3x], # 190
        [0],
        [L3z] # -0.55
    ])

    # Rotation matrix from motor 1 to motor3
    M1M3_R = sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0],
        [sp.sin(theta),  sp.cos(theta), 0],
        [          0,                0, 1]
    ])

    # Translation matrix from motor3 to end effector
    M3EE_T = sp.Matrix([
        [LEx], # 189
        [0],
        [LEz] # 39.35
    ])

    # Rotaion matrix from motor3 to end effector
    M3EE_R = sp.Matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    # Expanded matrix for homogeneous coordinates
    Expanded = sp.Matrix([
        [0, 0, 0, 1]
    ])

    # Create the transformation matrices for each segment
    TR = BM1_R.row_join(BM1_T)
    T_BM1 = TR.col_join(Expanded)
    TR = M1M3_R.row_join(M1M3_T)
    T_M1M3 = TR.col_join(Expanded)
    TR = M3EE_R.row_join(M3EE_T)
    T_M3EE = TR.col_join(Expanded)

    # Combine the transformation matrices to get the full transformation from base to end effector
    T_BEE = T_BM1 * T_M1M3 * T_M3EE # Calculate transformation matrix from base to end effector
    # T_BEE_subs = T_BEE.subs(subsL) # Substitute link lengths

    return T_BEE

"""
^                                        88 88b 88 Yb    dP 888888 88""Yb .dP"Y8 888888
^                                        88 88Yb88  Yb  dP  88__   88__dP `Ybo." 88__
^                                        88 88 Y88   YbdP   88""   88"Yb  o.`Y8b 88""
^                                        88 88  Y8    YP    888888 88  Yb 8bodP' 888888
"""
def inverse(T_BEE, LM1M3, LM3EE, xDesired, yDesired):
    # Extract x, y, z from T_BEE
    # xd, yd = sp.symbols("xd yd", real = True) # Desired destination in x, y directions
    #subsd = {xDesired:1, yDesired:1} # input desired x, y location

    # extract x, y, z components form T_BEE matrix
    x, y, z = T_BEE[:3, 3]

    # Set equations for x, y calculation
    eqx = sp.Eq(x, xDesired)
    eqy = sp.Eq(y, yDesired)

    # Display results
    sp.pprint(eqx)
    sp.pprint(eqy)

    # Calculate the distance form base to end effector
    r_sqr = xDesired**2 + yDesired**2
    # r = sp.sqrt(r_sqr)

    # Calculate the angle theta which is the angle of second motor
    cos_thera = (LM1M3**2 + LM3EE**2 - r_sqr) / (2 * LM1M3 * LM3EE)
    thetaDesired = sp.acos(cos_thera) # Returns +-

    # Calculate the angle phi which is the angle of first motor
    LM1M3 = sp.Integer(LM1M3)  # Convert to sympy Integer for better precision
    LM3EE = sp.Integer(LM3EE)  # Convert to sympy Integer for better precision
    beta = math.atan2(yDesired, xDesired)
    gamma = math.atan2(LM3EE * sp.sin(thetaDesired), LM1M3 + LM3EE * sp.cos(thetaDesired))
    phiDesired = beta - gamma

    return phiDesired, thetaDesired

def DEG2RAD(degree):

    rad = degree * (np.pi/180)
    return rad

def mm2m(mm):

    m = mm/1000
    return m

# Function sets default positions for active and in-active operation mode
def activePos(flag):
    # Check for activity flag
    if flag == 1:
        phiActive = DEG2RAD(0)
        alphaActive = DEG2RAD(0) # Active angle for motor 2 (pitch)
        thetaActive = DEG2RAD(0)
    else:
        phiActive = DEG2RAD(180) # Idle angle for motor 1
        alphaActive = DEG2RAD(270) # Idle angle for motor 2
        thetaActive = DEG2RAD(195) # Idle angle for motor 3

    return phiActive, alphaActive, thetaActive

# Function sends motor positions
def topicOut(x, y, z, phiDesired, thetaDesired):
    # No implementation yet
    #TODO: convert rad to motor readable angle

    return 0

# Function receives motor positions
def topicIn():
    # No implementation yet
    #TODO: convert motor angle to rad

    return 0

L1z = mm2m(41.9) # z-component of distance between base and motor 1
L3x = mm2m(190) # x-component of distance between motor 1 and motor 3
L3z = mm2m(-0.55) # z-component of distance between motor 1 and motor 3
LEx = mm2m(189) # x-component of distance between motor 3 and end effector
LEz = mm2m(39.35) # z-component of distance between motor 3 and end effector

LM1M3 = mm2m(190) # x-component of distance between motor 1 and motor 3
LM3EE = mm2m(189) # x-component of distance between motor 3 and end effector

# Working area limits
xLowerWS = mm2m(-25.487)
xUpperWS = mm2m(324.513)
xLowerB = mm2m(25.487)
xUpperB = mm2m(124.513)
yLowerWS = mm2m(-175)
yUpperWS = mm2m(175)
yLowerB = mm2m(-75)
yUpperB = mm2m(75)

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