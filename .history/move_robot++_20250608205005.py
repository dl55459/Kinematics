#!/usr/bin/env python3

from asyncio import PidfdChildWatcher
from matplotlib.transforms import LockableBbox
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
import time
import numpy as np
import sympy as sp
import math as math
from math import atan2, cos, sin, pi

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
def inverse(LM1M3, LM3EE, xDesired, yDesired):

    r_sqr = xDesired**2 + yDesired**2
    cos_theta = (LM1M3**2 + LM3EE**2 - r_sqr) / (2 * LM1M3 * LM3EE)
    thetaDesired = sp.acos(cos_theta)

    beta  = math.atan2(yDesired, xDesired)
    gamma = math.atan2(LM3EE * sp.sin(thetaDesired),
                       LM1M3 + LM3EE * sp.cos(thetaDesired))
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
        alphaActive = DEG2RAD(-90) # Idle angle for motor 2
        thetaActive = DEG2RAD(165) # Idle angle for motor 3

    return phiActive, alphaActive, thetaActive

# When sending commands to Brot joint:
def BrotOffset(thetaD):
    # Convert desired angle to motor command with 180° offset
    motor_zero = 180  # Physical 0° is at motor's 180°
    return (thetaD + motor_zero) % 360

L1z = mm2m(41.9) # z-component of distance between base and motor 1
L3x = mm2m(190) # x-component of distance between motor 1 and motor 3
L3z = mm2m(-0.55) # z-component of distance between motor 1 and motor 3
LEx = mm2m(189) # x-component of distance between motor 3 and end effector
LEz = mm2m(39.35) # z-component of distance between motor 3 and end effector

LM1M3 = mm2m(190) # x-component of distance between motor 1 and motor 3
LM3EE = mm2m(189) # x-component of distance between motor 3 and end effector

# Working area limits
# WS = Workspace
# B = Base
xLowerWS = mm2m(0) # -25.487)
xUpperWS = mm2m(350) # 324.513)
xLowerB = mm2m(100) # 25.487)
xUpperB = mm2m(275) # 124.513)
yLowerWS = mm2m(0) # -175)
yUpperWS = mm2m(350) # 175)
yLowerB = mm2m(200) # -75)
yUpperB = mm2m(350) # 75)

def R2B(xR, yR):

    xB = -xR + 324.513
    yB = -yR + 175
    return xB, yB

# Flags
finishedFlag = False

class prarobClientNode(Node):
    def __init__(self):
        super().__init__('prarob_client_node')

        # Define publishers and subscribers
        self.moveJoints_sub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory',
        10)
        self.path_sub = self.create_listener(Path, '/pathfinder/path', self.autoMove, 10)
# -----------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------
    def autoMove(self):

        Brot, Pitch, EErot = activePos(False) # Set arm to idle position (All positions taken to account)
        self.move_robot([phiD, alphaD, thetaD], durationS = 1.5)

        finishedFlag = False

        Brot, Pitch, EErot = activePos(False)
        self.move_robot([Brot, Pitch, BrotOffset(EErot)], durationS = 1.5)

        for i in Path:
            if (xLowerWS <= Path.x <= xUpperWS) and (yLowerWS <= Path.y <= yUpperWS):
                print("Inside work area")
                if (xLowerB <= Path.x <= xUpperB) and (yLowerB <= Path.y <= yUpperB):
                    print("Unacessible area")
                else:
                    print("Accessible area")
                    Brot, EErot = inverse(LM1M3, LM3EE, R2B(Path.x), R2B(Path.y))
                    self.move_robot([Brot, Pitch, BrotOffset(EErot)], durationS = 1.5)

                    if i == 1: # After first coordinate loaded set arm to active position
                        phiD, _, thetaD = activePos(True)
                        self.move_robot([Brot, Pitch, BrotOffset(EErot)], durationS = 1.5)
                        #self.get_clock().sleep_for(Duration(seconds = 6.0))
            else:
                print("Outside working area")

        finishedFlag = True

    def move_robot(self, q, durationS):

        goal_trajectory = JointTrajectory()
        goal_trajectory.joint_names.append('Brot')
        goal_trajectory.joint_names.append('pitch')
        goal_trajectory.joint_names.append('EErot')

        goal_point = JointTrajectoryPoint()
        goal_point.positions.append(q)
        goal_point.time_from_start = Duration(durationS).to_msg()

        goal_trajectory.points.append(goal_point)

        return self.robot_goal_publisher_.publish(goal_trajectory)

    def callback(mess):
        x = mess.x


def main(args=None):
    rclpy.init(args=args)
    node = prarobClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()