import sympy as sp
import numpy as np
import math as math
from math import atan2, cos, sin, pi

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

# Function sets default positions for active and in-active operation mode
def activePos(flag):
    # Check for activity flag
    if flag == 1:
        phiActive = 0
        alphaActive = 0 # Active angle for motor 2 (pitch)
        thetaActive = 0
    else:
        phiActive = 0 # Idle angle for motor 1
        alphaActive = pi/2 # Idle angle for motor 2
        thetaActive = 3*pi/2 # Idle angle for motor 3

    return phiActive, alphaActive, thetaActive



# Joint limits in radians
THETA_MIN = math.radians(160)
THETA_MAX = math.radians(195)
PHI_MIN = math.radians(0)
PHI_MAX = math.radians(360)

def inverse_with_limits(T_BEE, LM1M3, LM3EE, xDesired, yDesired):
    # Calculate raw angles
    r_sqr = xDesired**2 + yDesired**2
    cos_theta = (LM1M3**2 + LM3EE**2 - r_sqr) / (2 * LM1M3 * LM3EE)

    # Handle numerical errors that might take cos_theta outside [-1, 1]
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    thetaD = math.acos(cos_theta)

    # Calculate both possible solutions (elbow up/down)
    thetaD_solutions = [thetaD, -thetaD]

    beta = math.atan2(yDesired, xDesired)
    phiD_solutions = []

    for theta in thetaD_solutions:
        gamma = math.atan2(LM3EE * math.sin(theta),
                          LM1M3 + LM3EE * math.cos(theta))
        phiD = beta - gamma
        phiD_solutions.append(phiD)

    # Select solution that respects joint limits
    valid_solutions = []
    for phi, theta in zip(phiD_solutions, thetaD_solutions):
        # Wrap phi to [0, 2π]
        phi_wrapped = phi % (2*math.pi)
        # Check theta limits
        if THETA_MIN <= theta <= THETA_MAX:
            valid_solutions.append((phi_wrapped, theta))

    if not valid_solutions:
        raise ValueError(f"No valid IK solution found for target ({xDesired}, {yDesired}) within joint limits")

    # Select the solution with smallest phi movement (optional)
    return min(valid_solutions, key=lambda x: abs(x[0]))

def move_robot_with_limits(self, x, y):
    try:
        phiD, thetaD = inverse_with_limits(self.DIRECT_MAT, LM1M3, LM3EE, x, y)

        # Convert to degrees for logging
        phi_deg = math.degrees(phiD)
        theta_deg = math.degrees(thetaD)

        self.get_logger().info(
            f"Moving to ({x:.3f}, {y:.3f}) | "
            f"Phi: {phi_deg:.1f}° | "
            f"Theta: {theta_deg:.1f}°"
        )

        # Execute movement
        self.move_robot([phiD, 0, thetaD], 5)
        return True

    except ValueError as e:
        self.get_logger().error(str(e))
        return False




# Main section of the code
phiD, alphaD, thetaD = activePos(False) # Set arm to idle position (All positions taken to account)

L1z = 41.9 # z-component of distance between base and motor 1
L3x = 190 # x-component of distance between motor 1 and motor 3
L3z = -0.55 # z-component of distance between motor 1 and motor 3
LEx = 189 # x-component of distance between motor 3 and end effector
LEz = 39.35 # z-component of distance between motor 3 and end effector
DIRECT_MAT = direct(L1z, L3x, L3z, LEx, LEz) # Get transformation matrix form base to end effector
sp.pprint(DIRECT_MAT) # Display transformation matrix form base to end effector

xDesired = 10 # Desired x coordinate
yDesired = 10 # Desired y coordinate
LM1M3 = 190 # x-component of distance between motor 1 and motor 3
LM3EE = 189 # x-component of distance between motor 3 and end effector

if (-25.487 <= xDesired <= 324.513) and (-175 <= yDesired <= 175):
    print("Inside work area")

    if (25.487 <= xDesired <= 124.513) and (-75 <= yDesired <= 75):
        print("Inaccessible area")

    else:
        print("Bingo")

        phiD, thetaD = inverse(DIRECT_MAT, LM1M3, LM3EE, xDesired, yDesired) # Get angles for motor 1 and motor 3 based on desired x, y destination
        sp.pprint(phiD) # Display angle for motor 1
        sp.pprint(thetaD) # Display angle for motor 3

else:
    print("Outside working area")

phiD, alphaD, thetaD = activePos(True) # Set arm to active position (Only motor 2 taken to account)