import sympy as sp
import math as math
from math import atan2, cos, sin, pi
import numpy as np

def DEG2RAD(degree):
    rad = degree * (np.pi/180)
    return rad

def RAD2DEG(rad):
    degree = rad * (180/np.pi)
    return degree

def mm2m(mm):
    m = mm/1000
    return m

# Distances for calculation (global constants as they are used by direct and inverse)
L1z = mm2m(41.9) # z-component of distance between base and motor 1
L3x = mm2m(190) # x-component of distance between motor 1 and motor 3
L3z = mm2m(-0.55) # z-component of distance between motor 1 and motor 3
LEx = mm2m(189) # x-component of distance between motor 3 and end effector
LEz = mm2m(39.35) # z-component of distance between motor 3 and end effector

"""
^                                        8888b.  88 88""Yb 888888  dP""b8 888888
^                                         8I  Yb 88 88__dP 88__   dP   `"   88
^                                         8I  dY 88 88"Yb  88""  Yb         88
^                                        8888Y"  88 88  Yb 888888  YboodP   88
"""
# Function for calculating transformation matrix from base to end effector
def direct(L1z, L3x, L3z, LEx, LEz):
    phi, theta = sp.symbols("phi theta", real = True) # Angles

    # Translation matrix from base to motor 1
    BM1_T = sp.Matrix([
        [0],
        [0],
        [L1z]
    ])

    # Rotation matrix from base to motor 1
    BM1_R = sp.Matrix([
        [sp.cos(phi), -sp.sin(phi), 0],
        [sp.sin(phi),  sp.cos(phi), 0],
        [          0,            0, 1]
    ])

    # Translation matrix from motor 1 to motor3
    M1M3_T = sp.Matrix([
        [L3x],
        [0],
        [L3z]
    ])

    # Rotation matrix from motor 1 to motor3
    M1M3_R = sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0],
        [sp.sin(theta),  sp.cos(theta), 0],
        [          0,               0, 1]
    ])

    # Translation matrix from motor3 to end effector
    M3EE_T = sp.Matrix([
        [LEx],
        [0],
        [LEz]
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
    T_BEE = T_BM1 * T_M1M3 * T_M3EE

    return T_BEE, phi, theta # Return phi and theta symbols as well

"""
^                                        88 88b 88 Yb     dP 888888 88""Yb .dP"Y8 888888
^                                        88 88Yb88  Yb   dP  88__   88__dP `Ybo." 88__
^                                        88 88 Y88   YbdP   88""   88"Yb  o.`Y8b 88""
^                                        88 88  Y8    YP   888888 88  Yb 8bodP' 888888
"""
def inverse(xDesired, yDesired, T_BEE_symbolic, phi_sym, theta_sym):
    """
    Solves for phi and theta using the provided symbolic transformation matrix.
    Args:
        xDesired (float): Desired x-coordinate of the end effector.
        yDesired (float): Desired y-coordinate of the end effector.
        T_BEE_symbolic (sympy.Matrix): The symbolic transformation matrix from base to end effector.
        phi_sym (sympy.Symbol): The symbolic variable for phi.
        theta_sym (sympy.Symbol): The symbolic variable for theta.
    Returns:
        tuple: (phiDesired, thetaDesired) in radians, or (None, None) if no solution.
    """
    # Extract the symbolic x and y expressions from the transformation matrix
    x_expression = T_BEE_symbolic[0, 3]
    y_expression = T_BEE_symbolic[1, 3]

    # Create equations
    eq1 = sp.Eq(x_expression, xDesired)
    eq2 = sp.Eq(y_expression, yDesired)

    # Solve the system of equations for phi and theta
    solutions = sp.solve([eq1, eq2], (phi_sym, theta_sym), dict=True)

    if not solutions:
        # Handle cases where no real solutions are found (e.g., unreachable point)
        return None, None

    # Filter for real solutions and pick the first valid one
    for sol in solutions:
        phi_val = sol[phi_sym]
        theta_val = sol[theta_sym]

        if phi_val.is_real and theta_val.is_real:
            # Convert sympy expressions to float
            phiDesired = float(phi_val)
            thetaDesired = float(theta_val)
            return phiDesired, thetaDesired

    # If no real solutions were found
    return None, None


if __name__ == "__main__":
    print("--- Testing Direct and Inverse Kinematics ---")

    # 1. Get symbolic T_BEE and angle symbols
    T_BEE_sym, phi_sym, theta_sym = direct(L1z, L3x, L3z, LEx, LEz)
    print("\nSymbolic T_BEE matrix (top-left 3x3 for rotation, last column for position):")
    print(T_BEE_sym)
    print(f"Symbolic phi: {phi_sym}, Symbolic theta: {theta_sym}")

    # 2. Test Direct Kinematics with sample angles
    print("\n--- Direct Kinematics Test ---")
    test_phi = DEG2RAD(30)  # 30 degrees
    test_theta = DEG2RAD(60) # 60 degrees

    # Substitute numerical values into the symbolic matrix
    T_BEE_numerical = T_BEE_sym.subs({phi_sym: test_phi, theta_sym: test_theta})

    x_calculated_direct = T_BEE_numerical[0, 3]
    y_calculated_direct = T_BEE_numerical[1, 3]

    print(f"Given phi: {RAD2DEG(test_phi):.2f} deg, theta: {RAD2DEG(test_theta):.2f} deg")
    print(f"Calculated End Effector Position (x, y) from direct kinematics: ({x_calculated_direct:.4f} m, {y_calculated_direct:.4f} m)")


    # 3. Test Inverse Kinematics with desired (x, y) coordinates
    print("\n--- Inverse Kinematics Test ---")
    # Use the calculated x,y from direct kinematics to test inverse kinematics
    # This should ideally give back the original test_phi and test_theta
    x_desired_inverse = x_calculated_direct
    y_desired_inverse = y_calculated_direct

    print(f"Desired End Effector Position (x, y) for inverse kinematics: ({x_desired_inverse:.4f} m, {y_desired_inverse:.4f} m)")

    phi_calculated_inverse, theta_calculated_inverse = inverse(x_desired_inverse, y_desired_inverse, T_BEE_sym, phi_sym, theta_sym)

    if phi_calculated_inverse is not None and theta_calculated_inverse is not None:
        print(f"Calculated Angles from inverse kinematics:")
        print(f"  phi: {RAD2DEG(phi_calculated_inverse):.2f} degrees ({phi_calculated_inverse:.4f} rad)")
        print(f"  theta: {RAD2DEG(theta_calculated_inverse):.2f} degrees ({theta_calculated_inverse:.4f} rad)")
        print(f"Original angles: phi {RAD2DEG(test_phi):.2f} deg, theta {RAD2DEG(test_theta):.2f} deg")
        print("\nNote: Multiple solutions might exist for inverse kinematics. The first real solution found is returned.")
        print("Due to floating point precision in sympy, the results might have small discrepancies.")
    else:
        print("No valid real solution found for the desired (x, y) position.")

    print("\n--- Additional Inverse Kinematics Test (Unreachable Point) ---")
    x_unreachable = 1000.0 # Far away point in meters
    y_unreachable = 1000.0
    print(f"Desired End Effector Position (x, y) for inverse kinematics: ({x_unreachable:.4f} m, {y_unreachable:.4f} m)")
    phi_unreachable, theta_unreachable = inverse(x_unreachable, y_unreachable, T_BEE_sym, phi_sym, theta_sym)
    if phi_unreachable is None:
        print("As expected, no valid real solution found for the unreachable point.")