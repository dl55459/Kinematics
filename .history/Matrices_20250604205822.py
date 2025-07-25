import sympy as sp

# L1z, L3x, L3z, LEx, LEz = sp.symbols("L1z, L3x, L3z, LEx, LEz", real = True) # Lengths
phi, theta = sp.symbols("phi theta", real = True) # Angles
# subsL = {L1z : 41.9, L3x : 190, L3z : -0.55, LEx : 189, LEz : 39.35} # Link lengths in mm

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
def direct(L1z, L3x, L3z, LEx, LEz):
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

    # Display the result
    sp.pprint(T_BEE)
    sp.pprint(T_BEE_subs)

    return T_BEE_subs

"""
^                                        88 88b 88 Yb    dP 888888 88""Yb .dP"Y8 888888
^                                        88 88Yb88  Yb  dP  88__   88__dP `Ybo." 88__
^                                        88 88 Y88   YbdP   88""   88"Yb  o.`Y8b 88""
^                                        88 88  Y8    YP    888888 88  Yb 8bodP' 888888
"""
def inverse(T_BEE_subs, LM1M3, LM3EE, xDesired, yDesired, subsLen):
    # Extract x, y, z from T_BEE
    # xd, yd = sp.symbols("xd yd", real = True) # Desired destination in x, y directions
    #subsd = {xDesired:1, yDesired:1} # input desired x, y location

    # extract x, y, z components form T_BEE matrix
    x, y = T_BEE_subs[:3, 2]

    # store x, y to different variables
    x_expr = x
    y_expr = y

    # Substitute the link lengths
    x_expr = x_expr.subs(subsLen)
    y_expr = y_expr.subs(subsLen)

    # Set equations for x, y calculation
    eqx = sp.Eq(x_expr, xDesired)
    eqy = sp.Eq(y_expr, yDesired)

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
    beta = sp.atan2(yDesired, xDesired)
    gamma = sp.atan2(LM3EE * sp.sin(thetaDesired), LM1M3 + LM3EE * sp.cos(thetaDesired))
    phiDesired = beta - gamma

    # Display results
    sp.pprint(phiDesired)
    sp.pprint(thetaDesired)

    return phiDesired, thetaDesired

# Function sets default positions for active and in-active operation mode
def activePos(flag):
    # Check for activity flag
    if flag == 1:
        phiActive = 0
        thetaActive = 0
    else:
        phiActive = 0
        thetaActive = 90

    return phiActive, thetaActive

# Function sends motor positions
def topicOut(x, y, z, phiDesired, thetaDesired):
    # No implementation yet

    return 0

# Function receives motor positions
def topicIn():
    # No implementation yet

    return 0

# Main section of the code
phiD, thetaD = activePos(False) # Set arm to idle position

L1z = 41.9
L3x = 190
L3z = -0.55
LEx = 189
LEz = 39.35
DIRECT_MAT = direct(L1z, L3x, L3z, LEx, LEz) # Get transformation matrix form base to end effector

xDesired = 10
yDesired = 10
phiD, thetaD = inverse(DIRECT_MAT, 190, 189, xDesired, yDesired, subsL)