import sympy as sp

L1z, L3x, L3z, LEx, LEz = sp.symbols("L1z, L3x, L3z, LEx, LEz", real = True) # Lengths
phi, theta = sp.symbols("phi theta", real = True) # Rotaions
subsL = {L1z : 41.9, L3x : 190, L3z : -0.55, LEx : 189, LEz : 39.35} # Link lengths in mm

    # xi(xi-1)
    # yi(yi-1)
    # zi(zi-1)

    # [xi(xi-1), xi(yi-1), xi(zi-1)]
    # [yi(xi-1), yi(yi-1), yi(zi-1)]
    # [zi(xi-1), zi(yi-1), zi(zi-1)]

"""
^                                        8888b.  88 88""Yb 888888  dP""b8 888888
^                                         8I  Yb 88 88__dP 88__   dP   `"   88
^                                         8I  dY 88 88"Yb  88""   Yb        88
^                                        8888Y"  88 88  Yb 888888  YboodP   88
"""
def direct():
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

    # Idle and active positions of joints (Phi and Theta)
    IdlePos = 0;
    ActivePos = 0;

    # Create the transformation matrices for each segment
    TR = BM1_R.row_join(BM1_T)
    T_BM1 = TR.col_join(Expanded)
    TR = M1M3_R.row_join(M1M3_T)
    T_M1M3 = TR.col_join(Expanded)
    TR = M3EE_R.row_join(M3EE_T)
    T_M3EE = TR.col_join(Expanded)

    # Combine the transformation matrices to get the full transformation from base to end effector
    T_BEE = T_BM1 * T_M1M3 * T_M3EE # Calculate transformation matrix from base to end effector
    T_BEE_subs = T_BEE.subs(subsL) # Substitute link lengths

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
def inverse(T_BEE_subs, LM1M3, LM3EE):
    # Extract x, y, z from T_BEE
    xd, yd = sp.symbols("xd yd", real = True) # Desired destination in x, y directions
    subsd = {xd:1, yd:1} # input desired x, y location

    # extract x, y, z components form T_BEE matrix
    x, y, z = T_BEE_subs[:3, 3]

    # store x, y to different variables
    x_expr = x
    y_expr = y

    # Substitute the link lengths
    x_expr = x_expr.subs(subsL)
    y_expr = y_expr.subs(subsL)

    # Set equations for x, y calculation
    eqx = sp.Eq(x_expr, xd)
    eqy = sp.Eq(y_expr, yd)

    # Display results
    sp.pprint(eqx)
    sp.pprint(eqy)

    # Calculate the distance form base to end effector
    r_sqr = xd**2 + yd**2
    r = sp.sqrt(r_sqr)

    # Lengths of the links
    L1 = 190   # L3x
    L2 = 189   # LEx

    # Calculate the angle theta which is the angle of second motor
    cos_thera = (L1**2 + L2**2 - r_sqr) / (2 * L1 * L2)
    thetaD = sp.acos(cos_thera) # Returns +-

    # Calculate the angle phi which is the angle of first motor
    L1 = sp.Integer(L1)  # Convert to sympy Integer for better precision
    L2 = sp.Integer(L2)  # Convert to sympy Integer for better precision
    beta = sp.atan2(yd, xd)
    gamma = sp.atan2(L2 * sp.sin(thetaD), L1 + L2 * sp.cos(thetaD))
    phiD = beta - gamma

    # Display results
    sp.pprint(phiD)
    sp.pprint(thetaD)

    return phiD, thetaD