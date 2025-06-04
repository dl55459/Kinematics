import sympy as sp

L1z, L3x, L3z, LEx, LEz = sp.symbols("L1z, L3x, L3z, LEx, LEz", real = True)
phi, theta = sp.symbols("phi theta", real = True)

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
T_BEE = T_BM1 * T_M1M3 * T_M3EE
T_BEE = T_BEE.subs({L1z : 41.9, L3x : 190, L3z : -0.55, LEx : 189, LEz : 39.35}) # in mm
# Display the result
sp.pprint(T_BEE)

"""
^                                        88 88b 88 Yb    dP 888888 88""Yb .dP"Y8 888888
^                                        88 88Yb88  Yb  dP  88__   88__dP `Ybo." 88__
^                                        88 88 Y88   YbdP   88""   88"Yb  o.`Y8b 88""
^                                        88 88  Y8    YP    888888 88  Yb 8bodP' 888888
"""
# Extract x, y, z from T_BEE
x, y, z = T_BEE[:3, 3]