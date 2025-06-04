import sympy as sp

phi, theta = sp.symbols("phi theta", real=True)

    # xi(xi-1)
    # yi(yi-1)
    # zi(zi-1)

    # [xi(xi-1), xi(yi-1), xi(zi-1)]
    # [yi(xi-1), yi(yi-1), yi(zi-1)]
    # [zi(xi-1), zi(yi-1), zi(zi-1)]

# Translation matrix from base to motor 1
BM1_T = sp.Matrix([
    [0],
    [0],
    [41.9]
])

# Rotation matrix from base to motor 1
BM1_R = sp.Matrix([
    [sp.cos(phi), -sp.sin(phi), 0],
    [sp.sin(phi),  sp.cos(phi), 0],
    [          0,            0, 1]
])

# Translation matrix from motor 1 to motor3
M1M3_T = sp.Matrix([
    [190],
    [0],
    [-0.55]
])

# Rotation matrix from motor 1 to motor3
M1M3_R = sp.Matrix([
    [sp.cos(theta), -sp.sin(theta), 0],
    [sp.sin(theta),  sp.cos(theta), 0],
    [          0,                0, 1]
])

# Translation matrix from motor3 to end effector
M3EE_T = sp.Matrix([
    [189],
    [0],
    [39.35]
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

# Display the result
sp.pprint(T_BEE)