import sympy as sp
x, y, z, r1, r2, r3, r4, r5, r6, r7, r8, r9 = sp.symbols("x y z r1 r2 r3 r4 r5 r6 r7 r8 r9", real=True)
phi, theta, alpha, beta = sp.symbols("phi theta alpha beta", real=True)

BM1_T = sp.Matrix([
    [0],
    [0],
    [41.9] # mm
])

    # [x1(x0), x1(y0), x1(z0)]
    # [y1(x0), y1(y0), y1(z0)]
    # [z1(x0), z1(y0), z1(z0)]

BM1_R = sp.Matrix([
    [sp.cos(phi), -sp.sin(phi), 0],
    [sp.sin(phi), sp.cos(phi), 0],
    [0, 0, 1]
])
M1M3_T = sp.Matrix([
    [1],
    [0],
    [0]
])
M1M3_R = sp.Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])
M3EE_T = sp.Matrix([
    [1],
    [0],
    [0]
])
M3EE_R = sp.Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])
Expanded = sp.Matrix([
    [0, 0, 0, 1]
])
IdlePos = 0;
ActivePos = 0;

# Step 1: Horizontally stack R and T to form the top 3 rows
Top = BaseMotor_R.row_join(BaseMotor_T)

# Step 2: Vertically stack Top and Expanded to form the 4x4 matrix
Homogeneous_Matrix = Top.col_join(Expanded)

# Display the result
sp.pprint(Homogeneous_Matrix)

# BasMotor = sp.Matrix([
#     [sp.cos(th), -sp.sin(th)*sp.cos(al),  sp.sin(th)*sp.sin(al),  a*sp.cos(th)],
#     [sp.sin(th),  sp.cos(th)*sp.cos(al), -sp.cos(th)*sp.sin(al),  a*sp.sin(th)],
#     [0,           sp.sin(al),             sp.cos(al),             d           ],
#     [0,           0,                     0,                      1]
#     ])
