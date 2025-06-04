import sympy as sp

x, y, z, r1, r2, r3, r4, r5, r6, r7, r8, r9 = sp.symbols("x y z r1 r2 r3 r4 r5 r6 r7 r8 r9", real=True)

BM1_T = sp.Matrix([
    [x],
    [y],
    [z]
])
BM1_R = sp.Matrix([
    [r1, r2, r3],
    [r4, r5, r6],
    [r7, r8, r9]
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
