import sympy as sp
phi, theta = sp.symbols("phi theta", real=True)

    # xi(xi-1)
    # yi(yi-1)
    # zi(zi-1)

    # [xi(xi-1), xi(yi-1), xi(zi-1)]
    # [yi(xi-1), yi(yi-1), yi(zi-1)]
    # [zi(xi-1), zi(yi-1), zi(zi-1)]

BM1_T = sp.Matrix([
    [0],
    [0],
    [41.9]
])

BM1_R = sp.Matrix([
    [sp.cos(phi), -sp.sin(phi), 0],
    [sp.sin(phi),  sp.cos(phi), 0],
    [          0,            0, 1]
])
M1M3_T = sp.Matrix([
    [190],
    [0],
    [-0.55]
])
M1M3_R = sp.Matrix([
    [sp.cos(theta), -sp.sin(theta), 0],
    [sp.sin(theta),  sp.cos(theta), 0],
    [          0,                0, 1]
])
M3EE_T = sp.Matrix([
    [189],
    [0],
    [39.35]
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

TR = BM1_R.row_join(BM1_T)
T_BM1 = TR.col_join(Expanded)
TR = M1M3_R.row_join(M1M3_T)
T_M1M3 = TR.col_join(Expanded)
TR = M3EE_R.row_join(M3EE_T)
T_M3EE = TR.col_join(Expanded)

# Display the result
sp.pprint(Homogeneous_Matrix)

# BasMotor = sp.Matrix([
#     [sp.cos(th), -sp.sin(th)*sp.cos(al),  sp.sin(th)*sp.sin(al),  a*sp.cos(th)],
#     [sp.sin(th),  sp.cos(th)*sp.cos(al), -sp.cos(th)*sp.sin(al),  a*sp.sin(th)],
#     [0,           sp.sin(al),             sp.cos(al),             d           ],
#     [0,           0,                     0,                      1]
#     ])
