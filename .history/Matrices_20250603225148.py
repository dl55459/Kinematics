import sympy as sp

BaseMotor_T = sp.Matrix([
    [1],
    [0],
    [0]
])
BaseMotor_R = sp.Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])
PitchMotor_T = sp.Matrix([
    [1],
    [0],
    [0]
])
PitchMotor_R = sp.Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])
EndMotor_T = sp.Matrix([
    [1],
    [0],
    [0]
])
EndMotor_R = sp.Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])
Expanded = sp.Matrix([
    [0, 0, 0, 1]
])

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