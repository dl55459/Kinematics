import sympy as sp

BasMotor = sp.Matrix([
    [sp.cos(th), -sp.sin(th)*sp.cos(al),  sp.sin(th)*sp.sin(al),  a*sp.cos(th)],
    [sp.sin(th),  sp.cos(th)*sp.cos(al), -sp.cos(th)*sp.sin(al),  a*sp.sin(th)],
    [0,           sp.sin(al),             sp.cos(al),             d],
    [0,           0,                     0,                      1]
    ])