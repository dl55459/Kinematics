import sympy as sp

q1, q2, q3, d1, d2, d3, d4 = sp.symbols("q1 q2 q3 d1 d2 d3 d4", real=True)

alpha90 = sp.pi/2
alpha0  = 0

# 2. Denavit-Hartenberg helper
def DH(th, d, al, a):
    return sp.Matrix([
        [sp.cos(th), -sp.sin(th)*sp.cos(al),  sp.sin(th)*sp.sin(al),  a*sp.cos(th)],
        [sp.sin(th),  sp.cos(th)*sp.cos(al), -sp.cos(th)*sp.sin(al),  a*sp.sin(th)],
        [0,           sp.sin(al),            sp.cos(al),             d],
        [0,           0,                     0,                      1]
    ])

# 3. Individual link transforms
T1 = DH(q1, d1, alpha90, 0)
T2 = DH(q2, 0,  alpha90, d2)
T3 = DH(q3, d3, alpha0,  d4)

# 4. Composite transform and raw position vector
T    = sp.simplify(T1 * T2 * T3)
Praw = T[:3, 3]

    sp.pprint(T, use_unicode=True)
    print("\nPraw =")
    sp.pprint(Praw, use_unicode=True)