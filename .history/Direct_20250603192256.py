from math import atan2, acos, cos, sin, pi, hypot
import numpy as np

def forward_kinematics(q1, q3, d1, d2, d3, d4):
    x = d2 * cos(q1) + d4 * cos(q1 - q3)
    y = d2 * sin(q1) + d4 * sin(q1 - q3)
    z = d1 - d3
    return np.array([x, y, z])

def inverse_kinematics(x, y, z, d1, d2, d4, elbow="down"):
    r_sq = x ** 2 + y ** 2
    r = hypot(x, y)
    if r > d2 + d4 or r < abs(d2 - d4):
        raise ValueError("Target is outside the planar workspace")

    cos_gamma = (d2 ** 2 + d4 ** 2 - r_sq) / (2 * d2 * d4)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = acos(cos_gamma)
    if elbow == "down":
        q3 = pi - gamma
    elif elbow == "up":
        q3 = pi + gamma - 2 * pi
    else:
        raise ValueError("elbow must be 'down' or 'up'")


    k1 = d2 + d4 * cos(q3)
    k2 = d4 * sin(q3)

    # 4. base rotation
    q1 = atan2(k1 * y + k2 * x, k1 * x - k2 * y)


    d3 = d1 - z

    return q1, 0.0, q3, d3

# link lengths (metres)
d1, d2, d4 = 1, 1, 1
# pick any joint pose you like
q1_true = np.deg2rad(30)
q2_true = 0.0
q3_true = np.deg2rad(40)      #
d3_true = 1
# forward → position
P = forward_kinematics(q1_true, q3_true, d1, d2, d3_true, d4)
print("Forward kinematics gives P =", np.round(P, 4))
# inverse → back to joints
q1_sol, q2_sol, q3_sol, d3_sol = inverse_kinematics(P[:2], d1, d2, d4, elbow="down", )
print("\nInverse kinematics recovers:")
print(f"  q1 = {np.rad2deg(q1_sol):7.3f}°   (true {np.rad2deg(q1_true):.3f}°)")
print(f"  q2 = {q2_sol:7.3f} rad           (fixed)")
print(f"  q3 = {np.rad2deg(q3_sol):7.3f}°   (true {np.rad2deg(q3_true):.3f}°)")
print(f"  d3 = {d3_sol:7.3f} m             (true {d3_true:.3f} m)")