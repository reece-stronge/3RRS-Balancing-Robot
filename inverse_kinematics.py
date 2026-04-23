import numpy as np
from adafruit_servokit import ServoKit

# Define leg kinematics functions
def leg1_angle(n, h, L, L1, L2, L3):
    den_sqrt = np.sqrt(n[0]**2 + n[2]**2)
    a1 = (L / den_sqrt) * n[2]
    b1 = 0
    c1 = h + (L / den_sqrt) * (-n[0])
    A_m = np.array([a1, b1, c1])
    A = (L3 - A_m[0]) / A_m[2]
    B = (A_m[0]**2 + A_m[1]**2 + A_m[2]**2 - L1**2 - L3**2 + L2**2) / (2 * A_m[2])
    C = A**2 + 1
    D = 2 * (A * B - L3)
    E = B**2 + L3**2 - L2**2
    x1 = (-D + np.sqrt(D**2 - 4 * C * E)) / (2 * C)
    y1 = 0
    z1 = np.sqrt(L2**2 - x1**2 + 2 * L3 * x1 - L3**2)
    A_2 = np.array([x1, y1, z1])
    angle1 = 90 - np.degrees(np.arctan2(A_2[0] - L3, A_2[2]))
    return angle1

def leg2_angle(n, h, L, L1, L2, L3):
    common_denom = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*np.sqrt(3)*n[0]*n[1])
    a2 = (L / common_denom) * (-n[2])
    b2 = (L / common_denom) * (-np.sqrt(3) * n[2])
    c2 = h + (L / common_denom) * (np.sqrt(3) * n[1] + n[0])
    B_m = np.array([a2, b2, c2])
    A2 = -(B_m[0] + np.sqrt(3) * B_m[1] + 2 * L3) / B_m[2]
    B2 = (B_m[0]**2 + B_m[1]**2 + B_m[2]**2 + L2**2 - L1**2 - L3**2) / (2 * B_m[2])
    C2 = A2**2 + 4
    D2 = 2 * A2 * B2 + 4 * L3
    E2 = B2**2 + L3**2 - L2**2
    x2 = (-D2 - np.sqrt(D2**2 - 4 * C2 * E2)) / (2 * C2)
    y2 = np.sqrt(3) * x2
    z2 = np.sqrt(L2**2 - 4 * x2**2 - 4 * L3 * x2 - L3**2)
    B_2 = np.array([x2, y2, z2])
    angle2 = 90 - np.degrees(np.arctan2(np.sqrt(B_2[0]**2 + B_2[1]**2) - L3, B_2[2]))
    return angle2

def leg3_angle(n, h, L, L1, L2, L3):
    common_denom = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*np.sqrt(3)*n[0]*n[1])
    a3 = (L / common_denom) * (-n[2])
    b3 = (L / common_denom) * (np.sqrt(3) * n[2])
    c3 = h + (L / common_denom) * (-np.sqrt(3) * n[1] + n[0])
    C_m = np.array([a3, b3, c3])
    A3 = -(C_m[0] - np.sqrt(3) * C_m[1] + 2 * L3) / C_m[2]
    B3 = (C_m[0]**2 + C_m[1]**2 + C_m[2]**2 + L2**2 - L1**2 - L3**2) / (2 * C_m[2])
    C3 = A3**2 + 4
    D3 = 2 * A3 * B3 + 4 * L3
    E3 = B3**2 + L3**2 - L2**2
    x3 = (-D3 - np.sqrt(D3**2 - 4 * C3 * E3)) / (2 * C3)
    y3 = -np.sqrt(3) * x3
    z3 = np.sqrt(L2**2 - 4 * x3**2 - 4 * L3 * x3 - L3**2)
    C_2 = np.array([x3, y3, z3])
    angle3 = 90 - np.degrees(np.arctan2(np.sqrt(C_2[0]**2 + C_2[1]**2) - L3, C_2[2]))
    return angle3

# Define geometry
L =  0.09125       # Top radius
L1 = 0.075         # Upper link
L2 = 0.05          # Lower link
L3 = 0.075         # Base radius

# Inputs
h = 0.122
tilt_theta = 0 # Magnitude
tilt_phi = 0   # Direction

tilt_theta_rad = np.radians(tilt_theta)
tilt_phi_rad = np.radians(tilt_phi)

# Normal vector components
alpha = np.sin(tilt_theta_rad) * np.cos(tilt_phi_rad)
beta = np.sin(tilt_theta_rad) * np.sin(tilt_phi_rad)
gamma = np.cos(tilt_theta_rad)

n = np.array([alpha, beta, gamma])

# Calculate motor angles
angle1 = leg1_angle(n, h, L, L1, L2, L3)
angle2 = leg2_angle(n, h, L, L1, L2, L3)
angle3 = leg3_angle(n, h, L, L1, L2, L3)

thetas_deg = np.array([angle1, angle2, angle3])
print(f"Motor Angles: {thetas_deg}")


# Servo Setup
kit = ServoKit(channels=16)
servo1 = kit.servo[0]
servo2 = kit.servo[1]
servo3 = kit.servo[2]

# Servo offsets: kinematics 0 = parallel with floor
servo1_0 = 40
servo2_0 = 22
servo3_0 = 20

servo1.angle = servo1_0 + angle1
servo2.angle = servo2_0 + angle2
servo3.angle = servo3_0 + angle3