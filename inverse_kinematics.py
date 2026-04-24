import numpy as np
from adafruit_servokit import ServoKit

# Define leg kinematics functions
def leg1_angle(n, h, L, L1, L2, L3):
    den_sqrt = np.sqrt(n[0]**2 + n[2]**2)
    a1 = (L / den_sqrt) * n[2]
    b1 = 0
    c1 = h + (L / den_sqrt) * (-n[0])
    S1 = np.array([a1, b1, c1])
    A = (L3 - S1[0]) / S1[2]
    B = (S1[0]**2 + S1[1]**2 + S1[2]**2 - L1**2 - L3**2 + L2**2) / (2 * S1[2])
    C = A**2 + 1
    D = 2 * (A * B - L3)
    E = B**2 + L3**2 - L2**2
    x1 = (-D + np.sqrt(D**2 - 4 * C * E)) / (2 * C)
    y1 = 0
    z1 = np.sqrt(L2**2 - x1**2 + 2 * L3 * x1 - L3**2)
    P1 = np.array([x1, y1, z1])
    angle1 = 90 - np.degrees(np.arctan2(P1[0] - L3, P1[2]))
    return angle1

def leg2_angle(n, h, L, L1, L2, L3):
    common_denom = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*np.sqrt(3)*n[0]*n[1])
    a2 = (L / common_denom) * (-n[2])
    b2 = (L / common_denom) * (-np.sqrt(3) * n[2])
    c2 = h + (L / common_denom) * (np.sqrt(3) * n[1] + n[0])
    S2 = np.array([a2, b2, c2])
    A2 = -(S2[0] + np.sqrt(3) * S2[1] + 2 * L3) / S2[2]
    B2 = (S2[0]**2 + S2[1]**2 + S2[2]**2 + L2**2 - L1**2 - L3**2) / (2 * S2[2])
    C2 = A2**2 + 4
    D2 = 2 * A2 * B2 + 4 * L3
    E2 = B2**2 + L3**2 - L2**2
    x2 = (-D2 - np.sqrt(D2**2 - 4 * C2 * E2)) / (2 * C2)
    y2 = np.sqrt(3) * x2
    z2 = np.sqrt(L2**2 - 4 * x2**2 - 4 * L3 * x2 - L3**2)
    P2 = np.array([x2, y2, z2])
    angle2 = 90 - np.degrees(np.arctan2(np.sqrt(P2[0]**2 + P2[1]**2) - L3, P2[2]))
    return angle2

def leg3_angle(n, h, L, L1, L2, L3):
    common_denom = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*np.sqrt(3)*n[0]*n[1])
    a3 = (L / common_denom) * (-n[2])
    b3 = (L / common_denom) * (np.sqrt(3) * n[2])
    c3 = h + (L / common_denom) * (-np.sqrt(3) * n[1] + n[0])
    S3 = np.array([a3, b3, c3])
    A3 = -(S3[0] - np.sqrt(3) * S3[1] + 2 * L3) / S3[2]
    B3 = (S3[0]**2 + S3[1]**2 + S3[2]**2 + L2**2 - L1**2 - L3**2) / (2 * S3[2])
    C3 = A3**2 + 4
    D3 = 2 * A3 * B3 + 4 * L3
    E3 = B3**2 + L3**2 - L2**2
    x3 = (-D3 - np.sqrt(D3**2 - 4 * C3 * E3)) / (2 * C3)
    y3 = -np.sqrt(3) * x3
    z3 = np.sqrt(L2**2 - 4 * x3**2 - 4 * L3 * x3 - L3**2)
    P3 = np.array([x3, y3, z3])
    angle3 = 90 - np.degrees(np.arctan2(np.sqrt(P3[0]**2 + P3[1]**2) - L3, P3[2]))
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
