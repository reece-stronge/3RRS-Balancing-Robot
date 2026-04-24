import time
import numpy as np
from adafruit_servokit import ServoKit

class Pose:
    def __init__(self):
        # Initialize servo kit (16 channels)
        self.kit = ServoKit(channels=16)
        self.s1_index = 0
        self.s2_index = 1
        self.s3_index = 2

        # --- Physical Geometry ---
        self.L = 0.09125   # Top radius
        self.L1 = 0.075    # Upper link
        self.L2 = 0.05     # Lower link
        self.L3 = 0.075    # Base radius

        # Initial position [tilt_phi (direction), tilt_theta (magnitude), h (height)]
        self.ini_pos = [0, 0, 0.10]
        self.h_max = 0.1228
        self.h_min = 0.080
        self.tilt_theta_max = 14

    def clean_up(self):
        print("Release servo holding torque")
        self.kit.servo[self.s1_index].angle = None
        self.kit.servo[self.s2_index].angle = None
        self.kit.servo[self.s3_index].angle = None
        pass

    def kinematics(self, n, h):
        # Leg inversion boundary
        A0 = (self.L3 + self.L2) / h
        B0 = (h**2 + self.L1**2 - (self.L3 + self.L2)**2 - self.L**2) / (2*h)
        C0 = A0**2 + 1
        D0 = 2 * (A0 * B0 - (self.L3 + self.L2))
        E0 = B0**2 + (self.L3 + self.L2)**2 - self.L1**2
        
        try:
            P0_x = (-D0 + np.sqrt(D0**2 - 4 * C0 * E0)) / (2 * C0)
            P0_z = np.sqrt(self.L1**2 - P0_x**2 + 2 * (self.L3 + self.L2) * P0_x - (self.L3 + self.L2)**2)
        except ValueError:
            return [0, 0, 0] # Return safe values if math domain error

        # --- LEG 1 ---
        den_sqrt_1 = np.sqrt(n[0]**2 + n[2]**2)
        a1 = (self.L / den_sqrt_1) * n[2]
        c1 = h + (self.L / den_sqrt_1) * (-n[0])
        S1 = np.array([a1, 0, c1])
        A1 = (self.L3 - S1[0]) / S1[2]
        B1 = (S1[0]**2 + S1[1]**2 + S1[2]**2 - self.L1**2 - self.L3**2 + self.L2**2) / (2 * S1[2])
        C1 = A1**2 + 1
        D1 = 2 * (A1 * B1 - self.L3)
        E1 = B1**2 + self.L3**2 - self.L2**2
        x1 = (-D1 + np.sqrt(D1**2 - 4 * C1 * E1)) / (2 * C1)
        z1 = np.sqrt(self.L2**2 - x1**2 + 2 * self.L3 * x1 - self.L3**2)
        
        if c1 < P0_z: z1 = -z1
        P1 = np.array([x1, 0, z1])
        theta1 = 90 - np.degrees(np.arctan2(P1[0] - self.L3, P1[2]))

        # --- LEG 2 ---
        common_denom_2 = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*np.sqrt(3)*n[0]*n[1])
        a2 = (self.L / common_denom_2) * (-n[2])
        b2 = (self.L / common_denom_2) * (-np.sqrt(3) * n[2])
        c2 = h + (self.L / common_denom_2) * (np.sqrt(3) * n[1] + n[0])
        S2 = np.array([a2, b2, c2])
        A2 = -(S2[0] + np.sqrt(3) * S2[1] + 2 * self.L3) / S2[2]
        B2 = (S2[0]**2 + S2[1]**2 + S2[2]**2 + self.L2**2 - self.L1**2 - self.L3**2) / (2 * S2[2])
        C2 = A2**2 + 4
        D2 = 2 * A2 * B2 + 4 * self.L3
        E2 = B2**2 + self.L3**2 - self.L2**2
        x2 = (-D2 - np.sqrt(D2**2 - 4 * C2 * E2)) / (2 * C2)
        y2 = np.sqrt(3) * x2
        z2 = np.sqrt(self.L2**2 - 4 * x2**2 - 4 * self.L3 * x2 - self.L3**2)
        
        if c2 < P0_z: z2 = -z2
        P2 = np.array([x2, y2, z2])
        theta2 = 90 - np.degrees(np.arctan2(np.sqrt(P2[0]**2 + P2[1]**2) - self.L3, P2[2]))

        # --- LEG 3 ---
        common_denom_3 = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*np.sqrt(3)*n[0]*n[1])
        a3 = (self.L / common_denom_3) * (-n[2])
        b3 = (self.L / common_denom_3) * (np.sqrt(3) * n[2])
        c3 = h + (self.L / common_denom_3) * (-np.sqrt(3) * n[1] + n[0])
        S3 = np.array([a3, b3, c3])
        A3 = -(S3[0] - np.sqrt(3) * S3[1] + 2 * self.L3) / S3[2]
        B3 = (S3[0]**2 + S3[1]**2 + S3[2]**2 + self.L2**2 - self.L1**2 - self.L3**2) / (2 * S3[2])
        C3 = A3**2 + 4
        D3 = 2 * A3 * B3 + 4 * self.L3
        E3 = B3**2 + self.L3**2 - self.L2**2
        x3 = (-D3 - np.sqrt(D3**2 - 4 * C3 * E3)) / (2 * C3)
        y3 = -np.sqrt(3) * x3
        z3 = np.sqrt(self.L2**2 - 4 * x3**2 - 4 * self.L3 * x3 - self.L3**2)
        
        if c3 < P0_z: z3 = -z3
        P3 = np.array([x3, y3, z3])
        theta3 = 90 - np.degrees(np.arctan2(np.sqrt(P3[0]**2 + P3[1]**2) - self.L3, P3[2]))

        return [theta1, theta2, theta3]

    def pose_platform(self, pos, t):
        tilt_phi = pos[0]    # Direction
        tilt_theta = pos[1]  # Magnitude
        
        if tilt_theta > self.tilt_theta_max:
            tilt_theta = self.tilt_theta_max
            
        h = pos[2]
        if h > self.h_max:
            h = self.h_max
        elif h < self.h_min:
            h = self.h_min

        # Normal vector components
        tilt_theta_rad = np.radians(tilt_theta)
        tilt_phi_rad = np.radians(tilt_phi)

        alpha = np.sin(tilt_theta_rad) * np.cos(tilt_phi_rad)
        beta = np.sin(tilt_theta_rad) * np.sin(tilt_phi_rad)
        gamma = np.cos(tilt_theta_rad)
        
        n = np.array([alpha, beta, gamma])
        angles = self.kinematics(n, h)
        
        plot_angle_1 = angles[0]
        plot_angle_2 = angles[1]
        plot_angle_3 = angles[2]

        # --- Servo Offsets  ---
        final_angle_1 = angles[0] + 40
        final_angle_2 = angles[1] + 22
        final_angle_3 = angles[2] + 20

        # Move the servos with clamping to safe limits
        self.kit.servo[self.s1_index].angle = max(0, min(180, final_angle_1))
        self.kit.servo[self.s2_index].angle = max(0, min(180, final_angle_2))
        self.kit.servo[self.s3_index].angle = max(0, min(180, final_angle_3))
        
        time.sleep(t) # Only sleeps during initialisation and reset - 0 during active operation
        
        return plot_angle_1, plot_angle_2, plot_angle_3

    def initialize_platform(self):
        self.pose_platform(self.ini_pos, 1)
