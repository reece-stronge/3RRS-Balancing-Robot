import numpy as np
import time
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from adafruit_servokit import ServoKit
from pynput import keyboard

# --- Geometry & Calibration ---
L, L1, L2, L3 = 0.09125, 0.075, 0.05, 0.075
servo1_0, servo2_0, servo3_0 = 40, 22, 20

# --- Trajectory parameters ---
h = 0.10           # Fixed height
tilt_theta = 15.0  # Fixed tilt magnitude
tilt_phi = 0.0     # Starting tilt direction
PHI_SPEED = 2.0    # Degrees to rotate per frame

# --- Data logging setup ---
log_time = []
log_angles = []
log_normal_xy = []
log_loop_dt = []

# --- Leg functions ---
def leg1_angle(n, h, L, L1, L2, L3):
    den_sqrt = np.sqrt(n[0]**2 + n[2]**2)
    a1 = (L/den_sqrt)*n[2]
    b1 = 0
    c1 = h + (L/den_sqrt)*(-n[0])
    S1 = np.array([a1, b1, c1])
    A = (L3 - S1[0]) / S1[2]
    B = (S1[0]**2 + S1[1]**2 + S1[2]**2 - L1**2 - L3**2 + L2**2) / (2 * S1[2])
    C = A**2 + 1
    D = 2*(A*B - L3)
    E = B**2 + L3**2 - L2**2
    x1 = (-D + np.sqrt(D**2 - 4*C*E)) / (2*C)
    y1 = 0
    z1 = np.sqrt(L2**2 - x1**2 + 2*L3*x1 - L3**2)
    P1 = np.array([x1, y1, z1])
    angle1 = 90 - np.degrees(np.arctan2(P1[0]-L3, P1[2]))
    return angle1

def leg2_angle(n, h, L, L1, L2, L3):
    common_denom = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*np.sqrt(3)*n[0]*n[1])
    a2 = (L/common_denom)*(-n[2])
    b2 = (L/common_denom)*(-np.sqrt(3)*n[2])
    c2 = h + (L/common_denom)*(np.sqrt(3)*n[1] + n[0])
    S2 = np.array([a2, b2, c2])
    A2 = -(S2[0] + np.sqrt(3)*S2[1] + 2*L3) / S2[2]
    B2 = (S2[0]**2 + S2[1]**2 + S2[2]**2 + L2**2 - L1**2 - L3**2) / (2*S2[2])
    C2 = A2**2 + 4
    D2 = 2*A2*B2 + 4*L3
    E2 = B2**2 + L3**2 - L2**2
    x2 = (-D2 - np.sqrt(D2**2 - 4*C2*E2)) / (2*C2)
    y2 = np.sqrt(3) * x2
    z2 = np.sqrt(L2**2 - 4*x2**2 - 4*L3*x2 - L3**2)
    P2 = np.array([x2, y2, z2])
    angle2 = 90 - np.degrees(np.arctan2(np.sqrt(P2[0]**2 + P2[1]**2) - L3, P2[2]))
    return angle2

def leg3_angle(n, h, L, L1, L2, L3):
    common_denom = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*np.sqrt(3)*n[0]*n[1])
    a3 = (L/common_denom)*(-n[2])
    b3 = (L/common_denom)*(np.sqrt(3)*n[2])
    c3 = h + (L/common_denom)*(-np.sqrt(3)*n[1] + n[0])
    S3 = np.array([a3, b3, c3])
    A3 = -(S3[0] - np.sqrt(3)*S3[1] + 2*L3) / S3[2]
    B3 = (S3[0]**2 + S3[1]**2 + S3[2]**2 + L2**2 - L1**2 - L3**2) / (2*S3[2])
    C3 = A3**2 + 4
    D3 = 2*A3*B3 + 4*L3
    E3 = B3**2 + L3**2 - L2**2
    x3 = (-D3 - np.sqrt(D3**2 - 4*C3*E3)) / (2*C3)
    y3 = -np.sqrt(3)*x3
    z3 = np.sqrt(L2**2 - 4*x3**2 - 4*L3*x3 - L3**2)
    P3 = np.array([x3, y3, z3])
    angle3 = 90 - np.degrees(np.arctan2(np.sqrt(P3[0]**2 + P3[1]**2) - L3, P3[2]))
    return angle3

# --- Servo setup ---
kit = ServoKit(channels=16)
servos = [kit.servo[0], kit.servo[1], kit.servo[2]]

# --- Keyboard Handling ---
running = True

def on_press(key):
    global running
    if key == keyboard.Key.esc:
        running = False

listener = keyboard.Listener(on_press=on_press)
listener.start()

print("Continuous trajectory started.")
print(f"Running at tilt magnitude: {tilt_theta} deg. ESC to Quit & Generate Plots.")

start_time = time.time()

try:
    while running:
        loop_start = time.time() # Start timing the execution
        
        # Update trajectory (Rotate phi)
        tilt_phi += PHI_SPEED
        if tilt_phi >= 360.0:
            tilt_phi -= 360.0
        
        # Re-calculate normal vector
        tr, pr = np.radians(tilt_theta), np.radians(tilt_phi)
        n = np.array([np.sin(tr)*np.cos(pr), np.sin(tr)*np.sin(pr), np.cos(tr)])
        
        # Calculate and Move
        try:
            a1 = leg1_angle(n, h, L, L1, L2, L3)
            a2 = leg2_angle(n, h, L, L1, L2, L3)
            a3 = leg3_angle(n, h, L, L1, L2, L3)
            
            servos[0].angle = np.clip(servo1_0 + a1, 0, 180)
            servos[1].angle = np.clip(servo2_0 + a2, 0, 180)
            servos[2].angle = np.clip(servo3_0 + a3, 0, 180)
            
            # --- Data logging ---
            current_time = time.time() - start_time
            log_time.append(current_time)
            log_angles.append([a1, a2, a3])
            log_normal_xy.append([n[0], n[1]])
            
        except:
            pass 
        
        # --- Log execution time ---
        calc_time = (time.time() - loop_start) * 1000 # Milliseconds
        log_loop_dt.append(calc_time)

        time.sleep(0.02) # 50 Hz refresh

finally:
    listener.stop()
    print("Stopping... Resetting platform to neutral (0 tilt).")
    
    n_reset = np.array([0, 0, 1])
    h_reset = 0.122

    try:
        a1_r = leg1_angle(n_reset, h_reset, L, L1, L2, L3)
        a2_r = leg2_angle(n_reset, h_reset, L, L1, L2, L3)
        a3_r = leg3_angle(n_reset, h_reset, L, L1, L2, L3)
        
        servos[0].angle = np.clip(servo1_0 + a1_r, 0, 180)
        servos[1].angle = np.clip(servo2_0 + a2_r, 0, 180)
        servos[2].angle = np.clip(servo3_0 + a3_r, 0, 180)
        time.sleep(0.5)
        servos[0].angle = None
        servos[1].angle = None
        servos[2].angle = None
    except Exception as e:
        print("Could not reset platform:", e)

    print("Generating performance plots...")
    
    # Convert lists to numpy arrays
    times = np.array(log_time)
    angles = np.array(log_angles)
    normals = np.array(log_normal_xy)
    calc_times = np.array(log_loop_dt)

# --- PLOTTING ---
    
    # Plot 1 - Commanded motor angles over time
    plt.figure(1, figsize=(8, 5))
    plt.plot(times, angles[:, 0], label='Leg 1 (a1)', color='r')
    plt.plot(times, angles[:, 1], label='Leg 2 (a2)', color='g')
    plt.plot(times, angles[:, 2], label='Leg 3 (a3)', color='b')
    plt.title('Commanded Motor Angles')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (Degrees)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('plot_angles.png')

    # Plot 2 - Trajectory path (Nx vs Ny)
    plt.figure(2, figsize=(6, 6))
    plt.plot(normals[:, 0], normals[:, 1], color='purple')
    plt.title('Platform Normal Vector Path')
    plt.xlabel('Nx')
    plt.ylabel('Ny')
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('plot_trajectory.png')

    # Plot 3 - Loop execution time
    plt.figure(3, figsize=(8, 5))
    plt.plot(times, calc_times, color='orange', alpha=0.7)
    plt.title('Kinematics Calculation Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Execution Time (ms)')
    plt.axhline(y=20, color='r', linestyle='--', label='20ms Limit (50Hz)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('plot_execution_time.png')
    
    try: plt.show() 
    except: pass