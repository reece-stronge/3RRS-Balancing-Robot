import numpy as np
import time
from adafruit_servokit import ServoKit
from pynput import keyboard

# --- Geometry & Calibration ---
L, L1, L2, L3 = 0.09125, 0.075, 0.05, 0.075
servo1_0, servo2_0, servo3_0 = 40, 22, 20

# --- Initial State ---
h = 0.122
tilt_theta = 0.0
tilt_phi = 0.0

# Step sizes for control
H_STEP = 0.001     # 1mm change
TILT_STEP = 0.5    # 0.5 degree change
PHI_STEP = 2.0     # 2.0 degree change

# Track which keys are pressed
pressed_keys = set()

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
    return 90 - np.degrees(np.arctan2(np.sqrt(P3[0]**2 + P3[1]**2) - L3, P3[2]))

# --- Servo Setup ---
kit = ServoKit(channels=16)
servos = [kit.servo[0], kit.servo[1], kit.servo[2]]

# --- Keyboard Handling ---
def on_press(key):
    try: pressed_keys.add(key.char)
    except AttributeError: pressed_keys.add(key)

def on_release(key):
    try: pressed_keys.remove(key.char)
    except (AttributeError, KeyError): 
        if key in pressed_keys: pressed_keys.remove(key)

# Start keyboard listener in background
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("Control started. Use:")
print("W/S: Height | UP/DOWN: Tilt Magnitude | LEFT/RIGHT: Tilt Direction | ESC: Quit")

try:
    while True:
        # 1. Update variables based on held keys
        if 'w' in pressed_keys: h += H_STEP
        if 's' in pressed_keys: h -= H_STEP
        if keyboard.Key.up in pressed_keys: tilt_theta += TILT_STEP
        if keyboard.Key.down in pressed_keys: tilt_theta -= TILT_STEP
        if keyboard.Key.right in pressed_keys: tilt_phi += PHI_STEP
        if keyboard.Key.left in pressed_keys: tilt_phi -= PHI_STEP
        
        # Constrain variables
        tilt_theta = np.clip(tilt_theta, 0, 20)
        h = np.clip(h, 0.08, 0.1228)
        
        # 2. Re-calculate Normal Vector
        tr, pr = np.radians(tilt_theta), np.radians(tilt_phi)
        n = np.array([np.sin(tr)*np.cos(pr), np.sin(tr)*np.sin(pr), np.cos(tr)])
        
        # 3. Calculate and Move
        try:
            a1 = leg1_angle(n, h, L, L1, L2, L3)
            a2 = leg2_angle(n, h, L, L1, L2, L3)
            a3 = leg3_angle(n, h, L, L1, L2, L3)
            
            servos[0].angle = np.clip(servo1_0 + a1, 0, 180)
            servos[1].angle = np.clip(servo2_0 + a2, 0, 180)
            servos[2].angle = np.clip(servo3_0 + a3, 0, 180)
        except:
            pass # Skip frame if math is out of bounds

        if keyboard.Key.esc in pressed_keys:
            n_reset = np.array([0, 0, 1])
            h_reset = 0.1228
            a1_r = leg1_angle(n_reset, h_reset, L, L1, L2, L3)
            a2_r = leg2_angle(n_reset, h_reset, L, L1, L2, L3)
            a3_r = leg3_angle(n_reset, h_reset, L, L1, L2, L3)            
            servos[0].angle = a1_r + 40
            servos[1].angle = a2_r + 22
            servos[2].angle = a3_r + 20
            time.sleep(0.5)
            servos[0].angle = None
            servos[1].angle = None
            servos[2].angle = None
            break
        
        time.sleep(0.01) # 100 Hz Refresh Rate

finally:
    listener.stop()