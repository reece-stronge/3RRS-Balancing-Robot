import pybullet as p
import pybullet_data
import time
import math
import matplotlib.pyplot as plt
from collections import deque

# === VALIDATION TOGGLES ===
ENABLE_LATENCY = True
ENABLE_SERVO_SLEW = True
ENABLE_STEP_RESPONSE = True

LATENCY_SECONDS = 0.1125

# Camera mapping
PIXELS_PER_METER = 640 / 0.1625

# === MATCHED PID CLASS ===
class PID:
    def __init__(self, K_PID, k_scale, alpha):
        self.kp = K_PID[0]
        self.ki = K_PID[1]
        self.kd = K_PID[2]
        self.k_scale = k_scale
        self.alpha = alpha
        self.last_output_x = 0
        self.last_output_y = 0
        self.last_derivative_x = 0
        self.last_derivative_y = 0
        self.last_error_x = 0
        self.integral_x = 0
        self.last_error_y = 0
        self.integral_y = 0
        self.last_time = None

    def compute(self, Target, current_pos, current_time):
        if self.last_time is None:
            self.last_time = current_time
            return 0, 0

        error_x = Target[0] - current_pos[0]
        error_y = Target[1] - current_pos[1]

        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.001

        self.integral_x += error_x * dt
        self.integral_y += error_y * dt

        raw_derivative_x = (error_x - self.last_error_x) / dt
        raw_derivative_y = (error_y - self.last_error_y) / dt

        derivative_x = self.alpha * raw_derivative_x + (1 - self.alpha) * self.last_derivative_x
        derivative_y = self.alpha * raw_derivative_y + (1 - self.alpha) * self.last_derivative_y

        output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y

        tilt_phi = math.degrees(math.atan2(output_y, output_x))
        if tilt_phi < 0:
            tilt_phi += 360
        tilt_theta = self.k_scale * math.sqrt(output_x ** 2 + output_y ** 2)

        self.last_derivative_x = derivative_x
        self.last_derivative_y = derivative_y
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_output_x = output_x
        self.last_output_y = output_y
        self.last_time = current_time

        return tilt_phi, tilt_theta


# === SIMULATION SETUP & LOOP ===
def run_simulation():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    PHYSICS_HZ = 240.0
    CAMERA_HZ = 14.3
    physics_step = 1.0 / PHYSICS_HZ
    camera_step = 1.0 / CAMERA_HZ
    p.setTimeStep(physics_step)

    plate_radius = 0.09125
    initial_height = 0.10
    tilt_theta_max = 14

    p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=45, cameraPitch=-30,
                                 cameraTargetPosition=[0, 0, initial_height])
    p.loadURDF("plane.urdf")

    plate_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=plate_radius, height=0.005)
    plate_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=plate_radius, length=0.005, rgbaColor=[0.8, 0.8, 0.8, 1])
    plateId = p.createMultiBody(baseMass=0.5, baseCollisionShapeIndex=plate_shape, baseVisualShapeIndex=plate_visual,
                                basePosition=[0, 0, initial_height])

    constraintId = p.createConstraint(parentBodyUniqueId=p.loadURDF("plane.urdf"), parentLinkIndex=-1,
                                      childBodyUniqueId=plateId, childLinkIndex=-1,
                                      jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                                      parentFramePosition=[0, 0, initial_height], childFramePosition=[0, 0, 0])

    ball_radius = 0.02
    ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
    ball_visual = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 0.5, 0, 1])

    # --- Initial Conditions ---
    start_x_m = 250 / PIXELS_PER_METER
    start_y_m = 150 / PIXELS_PER_METER
    ballId = p.createMultiBody(baseMass=0.0027, baseCollisionShapeIndex=ball_shape, baseVisualShapeIndex=ball_visual,
                               basePosition=[start_x_m, start_y_m, initial_height + 0.05])

    p.changeDynamics(plateId, -1, lateralFriction=1.0, rollingFriction=0.0)
    p.changeDynamics(ballId, -1,
                     lateralFriction=0.5,
                     rollingFriction=0.0001,
                     restitution=0.8,
                     linearDamping=0.0,
                     angularDamping=0.0,
                     activationState=p.ACTIVATION_STATE_DISABLE_SLEEPING)

    m = 0.0027
    r = 0.02
    inertia = (2.0 / 3.0) * m * (r ** 2)
    p.changeDynamics(ballId, -1, localInertiaDiagonal=[inertia, inertia, inertia])

    # Gains
    K_PID = [0.018, 0.0015, 0.01]
    k_scale = 1
    a = 1.0
    pid = PID(K_PID, k_scale, a)
    goal = [0, 0]

    # --- Logging arrays ---
    time_log = []
    x_error_log = []
    y_error_log = []
    x_pos_log = []
    y_pos_log = []
    x_goal_log = []
    y_goal_log = []

    command_queue = deque()

    cur_nx, cur_ny, cur_nz = 0.0, 0.0, 1.0
    active_nx, active_ny, active_nz = 0.0, 0.0, 1.0

    sim_time = 0.0
    last_camera_time = -camera_step

    if ENABLE_STEP_RESPONSE:
        max_sim_time = 45.0
    else:
        max_sim_time = 6.0

    print("Starting simulation... Waiting for ball to drop.")
    for _ in range(100):
        p.stepSimulation()

    while sim_time < max_sim_time:

        # === THREAD 1: CAMERA & PID ===
        if sim_time - last_camera_time >= camera_step:
            ball_pos, _ = p.getBasePositionAndOrientation(ballId)
            plate_pos, _ = p.getBasePositionAndOrientation(plateId)

            x_m = ball_pos[0] - plate_pos[0]
            y_m = ball_pos[1] - plate_pos[1]

            # Inverted for bottom-up camera perspective
            x_px = int(-x_m * PIXELS_PER_METER)
            y_px = int(-y_m * PIXELS_PER_METER)

            # --- DYNAMIC SETPOINT ---
            if ENABLE_STEP_RESPONSE:
                if sim_time < 15.0:
                    goal = [0, 0]
                elif sim_time < 25.0:
                    goal = [200, 0]
                elif sim_time < 35.0:
                    goal = [-200, 0]
                else:
                    goal = [0, 0]
            else:
                goal = [0, 0]

            current_pos = [x_px, y_px, 0]
            tilt_phi, tilt_theta = pid.compute(goal, current_pos, sim_time)
            tilt_theta = min(tilt_theta, tilt_theta_max)

            tilt_phi_rad = math.radians(tilt_phi)
            tilt_theta_rad = math.radians(tilt_theta)

            target_nx = math.sin(tilt_theta_rad) * math.cos(tilt_phi_rad)
            target_ny = math.sin(tilt_theta_rad) * math.sin(tilt_phi_rad)
            target_nz = math.cos(tilt_theta_rad)

            if ENABLE_LATENCY:
                command_queue.append((sim_time + LATENCY_SECONDS, target_nx, target_ny, target_nz))
            else:
                active_nx, active_ny, active_nz = target_nx, target_ny, target_nz

            # Log dynamic error, position, and goals
            time_log.append(sim_time)
            x_error_log.append(goal[0] - x_px)
            y_error_log.append(goal[1] - y_px)
            x_pos_log.append(x_px)
            y_pos_log.append(y_px)
            x_goal_log.append(goal[0])
            y_goal_log.append(goal[1])

            last_camera_time = sim_time

        # === THREAD 2: SERVO ACTUATION (Runs continuously 240 Hz)
        if ENABLE_LATENCY:
            if len(command_queue) > 0 and command_queue[0][0] <= sim_time:
                _, active_nx, active_ny, active_nz = command_queue.popleft()

        if ENABLE_SERVO_SLEW:
            cur_nx += (active_nx - cur_nx) * 0.15
            cur_ny += (active_ny - cur_ny) * 0.15
            cur_nz += (active_nz - cur_nz) * 0.15

            mag = math.sqrt(cur_nx ** 2 + cur_ny ** 2 + cur_nz ** 2)
            cur_nx /= mag
            cur_ny /= mag
            cur_nz /= mag
        else:
            cur_nx, cur_ny, cur_nz = active_nx, active_ny, active_nz

        axis_x = -cur_ny
        axis_y = cur_nx
        axis_length = math.sqrt(axis_x ** 2 + axis_y ** 2)

        if axis_length < 1e-6:
            target_quat = [0, 0, 0, 1]
        else:
            axis_x /= axis_length
            axis_y /= axis_length
            angle = math.acos(cur_nz)
            sin_half = math.sin(angle / 2.0)
            target_quat = [axis_x * sin_half, axis_y * sin_half, 0, math.cos(angle / 2.0)]

        p.changeConstraint(constraintId, jointChildFrameOrientation=target_quat, maxForce=50000)

        # Physics integration
        p.stepSimulation()
        time.sleep(physics_step)
        sim_time += physics_step

    p.disconnect()

    # === PLOTTING ===
    plant_type = "Non-Ideal System (Latency & Servo Speed)" if (
            ENABLE_LATENCY or ENABLE_SERVO_SLEW) else "Ideal System (Zero Latency)"

    # Plot 1: Error over Time
    plt.figure(1, figsize=(10, 6))
    plt.plot(time_log, x_error_log, label='X Axis Error', color='blue', alpha=0.7)
    plt.plot(time_log, y_error_log, label='Y Axis Error', color='red', alpha=0.7)
    plt.axhline(0, color='black', linestyle='--', linewidth=1)
    plt.title(f'Simulated PID Response (Error vs Time): {plant_type}')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (px)')
    plt.grid(True)
    plt.legend()

    # Plot 2: Position Tracking Step Response
    plt.figure(2, figsize=(10, 6))
    plt.plot(time_log, x_goal_log, label='X Setpoint', color='black', linestyle='--', linewidth=1.5)
    plt.plot(time_log, y_goal_log, label='Y Setpoint', color='grey', linestyle='--', linewidth=1.5)
    plt.plot(time_log, x_pos_log, label='X Position', color='purple', alpha=1, linewidth=2)
    plt.plot(time_log, y_pos_log, label='Y Position', color='orange', alpha=1, linewidth=2)
    plt.title(f'Simulated Step Response (Position Tracking): {plant_type}')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (px)')
    plt.grid(True)
    plt.legend()

    plt.show()

if __name__ == "__main__":
    run_simulation()
