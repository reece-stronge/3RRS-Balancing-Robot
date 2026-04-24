import time
import numpy as np

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
        self.count = 0

    def compute(self, Target, current_pos):
        current_time = time.perf_counter()
        if self.last_time is None:
            self.last_time = current_time
            return 0, 0
        
        error_x = Target[0] - current_pos[0]
        error_y = Target[1] - current_pos[1]
        
        self.integral_x += error_x * (current_time - self.last_time)
        self.integral_y += error_y * (current_time - self.last_time)
        
        raw_derivative_x = (error_x - self.last_error_x) / (current_time - self.last_time)
        raw_derivative_y = (error_y - self.last_error_y) / (current_time - self.last_time)
        
        # EMA filter on strictly derivative term
        derivative_x = self.alpha * raw_derivative_x + (1 - self.alpha) * self.last_derivative_x
        derivative_y = self.alpha * raw_derivative_y + (1 - self.alpha) * self.last_derivative_y
        
        output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
        
        ''' EMA Filter on entire controller outputs
        output_x = self.alpha * output_x + (1 - self.alpha) * self.last_output_x
        output_y = self.alpha * output_y + (1 - self.alpha) * self.last_output_y
        '''
        
        # Calculate direction (phi)
        tilt_phi = np.degrees(np.arctan2(output_y, output_x))
        if tilt_phi < 0:
            tilt_phi += 360
            
        # Calculate magnitude (theta)
        tilt_theta = self.k_scale * np.hypot(output_x, output_y)
        
        self.last_derivative_x = derivative_x
        self.last_derivative_y = derivative_y
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_output_x = output_x
        self.last_output_y = output_y
        self.last_time = current_time

        return tilt_phi, tilt_theta
