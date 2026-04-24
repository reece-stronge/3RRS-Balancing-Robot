import time
import cv2
import numpy as np
import threading
import matplotlib.pyplot as plt

# Import modules
from pose import Pose
from pid import PID
from camera import Camera

# --- Global Setup Variables ---
height = 480
width = 480
channels = 3
image = np.zeros((height, width, channels), dtype=np.uint8)

k_scale = 1
K_PID = [0.018, 0.0015, 0.01]
a = 1

# Step Response Toggle
ENABLE_STEP_RESPONSE = False

Platform = Pose()
camera_module = Camera()
pid = PID(K_PID, k_scale, a)

Platform.initialize_platform()
h_ini = Platform.ini_pos[2]

frame_count = 0
start_time = time.time()
img_start_time = time.time()
rob_start_time = time.time()
fps = 0
img_fps = 0
rob_fps = 0

running = True
x = -1
y = -1
area = -1
goal = [0, 0]

log_time = []
log_error_x = []
log_error_y = []
log_cv_time = []
log_pos_x = []
log_pos_y = []
log_goal_x = []
log_goal_y = []
log_ctrl_time = []
log_s1 = []
log_s2 = []
log_s3 = []
latest_cv_time = 0.0
plot_start_time = time.time()

def main():
    global running
    
    image_event = threading.Event()
    data_event = threading.Event()
    
    def get_img():
        global image, img_fps, img_start_time
        img_frame_count = 0
        while running:
            image = camera_module.take_pic()
            
            image_event.set()
            
            img_frame_count += 1
            if img_frame_count == 100:
                img_end_time = time.time()
                img_elapsed_time = img_end_time - img_start_time
                img_fps = 100 / img_elapsed_time
                img_start_time = img_end_time
                img_frame_count = 0

    def control_robot():
        global x, y, area, rob_fps, rob_start_time, latest_cv_time
        rob_frame_count = 0
        while running:
            
            image_event.wait()
            if not running:
                break
            image_event.clear()
            
            # --- Start CV Stopwatch ---
            t_start_cv = time.perf_counter()
            
            x, y, area = camera_module.find_object(image)
            
            # --- End CV Stopwatch ---
            t_end_cv = time.perf_counter()
            latest_cv_time = (t_end_cv - t_start_cv) * 1000
            
            data_event.set()
            
            rob_frame_count += 1
            if rob_frame_count == 100:
                rob_end_time = time.time()
                rob_elapsed_time = rob_end_time - rob_start_time
                rob_fps = 100 / rob_elapsed_time
                rob_start_time = rob_end_time
                rob_frame_count = 0

    try:
        camera_thread = threading.Thread(target=get_img)
        robot_thread = threading.Thread(target=control_robot)
        camera_thread.start()
        robot_thread.start()

        while running:
            
            data_event.wait()
            data_event.clear()
            
            current_pos = [x, y, area]
            
            # Safe camera monitoring - Main thread for GUI safety
            cv2.imshow("Debug Feed", image)
            
            # Exit logic
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                running = False
                break
            
            if x != -1:
                
                current_time = time.time() - plot_start_time
                
                # --- Dynamic setpoint (Step Response) ---
                if ENABLE_STEP_RESPONSE:
                    if current_time < 15.0:
                        goal = [0, 0]
                    elif current_time < 25.0:
                        goal = [0, 200]
                    elif current_time < 35.0:
                        goal = [0, -200]
                    else:
                        goal = [0, 0]
                else:
                    goal = [0, 0]
                    
                logged_error_x = goal[0] - current_pos[0]
                logged_error_y = goal[1] - current_pos[1]
                
                log_time.append(current_time)
                log_error_x.append(logged_error_x)
                log_error_y.append(logged_error_y)
                log_pos_x.append(current_pos[0])
                log_pos_y.append(current_pos[1])
                log_goal_x.append(goal[0])
                log_goal_y.append(goal[1])
                
                # --- Start control stopwatch ---
                t_start_ctrl = time.perf_counter()
                
                tilt_phi, tilt_theta = pid.compute(goal, current_pos)
                pos = [tilt_phi, tilt_theta, h_ini]
                
                # No sleep time needed - timed by event wait
                s1, s2, s3 = Platform.pose_platform(pos, 0)
                
                # --- End control stopwatch ---
                t_end_ctrl = time.perf_counter()
                ctrl_time = (t_end_ctrl - t_start_ctrl) * 1000
                
                log_cv_time.append(latest_cv_time)
                log_ctrl_time.append(ctrl_time)
                
                # --- Log commanded angles ---
                log_s1.append(s1)
                log_s2.append(s2)
                log_s3.append(s3)
            else:
                pass

    finally:
        
        print("Stopping threads...")
        running = False
        
        image_event.set()
        data_event.set()
        
        if camera_thread.is_alive():
            camera_thread.join()
        if robot_thread.is_alive():
            robot_thread.join()
            
        # --- Reset platform pose ---
        Platform.pose_platform([0, 0, 0.11], 0.5)
        
        print("Stop and generate plots")
        Platform.clean_up()
        camera_module.clean_up_cam()
        
        if len(log_time) > 0:
            # Plot 1 - Error Response
            plt.figure(1, figsize=(10, 6))
            plt.plot(log_time, log_error_x, label='X Axis Error', color='blue', alpha=0.7)
            plt.plot(log_time, log_error_y, label='Y Axis Error', color='red', alpha=0.7)
            plt.axhline(0, color='black', linestyle='--', linewidth=1)
            plt.title('PID Response: Error vs Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Error (px)')
            plt.legend()
            plt.grid(True)
            plt.savefig('error_plot.png')
            
            # Plot 2 - Commanded Motor Angles
            plt.figure(2, figsize=(10, 6))
            plt.plot(log_time, log_s1, label='Servo 1', color='red', alpha=0.8, linewidth=1)
            plt.plot(log_time, log_s2, label='Servo 2', color='green', alpha=0.8, linewidth=1)
            plt.plot(log_time, log_s3, label='Servo 3', color='blue', alpha=0.8, linewidth=1)
            plt.title('Control Effort: Commanded Motor Angles (Sync Threads)')
            plt.xlabel('Time (s)')
            plt.ylabel('Commanded Angle (Degrees)')
            plt.legend()
            plt.grid(True)
            plt.savefig('angles_plot.png')
            
            # Plot 3 - Absolute Position (Step Response)
            plt.figure(3, figsize=(10, 6))
            plt.plot(log_time, log_goal_x, label='X Target', color='black', linestyle='--', linewidth=1.5)
            plt.plot(log_time, log_goal_y, label='Y Target', color='grey', linestyle='--', linewidth=1.5)
            plt.plot(log_time, log_pos_x, label='X Position', color='blue', alpha=0.7)
            plt.plot(log_time, log_pos_y, label='Y Position', color='red', alpha=0.7)
            plt.title('System Step Response: Position Tracking')
            plt.xlabel('Time (s)')
            plt.ylabel('Position (px from centre)')
            plt.legend()
            plt.grid(True)
            plt.savefig('step_response.png')
            
        else:
            print("No data recorded to plot")
            
        # Plot 3 - Computational Load (Execution Times)
        if len(log_cv_time) > 0:
            plt.figure(4, figsize=(10, 6))
            plt.plot(log_time, log_cv_time, label='Computer Vision Execution', color='purple', alpha=0.8)
            plt.plot(log_time, log_ctrl_time, label='PID and Kinematics Execution', color='orange', alpha=0.8)
            avg_cv = sum(log_cv_time) / len(log_cv_time)
            avg_ctrl = sum(log_ctrl_time) / len(log_ctrl_time)
            plt.title(f'System Execution Time Breakdown: Avg CV: {avg_cv:.2f}ms Avg Control: {avg_ctrl:.2f}ms')
            plt.xlabel('Time (s)')
            plt.ylabel('Execution Time (ms)')
            plt.legend()
            plt.grid(True)
            plt.savefig('execution_times.png')
        
        plt.show()

if __name__ == "__main__":
    main()
