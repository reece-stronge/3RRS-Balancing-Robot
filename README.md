# 3-RRS Ball Balancing Robot 

<video src="https://github.com/user-attachments/assets/cb008f77-76ea-4a3e-9e36-dd4379848b34" width="100" controls></video>

## 🎯 Overview

This repository contains the software and other relevant materials from my MEng final year project, a 3-RRS moving platform that autonomously balances a ball and tracks dynamic setpoints in real time. To achieve this, the project combines inverse kinematics with mechanical design, classical control theory, real-time computer vision and software development, which culminated in a multi-threaded, event-synchronised software architecture that uses PID control to correct the ball's position using camera feedback.

## ⚙️ Hardware and Dependencies
|No|Component|Quantity|Purpose|
|---|---|---|---|
|1|Savox SA-1258TG Coreless Digital Servo|3|Actuators|
|2|PCA9685 Servo Driver|1|Servo PWM Control
|3|Raspberry Pi 5|1|Host Controller|
|4|Raspberry Pi 5 Active Cooler|1|Cooling|
|5|Raspberry Pi Camera Module 3 Wide|1|Object Detection with CV|
|6|22-way 0.5mm-pitch to 15-way 1mm-pitch FPC Camera Cable|1|Camera module to Pi 5 MIPI connection|
|7|SA5T/K M5x0.8 Rod End Bearing|3|Spherical Joints|
|8|625RS 5x16x5 (5mm I.D.) Ball Bearing|3|Revolute Joints|
|9|6V, 5A External Power Supply|1|Servo Power|

### Software Dependencies
- Python, OpenCV, NumPy, Matplotlib, picamera2, Adafruit ServoKit, pynput, threading

## 📏 CAD Files
All original bodies found in the CAD directory were newly designed in Autodesk Fusion and can be 3D printed/lasercut.  
Note the lower links were specifically designed to fit the particular servo horns used for this project.

## 🖥️ MATLAB Kinematics Simulation
Separate functions define the sets of equations to calculate the necessary motor angle for each leg individually based on a desired orientation.  
Desired platform pose is defined as its centre height, tilt magnitude and tilt direction in the main solver script, which calculates the necessary motor angles and plots them in 3D vector space to ensure a physically logical pose.  
This static pose generation was directly translated from MATLAB into the Python environment on the Raspberry Pi in inverse_kinematics.py

## ⌨️ Keyboard Manual Control
    keyboard_control.py
Provides open-loop, discrete manual control of the platform's orientation using threaded keyboard listening to map the positional variables to pre-defined keystrokes:  
Height: W/S | Tilt Magnitude: ↑/↓ | Tilt Direction: ←/→  
Listening for keyboard input on a separate background thread from the kinematics solver allows them to run in parallel and not interfere with one another.

## ⭕ Fixed Trajectory Following
    fixed_trajectory.py
Commands the platform to continuously follow a pre-defined trajectory - tilt direction is continuously incremented through 360 degrees while maintaining a fixed height and tilt magnitude.  
Calculates the execution time of the kinematics when running continuously.

## 👁️ Computer Vision Calibration
    camera_tuning.py
A tuning environment that provides a GUI with trackbars for the user to calibrate the necessary HSV colour thresholds to detect the ball, along with preview windows for the subsequent binary mask and contour bounding box computed by the OpenCV algorithms. 

## 🎯 Main Balancing Program
    main.py | pose.py | pid.py | camera.py
The core application of the project, responsible for the real-time closed-loop stabilisation of the ball. To separate the concerns the system is modularised into four distinct components:
- `main.py`: The main execution loop that manages event-driven thread synchronisation, executes the main control loop and handles end-of-run data logging and plotting.
- `camera.py`: Interfaces with the camera module to execute the OpenCV colour masking and contour detection algorithms.
- `pid.py`: Houses the PID control theory to compute the necessary tilt magnitude and direction and applies EMA low-lass filtering (currently disabled for this project via `a = 1` in `main.py`).
- `pose.py`: Translates the required control effort into physical actuation via the inverse kinematics solver and I2C servo commands.
### Configuration Notes
- **Colour Thresholds:** The HSV colour bounds used to detect the ball are hardcoded within the `Camera` class in `camera.py`. The correct thresholds for the current lighting environment/ball colour can be identified using `camera_tuning.py` and the arrays in `camera.py` can be updated accordingly.
- **Step Response Testing:** To analyse dynamic tracking performance, a predefined step response can be triggered by setting the `ENABLE_STEP_RESPONSE` boolean to `True` within `main.py`. The specific target pixel coordinates and timing intervals for the sequence are manually configured inside the main loop.

## 🚀 Installation and Usage
**1. Create and activate a virtual environment**  
This project was run inside a Python virtual environment to prevent package conflicts on the Raspberry Pi. This is now the required standard for Raspberry Pi S Bookworm which the Raspberry Pi 5 runs. Within a terminal:  
```
python3 -m venv --system-site-packages ~/.venv
source ~/.venv/bin/activate
```
**2. Install dependencies**  
With the virtual environment activated, install the required packages e.g:
```
pip install adafruit-circuitpython-servokit
```
_(picamera2 is typically pre-installed on modern Raspberry Pi OS releases, but may require specific system configuration to enable)_  
  
**3. Usage**  
With the hardware connected and virtual environment activated, the scripts can be opened and executed directly within a Python IDE like Thonny that comes pre-installed on the Raspberry Pi OS.

## PyBullet Physics Engine Simulation
```
balance_sim.py
```
A "digital twin" constructed in the PyBullet rigid-body physics engine for system validation. It uses approximations of the physics of a ping-pong ball rolling on an acrylic plate along with a 1:1 mirror of the `PID` class in `pid.py` and the measured end-to-end hardware latency of the real system to isolate the performance of the controller. Different initial conditions as well as a step response can be enabled by adjusting the relevant variables.  
  
<img width="332" height="322" alt="Screenshot 2026-04-24 222939" src="https://github.com/user-attachments/assets/6a99f404-2183-42c0-a005-51d1ede3bb7f" />
