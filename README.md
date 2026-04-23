# 3-RRS Ball Balancing Robot 

<video src="https://github.com/user-attachments/assets/cb008f77-76ea-4a3e-9e36-dd4379848b34" width="100" controls></video>

## 🎯 Overview

This repository contains the software and other relevant materials from my MEng final year project, a 3-RRS moving platform that autonomously balances a ball and tracks dynamic setpoints in real time. To achieve this, the project combines inverse kinematics with mechanical design, classical control theory, real-time computer vision and software development, which culminated in a multi-threaded, event-synchronised software architecture that uses PID control to correct the ball's position using camera feedback.

## 📏 CAD Files
All original bodies found in the CAD directory were newly designed in Autodesk Fusion and can be 3D printed/lasercut.  
Note the lower links were specifically designed to fit the particular servo horns used for this project.

## ⌨️ Keyboard Manual Control
    keyboard_control.py
Provides open-loop, discrete manual control of the platform's orientation using threaded keyboard listening to map the positional variables to pre-defined keystrokes:  
Height: W/S | Tilt Magnitude: ↑/↓ | Tilt Direction: ←/→  
Listening for keyboard input on a separate background thread from the kinematics solver allows them to run in parallel and not interfere with one another.

## ⭕ Fixed Trajectory Following
    fixed_trajectory.py
Commands the platform to continuously follow a pre-defined trajectory - tilt direction is continuously incremented through 360 degrees while maintaining a fixed height and tilt magnitude.  
Calculates the execution time of the kinematics in real-time.

## 👁️ Computer Vision Calibration
    camera_tuning.py
A tuning environment that provides a GUI with trackbars for the user to calibrate the necessary HSV colour thresholds to detect the ball, along with preview windows for the subsequent binary mask and contour bounding box computed by the OpenCV algorithms. 
