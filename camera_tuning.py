import cv2
import numpy as np
from picamera2 import Picamera2

def nothing(x):
    pass

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"size": (640, 640)},
    sensor={"output_size": (2592, 2592)}
)
picam2.configure(config)

picam2.set_controls({"AfMode": 2})

picam2.start()

cv2.namedWindow("Tuning")
cv2.createTrackbar("L-H", "Tuning", 140, 255, nothing)
cv2.createTrackbar("L-S", "Tuning", 100, 255, nothing)
cv2.createTrackbar("L-V", "Tuning", 100, 255, nothing)
cv2.createTrackbar("U-H", "Tuning", 179, 255, nothing)
cv2.createTrackbar("U-S", "Tuning", 255, 255, nothing)
cv2.createTrackbar("U-V", "Tuning", 255, 255, nothing)

try:
    while True:
        frame = picam2.capture_array()
        
        display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        lh = cv2.getTrackbarPos("L-H", "Tuning")
        ls = cv2.getTrackbarPos("L-S", "Tuning")
        lv = cv2.getTrackbarPos("L-V", "Tuning")
        uh = cv2.getTrackbarPos("U-H", "Tuning")
        us = cv2.getTrackbarPos("U-S", "Tuning")
        uv = cv2.getTrackbarPos("U-V", "Tuning")
        
        lower_bound = np.array([lh, ls, lv])
        upper_bound = np.array([uh, us, uv])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + (w // 2)
                
                cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(display_frame, (cx, y + h//2), 5, (0, 0, 255), -1)
                
        cv2.imshow("Mask (Black and White)", mask)
        cv2.imshow("Result (Detection)", display_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"Final Bounds: Lower[{lh}, {ls}, {lv}], Upper[{uh}, {us}, {uv}]")
            break
        
finally:
    picam2.stop()
    cv2.destroyAllWindows()