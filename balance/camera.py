import cv2
import numpy as np
from picamera2 import Picamera2

class Camera:
    def __init__(self):
        self.picam2 = Picamera2()
        self.height = 640
        self.width = 640
        
        self.config = self.picam2.create_video_configuration(
            main={"format": 'RGB888', "size": (self.width, self.height)},
            sensor={"output_size": (2592, 2592)}
        )
        self.picam2.configure(self.config)
        self.picam2.set_controls({"AfMode": 2}) # Lock autofocus

        # --- Colour limits ---
        self.lower_pink = np.array([140, 30, 80])
        self.upper_pink = np.array([200, 255, 255])

        self.picam2.start()

    def take_pic(self):
        image = self.picam2.capture_array()
        return image

    def show_video(self, image):
        img_align = image.copy()
        cv2.line(img_align, (0, 320), (640, 320), (0, 0, 0), thickness=2)
        cv2.line(img_align, (320, 0), (320, 640), (0, 0, 0), thickness=2)
        cv2.circle(img_align, (320, 320), radius=5, color=(0, 255, 255), thickness=-1)
        cv2.imshow("Align", img_align)
        cv2.waitKey(1)

    def find_object(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image_hsv, self.lower_pink, self.upper_pink)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            area = cv2.contourArea(largest_contour)

            if area > 500:
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                
                # --- Centre coordinates ---
                x -= self.width / 2
                y -= self.height / 2
                
                # Bottom-Up Camera Axis Adjustment
                x = -x
                
                return int(x), int(y), int(area)
        
        return -1, -1, 0

    def clean_up_cam(self):
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()
        
        for _ in range(5):
            cv2.waitKey(1)
