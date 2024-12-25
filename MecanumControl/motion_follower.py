#!/usr/bin/python3
import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import numpy as np
import HiwonderSDK.mecanum as mecanum

class MotionFollower:
    def __init__(self):
        # EpocCam streams video at http://IPHONE_IP:8080/video
        camera_url = 'http://IPHONE_IP:8080/video'
        print(f"Connecting to iPhone camera at {camera_url}")
        
        self.cap = cv2.VideoCapture(camera_url)
        
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open iPhone camera stream! Make sure:\n"
                             "1. EpocCam is running on your iPhone\n"
                             "2. iPhone and Raspberry Pi are on the same network\n"
                             "3. The IP address is correct")
        
        print("Successfully connected to iPhone camera")
        
        self.chassis = mecanum.MecanumChassis()
        
        # Configuration
        self.frame_width = 640
        self.center_threshold = 100
        self.distance_threshold = 100000
        
        # Black detection parameters
        self.min_area = 2000
        self.lower_black = np.array([0, 0, 0])      
        self.upper_black = np.array([180, 50, 50])
        
        # Create window
        cv2.namedWindow('Robot Camera View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Robot Camera View', 640, 480)

    def detect_black_object(self, frame):
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for black
        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)
        
        # Remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=3)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_contour = None
        largest_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area and area > largest_area:
                largest_contour = contour
                largest_area = area
        
        if largest_contour is not None:
            (x, y, w, h) = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_x = x + w//2
            center_y = y + h//2
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            return (center_x, center_y, w * h), frame
        
        return None, frame

    def follow(self):
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                black_object_data, annotated_frame = self.detect_black_object(frame)
                
                # Display the frame
                cv2.imshow('Robot Camera View', annotated_frame)
                
                if black_object_data is not None:
                    center_x, _, area = black_object_data
                    frame_center = self.frame_width // 2
                    
                    # Movement logic
                    if area < self.distance_threshold * 0.8:  # Far away
                        if abs(center_x - frame_center) > self.center_threshold:
                            if center_x > frame_center:
                                self.chassis.set_velocity(30, 90, 0)  # Move right
                            else:
                                self.chassis.set_velocity(30, 270, 0)  # Move left
                        else:
                            self.chassis.set_velocity(30, 0, 0)  # Move forward
                    elif area > self.distance_threshold * 1.8:  # Too close
                        self.chassis.set_velocity(30, 180, 0)  # Move backward
                    else:
                        if center_x > frame_center + self.center_threshold:
                            self.chassis.set_velocity(30, 90, 0)  # Move right
                        elif center_x < frame_center - self.center_threshold:
                            self.chassis.set_velocity(30, 270, 0)  # Move left
                        else:
                            self.chassis.set_velocity(0, 0, 0)  # Stop
                else:
                    self.chassis.set_velocity(0, 0, 0)
                
                # Break loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.cleanup()
    
    def cleanup(self):
        print("\nCleaning up...")
        self.chassis.set_velocity(0, 0, 0)
        self.cap.release()
        cv2.destroyAllWindows()
        print("Cleanup complete")

if __name__ == '__main__':
    follower = MotionFollower()
    follower.follow() 