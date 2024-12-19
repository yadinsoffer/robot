#!/usr/bin/python3
import cv2
import numpy as np
import time
import os
from mecanum_movements import MecanumMovements

class PersonFollower:
    def __init__(self):
        self.robot = MecanumMovements()
        print("Initializing camera...")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera!")
        print("Camera initialized successfully")
        
        # Initialize HOG descriptor for person detection
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        print("Person detector initialized")
        
        # Camera frame dimensions
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_center = self.frame_width // 2
        
        # Movement parameters
        self.current_movement = None
        self.MIN_PERSON_HEIGHT = 200  # Minimum height for person detection
        self.FOLLOW_DISTANCE = 300    # Target height of person in pixels
        self.CENTER_THRESHOLD = 100   # Acceptable distance from center
        self.speed = 40              # Base movement speed
        
        # Make sure robot is stopped at start
        self.robot.stop()
        
    def detect_person(self, frame):
        """Detect largest person in frame and return their bounding box"""
        # Detect people in the frame
        boxes, weights = self.hog.detectMultiScale(
            frame, 
            winStride=(8, 8),
            padding=(4, 4),
            scale=1.05
        )
        
        if len(boxes) == 0:
            return None
            
        # Find the largest detection (closest person)
        largest_box = max(boxes, key=lambda x: x[2] * x[3])
        return largest_box
        
    def follow_person(self, box):
        """Adjust robot movement based on person position"""
        x, y, w, h = box
        person_center = x + w//2
        person_height = h
        center_offset = person_center - self.frame_center
        
        print(f"Person height: {person_height}, Center offset: {center_offset}")
        
        # Determine forward/backward movement based on person height
        if person_height < self.FOLLOW_DISTANCE - 50:  # Too far
            print("Moving forward - person too far")
            self.robot.move_forward(self.speed)
            self.current_movement = "forward"
        elif person_height > self.FOLLOW_DISTANCE + 50:  # Too close
            print("Moving backward - person too close")
            self.robot.move_backward(self.speed)
            self.current_movement = "backward"
        else:
            if self.current_movement is not None:
                print("Stopping - good distance")
                self.robot.stop()
                self.current_movement = None
    
    def run(self):
        print("\nStarting person follower. Press Ctrl+C to quit.")
        print("Stand in front of the camera to be detected")
        print("The robot will maintain distance and follow you\n")
        
        try:
            while self.cap.isOpened():
                success, frame = self.cap.read()
                if not success:
                    print("Failed to read camera frame")
                    continue
                
                # Detect person
                box = self.detect_person(frame)
                
                if box is not None:
                    print("\nPerson detected!")
                    self.follow_person(box)
                else:
                    if self.current_movement is not None:
                        print("No person detected - stopping")
                        self.robot.stop()
                        self.current_movement = None
                
                # Add a small delay to prevent CPU overload
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nStopping on keyboard interrupt...")
                
        finally:
            self.cleanup()
    
    def cleanup(self):
        print("\nCleaning up...")
        self.robot.stop()
        self.cap.release()
        print("Cleanup complete")

if __name__ == '__main__':
    follower = PersonFollower()
    follower.run() 