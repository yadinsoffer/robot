#!/usr/bin/python3
import cv2
import mediapipe as mp
import time
from mecanum_movements import MecanumMovements
import os

# Disable GUI requirements
os.environ["QT_QPA_PLATFORM"] = "offscreen"

class GestureController:
    def __init__(self):
        self.robot = MecanumMovements()
        print("Initializing camera...")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera!")
        print("Camera initialized successfully")
        
        # Initialize mediapipe hand detection
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # Movement state
        self.current_movement = None
        
        # Gesture stability tracking
        self.last_gesture = None
        self.gesture_start_time = 0
        self.GESTURE_CONFIRMATION_TIME = 0.5  # Changed from 2.0 to 0.5 seconds
        self.last_print_time = 0  # To control debug print frequency
        
        # Make sure robot is stopped at start
        self.robot.stop()
        
    def count_fingers(self, hand_landmarks):
        """Count number of raised fingers"""
        finger_tips = [8, 12, 16, 20]  # Index, middle, ring, pinky tip indices
        finger_mids = [6, 10, 14, 18]  # Mid points of fingers
        thumb_tip = 4
        thumb_mid = 3
        count = 0
        
        # Check thumb separately - compare with thumb base
        thumb_base = hand_landmarks.landmark[2]
        thumb_tip_point = hand_landmarks.landmark[thumb_tip]
        
        # For right hand, thumb is up if tip is more right (x is less) than base
        # For left hand, thumb is up if tip is more left (x is more) than base
        if abs(thumb_tip_point.x - thumb_base.x) > 0.05:  # Add threshold to prevent jitter
            count += 1
            
        # Check other fingers by comparing tip position with middle knuckle
        for tip, mid in zip(finger_tips, finger_mids):
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[mid].y:
                count += 1
                
        return count
    
    def process_gesture(self, finger_count):
        """Convert finger count to robot movement"""
        current_time = time.time()
        
        # Print debug info every 0.5 seconds
        if current_time - self.last_print_time >= 0.5:
            print(f"Current gesture: {finger_count} fingers")
            if self.last_gesture == finger_count:
                time_held = current_time - self.gesture_start_time
                print(f"Holding for: {time_held:.1f} seconds")
            self.last_print_time = current_time
        
        # If this is a new gesture or different from the last one
        if self.last_gesture != finger_count:
            print(f"\nNew gesture detected: {finger_count} fingers")
            self.last_gesture = finger_count
            self.gesture_start_time = current_time
            return  # Wait for gesture to stabilize
        
        # Check if gesture has been held long enough
        if current_time - self.gesture_start_time >= self.GESTURE_CONFIRMATION_TIME:
            if finger_count == 5 and self.current_movement != "forward":
                print("\n>>> Five fingers held for 0.5 seconds - Moving Forward <<<")
                self.robot.stop()
                time.sleep(0.1)
                self.robot.move_forward()
                self.current_movement = "forward"
                
            elif finger_count == 1 and self.current_movement != "backward":
                print("\n>>> One finger held for 0.5 seconds - Moving Backward <<<")
                self.robot.stop()
                time.sleep(0.1)
                self.robot.move_backward()
                self.current_movement = "backward"
                
            elif finger_count not in [1, 5] and self.current_movement is not None:
                print("\n>>> Invalid gesture - Stopping <<<")
                self.robot.stop()
                self.current_movement = None
    
    def run(self):
        print("\nStarting gesture control. Press Ctrl+C to quit.")
        print("Waiting for hand gestures...")
        print("Show 5 fingers for forward, 1 finger for backward")
        print("Hold gesture steady for 0.5 seconds to activate\n")
        
        try:
            while self.cap.isOpened():
                success, image = self.cap.read()
                if not success:
                    print("Failed to read camera frame")
                    continue

                # Convert to RGB for MediaPipe
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = self.hands.process(image)
                
                if results.multi_hand_landmarks:
                    if self.last_gesture is None:
                        print("\nHand detected!")
                    for hand_landmarks in results.multi_hand_landmarks:
                        finger_count = self.count_fingers(hand_landmarks)
                        self.process_gesture(finger_count)
                else:
                    if self.last_gesture is not None:
                        print("\nHand lost from view")
                    # Reset gesture tracking when no hand is detected
                    self.last_gesture = None
                    self.gesture_start_time = 0
                    
                    # Stop the robot if it was moving
                    if self.current_movement is not None:
                        print(">>> Stopping movement - no hand detected <<<")
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
    controller = GestureController()
    controller.run() 