#!/usr/bin/python3
import cv2
import mediapipe as mp
import time
from mecanum_movements import MecanumMovements
import os
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import socketserver
import io
from PIL import Image

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write("""
                <html>
                <head>
                    <title>Robot Gesture Control View</title>
                    <style>
                        body { text-align: center; background-color: #f0f0f0; }
                        img { max-width: 100%; height: auto; margin: 20px 0; }
                    </style>
                </head>
                <body>
                    <h1>Robot Gesture Control View</h1>
                    <img src="/stream.mjpg" />
                    <p>Show 5 fingers for forward, 1 finger for backward</p>
                </body>
                </html>
            """.encode())
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    if hasattr(self.server, 'frame') and self.server.frame is not None:
                        _, jpeg = cv2.imencode('.jpg', self.server.frame)
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(jpeg))
                        self.end_headers()
                        self.wfile.write(jpeg.tobytes())
                        self.wfile.write(b'\r\n')
                    else:
                        time.sleep(0.1)
            except Exception as e:
                print(f"Streaming error: {e}")
                pass

class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True
    frame = None

class GestureController:
    def __init__(self):
        self.robot = MecanumMovements()
        print("Initializing camera...")
        
        # DroidCam settings
        camera_url = 'http://192.168.12.228:4747/video'
        print(f"Connecting to DroidCam at {camera_url}")
        
        self.cap = cv2.VideoCapture(camera_url)
        
        if not self.cap.isOpened():
            raise RuntimeError("Failed to connect to DroidCam! Make sure:\n"
                             "1. DroidCam app is running on your iPhone\n"
                             "2. iPhone and Raspberry Pi are on the same network\n"
                             "3. The IP address (192.168.12.228) and port (4747) are correct")
        
        # Test camera connection
        ret, test_frame = self.cap.read()
        if ret:
            print("\nDroidCam connected successfully!")
            print(f"Resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
        else:
            raise RuntimeError("Could not read frame from DroidCam")
            
        print("Camera initialized successfully")
        
        # Initialize streaming server
        self.server = StreamingServer(('', 8000), StreamingHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        print("\nCamera stream available at http://raspberrypi:8000")
        
        # Initialize mediapipe hand detection
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
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
        self.GESTURE_CONFIRMATION_TIME = 0.5
        self.last_print_time = 0
        
        # Make sure robot is stopped at start
        self.robot.stop()
        
    def count_fingers(self, hand_landmarks):
        """Count number of raised fingers"""
        # Finger indices
        finger_tips = [8, 12, 16, 20]  # Index, middle, ring, pinky tips
        finger_pips = [6, 10, 14, 18]  # Second joints (PIP)
        finger_mcps = [5, 9, 13, 17]   # Knuckles (MCP)
        thumb_tip = 4
        thumb_ip = 3    # Thumb IP joint
        thumb_mcp = 2   # Thumb MCP joint
        wrist = 0
        count = 0
        
        # Get wrist position for reference
        wrist_y = hand_landmarks.landmark[wrist].y
        
        # Check thumb
        # Thumb is up if its tip is significantly to the side of the IP joint
        thumb_tip_point = hand_landmarks.landmark[thumb_tip]
        thumb_ip_point = hand_landmarks.landmark[thumb_ip]
        thumb_mcp_point = hand_landmarks.landmark[thumb_mcp]
        
        # Calculate thumb angle
        thumb_angle = abs(thumb_tip_point.x - thumb_mcp_point.x)
        if thumb_angle > 0.1:  # Threshold for thumb being clearly extended
            count += 1
            
        # Check other fingers
        # A finger is considered raised if:
        # 1. The tip is higher than both PIP and MCP joints
        # 2. The tip is significantly higher than the wrist
        for tip, pip, mcp in zip(finger_tips, finger_pips, finger_mcps):
            tip_y = hand_landmarks.landmark[tip].y
            pip_y = hand_landmarks.landmark[pip].y
            mcp_y = hand_landmarks.landmark[mcp].y
            
            # Finger must be clearly extended (tip higher than other joints)
            # and raised significantly above the wrist
            if tip_y < pip_y and tip_y < mcp_y and (wrist_y - tip_y) > 0.1:
                count += 1
                
        return count
    
    def process_gesture(self, finger_count):
        """Convert finger count to robot movement"""
        current_time = time.time()
        
        if current_time - self.last_print_time >= 0.5:
            print(f"Current gesture: {finger_count} fingers")
            if self.last_gesture == finger_count:
                time_held = current_time - self.gesture_start_time
                print(f"Holding for: {time_held:.1f} seconds")
            self.last_print_time = current_time
        
        if self.last_gesture != finger_count:
            print(f"\nNew gesture detected: {finger_count} fingers")
            self.last_gesture = finger_count
            self.gesture_start_time = current_time
            return
        
        if current_time - self.gesture_start_time >= self.GESTURE_CONFIRMATION_TIME:
            if finger_count == 5 and self.current_movement != "forward":
                print("\n>>> Five fingers held - Moving Forward <<<")
                self.robot.stop()
                time.sleep(0.1)
                self.robot.move_forward()
                self.current_movement = "forward"
                
            elif finger_count == 1 and self.current_movement != "backward":
                print("\n>>> One finger held - Moving Backward <<<")
                self.robot.stop()
                time.sleep(0.1)
                self.robot.move_backward()
                self.current_movement = "backward"
                
            elif finger_count not in [1, 5] and self.current_movement is not None:
                print("\n>>> Invalid gesture - Stopping <<<")
                self.robot.stop()
                self.current_movement = None
    
    def draw_status(self, image, results):
        """Draw hand landmarks and status on frame"""
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_draw.DrawingSpec(color=(0,255,0), thickness=2),
                    self.mp_draw.DrawingSpec(color=(0,0,255), thickness=2)
                )
                
                # Draw finger count
                finger_count = self.count_fingers(hand_landmarks)
                cv2.putText(image, f"Fingers: {finger_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                
        # Draw movement status
        status = f"Movement: {self.current_movement if self.current_movement else 'Stopped'}"
        cv2.putText(image, status, (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        
        if self.last_gesture is not None:
            hold_time = time.time() - self.gesture_start_time
            cv2.putText(image, f"Hold time: {hold_time:.1f}s", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        
        return image
    
    def run(self):
        print("\nStarting gesture control. Press Ctrl+C to quit.")
        print("Waiting for hand gestures...")
        print("Show 5 fingers for forward, 1 finger for backward")
        print("Hold gesture steady for 0.5 seconds to activate\n")
        print("View the camera feed at http://raspberrypi:8000")
        
        try:
            while self.cap.isOpened():
                success, image = self.cap.read()
                if not success:
                    print("Failed to read camera frame")
                    continue

                # Convert to RGB for MediaPipe
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = self.hands.process(image_rgb)
                
                if results.multi_hand_landmarks:
                    if self.last_gesture is None:
                        print("\nHand detected!")
                    for hand_landmarks in results.multi_hand_landmarks:
                        finger_count = self.count_fingers(hand_landmarks)
                        self.process_gesture(finger_count)
                else:
                    if self.last_gesture is not None:
                        print("\nHand lost from view")
                    self.last_gesture = None
                    self.gesture_start_time = 0
                    
                    if self.current_movement is not None:
                        print(">>> Stopping movement - no hand detected <<<")
                        self.robot.stop()
                        self.current_movement = None

                # Draw status and update stream
                image = self.draw_status(image, results)
                self.server.frame = image
                
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nStopping on keyboard interrupt...")
                
        finally:
            self.cleanup()
    
    def cleanup(self):
        print("\nCleaning up...")
        self.robot.stop()
        self.cap.release()
        self.server.shutdown()
        print("Cleanup complete")

if __name__ == '__main__':
    controller = GestureController()
    controller.run() 