#!/usr/bin/python3
import cv2
import numpy as np
import time
import os
from mecanum_movements import MecanumMovements
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
                    <title>Robot Camera View</title>
                </head>
                <body>
                    <h1>Robot Camera View</h1>
                    <img src="/stream" style="width:640px;height:480px"/>
                </body>
                </html>
            """.encode())
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    if hasattr(self.server, 'frame') and self.server.frame is not None:
                        # Encode frame as JPEG
                        _, jpeg = cv2.imencode('.jpg', self.server.frame)
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', len(jpeg))
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

class PersonFollower:
    def __init__(self):
        self.robot = MecanumMovements()
        print("Initializing camera...")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera!")
        print("Camera initialized successfully")
        
        # Initialize streaming server
        self.server = StreamingServer(('', 8000), StreamingHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        print("\nCamera stream available at http://raspberrypi:8000")
        
        # Initialize upper body detection
        self.body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_upperbody.xml')
        print("Upper body detector initialized")
        
        # Camera frame dimensions
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_center = self.frame_width // 2
        
        # Movement parameters
        self.current_movement = None
        
        # Position thresholds - using vertical position in frame
        self.VERTICAL_TOO_CLOSE = self.frame_height // 3  # Top third of frame
        self.VERTICAL_TOO_FAR = (self.frame_height * 2) // 3  # Bottom third of frame
        
        # Smoothing parameters
        self.last_valid_y = None
        self.consecutive_detections = 0
        self.consecutive_losses = 0
        self.REQUIRED_CONSECUTIVE_DETECTIONS = 3
        self.MAX_CONSECUTIVE_LOSSES = 5
        
        # Make sure robot is stopped at start
        self.robot.stop()
        
    def detect_person(self, frame):
        """Detect upper body in frame and return bounding box"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect upper bodies with different scale parameters
        bodies = self.body_cascade.detectMultiScale(
            gray,
            scaleFactor=1.05,      # Smaller scale factor for more detections
            minNeighbors=2,        # More lenient detection
            minSize=(60, 60),      # Minimum size for upper body
            maxSize=(400, 400)     # Maximum size for upper body
        )
        
        if len(bodies) == 0:
            return None
            
        # Get the largest upper body detection
        largest_body = max(bodies, key=lambda x: x[2] * x[3])
        return largest_body
        
    def is_valid_position_change(self, new_y):
        """Check if position change is reasonable"""
        if self.last_valid_y is None:
            return True
        
        # Don't allow more than 20% change in vertical position between detections
        max_change = self.frame_height * 0.2
        return abs(new_y - self.last_valid_y) <= max_change
        
    def follow_person(self, box):
        """Adjust robot movement based on face position"""
        x, y, w, h = box
        face_y = y + h//2  # Vertical center of face
        face_center = x + w//2  # Horizontal center of face
        
        print(f"Face vertical position: {face_y}px (0=top, {self.frame_height}=bottom)")
        
        # Check if position change is valid
        if not self.is_valid_position_change(face_y):
            print("Invalid position change detected - ignoring")
            self.consecutive_detections = 0
            return
            
        self.last_valid_y = face_y
        
        # Movement based on vertical position in frame
        if face_y > self.VERTICAL_TOO_FAR:  # Face is low in frame = too far
            self.consecutive_detections += 1
            if self.consecutive_detections >= self.REQUIRED_CONSECUTIVE_DETECTIONS:
                print("Moving forward - person too far (face low in frame)")
                self.robot.move_forward()
                self.current_movement = "forward"
        elif face_y < self.VERTICAL_TOO_CLOSE:  # Face is high in frame = too close
            self.consecutive_detections += 1
            if self.consecutive_detections >= self.REQUIRED_CONSECUTIVE_DETECTIONS:
                print("Moving backward - person too close (face high in frame)")
                self.robot.move_backward()
                self.current_movement = "backward"
        else:
            if self.current_movement is not None:
                print("Stopping - good distance")
                self.robot.stop()
                self.current_movement = None
            self.consecutive_detections = 0
    
    def draw_status(self, frame, box=None):
        """Draw status information on frame"""
        # Draw zone lines
        cv2.line(frame, (0, self.VERTICAL_TOO_CLOSE), (self.frame_width, self.VERTICAL_TOO_CLOSE), 
                (255, 0, 0), 2)  # Too close line
        cv2.line(frame, (0, self.VERTICAL_TOO_FAR), (self.frame_width, self.VERTICAL_TOO_FAR), 
                (255, 0, 0), 2)  # Too far line
        
        if box is not None:
            x, y, w, h = box
            # Draw a thick box around the upper body
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            
            # Draw body center point and crosshair
            body_y = y + h//2
            body_center = x + w//2
            cv2.circle(frame, (body_center, body_y), 6, (0, 255, 0), -1)
            cv2.line(frame, (body_center - 15, body_y), (body_center + 15, body_y), (0, 255, 0), 2)
            cv2.line(frame, (body_center, body_y - 15), (body_center, body_y + 15), (0, 255, 0), 2)
            
            # Draw position indicator with background
            text = f"Body pos: {body_y}/{self.frame_height}"
            cv2.putText(frame, text, (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(frame, text, (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw size indicator
            size_text = f"Size: {h}px"
            cv2.putText(frame, size_text, (x, y+h+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(frame, size_text, (x, y+h+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw movement status
        status = f"Movement: {self.current_movement if self.current_movement else 'Stopped'}"
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Draw detection counter
        detections = f"Detections: {self.consecutive_detections}/{self.REQUIRED_CONSECUTIVE_DETECTIONS}"
        cv2.putText(frame, detections, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(frame, detections, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Add zone labels
        cv2.putText(frame, "Too Close", (10, self.VERTICAL_TOO_CLOSE - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(frame, "Too Far", (10, self.VERTICAL_TOO_FAR + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        return frame
    
    def run(self):
        print("\nStarting person follower. Press Ctrl+C to quit.")
        print("Stand in front of the camera to be detected")
        print("The robot will maintain distance and follow you\n")
        print("View the camera feed at http://raspberrypi:8000")
        
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
                    self.consecutive_losses = 0
                    self.follow_person(box)
                else:
                    self.consecutive_losses += 1
                    self.consecutive_detections = 0
                    if self.consecutive_losses >= self.MAX_CONSECUTIVE_LOSSES:
                        if self.current_movement is not None:
                            print("Lost person - stopping")
                            self.robot.stop()
                            self.current_movement = None
                            self.last_valid_y = None
                
                # Draw status and update stream
                frame = self.draw_status(frame, box)
                self.server.frame = frame
                
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
    follower = PersonFollower()
    follower.run() 