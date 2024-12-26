#!/usr/bin/python3
import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import numpy as np
from ultralytics import YOLO
from mecanum_movements import MecanumMovements
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import socketserver

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write("""
                <html>
                <head>
                    <title>Robot Person Following View</title>
                    <style>
                        body { text-align: center; background-color: #f0f0f0; }
                        img { max-width: 100%; height: auto; margin: 20px 0; }
                    </style>
                </head>
                <body>
                    <h1>Robot Person Following View</h1>
                    <img src="/stream.mjpg" />
                    <p>Press Ctrl+C in terminal to stop</p>
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

class MotionFollower:
    def __init__(self):
        camera_url = 'http://192.168.12.228:4747/video'
        print(f"Connecting to iPhone camera at {camera_url}")
        
        self.cap = cv2.VideoCapture(camera_url)
        
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open iPhone camera stream! Make sure:\n"
                             "1. DroidCam is running on your iPhone\n"
                             "2. iPhone and Raspberry Pi are on the same network\n"
                             "3. The IP address is correct")
        
        print("Successfully connected to iPhone camera")
        
        # Initialize YOLOv8 model
        print("Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt')  # Using the nano model for better performance
        print("YOLOv8 model loaded successfully")
        
        # Initialize robot movement controller
        self.robot = MecanumMovements()
        
        # Configuration
        self.frame_width = 640
        self.center_threshold = 100
        self.distance_threshold = 100000  # Will be adjusted based on person detection
        
        # Movement state tracking
        self.current_movement = None
        
        # Initialize streaming server
        self.server = StreamingServer(('', 8000), StreamingHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        print("\nCamera stream available at http://raspberrypi:8000")

    def detect_person(self, frame):
        # Run YOLOv8 inference
        results = self.model(frame, conf=0.5, classes=[0])  # class 0 is person in COCO dataset
        
        if len(results) > 0 and len(results[0].boxes) > 0:
            # Get the first detected person (highest confidence)
            box = results[0].boxes[0]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            confidence = float(box.conf[0])
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Calculate center and area
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            area = (x2 - x1) * (y2 - y1)
            
            # Draw center point
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # Add confidence text
            cv2.putText(frame, f"Conf: {confidence:.2f}", (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add movement state
            if self.current_movement:
                cv2.putText(frame, f"Movement: {self.current_movement}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            return (center_x, center_y, area), frame
        
        return None, frame

    def move(self, movement_type):
        """Helper function to change movement state and execute movement"""
        if self.current_movement != movement_type:
            self.current_movement = movement_type
            if movement_type == "forward":
                self.robot.move_forward()
            elif movement_type == "backward":
                self.robot.move_backward()
            elif movement_type == "left":
                self.robot.move_left()
            elif movement_type == "right":
                self.robot.move_right()
            elif movement_type == "stop":
                self.robot.stop()
            print(f"Movement changed to: {movement_type}")

    def follow(self):
        print("\nStarting person following. Press Ctrl+C to quit.")
        print("View the camera feed at http://raspberrypi:8000\n")
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                person_data, annotated_frame = self.detect_person(frame)
                
                # Update the streaming frame
                self.server.frame = annotated_frame
                
                if person_data is not None:
                    center_x, _, area = person_data
                    frame_center = self.frame_width // 2
                    
                    # Movement logic
                    if area < self.distance_threshold * 0.8:  # Person is far away
                        if abs(center_x - frame_center) > self.center_threshold:
                            if center_x > frame_center:
                                self.move("right")
                            else:
                                self.move("left")
                        else:
                            self.move("forward")
                    elif area > self.distance_threshold * 1.8:  # Person is too close
                        self.move("backward")
                    else:
                        if center_x > frame_center + self.center_threshold:
                            self.move("right")
                        elif center_x < frame_center - self.center_threshold:
                            self.move("left")
                        else:
                            self.move("stop")
                else:
                    self.move("stop")
                
                time.sleep(0.1)  # Small delay to prevent CPU overload
                    
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
    follower = MotionFollower()
    follower.follow() 