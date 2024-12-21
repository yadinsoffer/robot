#!/usr/bin/python3
import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import numpy as np
import threading
from http import server
import socketserver
import io
import logging
import HiwonderSDK.mecanum as mecanum

# Global variable for the latest frame
global_frame = None
condition = threading.Condition()

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/stream.mjpg')
            self.end_headers()
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with condition:
                        condition.wait()
                        frame = global_frame
                    ret, jpeg = cv2.imencode('.jpg', frame)
                    if not ret:
                        continue
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(jpeg))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(f'Removed streaming client {self.client_address}: {str(e)}')
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

class MotionFollower:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.chassis = mecanum.MecanumChassis()
        
        # Configuration
        self.frame_width = 640
        self.center_threshold = 100
        self.distance_threshold = 100000
        
        # Red detection parameters
        self.min_area = 2000
        # HSV ranges for red (need two ranges because red wraps around in HSV)
        self.lower_red1 = np.array([0, 100, 100])     # First red range (0-10)
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])   # Second red range (160-180)
        self.upper_red2 = np.array([180, 255, 255])
        
    def detect_red_object(self, frame):
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create masks for both red ranges
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        
        # Combine the masks
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
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
            # Draw rectangle around the red object
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Draw center point
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

                red_object_data, annotated_frame = self.detect_red_object(frame)
                
                # Update global frame for streaming
                global global_frame
                with condition:
                    global_frame = annotated_frame.copy()
                    condition.notify_all()
                
                if red_object_data is not None:
                    center_x, _, area = red_object_data
                    frame_center = self.frame_width // 2
                    
                    # Fixed movement logic
                    if area < self.distance_threshold * 0.8:  # Far away
                        # Need to move forward
                        if abs(center_x - frame_center) > self.center_threshold:
                            # Need to center first
                            if center_x > frame_center:
                                self.chassis.set_velocity(30, 90, 0)  # Move right
                            else:
                                self.chassis.set_velocity(30, 270, 0)  # Move left
                        else:
                            # Centered and far - move forward
                            self.chassis.set_velocity(30, 0, 0)  # Move forward (0 degrees)
                    elif area > self.distance_threshold * 1.8:  # Too close
                        # Move backward
                        self.chassis.set_velocity(30, 180, 0)  # Move backward (180 degrees)
                    else:
                        # Good distance, just adjust horizontal position
                        if center_x > frame_center + self.center_threshold:
                            self.chassis.set_velocity(30, 90, 0)  # Move right
                        elif center_x < frame_center - self.center_threshold:
                            self.chassis.set_velocity(30, 270, 0)  # Move left
                        else:
                            self.chassis.set_velocity(0, 0, 0)  # Stop
                else:
                    # No red object detected, stop
                    self.chassis.set_velocity(0, 0, 0)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.cap.release()
            self.chassis.set_velocity(0, 0, 0)

def start_server():
    address = ('', 8000)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()

if __name__ == '__main__':
    # Start the streaming server in a separate thread
    server_thread = threading.Thread(target=start_server)
    server_thread.daemon = True
    server_thread.start()
    
    # Start the motion follower
    follower = MotionFollower()
    follower.follow() 