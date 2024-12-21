#!/usr/bin/python3
import sys
import os
import time
import HiwonderSDK.ros_robot_controller_sdk as rrc

class MecanumMovements:
    def __init__(self):
        self.board = rrc.Board()
        self.default_speed = 80
        
    def stop(self):
        """Stop all motors"""
        self.board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])
        
    def move_forward(self, speed=None):
        """
        Move robot forward with verified motor configuration
        :param speed: Motor speed (0-100), defaults to 80 if not specified
        """
        if speed is None:
            speed = self.default_speed
            
        self.board.set_motor_duty([
            [1, speed],     # Front left
            [2, speed],     # Front right
            [3, -speed],    # Rear left (inverted)
            [4, -speed]     # Rear right (inverted)
        ])
        
    def move_backward(self, speed=None):
        """
        Move robot backward with verified motor configuration
        :param speed: Motor speed (0-100), defaults to 80 if not specified
        """
        if speed is None:
            speed = self.default_speed
            
        self.board.set_motor_duty([
            [1, -speed],    # Front left (inverted)
            [2, -speed],    # Front right (inverted)
            [3, speed],     # Rear left
            [4, speed]      # Rear right
        ])
        
    def set_speed(self, new_speed):
        """Change the default speed (0-100)"""
        self.default_speed = max(0, min(100, new_speed))  # Clamp between 0-100
        
    def move_left(self, speed=None):
        """
        Move robot left using the mecanum chassis velocity control
        :param speed: Motor speed (0-100), defaults to 80 if not specified
        """
        if speed is None:
            speed = self.default_speed
            
        # Use mecanum chassis velocity control: speed, direction angle (180 = left), no rotation
        self.board.set_motor_duty([
            [1, speed],     # Front left
            [2, -speed],    # Front right
            [3, speed],     # Rear left
            [4, -speed]     # Rear right
        ])

# Example usage
if __name__ == '__main__':
    robot = MecanumMovements()
    
    try:
        print("Moving forward for 4 seconds...")
        robot.move_forward()
        time.sleep(4)
        
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        robot.stop()
        print("Motors stopped") 