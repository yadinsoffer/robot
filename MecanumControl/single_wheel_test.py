#!/usr/bin/python3
import sys
import os
import time
import HiwonderSDK.ros_robot_controller_sdk as rrc

# Initialize the board
board = rrc.Board()

try:
    # Test only motor 1 (front left wheel)
    print("Moving only motor 1 (front left wheel)")
    print("Press Ctrl+C to stop")
    
    while True:
        board.set_motor_duty([[1, 6]])  # Very low speed on motor 1 only
        time.sleep(0.1)  # Small delay to prevent CPU overload

except KeyboardInterrupt:
    print("\nStopping motor...")
    board.set_motor_duty([[1, 0]])  # Stop the motor
    print("Motor stopped") 