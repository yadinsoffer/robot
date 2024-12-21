#!/usr/bin/python3
# coding=utf8
import sys
import os
import time
import signal
from mecanum_movements import MecanumMovements

robot = MecanumMovements()
start = True

def Stop(signum, frame):
    global start
    start = False
    print('Stopping...')
    robot.stop()

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    while start:
        print("Moving left at medium speed...")
        # For mecanum wheels, to move left:
        # Front left motor: backward
        # Front right motor: forward
        # Rear left motor: forward
        # Rear right motor: backward
        speed = 50  # Medium speed
        robot.board.set_motor_duty([
            [1, -speed],    # Front left (backward)
            [2, speed],     # Front right (forward)
            [3, speed],     # Rear left (forward)
            [4, -speed]     # Rear right (backward)
        ])
        time.sleep(4)
        break

    robot.stop()
    print('Stopped') 