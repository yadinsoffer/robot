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
    print('关闭中...')
    robot.stop()

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    while start:
        print("Moving forward at high speed...")
        robot.move_forward()
        time.sleep(4)
        break

    robot.stop()
    print('已关闭')
        
