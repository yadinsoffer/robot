#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import signal
import Camera
import threading
import yaml_handle
import mediapipe as mp
import HiwonderSDK.PID as PID
import HiwonderSDK.mecanum as mecanum

# 人脸追踪
board = None
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
 
car = mecanum.MecanumChassis()
# 导入人脸识别模块
Face = mp.solutions.face_detection
# 自定义人脸识别方法，最小的人脸检测置信度0.5
faceDetection = Face.FaceDetection(min_detection_confidence=0.5)

servo1 = 1500
servo2 = 1500
servo_x = servo2
servo_y = servo1

size = (640, 480)
__isRunning = False
center_x, center_y, area = -1, -1, 0

car_x_pid = PID.PID(P=0.150, I=0.001, D=0.0001)
car_y_pid = PID.PID(P=0.002, I=0.001, D=0.0001)
servo_x_pid = PID.PID(P=0.1, I=0.0000, D=0.0000)  # pid初始化
servo_y_pid = PID.PID(P=0.1, I=0.000, D=0.000)

servo_data = None
def load_config():
    global servo_data
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


# 初始位置
def initMove():
    board.pwm_servo_set_position(1, [[1, servo1], [2, servo2]])

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# 关闭电机
def car_stop():
    car.set_velocity(0,90,0)  # 关闭所有电机

# 变量重置
def reset():
    global servo1, servo2
    global servo_x, servo_y
    global center_x, center_y, area
    
    servo1 = servo_data['servo1'] - 350
    servo2 = servo_data['servo2']
    servo_x = servo2
    servo_y = servo1
    car_x_pid.clear()
    car_y_pid.clear()
    servo_x_pid.clear()
    servo_y_pid.clear()
    center_x, center_y, area = -1, -1, 0
    

# app初始化调用
def init():
    print("FaceTracking Init")
    load_config()
    reset()
    initMove()

# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("FaceTracking Start")

# app停止玩法调用
def stop():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    print("FaceTracking Stop")

# app退出玩法调用
def exit():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    print("FaceTracking Exit")

# 机器人移动逻辑处理
car_en = False
def move():
    global __isRunning,car_en
    global servo_x, servo_y
    global center_x, center_y, area
    
    img_w, img_h = size[0], size[1]
    
    while True:
        if __isRunning:
            if center_x != -1 and center_y != -1:
                # 摄像头云台追踪
                # 根据摄像头X轴坐标追踪
                if abs(center_x - img_w/2.0) < 15: # 移动幅度比较小，则不需要动
                    center_x = img_w/2.0
                servo_x_pid.SetPoint = img_w/2.0 # 设定
                servo_x_pid.update(center_x)     # 当前
                servo_x += int(servo_x_pid.output)  # 获取PID输出值
                
                servo_x = 800 if servo_x < 800 else servo_x # 设置舵机范围
                servo_x = 2200 if servo_x > 2200 else servo_x
                
                # 根据摄像头Y轴坐标追踪
                if abs(center_y - img_h/2.0) < 10: # 移动幅度比较小，则不需要动
                    center_y = img_h/2.0
                servo_y_pid.SetPoint = img_h/2.0  
                servo_y_pid.update(center_y)
                servo_y -= int(servo_y_pid.output) # 获取PID输出值
                
                servo_y = 1000 if servo_y < 1000 else servo_y # 设置舵机范围
                servo_y = 1900 if servo_y > 1900 else servo_y
                # print(servo_y, center_y) 
                board.pwm_servo_set_position(0.02, [[1, servo_y], [2, servo_x]])  # 设置舵机移动
                # 车身跟随追踪
                # 根据目标大小进行远近追踪
                if abs(area - 30000) < 2000 or servo_y < 1100:
                    car_y_pid.SetPoint = area
                else:
                    car_y_pid.SetPoint = 30000
                car_y_pid.update(area)
                dy = car_y_pid.output   # 获取PID输出值
                dy = 0 if abs(dy) < 20 else dy # 设置速度范围
                
                # 根据X轴舵机值进行追踪
                if abs(servo_x - servo2) < 15:
                    car_x_pid.SetPoint = servo_x
                else:
                    car_x_pid.SetPoint = servo2
                car_x_pid.update(servo_x)
                dx = car_x_pid.output   # 获取PID输出值
                dx = 0 if abs(dx) < 20 else dx # 设置速度范围
                                
                car.translation(dx, dy) # 设置机器人移动（X轴速度，Y轴速度）
                car_en = True
                
                time.sleep(0.02)
            else:
                if car_en:
                    car_stop()
                    car_en = False
                time.sleep(0.01)
        else:
            if car_en:
                car_stop()
                car_en = False
            time.sleep(0.01)

# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# 机器人图像处理
def run(img):
    global __isRunning, area
    global center_x, center_y
    global center_x, center_y, area
    
    if not __isRunning:   # 检测是否开启玩法，没有开启则返回原图像
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
     
    imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB) # 将BGR图像转为RGB图像
    results = faceDetection.process(imgRGB) # 将每一帧图像传给人脸识别模块
    if results.detections:   # 如果检测不到人脸那就返回None
        for index, detection in enumerate(results.detections): # 返回人脸索引index(第几张脸)，和关键点的坐标信息
            bboxC = detection.location_data.relative_bounding_box # 设置一个边界框，接收所有的框的xywh及关键点信息
            
            # 将边界框的坐标点,宽,高从比例坐标转换成像素坐标
            bbox = (int(bboxC.xmin * img_w), int(bboxC.ymin * img_h),  
                   int(bboxC.width * img_w), int(bboxC.height * img_h))
            cv2.rectangle(img, bbox, (0,255,0), 2)  # 在每一帧图像上绘制矩形框
            x, y, w, h = bbox  # 获取识别框的信息,xy为左上角坐标点
            center_x =  int(x + (w/2))
            center_y =  int(y + (h/2))
            area = int(w * h)
    else:
        center_x, center_y, area = -1, -1, 0
            
    return img


#关闭前处理
def manual_stop(signum, frame):
    global __isRunning
    
    print('关闭中...')
    __isRunning = False
    car_stop()  # 关闭所有电机
    initMove()  # 舵机回到初始位置

if __name__ == '__main__':
    import HiwonderSDK.ros_robot_controller_sdk as rrc
    board = rrc.Board()
    init()
    start()
    camera = Camera.Camera()
    camera.camera_open(correction=True) # 开启畸变矫正,默认不开启
    signal.signal(signal.SIGINT, manual_stop)
    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240)) # 画面缩放到320*240
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
