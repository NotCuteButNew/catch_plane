#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import time
import threading
import numpy as np
import apriltag
import HiwonderSDK.Board as Board
import HiwonderSDK.PID as PID
import HiwonderSDK.Misc as Misc
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

#apriltag检测

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

x_dis = 60
y_dis = 0 
Z_DIS = 18
z_dis = Z_DIS
x_pid = PID.PID(P=0.1, I=0.05, D=0.008)  # pid初始化
y_pid = PID.PID(P=0.00001, I=0, D=0)
z_pid = PID.PID(P=0.005, I=0, D=0)
grab_dis = 25
cur_times = 0
cur_x_dis=x_dis
x_offest=-20

def grabMove():
    print("DEBUG grabMove")
    cur_target = AK.setPitchRange((0,round(y_dis+grab_dis, 2), round(z_dis, 2)), -90, 0)
    servo_data = cur_target[0]
    Board.setBusServoPulse(1, 200, 800)
    Board.setBusServoPulse(3, servo_data['servo3'], 1000)
    Board.setBusServoPulse(4, servo_data['servo4'], 1000)
    Board.setBusServoPulse(5, servo_data['servo5'], 1000)
    Board.setBusServoPulse(6, int(x_dis+x_offest), 1000)
    time.sleep(1)
    Board.setBusServoPulse(1, 600, 800)
    time.sleep(1)
    AK.setPitchRangeMoving((30,0,12),0,-90,0,2000)
    time.sleep(2)
    AK.setPitchRangeMoving((0,15,12),0,-90,0,2000)
    time.sleep(2)
    AK.setPitchRangeMoving((0,30,6),0,-90,0,1000)
    Board.setBusServoPulse(1, 200, 800)
    cap.release()
    cv2.destroyAllWindows()
    sys.exit()

# 初始位置
def initMove():
    print("DEBUG: initMove")
    AK.setPitchRangeMoving((10,0,18),0,-90,0,3000)
    time.sleep(3)
    
    
st = True
object_center_x = 0.0
object_center_y = 0.0
object_area = 0.0
check_is_grab=0
# 检测apriltag
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):
    global object_area
    global object_center_x, object_center_y
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)

    if len(detections) != 0:
        for detection in detections:                       
            corners = np.rint(detection.corners)  # 获取四个角点
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)
            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family
            tag_id = int(detection.tag_id)  # 获取tag_id
            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点  
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角
            object_area = abs(int((corners[3][1] - corners[0][1]) * (corners[3][0] - corners[0][0])))
            print(object_area)
            return tag_family, tag_id

            
    return None, None

def run(img):
    global st 
    global state
    global tag_id
    global object_area
    global action_finish
    global x_dis, y_dis, z_dis
    global cur_x_dis, cur_y_dis, cur_z_dis#cur_ current
    global object_center_x, object_center_y
    global cur_times
    
    img_h, img_w = img.shape[:2]
    tag_family, tag_id = apriltagDetect(img) # apriltag检测
    
    if tag_id is not None:
        x_pid.SetPoint = img_w / 2.0  # 设定
        x_pid.update(object_center_x)  # 当前
        dx = x_pid.output
        x_dis += int(dx)  # 输出

        x_dis = 0 if x_dis < 0 else x_dis
        x_dis = 1000 if x_dis > 1000 else x_dis

        y_pid.SetPoint = 15000  # 设定
        if abs(object_area - 15000) < 50:
            object_area = 15000
        y_pid.update(object_area)  # 当前
        dy = y_pid.output
        y_dis += dy  # 输出
        y_dis = 5.00 if y_dis < 5.00 else y_dis
        y_dis = 10.00 if y_dis > 10.00 else y_dis
        
        if abs(object_center_y - img_h/2.0) < 20:
            z_pid.SetPoint = object_center_y
        else:
            z_pid.SetPoint = img_h / 2.0
            
        z_pid.update(object_center_y)
        dy = z_pid.output
        z_dis += dy

        z_dis = 32.00 if z_dis > 32.00 else z_dis
        z_dis = 10.00 if z_dis < 10.00 else z_dis
        
        target = AK.setPitchRange((0, round(y_dis, 2), round(z_dis, 2)), -90, 0)
        if cur_times == 0:
            cur_x_dis=x_dis
        cur_times = cur_times + 1;
            
        if target:
            servo_data = target[0]
            if st:
                print("***********st is true************\n")
                print("x_dis: ",x_dis,",y_dis: ",y_dis,",z_dis: ",z_dis)
                Board.setBusServoPulse(3, servo_data['servo3'], 1000)
                Board.setBusServoPulse(4, servo_data['servo4'], 1000)
                Board.setBusServoPulse(5, servo_data['servo5'], 1000)
                Board.setBusServoPulse(6, int(x_dis), 1000)
                time.sleep(1)
                st = False
            else:
                print("***********st is false************\n")
                print("x_dis: ",x_dis,",y_dis: ",y_dis,",z_dis: ",z_dis)
                Board.setBusServoPulse(3, servo_data['servo3'], 20)
                Board.setBusServoPulse(4, servo_data['servo4'], 20)
                Board.setBusServoPulse(5, servo_data['servo5'], 20)
                Board.setBusServoPulse(6, int(x_dis), 20)
                time.sleep(0.03)
            if cur_times > 3:
                if cur_x_dis - x_dis < 5:
                    print("**********grab********\n")
                    time.sleep(1)
                    grabMove()
                else:
                    cur_times=0


                    
        cv2.putText(img, "tag_id: " + str(tag_id), (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: " + tag_family, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
    else:
        cv2.putText(img, "tag_id: None", (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: None", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)



    return img

if __name__ == '__main__':
    initMove()
    cap = cv2.VideoCapture(-1) #读取摄像头

    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
