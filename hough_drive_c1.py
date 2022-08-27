#!/usr/bin/env python
# -*- coding: utf-8 -*-
####################################################################
# 프로그램명 : hough_drive_c1.py
# 작 성 자 : (주)자이트론, S2Y (2022 대상)
# 생 성 일 : 2020년 07월 23일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, rospkg, time
import numpy as np
import cv2, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray
from math import *
import signal
import sys
import os

def signal_handler(sig, frame):#close exit kill switch
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None
Width = 320
Height = 240
Offset = 170 #sanghansun
Gap = 60 #hahansun 160+40
Cnt = 0

cam = False
cam_debug = True

sub_f = 0
time_c = 0

def img_callback(data):
    global image   
    global sub_f 
    global time_c

    sub_f += 1
    if time.time() - time_c > 1:
        #print("pub fps :", sub_f)
        time_c = time.time()
        sub_f = 0

    image = bridge.imgmsg_to_cv2(data, "bgr8")
    
def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

# publish xycar_motor msg
def drive(Angle, Speed): 
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), (0, 255, 0), 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 2, 7 + offset),
                       (lpos + 2, 12 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 2, 7 + offset),
                       (rpos + 2, 12 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-2, 7 + offset),
                       (center+2, 12 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (157, 7 + offset),
                       (162, 12 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 20

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if low_slope_threshold < abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []
    th = -20 #sonbayaham 

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - th):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + th):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines
#2:01
# get average m, b of line, sum of x, y, mget lpos, rpos
def get_slope(llines, rlines):
    lm_sum=0.0
    rm_sum=0.0
    
    lsize = len(llines)
    rsize = len(rlines)
    
    for line in llines:
            xl1, yl1, xl2, yl2 = line[0]
            lm_sum += float(yl2 - yl1) / float(xl2 - xl1)
    
    for line in rlines:
            xr1, yr1, xr2, yr2 = line[0]
            rm_sum += float(yr2 - yr1) / float(xr2 - xr1)
    
    if (lsize==0):
        lsize=1
    if (rsize==0):
        rsize=1
    if (lm_sum==0):
        lm_sum=100
    if(rm_sum==0):
        rm_sum=100
    
    lm_av = lm_sum / lsize
    rm_av = rm_sum / rsize
    
    if (lsize==0) and (rsize==0):
        result_x=1
        return result_x
    else:
        return (abs(lm_av)-abs(rm_av))
    
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap, cam_debug

    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    
    m = 0
    b = 0

    if size != 0:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2) #2:03
        y_avg = y_sum / (size * 2)

        m = m_sum / size
        b = y_avg - m * x_avg

    if m == 0 and b == 0:
        if left:
            pos = 0
        elif right:
            pos = Width
    else:
        y = Gap / 2

        pos = (y - b) / m

        if cam_debug:
            b += Offset
            xs = (Height - b) / float(m)
            xe = ((Height/2) - b) / float(m)

            cv2.line(img, (int(xs), Height), (int(xe), (Height/2)), (255, 0,0), 3)

    return img, int(pos)
##################################################################################
#####################################################################################

def stop(all_lines, flag, line_count):#정지선 인식 함수
    line_len=all_lines
    print("all_lines",line_len)
    
    if (line_count == 1) and (line_len > 49): #출발후 1바퀴 돌고-> 1번째 바퀴 완주전 2번-> 2번째 바퀴 인식하고 정지
        flag=0
        line_count = 2
        return line_count, flag
    
    if (line_len > 49): #출발후  1바퀴 돌고
        flag = 1
        print("flag up")
        print("flag up")
        
    if(line_len < 43) and (flag==1): #출발후 1바퀴 돌고-> 1번째 바퀴 완주전 2번
        line_count += 1
        flag = 0
        print("count up")

    return line_count, flag
        
######################################################################
#####################################################################
# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    global cam, cam_debug, img

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    roi = gray[Offset : Offset+Gap, 0 : Width]
    roi2 = gray[Offset-Gap+20 : Offset, 100 : Width-100]

    # blur
    kernel_size = 5
    standard_deviation_x = 3     #Kernel standard deviation along X-axis
    blur_gray = cv2.GaussianBlur(roi, (kernel_size, kernel_size), standard_deviation_x)
    blur_gray2 = cv2.GaussianBlur(roi2, (kernel_size, kernel_size), standard_deviation_x)

    # canny edge
    low_threshold = 170
    high_threshold = 200
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold, kernel_size)
    '''edge_img2 = cv2.Canny(np.uint8(blur_gray2), 70, high_threshold, kernel_size)'''

#     cv2.imshow('edge_img', edge_img)
    '''cv2.imshow('edge_img2', edge_img2)'''

    # HoughLinesP
    #all_lines = cv2.HoughLinesP(edge_img, 1, math.pi/180,30,30,10)
    all_lines = cv2.HoughLinesP(edge_img, 0.7, math.pi/180,8,28,2)
    '''all_lines2 = cv2.HoughLinesP(edge_img2, 0.7, math.pi/180,8,16,2)
    all_lines3 = cv2.HoughLinesP(edge_img2, 0.7, math.pi/180,20,20,2)'''
##########################################################################
##########################################################################

###################################################################
##################################################################
    if cam:
        cv2.imshow('calibration', frame)
    # divide left, right lines
    if all_lines is None:
        return (Width)/2, (Width)/2, False
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    if cam_debug:
        # draw lines
        frame = draw_lines(frame, left_lines)
        frame = draw_lines(frame, right_lines)
        frame = cv2.line(frame, (115, 117), (205, 117), (0,255,255), 2)
        #frame = cv2.rectangle(frame, (60, Offset), (Width-30, Offset+Gap), (0, 255, 0), 2)
       
        # draw rectangle
        frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
        frame = cv2.rectangle(frame, (0, Offset), (Width, Offset+Gap), (0, 255, 0), 2)

    img = frame        
    #slope = abs(get_slope(left_lines, right_lines))
    return lpos, rpos, len(all_lines),True
###################################################################
###################################################################

def draw_steer(steer_angle):
    global Width, Height, img
    #img = cv_image

    arrow = cv2.imread('/home/pi/xycar_ws/src/auto_drive/src/steer_arrow.png')

    origin_Height = arrow.shape[0]
    origin_Width = arrow.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (-steer_angle) * 1.5, 0.7)    
    arrow = cv2.warpAffine(arrow, matrix, (origin_Width+60, origin_Height))
    arrow = cv2.resize(arrow, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = img[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow)
    img[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res

    cv2.imshow('steer', img)

def pid_angle(ITerm, error, b_angle, b_error, Cnt):
    angle = 0
    Kp = 0.5 #0.5 good
    Ki = 0.0001 #0.0001 good #0.0002
    Kd = 1.0 #1.0 good #2.0
    dt = 1

    PTerm = Kp * error
    ITerm += Ki * error * dt
    derror = error - b_error
    DTerm = Kd * (derror / dt)
    angle = PTerm + ITerm + DTerm

    return angle, ITerm

def start():
    global motor
    global image
    global Width, Height

    rospy.init_node('auto_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)

    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    print ("---------- Xycar C1 HD v1.0 ----------")
    time.sleep(1)#3

    #sq = rospy.Rate(30)
    rospy.Rate(100)
    t_check = time.time()
    f_n = 0
    p_angle = 0
    flag = 0
    line_count = 0
    b_angle = 0
    b_error = 0
    ITerm = 0
    avoid_time =time.time() + 3.8
    Cnt = 0
    turn_right = time.time()
    stop_time = time.time() + 10.5
    while not rospy.is_shutdown():
        while not image.size == (Width*Height*3):
            continue

################### ultra_sonic ultra node
        if ultra_msg == None:
            continue
        
        #print(ultra_msg)
#         time.sleep(0.5)
################### 

        f_n += 1
        if (time.time() - t_check) > 1:
            #print("fps : ", f_n)
            t_check = time.time()
            f_n = 0
        
        draw_img = image.copy()
        try:
            lpos, rpos, len_all_lines, go = process_image(draw_img)
        except:
            lpos, rpos, go = process_image(draw_img)
        
        if time.time() > stop_time:
            line_count, flag = stop(len_all_lines, flag, line_count)
         #stop
        if (line_count==2):# 라인 카운트 2개시 정지 
            drive(0,-16)
            cv2.waitKey(0)
            line_count = 0

        center = ((lpos + rpos) / 2)
        original_angle = -((Width/2 - center))
        error = (center - Width/2)
        angle, ITerm = pid_angle(ITerm, error, b_angle, b_error, Cnt)
        
        if lpos == 0 and rpos == 320:
            angle = 70
            drive(angle, 21)
        else:
##################  avoid car
                        
            if time.time() > avoid_time and Cnt == 0:
                Cnt = 1
                print("------------------------CNT: ",Cnt)
                print("------------------------CNT: ",Cnt)
                
            if (ultra_msg[2] < 75 or ultra_msg[3] < 60) and Cnt == 1:
                Cnt = 2
#                 avoid_drive_left()
                max_time_end = time.time() + 0.30
                while True:
                    drive(-70,3)
                    if time.time() > max_time_end:
                        break

                max_time_end = time.time() + 0.68 #start(True)
                while True:
                    drive(-70,21)
                    if time.time() > max_time_end:
                        break
                
                max_time_end = time.time() + 0.5  # changed line and to be stable
                while True:
                    drive(50,19)
                    if time.time() > max_time_end:
                        break
                turn_right = time.time() + 1.3

            if ultra_msg[3] >100 and Cnt == 2 and time.time() > turn_right :
                max_time_end = time.time() + 0.6    #go back to the line
                while True:
                    drive(45,18)
                    if time.time() > max_time_end:
                        break
                    
                max_time_end = time.time() + 0.5    #go back to the line
                while True:
                    drive(-50,19)
                    if time.time() > max_time_end:
                        break   
                
                Cnt = 3
                continue
################## 
            if Cnt == 3:
                ang = angle * 0.8
                drive(angle,22)
            else:
                drive(angle, 21)
        
        steer_angle = angle * 0.4
        draw_steer(steer_angle)
        
        cv2.waitKey(1)
        #sq.sleep()
        b_angle =angle
        b_error = error

if __name__ == '__main__':
    start()

