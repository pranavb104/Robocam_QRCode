# -*- coding: utf-8 -*-
"""
Created on Fri Aug  9 15:21:06 2019

@author: zzh
"""
#!/usr/bin/env python
import rospy
import cv2
from six.moves import urllib
import numpy as np
from numpy import array
from std_msgs.msg import Int16
from server.msg import Correction
import copy
import math
import pyzbar.pyzbar as pyzbar

"""parameters"""
threshold_line = 10;
threshold_area = 15;
threshold_ratio_out = 1.2
threshold_ratio_in =  2.0

click_pic = 0;


def callback(x):
    global click_pic
    click_pic = x.data

def decodeDisplay(image):
    barcodes = pyzbar.decode(image)

    # judge if there is qrcode in the image
    if barcodes == []:
        flag_qr = 0
        return image, flag_qr

    else:
        flag_qr = 1
        for barcode in barcodes:
            # mark the edge of the qrcodes in the image
            (x, y, w, h) = barcode.rect
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # get the data an type of the qrcode
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type

            """ display the data and type of the qrcode on the image """
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0, 0, 125), 2)

            """ print the info in the terminal """
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return image, flag_qr

def detect_qr(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                #将图像转化为灰度图
    #cv2.namedWindow('denoised_img', cv2.WINDOW_GUI_NORMAL)
    #cv2.imshow('denoised_img', frame)
    im, flag_qr = decodeDisplay(gray_img)
    return flag_qr

def load_cam_bot():
    f = open('esp32_cam_para.txt', 'r')
    a = f.read()
    bot_cam_para = eval(a)
    f.close()
    return bot_cam_para

def calculate_by_mtx(cx, cy, mtx_bot):
    uv_point = np.ones((3,1))
    uv_point[0,0] = cx
    uv_point[1,0] = cy
    K_intrin = np.linalg.inv(mtx_bot)
    Wc_point = np.matmul(K_intrin, uv_point)
    X_W = Wc_point[0, 0] * h_camera
    Y_W = Wc_point[1, 0] * h_camera
    Y_W = -Y_W
    return X_W, Y_W

""" calculate the obviate degree, anti-clockwise is positive """
def calculate_devi_deg(edge):
    corner0 = edge[0, :]
    corner1 = edge[1, :]

    if corner0[1] == corner1[1]:
        delta_deg = 0
    else:
        delta_rad = math.atan((corner0[0] - corner1[0]) / (corner0[1] - corner1[1]))
        delta_deg = delta_rad / 3.1415926 * 180

        if delta_deg > 45:
            delta_deg = delta_deg - 90
        elif delta_deg < -45:
            delta_deg = delta_deg + 90

    return delta_deg

""" using the mtx to calculate position """
def calculate_devi_pos(edge, mtx_bot):
    center_x = np.floor((edge[0, 1] + edge[2, 1]) / 2)
    center_y = np.floor((edge[0, 0] + edge[2, 0]) / 2)
    Xcen_W, Ycen_W = calculate_by_mtx(center_x, center_y, mtx_bot)
    dist = math.sqrt(Xcen_W**2 + Ycen_W**2)

    return dist, Xcen_W, Ycen_W

"""extract all contours"""
def detect_cnt(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY_INV)
    image, contours, hierachy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return image, contours, hierachy

"""the ratio of outer contour and its child """
def compute_1(contours, i, j):
    area1 = cv2.contourArea(contours[i])
    area2 = cv2.contourArea(contours[j])
    
    if area2==0:
        return False

    ratio = area1 * 1.0 / area2

    if abs(ratio - 49.0 / 25):
        return True

    return False

"""the ratio of child and its child"""
def compute_2(contours, i, j):
    area1 = cv2.contourArea(contours[i])
    area2 = cv2.contourArea(contours[j])

    if area2 == 0:
        return False
    ratio = area1 * 1.0 / area2
    if abs(ratio - 25.0 / 9):
        return True

    return False

""" calculate the center point of the contour """
def compute_center(contours,i):
    M = cv2.moments(contours[i])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    return cx,cy

""" calculate the distance among the family contours and judge if it's small enough """
def detect_contours(vec):
    distance_1 = np.sqrt((vec[0]-vec[2])**2+(vec[1]-vec[3])**2)
    distance_2 = np.sqrt((vec[0]-vec[4])**2+(vec[1]-vec[5])**2)
    distance_3 = np.sqrt((vec[2]-vec[4])**2+(vec[3]-vec[5])**2)
    
    if sum((distance_1, distance_2, distance_3)) / 3 < 4:
        return True
    return False

""" judge whether there are three points to form an equilateral right triangle """
def judge_angle(rec):
    if len(rec) < 3:
        return -1, -1, -1
    for i in range(len(rec)):
        for j in range(i+1,len(rec)):
            for k in range(j+1,len(rec)):
                distance_1 = np.sqrt((rec[i][0] - rec[j][0]) ** 2 + (rec[i][1] - rec[j][1]) ** 2)
                distance_2 = np.sqrt((rec[i][0] - rec[k][0]) ** 2 + (rec[i][1] - rec[k][1]) ** 2)
                distance_3 = np.sqrt((rec[j][0] - rec[k][0]) ** 2 + (rec[j][1] - rec[k][1]) ** 2)
                if abs(distance_1 - distance_2) < threshold_line:
                    if abs(np.sqrt(np.square(distance_1)+np.square(distance_2))-distance_3) < threshold_area:
                        return i,j,k
                elif abs(distance_1 - distance_3) < threshold_line:
                    if abs(np.sqrt(np.square(distance_1) + np.square(distance_3)) - distance_2) < threshold_area:
                        return i,j,k
                elif abs(distance_2 - distance_3) < threshold_line:
                    if abs(np.sqrt(np.square(distance_2) + np.square(distance_3)) - distance_1) < threshold_area:
                        return i,j,k
    return -1,-1,-1

""" find eligible contours """
def find_cnt(image, image_name, contours, hier , root = 0):
    
    rec = []

    for i in range(len(hier)):
        #print(shape(hier))
        #print(i)
        child = hier[i][2]
        child_child = hier[child][2]

        if child != -1 and child_child != -1:

            if compute_1(contours, i, child) and compute_2(contours,child,child_child):
                cx1, cy1 = compute_center(contours, i)
                cx2, cy2 = compute_center(contours, child)
                cx3, cy3 = compute_center(contours, child_child)

                if detect_contours([cx1, cy1, cx2, cy2, cx3, cy3]):
                    rec.append([cx1, cy1, cx2, cy2, cx3, cy3, i, child, child_child])       # 三个小正方形外框

    # find eligible center points
    i, j ,k = judge_angle(rec)
    if i == -1 or j == -1 or k == -1:
        print('Blur image')
        flag_clarity = 0
        return 0, 0, flag_clarity

    flag_clarity = 1
    ts = np.concatenate((contours[rec[i][6]], contours[rec[j][6]], contours[rec[k][6]]))
    rect = cv2.minAreaRect(ts)
    box = cv2.boxPoints(rect)
    box = np.int0(box)             # box : red outer contonr
    result = copy.deepcopy(image)
    cv2.drawContours(result, [box], 0, (0, 0, 255), 2)
    cv2.drawContours(image, contours, rec[i][6], (255,0,0), 2)
    cv2.drawContours(image, contours, rec[j][6], (255,0,0), 2)
    cv2.drawContours(image, contours, rec[k][6], (255,0,0), 2)

    """
    cv2.imshow('img', image)
    cv2.waitKey(0)
    cv2.imshow('img', result)
    cv2.waitKey(0)
    """

    return box, rec, flag_clarity

def url_to_image(url):
    resp = urllib.request.urlopen(url)
    print('got url')
    image = np.asarray(bytearray(resp.read()), dtype = "uint8")
    print('got byte')
    image = cv2.imdecode(image,cv2.IMREAD_COLOR)

    return image

""" Main """
img_result = 'results'
h_camera = 20
bot_cam_para = load_cam_bot()
mtx_bot = bot_cam_para['bot_mtx']
dist_bot = bot_cam_para['bot_dist']

pub = rospy.Publisher('correction_val', Correction,queue_size=10)
rospy.Subscriber("take_pic",Int16,callback)
rospy.init_node('cam_processor', anonymous=True)
print('rospy initiated')
error = Correction()

while not rospy.is_shutdown():

    if click_pic == 1:
        frame = url_to_image('http://172.20.10.9/capture')
        click_pic = 0
                               # No error
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        
        frame = cv2.undistort(frame, mtx_bot, dist_bot, None, mtx_bot)
        cv2.namedWindow('undist', cv2.WINDOW_GUI_NORMAL)
        cv2.imshow('undist', frame)
        flag_qr = detect_qr(frame)

        if flag_qr == 1:
            image = frame
            cv2.imshow('image', image)
            image, contours, hierachy = detect_cnt(image)
            hierachy_sq = np.squeeze(hierachy)
            edge, squares, flag_clarity = find_cnt(image, image, contours, hierachy_sq)     #获得外框信息

            if flag_clarity == 0:
                continue
            else:
                delta_deg = calculate_devi_deg(edge)      # calculate the obviate degree
                devi_dist, delta_x, delta_y = calculate_devi_pos(edge, mtx_bot)
                print('delta_deg = ', delta_deg)
                print('devi_dist = ', devi_dist)
                print('delta_x, delta_y = ', delta_x, ', ', delta_y)
                error.delta1 = delta_x
                error.delta2 = delta_y
                pub.publish(error)
                
                
        else:
                print('no qr code')

        cv2.destroyAllWindows()





































