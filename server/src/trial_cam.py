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
from std_msgs.msg import String, Int16, Float32
import copy
import math
import pyzbar.pyzbar as pyzbar


largestArea=75000.0

click_pic = "";


def callback(x):
    global click_pic
    click_pic = x.data

def decodeDisplay(image):
    barcodes = pyzbar.decode(image)

    # judge if there is qrcode in the image
    if barcodes == []:
        print "No barcodes"
        flag_qr = 0
        return image, flag_qr,0

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


        return image, flag_qr, barcodeData

def detect_qr(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                #将图像转化为灰度图
    #cv2.namedWindow('denoised_img', cv2.WINDOW_GUI_NORMAL)
    #cv2.imshow('denoised_img', frame)
    im, flag_qr, barcodeData = decodeDisplay(gray_img)
    return flag_qr, barcodeData


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
    _, contours, hierachy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return image, contours, hierachy

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

def get_edge(img,contours):
    for c in contours:

        # get the bounding rect
        area = cv2.contourArea(c)


        if(area>largestArea):
                print area
                # get the bounding rect
                x, y, w, h = cv2.boundingRect(c)
                # draw a green rectangle to visualize the bounding rect
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # get the min area rect
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                # convert all coordinates floating point values to int
                box = np.int0(box)
                # draw a red 'nghien' rectangle
                cv2.drawContours(img, [box], 0, (0, 0, 255))

    cv2.drawContours(img, contours, -1, (255, 255, 0), 1)

    #cv2.imshow("contours", img)
    #cv2.waitKey(0)


    return box,1

def url_to_image(url):
    resp = urllib.request.urlopen(url)
    print('got url')
    image = np.asarray(bytearray(resp.read()), dtype = "uint8")
    print('got byte')
    image = cv2.imdecode(image,cv2.IMREAD_COLOR)

    return image

""" Main starts here"""

pub1 = rospy.Publisher('correction_val', Float32 ,queue_size=10)
pub2 = rospy.Publisher('qr_number',Int16, queue_size=10)
rospy.Subscriber("take_pic",String,callback)
rospy.init_node('cam_processor', anonymous=True)
print('rospy initiated')


frame = url_to_image('http://192.168.0.159:8080/?action=snapshot')
   
flag_qr, barcodeData = detect_qr(frame)
barcode_number = int(barcodeData)

if flag_qr == 1:
    #Publish barcodeData number
    pub2.publish(barcode_number)
    image, contours, hierachy = detect_cnt(frame)
    hierachy_sq = np.squeeze(hierachy)
    
    edge,flag_clarity= get_edge(image,contours)

    if flag_clarity == 0:
        print "Flag 0"
    else:
        delta_deg = calculate_devi_deg(edge)      # calculate the obviate degree
        print('delta_deg = ', delta_deg)
        error_deg = float(delta_deg)
        pub1.publish(error_deg)
        
        
else:
        print('no qr code')







































