# -*- coding: utf-8 -*-
#!/usr/bin/env python
import rospy
import cv2
from six.moves import urllib
import numpy as np
from numpy import array
from std_msgs.msg import Float32, Int16
import pyzbar.pyzbar as pyzbar

click_pic = 0;


def callback(x):
    global click_pic
    click_pic = x.data

def decodeDisplay(image):
    barcodes = pyzbar.decode(image)
    for barcode in barcodes:
        # 提取条形码的边界框的位置
        # 画出图像中条形码的边界框
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # 条形码数据为字节对象，所以如果我们想在输出图像上
        # 画出来，就需要先将它转换成字符串
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type

        # 绘出图像上条形码的数据和条形码类型
        text = "{} ({})".format(barcodeData, barcodeType)
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (0, 0, 125), 2)

        # 向终端打印条形码数据和条形码类型
        print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
    return image

def detect():

    #denoised_img = cv2.fastNlMeansDenoising(frame, None, 10, 7, 21)     #图像去噪
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)                #将图像转化为灰度图
    #cv2.namedWindow('denoised_img', cv2.WINDOW_GUI_NORMAL)
    #cv2.imshow('denoised_img', frame)
    im = decodeDisplay(gray_img)
    cv2.namedWindow('im_im', cv2.WINDOW_GUI_NORMAL)
    cv2.imshow('im_im', im)


stream=urllib.request.urlopen('http://172.20.10.9/capture')
bts = b''
rospy.Subscriber("take_pic",Int16,callback)
rospy.init_node('bot_cam', anonymous=True)

while not rospy.is_shutdown():

    try:
        bts += stream.read(1024) #.decode('utf-8')

    except:
        print('incomplete read error')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()                    # 键入q，关闭窗口，退出主函数
            break

    else:
        a = bts.find(b'\xff\xd8')
        b = bts.find(b'\xff\xd9')
        
        if a!=-1 and b!=-1:
            jpg = bts[a:b+2]
            bts = bts[b+2:]
            
            try:
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8),1)

            except:
                print('imdecode error')
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                continue

            else:
                cv2.imshow("Frame", frame)
                detect()
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                   break




































