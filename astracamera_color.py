import os
import configparser
import cv2 as cv
import numpy as np
from openni import openni2
from openni import _openni2 as c_api

# 滑动条的回调函数，获取滑动条位置处的值
def empty(a):
    h_min = cv.getTrackbarPos("Hue Min", "TrackBars")
    h_max = cv.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv.getTrackbarPos("Val Max", "TrackBars")
    # print(h_min, h_max, s_min, s_max, v_min, v_max)
    return h_min, h_max, s_min, s_max, v_min, v_max

config = configparser.ConfigParser()
cur_path = os.path.dirname(os.path.realpath(__file__))
config.read(os.path.join(cur_path, "config.ini"))

threshold = list(
            map(int, config.get('using', 'hsv')[1:-1].split(',')))  # 红色阈值

# 创建一个窗口，放置6个滑动条
cv.namedWindow("TrackBars")
cv.resizeWindow("TrackBars",640,360)
cv.createTrackbar("Hue Min","TrackBars",threshold[0],179,empty)
cv.createTrackbar("Hue Max","TrackBars",threshold[1],179,empty)
cv.createTrackbar("Sat Min","TrackBars",threshold[2],255,empty)
cv.createTrackbar("Sat Max","TrackBars",threshold[3],255,empty)
cv.createTrackbar("Val Min","TrackBars",threshold[4],255,empty)
cv.createTrackbar("Val Max","TrackBars",threshold[5],255,empty)

# Init openni
openni_path = os.path.join(cur_path, "openni2_redist/arm64")
openni2.initialize(openni_path)
#openni2.initialize()

# Open astra depth stream (using openni)
depth_device = openni2.Device.open_any()

# Start the color stream
color_stream = depth_device.create_color_stream()
color_stream.start()
color_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX = 640, resolutionY = 480, fps = 30))

while True:
    # Grab a new color frame
    color_frame = color_stream.read_frame()
    color_frame_data = color_frame.get_buffer_as_uint8()

    # Put the color frame into a numpy array, reshape it, and convert from bgr to rgb
    color_img = np.frombuffer(color_frame_data, dtype=np.uint8)
    color_img.shape = (480, 640, 3)
    color_img = color_img[...,::-1]
    color_img = cv.flip(color_img, 1)

    imgHSV = cv.cvtColor(color_img,cv.COLOR_BGR2HSV)
    # 调用回调函数，获取滑动条的值
    h_min,h_max,s_min,s_max,v_min,v_max = empty(0)
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    # 获得指定颜色范围内的掩码
    mask = cv.inRange(imgHSV,lower,upper)

    # 对原图图像进行按位与的操作，掩码区域保留
    imgResult = cv.bitwise_or(color_img,color_img,mask=mask)
   
    cv.imshow("Mask", mask)
    cv.imshow("Result", imgResult)
    
    key = cv.waitKey(10) & 0xFF
    if int(key) == ord('q'):
        openni2.unload()
        cv.destroyAllWindows()
        break
    elif int(key) == ord('s'):
        print("save config")
        config.set('using', 'hsv', str((h_min,h_max,s_min,s_max,v_min,v_max)))
        with open(os.path.join(cur_path, "config.ini"), 'w') as f:
            config.write(f)
