import numpy as np
from openni import openni2
from openni import _openni2 as c_api
import cv2 as cv
import configparser
import os
import yaml
import time
import math
from threading import Thread
import serial

cv_ver = cv.__version__.split(".")[0]

# config = configparser.ConfigParser()
cur_path = os.path.dirname(os.path.realpath(__file__))
# config.read(os.path.join(cur_path, "config.ini"))

# threshold = list(
#             map(int, config.get('using', 'red_hsv')[1:-1].split(',')))  # red阈值
# threshold1 = list(
#             map(int, config.get('using', 'red2_hsv')[1:-1].split(',')))  # red1阈值
# threshold2 = list(
#             map(int, config.get('using', 'green_hsv')[1:-1].split(',')))  # green阈值
# threshold3 = list(
#             map(int, config.get('using', 'blue_hsv')[1:-1].split(',')))  # blue阈值

yaml_path = os.path.join(cur_path, "camera_640.yaml")
with open(yaml_path,"r") as file:
    parameter=yaml.load(file.read(),Loader=yaml.Loader)
    camera_matrix=np.array(parameter['camera_matrix']['data'], np.float64).reshape(3, 3)
    distortion_coefficients=np.array(parameter['distortion_coefficients']['data'], np.float32).reshape(-1, 5)
cx = camera_matrix[0][2]
cy = camera_matrix[1][2]
fx = camera_matrix[0][0]
fy = camera_matrix[1][1]

openni_path = os.path.join(cur_path, "openni2_redist/arm64")

fourcc = cv.VideoWriter_fourcc(*'XVID')

def eu_2(a,b):
    distance=np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
    return distance

def order_points(pts):

    xSorted = pts[np.argsort(pts[:, 0]), :]

    # grab the left-most and right-most points from the sorted
    # x-roodinate points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]
    if leftMost[0,1]!=leftMost[1,1]:
        leftMost=leftMost[np.argsort(leftMost[:,1]),:]
    else:
        leftMost=leftMost[np.argsort(leftMost[:,0])[::-1],:]
    (tl, bl) = leftMost
    if rightMost[0,1]!=rightMost[1,1]:
        rightMost=rightMost[np.argsort(rightMost[:,1]),:]
    else:
        rightMost=rightMost[np.argsort(rightMost[:,0])[::-1],:]
    (tr,br)=rightMost
    if eu_2(bl, tl) >  eu_2(tl, tr):
        look = True
    else:
        look = False
    return look, np.array([bl, tl, tr, br], dtype="float32")


# 230mm x 110mm
brick_pts_h = np.float32([[-40, -40, 0], [-40, 40, 0], [40, 40, 0], [40, -40, 0]])
brick_pts_v = np.float32([[-40, -40, 0], [-40, 40, 0], [40, 40, 0], [40, -40, 0]])

colors = ((0, 0, 255), (255, 0, 0), (0, 255, 0), (255, 255, 0))


frame_tail = b'\x0D'
color_start = b'\x1B\x32'
QR_start = b'\x1B\x31'
cir_start = b'\x1B\x33'
pos_start = b'\x1B\x34'
def rece():
    global response, Flag
    #res = None
    
    
    while Flag:
        res = None
        time.sleep(0.3)
        start_time = time.time()
        while Flag:
            #print("thread_1:{}".format(time.time()))
            if ser.inWaiting():
                read_bytes = ser.read(1)
                # print(read_bytes)
                #read_duration = time.time() - start_time
                #if (not read_bytes) or read_duration > 0.1:
                #    break
                #print(read_bytes)
                if res is None:
                    if read_bytes == b'\x1B':
                        res = read_bytes
                else:
                    res += read_bytes
                    if (len(res) == 2):
                        ser.write(b'\x06')
                        break
            
        if res != None:
            print(res)
            if res == b'\x1B\x30':
                response = None
            else:
                response = res
#ser = serial.Serial("COM2", 115200)    # 连接串口
ser = serial.Serial("/dev/XCOM2", 115200)
response = QR_start#None
Flag = True

# 返回坐标 x y 单位mm， angle 单位0.1度
def brick_detection(img):
    height, width = img.shape[:2]
    # img2=img[0:height//2,0:width]
    img2 = img

    hsv_image = cv.cvtColor(img2, cv.COLOR_BGR2HSV)
    #color_start = b'\x1B\x32'    QR_start = b'\x1B\x31'        cir_start = b'\x1B\x33'    pos_start = b'\x1B\x34'
    
    config = configparser.ConfigParser()
    cur_path = os.path.dirname(os.path.realpath(__file__))
    config.read(os.path.join(cur_path, "config.ini"))

    threshold = list(
                map(int, config.get('using', 'red_hsv')[1:-1].split(',')))  # red阈值
    threshold1 = list(
                map(int, config.get('using', 'red2_hsv')[1:-1].split(',')))  # red1阈值
    threshold2 = list(
                map(int, config.get('using', 'green_hsv')[1:-1].split(',')))  # green阈值
    threshold3 = list(
                map(int, config.get('using', 'blue_hsv')[1:-1].split(',')))  # blue阈值
    
    if response==QR_start: 
        up1_ = np.array(threshold[1::2])
        lower1_ = np.array(threshold[0::2])
        mask1 = cv.inRange(hsv_image, lower1_, up1_)
        up2_ = np.array(threshold1[1::2])
        lower2_ = np.array(threshold1[0::2])
        mask2 = cv.inRange(hsv_image, lower2_, up2_)
        mask = cv.bitwise_or(mask1,mask2)
    elif response==color_start:    
        up_ = np.array(threshold2[1::2])
        lower_ = np.array(threshold2[0::2])
        mask = cv.inRange(hsv_image, lower_, up_)
    elif response==cir_start:     
        up_ = np.array(threshold3[1::2])
        lower_ = np.array(threshold3[0::2])
        mask = cv.inRange(hsv_image, lower_, up_)
    else:
        mask = cv.inRange(hsv_image, 0, 0)



    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(5,5))
    morp = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    morp = cv.morphologyEx(morp, cv.MORPH_CLOSE, kernel)
    # cv.imshow("morp", morp)

    out_x = 0
    out_y = 0
    out_angle = 0
    origin_distance = 999999
    show_box = None
    ret = False
    if cv_ver == "4":
        contours, hierarchy = cv.findContours(morp, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    elif cv_ver == "3":
        _, contours, hierarchy = cv.findContours(morp, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    
    if len(contours) >0:
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < 5000:
                continue

            cv.drawContours(img2, cnt, -1, (0, 255, 0), 3)
            box = cv.minAreaRect(cnt)
            l, w= box[1]
            angle = box[2]
            if w > l:
                l, w = w, l
            lw_ratio = (l / w)
            # print(lw_ratio)

            if True: 
            # if 0.7<= lw_ratio <= 1.3:
                box_pst = cv.boxPoints(box)
                box_pst = np.array(box_pst, dtype="int")
                cv.drawContours(img2, [box_pst], -1, (0, 0, 255), 2)
                # cv.putText(img2, str(angle), np.int32(box[0]), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

                look, rect = order_points(box_pst)

                rect.astype("int")
                for ((x, y), color) in zip(rect, colors):
                    cv.circle(img2, (int(x), int(y)), 5, color, -1)

                brick_pts = brick_pts_h if look else brick_pts_v

                _, rvecs, tvecs = cv.solvePnP(brick_pts, np.float32(rect), camera_matrix, distortion_coefficients)
                cv.drawFrameAxes(img2, camera_matrix, distortion_coefficients, rvecs, tvecs, 100)

                rvec_matrix = cv.Rodrigues(rvecs)[0]
                proj_matrix = cv.hconcat((rvec_matrix, tvecs))
                eulerAngles = -cv.decomposeProjectionMatrix(proj_matrix)[6]
                yaw = eulerAngles[1][0]
                pitch = eulerAngles[0][0]
                roll = eulerAngles[2][0]
                rot_params = np.array([yaw, pitch, roll])


                position = -np.matrix(rvec_matrix).T * np.matrix(tvecs)
                

                # x_ = -position[0, 0]
                # y_ = -position[1, 0]
                # x_ = box[0][0] - (width / 2)
                # y_ = (height / 2) - box[0][1]

                px, py = (box[0][0] - cx) / fx * position[2, 0], (cy- box[0][1]) / fy * position[2, 0]
                x_ = -px
                y_ = py
                # cv.putText(img2, str((px, py)), (0, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                # cv.putText(img2, str(rot_params), (0, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                # cv.putText(img2, str(tvecs), (0, 60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                # cv.putText(img2, str(position), (0, 80), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                # cv.putText(img2, str((-x_, -y_)), (0, 100), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2) 

                center_x = int((box_pst[0][0] + box_pst[2][0]) / 2)
                center_y = int((box_pst[0][1] + box_pst[2][1]) / 2)
                
                # x_ = center_x - (width / 2)
                # y_ = (height / 4) - center_y
                
                cv.circle(img2, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)
                
                dis_ = eu_2((x_, y_), (0, 0))
                if dis_ < origin_distance:
                    origin_distance = dis_
                    out_x = x_
                    out_y = y_
                    out_angle = -roll
                    # out_angle = roll if look else roll-90
                    # if out_angle > 90:
                    #     out_angle = -180 + out_angle
                    # elif out_angle < -90:
                    #     out_angle = 180 + out_angle
                    show_box = box_pst
                    ret = True


        if ret:
            # out_angle = -out_angle
            cv.drawContours(img2, [show_box], -1, (255, 0, 255), 2)
            cv.putText(img2, str((out_x, out_y, out_angle, origin_distance)), (0, 20), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 2)           
    cv.imshow('detection', img2)
    return ret, (int(out_x), int(out_y), int(out_angle * 10)),mask



def main():
    global Flag, response
    # Init openni
    openni2.initialize(openni_path)
    #openni2.initialize()

    # Open astra depth stream (using openni)
    depth_device = openni2.Device.open_any()

    # Start the color stream
    color_stream = depth_device.create_color_stream()
    color_stream.start()
    color_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX = 640, resolutionY = 480, fps = 30))
    
    video_flag = False

    if ser.is_open:
        ser.close()
    ser.open()

    t = Thread(target=rece)
    t.start()

    start_time = time.time()

    while True:
        # Grab a new color frame
        color_frame = color_stream.read_frame()
        color_frame_data = color_frame.get_buffer_as_uint8()

        # Put the color frame into a numpy array, reshape it, and convert from bgr to rgb
        color_img = np.frombuffer(color_frame_data, dtype=np.uint8)
        color_img.shape = (480, 640, 3)
        color_img = color_img[...,::-1]
        color_img = cv.flip(color_img, 1)
        # Display the reshaped depth frame using OpenCV
        # cv.imshow("Color Image", color_img)
        now_time = time.time()
        if now_time - start_time > 0.050:
            start_time = now_time
            if response == QR_start or response == color_start  or response == cir_start or response == pos_start:
                ret, brick ,caq= brick_detection(color_img)
                # cv.imshow("op",caq)
                if ret:
                    brick = str(brick).replace(" ", "")
                    ser.write(brick.encode()+frame_tail)
#color_start = b'\x1B\x32'    QR_start = b'\x1B\x31'        cir_start = b'\x1B\x33'    pos_start = b'\x1B\x34'
        if video_flag:
            out.write(color_img)

        key = cv.waitKey(10) & 0xFF
        if int(key) == ord('q'):
            Flag = False
            ser.close()
            openni2.unload()
            cv.destroyAllWindows()
            break
        elif int(key) == ord('s'):
            cv.imwrite(os.path.join(cur_path, "test.jpg"), color_img)
            print("Save Image")
        elif int(key) == ord('v'):
            video_flag = True
            out = cv.VideoWriter(os.path.join(cur_path, 'output.avi'),fourcc, 30.0, (640, 480))
            print("video recording")
        elif int(key) == ord('p'):
            response = QR_start
            print("detection start")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
