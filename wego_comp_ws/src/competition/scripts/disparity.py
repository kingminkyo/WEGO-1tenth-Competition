#!/usr/bin/env python3

import rospy
import numpy as np
import math
import cv2
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import CompressedImage
# from morai_msgs.msg import EgoVehicleStatus
# from sensor_msgs.msg import CompressedImage
# from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import time
import matplotlib.pyplot as plt
# from fiducial_msgs.msg import FiducialArray
class Disparity():
    def __init__(self):
        rospy.Subscriber('/lidar2D_RB', LaserScan, self.disparity_extender_callback)
        # rospy.Subscriber('/usb_cam/image_rect_color', Image, self.camera_callback_dis) #################### (?) 무슨 용도일까?
        rospy.Subscriber('usb_cam/image_rect_color/compressed', CompressedImage, self.camera_callback_dis) #################### (?) 무슨 용도일까?

        self.speed_pub = rospy.Publisher("/request/motor/speed", Float64, queue_size=1) #################### 내가 추가함.
        self.steer_pub = rospy.Publisher('/request/steer/angle', Float64, queue_size=1) #################### (?) 큐 사이즈 변경하면 어떤 효과?
        self.bool_pub = rospy.Publisher('/control/Lane_Return', Bool, queue_size=1)
        self.RBbool_pub = rospy.Publisher('/detect/RB', Bool, queue_size=1)
        self.DISPARITY_DIF = 0.3 #################### (?) 추후 조정 필요할듯
        self.CAR_WIDTH = 0.4


        #################### 카메라 관련 ####################
        self.cam_key = True      # camera callback에서 라바콘 탐지하면 camera callback에 진입 못하도록 함.
        self.camRate = 0             # camera hz 낮추기 위한 변수
        self.bridge = CvBridge()
        self.threshold=0      # 필터에 걸린 픽셀 수


        #################### 라이다 관련 ####################
        self.rb_key = False       # 라바콘 탐지시 RBDetect 함수 내부 진입용 변수
        self.lidar = None              # 라이다 데이터 전역변수
        self.wall_data = 0          # 벽 세워진 라이다 데이터 전역변수 from MCRB.py
        self.lidarRate = 0           # lidar hz 낮추기 위한 변수
        self.minrange=100           # 라바콘 판단을 위한 정면 방향 최소거리

        #################### disparity 관련 ####################
        self.dp_key = False       # 라바콘 진입 이후 disparity 함수 내부 진입용 변수
        self.speed_param = rospy.get_param('~speed_param', 1000.0)
        self.P_param = rospy.get_param('~P_param', 0.0)
        # self.disparity_mode = rospy.get_param('disparity_mode', False)
        self.disparity_mode = rospy.get_param('disparity_mode', False)
        # self.offset = 19.4799995422     # 조향 오프셋
        # self.offset = 90     # 조향 오프셋
        self.offset = 20     # 조향 오프셋
        self.line = 0       # 벽 세울 때 생기는 연장선의 개수
        self.list_line = []
        
        self.print_cnt_1 = True
        self.print_cnt_2 = True
        
        
        
        # self.initialized = False      # 트랙바용 변수
        
        # self.avg_threshold = []
        rospy.Timer(rospy.Duration(0.2), self.timer_callback)       # 카메라 연산 줄이기

        # self.steer_pub = rospy.Publisher('/control/angle/RB', Float64, queue_size=1)



    def timer_callback(self, _event):
        # print('TIMER CALLBACK', time.time())
        self.camRate = 0
        # rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camera_callback)
        rospy.Subscriber('usb_cam/image_rect_color/compressed', CompressedImage, self.camera_callback_dis)
        # print('CAMERA CALLBACK')1
        


    def camera_callback_dis(self, _data):
        self.camRate += 1      # 50개 들어오는 데이터 중 5개만 쓸겨
        lidar_data = self.lidar
        # if self.cam_key == True and self.camRate <= 3:
        if self.camRate <= 3:
            # print(self.camRate)

            ######################## 트랙바 생성 ########################

            # if self.initialized == False:
            #     cv2.namedWindow("LANE", cv2.WINDOW_NORMAL)       # Simulator_Image를 원하는 크기로 조정가능하도록 만드는 코드
            #     # 흰색 차선을 검출할 때, 이미지를 보면서 트랙바를 움직이면서 흰색선이 검출되는 정도를 파악하고 코드안에 그 수치를 넣어준다.
            #     cv2.createTrackbar('low_H', 'LANE', 0, 255, nothing)    # Trackbar 만들기
            #     cv2.createTrackbar('low_S', 'LANE', 210, 255, nothing)
            #     cv2.createTrackbar('low_V', 'LANE', 175, 255, nothing)
            #     cv2.createTrackbar('high_H', 'LANE', 32, 255, nothing)    
            #     cv2.createTrackbar('high_S', 'LANE', 244, 255, nothing)
            #     cv2.createTrackbar('high_V', 'LANE', 255, 255, nothing)
            #     self.initialized = True  # 두 번 다시 여기 들어오지 않도록 처리
            
            ################################################################################


            #============== 카메라 imgmsg --> cv2 img --> hsv --> filtered hsv =========================

            
            # img = self.bridge.compressed_imgmsg_to_cv2(img,'bgr8') # [y, x, rgb]
            img = self.bridge.compressed_imgmsg_to_cv2(_data, 'bgr8')
            # img = self.bridge.imgmsg_to_cv2(img,'bgr8') # [y, x, rgb]
            #cv2.imshow("test", img)
            cv2.waitKey(1)
            # print(img.shape[0], img.shape[1])
            img = img[120:500,:,:]  # roi  1280 720
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


            # low_H = cv2.getTrackbarPos('low_H', 'LANE')
            # low_S = cv2.getTrackbarPos('low_S', 'LANE')
            # low_V = cv2.getTrackbarPos('low_V', 'LANE')
            # high_H = cv2.getTrackbarPos('high_H', 'LANE')
            # high_S = cv2.getTrackbarPos('high_S', 'LANE')
            # high_V = cv2.getTrackbarPos('high_V', 'LANE')
            # lower = np.array([low_H, low_S, low_V])
            # upper = np.array([high_H, high_S, high_V])
            lower = np.array([5, 150, 60])
            upper = np.array([15, 250, 200])

            mask = cv2.inRange(hsv, lower, upper)
            # masked = cv2.bitwise_and(img, img, mask=mask)
            # cv2.imshow('masked', masked)
            #cv2.imshow('mask',mask)
            self.threshold = np.nonzero(mask)[0]  #빨강인 픽셀이 몇개인지
            
            # cv2.imshow('img',img)
            # cv2.imshow('hsv', masked)
            # cv2.imshow('gray', gray_masked)
            # print(gray_masked.shape)
            # cv2.waitKey(1)
            # print(f'픽셀 개수 : {self.threshold.shape[0]}')

            
            # self.threshold = 0      # hsv_gray filter에서 값 잡히는 픽셀 갯수 임계값으로 신호등과 라바콘 구분
            
            
            # for i in gray_masked:
            #     if (i>0).any():
            #         # print('###################################')
            #         # print('RB DETECTED')
            #         # print('###################################')
            #         self.threshold += len(np.nonzero(i)[0])
            

            #############################################################################
            # 픽셀 50개 이상 잡히면 라바콘으로 인식하여 더이상 camera callback 연산 X

            # if (self.threshold.shape[0] > 13000)  and (self.minrange<=1.5 and self.cam_key==True):      # 라바콘 판단 조건
            # if self.threshold.shape[0] > 2000 and self.cam_key == True:
            # if self.threshold.shape[0] >5000 and self.cam_key == True: #============ 경기장
            if self.threshold.shape[0] >4000 and self.cam_key == True:
                self.rb_key=True                                          # 라바콘 판단
                self.cam_key=False
                rospy.set_param('disparity_mode', True)

                # a = rospy.get_param('disparity_mode')
                print('[라바콘 미션] 라바콘 발견')

                self.RBbool_pub.publish(True)

                    
            # print(self.threshold.shape[0])

            #############################################################################


        #############################################################################
        # 라바콘 인지하여 RBDetect 함수 진입

        if self.rb_key == True:
            self.RBDetect()
            self.lidarRate=0
            self.cam_key=False       # camera callback 진입 통제.
            

    ############################################################################################################################################################
    # 라바콘 내부로 진입
        
    def RBDetect(self):
        data = self.lidar
        if self.print_cnt_1:
            print('[라바콘 미션] 라바콘 진입 중')
            self.print_cnt_1 = False

        if self.rb_key == True and data != None :
            
            
            org_ranges = list(data.ranges)
            roi_gap = 45            # 정면 라이다 각도 roi 
            
            left_storage = []        # 정면 20도 안에 있는 라이다 데이터의 [angle, range]
            right_storage = []

            for angle in range(0,roi_gap+1):    # 0 ~ roi_gap
                left_storage.append([angle, org_ranges[angle]])

            for angle in range(360-roi_gap,360):    # 359-roi_gap ~ 359도
                right_storage.append([angle, org_ranges[angle]])

            left_storage.sort(key=lambda x: x[1])     # 거리순으로 오름차순
            right_storage.sort(key=lambda x: x[1])     # "
            # print(left_storage)
            
            closest = []        # 가장 가까운 두 라이다 데이터의 [[angle, range], [angle, rnage]]
            
            closest.append(left_storage.pop(0))  
            closest.append(right_storage.pop(0))
            # print(closest)
            closest[1][0] += -360 #################### (?) 왜 360을 빼지? --> 전방을 0도로 맞추기 위해 !!
            vector = (closest[0][0]+closest[1][0])//2
            # print(vector)
            vector = (-vector+self.offset)/(2*self.offset) #################### (?) vector은 무슨 용도지? 계산상 angle인데? 아래 나오는 angle과 같음!
            # print(vector)

            if vector>1.0:
                vector=1.0
            elif vector<0:
                vector=0
            
            angle = Float64()   
            angle.data = vector
            self.speed_pub.publish(3000)
            self.steer_pub.publish(angle)

            disparity_list = data.ranges[80:91]+data.ranges[270:281]
            stack = 0           # 옆구리쪽 거리가 1m 이내인 인덱스 개수
            for i in disparity_list:
                if i<=0.5:
                    stack += 1
            if stack >= 10:              # 디스파리티 진입 조건
                self.dp_key=True        # 디스파리티 진입
                self.rb_key=False
                self.print_cnt_1 = True

            # print('??')



    ############################################################################################################################################################|

    # 벽 세운 라이다 데이터 받아와서 전역변수로 지정
    def lidarRB_callback(self, data):
        self.wall_data = data


    # 벽 세울 때 생긴 연장선의 개수 받아와서 전역변수로 지정
    def line_callback(self, data):
        self.line = data.data
        # print(self.line)


    def disparity_extender_callback(self, data):
        # os.system('clear')
        self.lidar = data
        roiBox = sorted(list(data.ranges[0:50]) + list(data.ranges[329:359]))  #정면에서 제일 작은값찾는 과정
        self.minrange = roiBox[0]
        wall_data = data            # 라바콘 판단    

        if self.dp_key == True:
            # rospy.loginfo('DISPARITY ON')
            # len(dat1a.ranges) = 360, head 기준 반시계방향으로 0~359
            # print(np.argmin(da1ta.ranges))
            
            dis = list(wall_data.ranges[270:360] + wall_data.ranges[0:91])
            disparities = []

            for angle in range(len(dis)):
                if angle == len(dis)-1:
                    continue
                if abs(dis[angle] - dis[angle+1]) > self.DISPARITY_DIF:
                    # dis[angle], dis[angle+1]에 대한 조건 : 위 조건의 경우가 물체와 먼 외부 환경과의 차이에 의해 발생하는 상황 방지
                    min_dis = min(dis[angle], dis[angle+1])
                    angle_range = math.ceil(math.degrees(math.atan(self.CAR_WIDTH / 2 / min_dis)))
                    # angle_range += 5           # 설명..
                    angle_range += 16           # 설명..
                    side_range = range(int(angle-angle_range+1), angle+1) if dis[angle+1] == min_dis else range(angle+1, int(angle+1+angle_range))
                    disparities.append((min_dis, side_range))
                    
            for min_dis, side_range in disparities:
                # print(side_range)
                for i in side_range:
                    if i >= 0 and i < len(dis):
                        dis[i] = min(dis[i], min_dis)

            max_index = np.argmax(dis)
            max_dis = dis[max_index]
            
            angle = max_index-90 if abs(max_index - 90) > 3 else 0
            # print(angle)
            if self.print_cnt_2:
                print("[라바콘 미션] Disparity 시작")
                self.print_cnt_2 = False
            
            angle = (-angle+self.offset)/(2*self.offset)


            if angle>1.0:  ############################ -90도 미만일 때 
                angle=1.0
            elif angle<0: ############################ 90도 초과일 때 
                angle=0
            

            ang = Float64()
            ang.data = angle

            disparity_list = data.ranges[80:91]+data.ranges[270:281]
            stack = 0           # 옆구리쪽 거리가 1m 이상인 인덱스 개수
            for i in disparity_list:
                if i>=1:
                    stack += 1
            if stack >= 15:              # 디스파리티 탈출 조건
                self.dp_key=False        # 디스파리티 종료
                print("[라바콘 미션] 라바콘 탈출")
                self.cam_key = True ############ 라바콘에서 완전히 탈출 후, 해주는 걸로 바꿔야 함 ! ##############
                self.print_cnt_2 = True
                rospy.set_param('disparity_mode', False)
                # rospy.set_param('normal_drive_mode', True)

                # a = rospy.get_param('disparity_mode')
                # print(f'Disparity mode 2: {a}')
            else:
                self.speed_pub.publish(3000)
                self.steer_pub.publish(ang)
            
            # self.avg_threshold.append(self.threshold.shape[0]//2)

            # print('#############################')
            # print(self.threshold.shape[0])
            # print(self.avg_threshold[1])
            # print('#############################')
            # self.list_line.append(self.line)      # [이전 line, 현재 line]
            
            # print(angle)

            # if self.threshold.shape[0] <= self.avg_threshold[0]:
            # print('###############')
            # print(self.line)
            # print(self.list_line)
            # print('###############')


            # try:
            #     if self.list_line[0]//2 >= self.list_line[1]:
                    # self.dp_key=False
                    # print('DISPARITY OFF')
                    # self.bool_pub.publish(True)
                    # self.steer_pub.publish(Float64(999))
                    # self.dp_key = False
                    # self.cam_key = True
            #     else:   self.list_line.pop(0) 

            # except IndexError: pass

            # if self.threshold.shape[0] <= 200:
            # if self.threshold.shape[0] <= 5:
            #     self.dp_key=False
            #     print('DISPARITY OFF')
            #     self.bool_pub.publish(True)
            #     self.speed_pub.publish(0)
            #     # self.steer_pub.publish(Float64(999))
            #     self.steer_pub.publish(Float64(0.5))
            #     self.dp_key = False
            #     self.cam_key = True
                
    ############################################################################################################################################################|            

        # 두 좌표 거리 구하는 함수
    def distance(self, myXY, wayXY):
        return ((myXY[0]-wayXY[0])**2+(myXY[1]-wayXY[1])**2)**0.5

    ############################################################################################################################################################|


# createTrackbar를 위한 함수
def nothing(a=1):
    return
        
def run():
    rospy.init_node('Disparity')
    Disparity()
    rospy.spin()

if __name__=='__main__':
    run()