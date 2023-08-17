#!/usr/bin/env python3
#-*- coding: utf-8 -*-
####8월9일

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float64

class StopLine():
    def __init__(self):
        rospy.Subscriber("usb_cam/image_rect_color/compressed", CompressedImage, self.image_callback)
        self.request_pub = rospy.Publisher("/request/motor/speed", Float64, queue_size=1)
        self.last_stop_detected = False
        self.stop_detected_once = False  
        self.vertical_line_threshold = 6      # 안 밝을 때                    
        # self.vertical_line_threshold = 7       # 밝을 때                   

        self.bridge = CvBridge()
        self.frame_cnt = 0
        self.print_cnt = True
        # self.last_time = rospy.Time.now()
        

    def image_callback(self,msg):

        self.frame_cnt += 1
        # if self.frame_cnt%2 != 0:
        #     pass
        # else:
        disparity_mode = rospy.get_param('disparity_mode')
        mv_obs_mode = rospy.get_param('mv_obs_detect_mode')
        stat_obs_mode = rospy.get_param('stat_obs_detect_mode')

        if disparity_mode or mv_obs_mode or stat_obs_mode:
            pass
        else:

            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # cv2.imshow('test4',image)
            
            
            p1 = [290, 310]  # 좌상
            p2 = [250, 380] # 좌하
            p3 = [350, 310] # 우상
            p4 = [390, 380]  # 우하
            
            # corners_point_arr는 변환 이전 이미지 좌표 4개 
            corner_points_arr = np.float32([p1, p2, p3, p4])
            height = 480
            width = 640

            image_p1 = [0, 0]
            image_p2 = [0, height]
            image_p3 = [width, 0]
            image_p4 = [width, height]

            image_params = np.float32([image_p1, image_p2, image_p3, image_p4])

            mat = cv2.getPerspectiveTransform(corner_points_arr, image_params)
            # mat = 변환행렬(3*3 행렬) 반
            image_transformed = cv2.warpPerspective(image, mat, (width, height))
            # cv2.imshow('tes3',image_transformed)
            

            roi_top = 0
            roi_bottom = 340
            roi_mask = np.zeros_like(image[:, :, 0])
            roi_mask[:, :] = 255
            stop_line_region = cv2.bitwise_and(image_transformed, image_transformed, mask=roi_mask)
            cv2.circle(image, (290, 310), 2, (0, 255, 0), -1)
            cv2.circle(image, (250, 380), 2, (0, 255, 0), -1)
            cv2.circle(image, (350, 310), 2, (0, 255, 0), -1)
            cv2.circle(image, (390, 380), 2, (0, 255, 0), -1)

            # cv2.imshow('wunbon',image)

            gray_frame = cv2.cvtColor(stop_line_region, cv2.COLOR_BGR2GRAY)
            blur_img = cv2.GaussianBlur(gray_frame, (5, 5), 5)
            # threshold1 증가하면 떨어져 있는 직선끼리 서로 잘 연결됨, threshold2 올리면 올릴수록 더 뚜렷한 직선만 검출
            # edges = cv2.Canny(blur_img, threshold1=10, threshold2=25) # 어두컴컴 적운형 구름 일요일, 오후 6시
            # edges = cv2.Canny(blur_img, threshold1=15, threshold2=50) # 해가 쨍한데 구름에 가려졌을 때
            edges = cv2.Canny(blur_img, threshold1=25, threshold2=60) # 중간
            # edges = cv2.Canny(blur_img, threshold1=40, threshold2=75) # 해가 구름에 안가려져서 쨍할 때
            # cv2.imshow('edge',edges)

            stop_lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=20, minLineLength=35, maxLineGap=10)
            output_image = np.copy(stop_line_region)

            vertical_lines = 0

            if stop_lines is not None:
                for line in stop_lines:
                    x1, y1, x2, y2 = line[0]

                    slope = (y2 - y1) / (x2 - x1 + 1e-5)  

                    slope_threshold = 1.8
                    if slope_threshold <= abs(slope)  :
                        cv2.line(output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        vertical_lines += 1 
                        # print(vertical_lines)
                    else:
                        self.stop_detected = False  

                if vertical_lines >= self.vertical_line_threshold and not self.stop_detected_once:
                    self.stop_detected_once = True
                    self.last_stop_detected = True
                    # print('Stop !')
                    if self.print_cnt:
                        print("[어린이구역 미션] 정지선 발견")
                        self.print_cnt = False

                    print("[어린이구역 미션] 정지 중")
                    
                    # 6초 후에 self.last_stop_detected를 다시 False로 설정
                    rospy.Timer(rospy.Duration(6), self.reset_last_stop_detected, oneshot=True)
                    rospy.Timer(rospy.Duration(50), self.reset_stop_detected_once, oneshot=True)
                    

                if self.last_stop_detected and self.stop_detected_once:
                    rospy.set_param("stop_line_mode", True)
                    # 횡단보도(정지선) 검출했을 때 정지
                    self.request_pub.publish(0.0)
                elif self.stop_detected_once and not self.last_stop_detected :
                    # 정지 후 6m 14초 가게끔 줘야 하는 속도
                    if rospy.get_param("aruco_mode"):
                        self.request_pub.publish(1000.0)
                        self.print_cnt = True
                    else:
                        pass
                
                # cv2.imshow('Detected Stop Lines', output_image)
            # if rospy.get_param('stop_line_mode'):
                # cv2.imshow('result',output_image)
                cv2.waitKey(1)
            
    def reset_last_stop_detected(self, event):
        self.last_stop_detected = False
        rospy.set_param("stop_line_mode", False)
        print("[어린이구역 미션] 다시 출발") 
        self.print_cnt = True
    
    def reset_stop_detected_once(self, event):
        self.stop_detected_once = False

# 실제 코드 동작 시, 실행할 코드
def run():
    rospy.init_node("stop_line_node")     # camera_example이라는 이름으로 노드를 ROS_MASTER에 등록해주는 코드 (이름이 겹치지만 않으면 됨) 
    new_class = StopLine()          # 실제 동작을 담당할 Object
    rospy.spin()                         # ROS Node가 종료되지 않고, Callback 함수를 정상적으로 실행해주기 위한 부분 (코드를 유지하면서, 지속적으로 subscriber를 호출해주는 함수)

if __name__ == '__main__':               # 해당 Python 코드를 직접 실행할 때만 동작하도록 하는 부분
    run()                                # 실제 동작하는 코드