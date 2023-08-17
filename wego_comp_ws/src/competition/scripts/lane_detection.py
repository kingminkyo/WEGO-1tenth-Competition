#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge                  # ROS Image를 OpenCV의 Image로 변경하기 위해 필요한 코드
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import CameraInfo
from stop_line import *
from display import draw_arrow
import matplotlib.pyplot as plt
import time
import warnings

class CameraReceiver():
    def __init__(self):
        rospy.loginfo("Camera Receiver object is created")
        rospy.Subscriber("usb_cam/image_rect_color/compressed", CompressedImage, self.camera_callback)
        rospy.Subscriber("/commands/motor/speed", Float64, self.speed_callback)
        rospy.Subscriber("/commands/servo/position", Float64, self.steer_callback)
        self.bridge = CvBridge()
        self.initialized = False                                             
        # self.lane_moment_pub = rospy.Publisher("center_lane_moment_x", Int32, queue_size=3)
        # self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        # self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/request/steer/angle", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/request/motor/speed", Float64, queue_size=1)
        # self.steer_pub = rospy.Publisher("/request/steer/angle/node1", Float64, queue_size=1)
        self.lane_pub = rospy.Publisher("/request/lane", Int32, queue_size=1)
        self.motor_steer = 0
        self.smoothing_factor = 0.7
        self.stop_line = StopLine()
        self.frame_cnt = 0
        self.print_cnt = True

        self.current_speed = 0
        self.current_angle = 0.546

        self.old_left_fit = None
        self.old_right_fit = None

    def speed_callback(self, msg):
        self.current_speed = msg.data
        
    def steer_callback(self, msg):
        self.current_angle = msg.data

    def angle_filter(self, current_angle, computed_angle, max_angle_change):
        angle_diff = abs(computed_angle - current_angle)

        if angle_diff > max_angle_change:  # 급격한 변화가 있는 경우
            return current_angle  # 현재 차량 각도를 사용
        else:
            return computed_angle  # 계산된 주행 각도를 사용

    def plothistogram(self, image):

        bottom_quarter = image[image.shape[0] * 3 // 4:, :]
        blurred_image = cv2.GaussianBlur(bottom_quarter, (5, 5), 0)
        histogram = np.sum(blurred_image, axis=0)
        midpoint = np.int(histogram.shape[0] / 2)
        leftbase = np.argmax(histogram[:midpoint])
        rightbase = np.argmax(histogram[midpoint:]) + midpoint

        return leftbase, rightbase

    def find_lane_divider(self, binary_warped):
        # 이 부분에서 중앙 기준으로 좌우 나누고, 흰색 픽셀 수가 어디에 많은지를 계산하는 함수입니다.
        # 아래는 예시로 간단하게 구현한 것이며, 실제로 사용하는 이미지에 따라 파라미터와 로직을 조정해야 합니다.
        mid_point = binary_warped.shape[1] // 2
        left_pixels = np.sum(binary_warped[:, :mid_point] == 255)
        right_pixels = np.sum(binary_warped[:, mid_point:] == 255)

        return left_pixels, right_pixels

    def classify_lane(self, left_pixels, right_pixels):
        # 이 함수에서는 좌우 픽셀 수에 따라 현재 위치를 1차선과 2차선으로 구분합니다.
        # 예시로 1차선은 좌측에 있을 때, 2차선은 우측에 있을 때로 구분합니다.
        if left_pixels > right_pixels:
            # return "1st lane"  # 1차선 (가운데로부터 좌측에 위치)
            return 1  # 1차선 (가운데로부터 좌측에 위치)
        else:
            # return "2nd lane"  # 2차선 (가운데로부터 우측에 위치)
            return 2  # 2차선 (가운데로부터 우측에 위치)

    def slide_window_search(self, binary_warped, left_current, right_current):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        nwindows = 10
        window_height = np.int(binary_warped.shape[0] / nwindows) # 120
        nonzero = binary_warped.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])
        margin = 70
        minpix = 50
        left_lane = []  # 왼쪽 차선만 검출하는 배열로 수정
        right_lane = []  # 오른쪽 차선만 검출하는 배열로 수정

        # Define color variable for drawing rectangles
        color = [0, 255, 0]
        thickness = 2

        for w in range(nwindows):
            win_y_low = binary_warped.shape[0] - (w + 1) * window_height
            win_y_high = binary_warped.shape[0] - w * window_height
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), color, thickness)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), color, thickness)

            good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]

            left_lane.append(good_left)
            right_lane.append(good_right)

            

        left_lane = np.concatenate(left_lane) if len(left_lane) > 0 else None
        right_lane = np.concatenate(right_lane) if len(right_lane) > 0 else None

        if left_lane is None or right_lane is None:
            return None

        leftx = nonzero_x[left_lane]
        lefty = nonzero_y[left_lane]
        rightx = nonzero_x[right_lane]
        righty = nonzero_y[right_lane]
        
        try:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)

            self.old_left_fit = left_fit
            self.old_right_fit = right_fit

            ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

            # print("보정 전 좌측:", left_fitx[-1])
            # print("보정 전 우측:", right_fitx[-1])
            # print("보정 전 차이:",right_fitx[-1] - left_fitx[-1])
            # print("=")
            # 검출된 차선들의 중심 위치의 평균과 차선 사이의 차이를 확인하여 조정
            avg_position = (left_fitx[-1] + right_fitx[-1]) / 2
            if avg_position <= 320 and abs(right_fitx[-1] - left_fitx[-1]) < 130:
                # 예: 오른쪽 차선을 50픽셀만큼 오른쪽으로 이동시키기
                # print("오른쪽으로 바로잡자 현재는: ",abs(right_fitx[-1] - left_fitx[-1]) )
                # left_fitx -= 150
                right_fitx += 50
            elif avg_position > 320 and abs(right_fitx[-1] - left_fitx[-1]) < 130:
                # 예: 왼쪽 차선을 50픽셀만큼 왼쪽으로 이동시키기
                # print("왼쪽으로 바로잡자 현재는: ", abs(right_fitx[-1] - left_fitx[-1]) )
                
                left_fitx -= 50
                # right_fitx +=150

            if left_fitx[-1] >= right_fitx[1] or (abs(right_fitx[1] - left_fitx[-1]) < 10 ) or (abs(right_fitx[1] - right_fitx[-1]) > 100):
                right_fitx = left_fitx + 200
                # print("오른쪽 보정")


            elif left_fitx[1] >= right_fitx[-1] or (abs(right_fitx[-1] - left_fitx[1]) < 10 ) or (abs(left_fitx[1] - left_fitx[-1]) > 100):
                left_fitx = right_fitx - 200
                # print("왼쪽 보정")

            ltx = np.trunc(left_fitx)
            rtx = np.trunc(right_fitx)

            out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]
            out_img[nonzero_y[right_lane], nonzero_x[right_lane]] = [0, 0, 255]
            # print("보정 후 좌측:", left_fitx[-1])
            # print("보정 후 우측:", right_fitx[-1])
            # print("보정 전 차이:",right_fitx[-1] - left_fitx[-1])
            # print("================")
            ret = {'left_fitx': ltx, 'right_fitx': rtx, 'ploty': ploty}

            return ret

        except Exception as e:
            # left_fit = self.old_left_fit
            # right_fit = self.old_right_fit

            # self.old_left_fit = left_fit
            # self.old_right_fit = right_fit

            # ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
            # left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            # right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

            # # print("보정 전 좌측:", left_fitx[-1])
            # # print("보정 전 우측:", right_fitx[-1])
            # # print("보정 전 차이:",right_fitx[-1] - left_fitx[-1])
            # # print("=")
            # # 검출된 차선들의 중심 위치의 평균과 차선 사이의 차이를 확인하여 조정
            # avg_position = (left_fitx[-1] + right_fitx[-1]) / 2
            # if avg_position <= 320 and abs(right_fitx[-1] - left_fitx[-1]) < 170:
            #     # 예: 오른쪽 차선을 50픽셀만큼 오른쪽으로 이동시키기
            #     # print("오른쪽으로 바로잡자 현재는: ",abs(right_fitx[-1] - left_fitx[-1]) )
            #     # left_fitx += 170
            #     right_fitx += 250
            # elif avg_position > 320 and abs(right_fitx[-1] - left_fitx[-1]) < 170:
            #     # 예: 왼쪽 차선을 50픽셀만큼 왼쪽으로 이동시키기
            #     # print("왼쪽으로 바로잡자 현재는: ", abs(right_fitx[-1] - left_fitx[-1]) )
                
            #     left_fitx -= 250
            #     # right_fitx -=150

            # # if left_fitx[-1] >= right_fitx[1] or (abs(right_fitx[1] - left_fitx[-1]) < 10 ) or (abs(right_fitx[1] - right_fitx[-1]) > 100):
            # #     right_fitx = left_fitx + 100
            # #     print("오른쪽 보정")


            # # elif left_fitx[1] >= right_fitx[-1] or (abs(right_fitx[-1] - left_fitx[1]) < 10 ) or (abs(left_fitx[1] - left_fitx[-1]) > 100):
            # #     left_fitx = right_fitx - 100
            # #     print("왼쪽 보정")

            # ltx = np.trunc(left_fitx)
            # rtx = np.trunc(right_fitx)

            # out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]
            # out_img[nonzero_y[right_lane], nonzero_x[right_lane]] = [0, 0, 255]
            # # print("보정 후 좌측:", left_fitx)
            # # print("보정 후 우측:", right_fitx)
            # # print("보정 전 차이:",right_fitx[-1] - left_fitx[-1])
            # # print("================")
            # ret = {'left_fitx': ltx, 'right_fitx': rtx, 'ploty': ploty}

            # return ret
            return False






    def draw_lane_lines(self, original_image, warped_image, Minv, draw_info):
        left_fitx = draw_info['left_fitx']
        right_fitx = draw_info['right_fitx']
        ploty = draw_info['ploty']

        if left_fitx is None or right_fitx is None:
            return original_image

        left_ = left_fitx[120:150]
        right_ = right_fitx[120:150]

        left_x = np.mean(left_)
        right_x = np.mean(right_)
        left = left_x - float(320)
        right = right_x - float(320)
        ref_dist = left + right

        warp_zero = np.zeros_like(warped_image).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        mean_x = np.mean((left_fitx, right_fitx), axis=0)
        pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

        k = np.int_([pts_mean])
        ref_angle = float((k[0][0][150][0] - k[0][0][120][0])/30)

        cv2.fillPoly(color_warp, np.int_([pts]), (255, 255, 255))
        cv2.fillPoly(color_warp, k, (0, 0, 255))
        cv2.circle(color_warp, (k[0][0][150][0],k[0][0][150][1]), 5, (0,255,0), -1)
        cv2.circle(color_warp, (k[0][0][10][0],k[0][0][10][1]), 5, (0,255,0), -1)

        newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        result = cv2.addWeighted(original_image, 1, newwarp, 0.4, 0)

        return pts_mean, result, k[0][0][150][0]

    def camera_callback(self, _data):
        self.frame_cnt +=1
        if self.frame_cnt % 2 != False:
            pass
        else:
        # disparity_mode = rospy.get_param("/Disparity/disparity_mode")
            lane_changing_mode = rospy.get_param('lane_changing_mode')
            stop_line_mode = rospy.get_param('stop_line_mode')
            aruco_mode = rospy.get_param("aruco_mode")
            disparity_mode = rospy.get_param('disparity_mode')
            mv_obs_mode = rospy.get_param('mv_obs_detect_mode')
            stat_obs_mode = rospy.get_param('stat_obs_detect_mode')

            self.frame_cnt = 0

            ################ 추후에 Aruco 마커 추가해야
            if disparity_mode or lane_changing_mode or stop_line_mode or mv_obs_mode or stat_obs_mode:
            # if disparity_mode:
                rospy.set_param('normal_drive_mode',False)
                cv_image = self.bridge.compressed_imgmsg_to_cv2(_data, 'bgr8')

                if self.print_cnt == False:
                    self.print_cnt = True

                img_1 = draw_arrow(cv_image, self.current_angle, self.current_speed)
                # cv2.imshow("result", img_1)
                cv2.waitKey(1)
                # cv2.moveWindow("result", 1400, 300)

            else:
                rospy.set_param('normal_drive_mode',True)
                
                if self.initialized == False:       # 아래의 코드는 처음 실행했을 때, 한번만 실행되어야하므로 self.initialized를 사용해서 처음에만 실행되도록 짜준 코드
                    cv2.namedWindow("hsv", cv2.WINDOW_NORMAL)       # Simulator_Image를 원하는 크기로 조정가능하도록 만드는 코드
                    # 흰색 차선을 검출할 때, 이미지를 보면서 트랙바를 움직이면서 흰색선이 검출되는 정도를 파악하고 코드안에 그 수치를 넣어준다.
                    cv2.createTrackbar('low_H', 'hsv', 0, 255, nothing)    # Trackbar 만들기
                    cv2.createTrackbar('low_S', 'hsv', 0, 255, nothing)
                    # cv2.createTrackbar('low_V', 'hsv', 134, 255, nothing)
                    # cv2.createTrackbar('low_V', 'hsv', 48, 255, nothing) ### 23.08.12  night
                    cv2.createTrackbar('low_V', 'hsv', 50, 255, nothing) ### 
                    cv2.createTrackbar('high_H', 'hsv', 255, 255, nothing)    # Trackbar 만들기
                    cv2.createTrackbar('high_S', 'hsv', 255, 255, nothing)
                    cv2.createTrackbar('high_V', 'hsv', 255, 255, nothing)
                    self.initialized = True


                #============== 카메라 띄우기 =========================
                cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)
                # cv2.imshow("rect_camera", cv_image)                                                              # wun bon
                # canny = self.canny(cv_image)
                # cv2.imshow("canny", canny)

                #====================================================
                # 시점변환부터
                
                (h, w) = (cv_image.shape[0], cv_image.shape[1])
                Width = 640
                Height = 480
                source = np.float32([[275, 328], [155, 440], [365, 328], [485, 440]])   # [x,y]                        #     2  good  jom gip da
                destination = np.float32([[235, 10], [235, 470], [405, 10], [405, 470]])

                # 변환 행렬을 구하기 위해서 쓰는 함수
                transform_matrix = cv2.getPerspectiveTransform(source, destination)
                # minv값은 마지막에 warpping된 이미지를 다시 원근감을 
                minv = cv2.getPerspectiveTransform(destination, source)
                # 변환 행렬값을 적용하여 최종 결과 이미지를 얻을 수 있다.
                _image = cv2.warpPerspective(cv_image, transform_matrix, (w, h))
                #cv2.imshow("warp_image", _image)                                                                      # jja bu

                # # ===============================================================
                
                hsv = cv2.cvtColor(_image, cv2.COLOR_BGR2HSV)
                

                low_H = cv2.getTrackbarPos('low_H', 'hsv')
                low_S = cv2.getTrackbarPos('low_S', 'hsv')
                low_V = cv2.getTrackbarPos('low_V', 'hsv')
                high_H = cv2.getTrackbarPos('high_H', 'hsv')
                high_S = cv2.getTrackbarPos('high_S', 'hsv') 
                high_V = cv2.getTrackbarPos('high_V', 'hsv')

                lower_lane = np.array([low_H, low_S, low_V])
                upper_lane = np.array([high_H, high_S, high_V])
                blur_img = cv2.GaussianBlur(hsv, (3, 3), 5)
                masked = cv2.inRange(blur_img, lower_lane, upper_lane)
            
                cv2.imshow("lane_image", masked)
                cv2.moveWindow("lane_image", 700, 300)/

                x = int(masked.shape[1])    # 이미지 가로
                y = int(masked.shape[0])    # 이미지 세로

                #한붓그리기
                _shape = np.array(
                    [[int(0.15*x), int(y)], [int(0.15*x), int(0.1*y)], [int(0.49*x), int(0.1*y)], [int(0.49*x), int(y)], [int(0.51*x), int(y)], [int(0.51*x), int(0.1*y)], [int(0.75*x), int(0.1*y)], [int(0.75*x), int(y)], [int(0.4*x), int(y)]]
                )

                mask2 = np.zeros_like(masked)
                #print(masked.shape)

                if len(masked.shape) > 2:
                    channel_count = masked.shape[2]
                    ignore_mask_color = (255, ) * channel_count
                else:
                    ignore_mask_color = 255

                cv2.fillPoly(mask2, np.int32([_shape]), ignore_mask_color)
                #cv2.imshow("lane1", mask2)    
                masked_image = cv2.bitwise_and(masked, mask2)

                leftbase, rightbase = self.plothistogram(masked_image)

                left_pixels, right_pixels = self.find_lane_divider(masked_image)
                current_lane = self.classify_lane(left_pixels, right_pixels)


                # # ## histogram 기반 window roi 영역
                draw_info = self.slide_window_search(masked_image, leftbase, rightbase)
                
                if draw_info == False:
                    cv2.imshow("result", cv_image) 
                    self.speed_pub.publish(Float64(3000))
                    cv2.moveWindow("result", 1400, 300)/
                else:
                    meanPts, result, center_x = self.draw_lane_lines(cv_image, masked_image, minv, draw_info)
                    
                    self.lane_pub.publish(Int32(center_x))

                    # Calculate the error from the center of the image
                    # error = center_x - (cv_image.shape[1] // 2)
                    # normalized_error = error / (cv_image.shape[1] // 2)

                    steer_scale = 0.9
                    error = center_x - (cv_image.shape[1] // 2)
                    normalized_error = error / (cv_image.shape[1] // 2)
                    steer_angle = 0.546 + steer_scale * normalized_error

                    # Clip the steering angle to be between 0.0 and 1.0
                    steer_angle = np.clip(steer_angle, 0.1334, 0.9586)

                    new_steer = steer_angle

                    # Apply linear smoothing
                    self.motor_steer = (1 - self.smoothing_factor) * self.motor_steer + self.smoothing_factor * new_steer
                    steer_angle = self.angle_filter(self.current_angle, self.motor_steer, 0.8)

                    if self.current_angle is None:
                        self.steer_pub.publish(Float64(self.motor_steer))
                    else:
                        # self.steer_pub.publish(Float64(steer_angle))

                        if aruco_mode is True:
                            self.steer_pub.publish(Float64(steer_angle))
                        else:
                            self.steer_pub.publish(Float64(steer_angle))
                            self.speed_pub.publish(Float64(3000))
                    # self.lane.publish(Float64(current_lane))

                    if self.print_cnt and (aruco_mode is not True):
                            print("[일반 주행] 일반 주행 중")
                            self.print_cnt = False

                    # # ## 원본 이미지에 라인 넣기
                    meanPts, result, x1 = self.draw_lane_lines(cv_image, masked_image, minv, draw_info)
                    # self.lane_moment_pub.publish(Int32(x1))

                    if self.current_angle is not None:
                        result = draw_arrow(result, self.current_angle, self.current_speed)
                    else:
                        ############## 일단 패스. 생각해보자.
                        pass
                    
                    cv2.imshow("result", result) /
                    cv2.moveWindow("result", 1400, 300)

        cv2.waitKey(1) 
    

def nothing():
    pass

# 실제 코드 동작 시, 실행할 코드
def run():
    rospy.init_node("path_tracking")     # camera_example이라는 이름으로 노드를 ROS_MASTER에 등록해주는 코드 (이름이 겹치지만 않으면 됨) 
    warnings.filterwarnings("ignore", category=np.RankWarning)
    new_class = CameraReceiver()          # 실제 동작을 담당할 Object
    rospy.spin()                         # ROS Node가 종료되지 않고, Callback 함수를 정상적으로 실행해주기 위한 부분 (코드를 유지하면서, 지속적으로 subscriber를 호출해주는 함수)

if __name__ == '__main__':               # 해당 Python 코드를 직접 실행할 때만 동작하도록 하는 부분
    run()    
