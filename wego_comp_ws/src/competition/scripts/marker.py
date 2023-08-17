#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
from fiducial_msgs.msg import FiducialArray
from std_msgs.msg import Float64

class MarkerRecognitionNode:
    def __init__(self):
        rospy.Subscriber('/fiducial_vertices', FiducialArray, self.callback)
        self.request_pub = rospy.Publisher("/request/motor/speed", Float64, queue_size=1)

        self.marker_detected_1 = False
        self.marker_detected_2 = False
        self.after_stopped = False

        self.print_cnt_1 = True
        self.print_cnt_2 = True
        self.print_cnt_3 = True


    def callback(self, data):
        # os.system("clear")
        
        if not data.fiducials:
            pass

        elif data.fiducials[0].fiducial_id == 100: 
            self.marker_detected_1 = True
            self.marker_detected_2 = False

        elif data.fiducials[0].fiducial_id == 101:
            self.marker_detected_1 = False
            self.marker_detected_2 = True

        # 1번 마커 인식 전, 차선주행 속도
        if self.marker_detected_1 == False and self.marker_detected_2 == False:
            pass

        # 1번 마커 인식 후
        if self.marker_detected_1 == True and self.marker_detected_2 == False:
            rospy.set_param("aruco_mode", True)
            if self.print_cnt_1 is True:
                print("[어린이보호구역 미션] 1번 마커 탐지")
                self.print_cnt_1 = False

            if rospy.get_param("stop_line_mode"):
                pass
            else:
                if self.print_cnt_2 is True:
                    print("[어린이보호구역 미션] 1번 마커 탐지 후 속도 감소")
                    self.print_cnt_2 = False
                
                speed_msg = Float64()
                speed_msg.data = 1000 ### 느린 속도: 마커 1 ~ 정지선
                self.request_pub.publish(speed_msg)

        # 2번 마커 인식 후 = 1번 마커 인식 전
        if self.marker_detected_1 == False and self.marker_detected_2 == True:
            rospy.Timer(rospy.Duration(2),self.reset_marker_detected_2, oneshot = True)
            self.request_pub.publish(1000)
            if self.print_cnt_3 is True:
                print("[어린이보호구역 미션] 2번 마커 탐지 후 일반 주행")
                self.print_cnt_3 = False

            self.print_cnt_1 = True
            self.print_cnt_2 = True
            self.print_cnt_3 = True


    def reset_marker_detected_2(self, event):
        self.marker_detected_2 = False
        rospy.set_param("aruco_mode", False)

    def run(self):
        rospy.spin()

def run():
    rospy.init_node("marker_recognition_node")
    new_class = MarkerRecognitionNode()
    rospy.spin()

if __name__ == '__main__':
    run()
