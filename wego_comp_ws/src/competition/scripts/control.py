#!/usr/bin/env python3
#-*- coding: utf-8 -*-

# ============================= 기능추가 =====================================
### (완료) rev_1_obstacle_detection 파일에서 "정적 장애물 탐지 후 차선 변경 신호"를 받아옴. 
#         물론 라바콘에서는 이 코드 관련 함수가 실행 안되도록 막아놓을 필요 있음. 추후 진행  
### (예정) lane_changing 실행이 안 됐는데 차선이 바뀌어 출력되면 노이즈로 판단하고 필터링.
### (예정) key 값을 통해, lane_changing 함수가 실행되는 중에는 다른 판단 기능 모두 off.
# 실행이 끝나고 나서 차선 인식, 장애물 탐지 등 on.


import rospy
from std_msgs.msg import Float64
import os

class ControlNode:
    def __init__(self) :

        rospy.Subscriber("/request/motor/speed", Float64,self.speed_callback)
        rospy.Subscriber("/request/steer/angle", Float64, self.steer_callback)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)

        self.node3_speed = 1
        self.static_detected = False
        self.lane = None
        self.current_lane = 1 ############# 임시로 !
        self.start = False
        self.time_step_1 = rospy.Rate(0.5)
        self.motor_speed = 0.0

        self.smoothing_factor = 0.2  # Range [0, 1]. 0: no smoothing, 1: maximum smoothing.
        self.mv_obs_factor = 0.6 


    def start_delay(self, event):
        self.start = True


    def reset_static_detected(self, event):
            self.static_detected = False


    def lane_changing(self, current_lane):
        rospy.set_param("lane_changing_mode", True)
        if current_lane == 1:
            self.steer_pub.publish(0.7)    
            self.time_step_1.sleep()
            self.steer_pub.publish(0.3)    
            self.time_step_1.sleep()
            self.steer_pub.publish(0.5)    
            self.current_lane = 2

        elif current_lane == 2:
            self.steer_pub.publish(0.3)    
            self.time_step_1.sleep()
            self.steer_pub.publish(0.7)
            self.time_step_1.sleep()
            self.steer_pub.publish(0.5)    
            self.current_lane = 1
        rospy.set_param("lane_changing_mode", False)


    def speed_callback(self, msg):
        rospy.Timer(rospy.Duration(3), self.start_delay, oneshot=True)
        if self.start == True:
            requested_speed = msg.data

            # # Apply linear smoothing
            if rospy.get_param("mv_obs_detect_mode"):
                self.motor_speed = (1 - self.mv_obs_factor) * self.motor_speed + self.mv_obs_factor * requested_speed
            else:
                self.motor_speed = (1 - self.smoothing_factor) * self.motor_speed + self.smoothing_factor * requested_speed
            final_speed = self.motor_speed
            
            # final_speed = requested_speed
            self.speed_pub.publish(final_speed)


    def steer_callback(self, msg):
        
        if self.start == True:
            # os.system("clear")

            final_steer = msg.data
            self.steer_pub.publish(final_steer)


if __name__ == '__main__':
    rospy.init_node('control_node')
    Control = ControlNode()
    rospy.spin()