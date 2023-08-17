#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import LaserScan
from math import *
import os

class Obstruct:
    def __init__(self):
        rospy.Subscriber("/request/lane", Int32, self.lane_callback)
        self.speed_pub = rospy.Publisher("/request/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/request/steer/angle", Float64, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.laser_msg = LaserScan()
        self.detect_degree = 100 #963, 321
        self.safe_range =   1.1
    
        self.degrees = []
        self.degree_flag = False
        self.obsobs = False 

        self.current_lane = 1
        self.mid_steer = 0.546
        self.left_steer = 0.1334
        self.right_steer = 0.9586
       
        ## 차선 변경 ####
        self.changing_lane = False
        self.semi_changing_lane = False
        self.my_lane_number = 1
        self.cnt_1 = 0
        self.cnt_2 = 0
        self.print_cnt = True

    def lane_callback(self,msg):
        self.current_lane = msg.data

    def lidar_callback(self, msg):
        if rospy.get_param("disparity_mode") == False and rospy.get_param('mv_obs_detect_mode') == False:
            # os.system("clear")
            e_stop_degree_list = []
        
            if self.degree_flag == False :
                self.degrees = [msg.angle_min*180/pi + msg.angle_increment*180/pi * index for index, value in enumerate(msg.ranges)]
                self.degree_flag = True


            for index, value in enumerate(msg.ranges) :
                if 170 < abs(self.degrees[index]) and 0 < value < self.safe_range :   # 좌우당  10도씩
                    e_stop_degree_list.append(self.degrees[index])
                    
            # print('전방에 감지되는 인덱스 갯수 :',len(e_stop_degree_list))

            #===================1->2===================        
            if len(e_stop_degree_list) < 25:
                pass


            if len(e_stop_degree_list) > 25:
                self.obsobs = True
                rospy.set_param("stat_obs_detect_mode", True)
                if self.print_cnt == True:
                    print("[장애물 미션] 정적 장애물 발견")
                    self.print_cnt = False

            if self.obsobs ==True and self.my_lane_number == 1 and self.cnt_1 < 15:
                
                self.speed_pub.publish(1000.0)
                self.steer_pub.publish(self.right_steer)
                self.cnt_1+=1

            if self.obsobs ==True and self.my_lane_number == 1 and 15 <= self.cnt_1 < 30 :
                self.speed_pub.publish(1000.0)
                self.steer_pub.publish(self.left_steer)
                self.cnt_1+=1

            
            if self.my_lane_number == 1 and 30 == self.cnt_1:
                self.my_lane_number = 2
                self.obsobs = False
                rospy.set_param("stat_obs_detect_mode", False)
                self.print_cnt = True

            #===================1->2===================

            #===================2->1===================
            if self.obsobs ==True and self.my_lane_number == 2 and self.cnt_2 < 15:
                
                # self.speed_pub.publish(1500.0)
                self.speed_pub.publish(900.0)
                self.steer_pub.publish(self.left_steer)
                self.cnt_2+=1

            if self.obsobs ==True and self.my_lane_number == 2 and 15 <= self.cnt_2 < 30 :
                self.speed_pub.publish(900.0)
                self.steer_pub.publish(self.right_steer)
                self.cnt_2+=1

            if self.my_lane_number == 2 and 30 == self.cnt_2:
                self.cnt_1 = 0
                self.cnt_2 = 0
                self.obsobs = False
                self.my_lane_number = 1
                rospy.set_param("stat_obs_detect_mode", False)  
                self.print_cnt = True
            #===================2->1===================

        
def main():
    rospy.init_node('stat_obstacle')
    obstruct = Obstruct()
    rospy.spin()

if __name__ == "__main__":
    main()