#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from detection_msgs.msg import BoundingBoxes
from collections import deque
import time

class ObjectDetectionNode:
    def __init__(self):
        self.objects_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.bboxes_callback)
        self.speed_pub = rospy.Publisher("/request/motor/speed", Float64, queue_size=1)

        self.frequency_threshold = 3
        self.time_period = 2  # in seconds
        self.detections_queue = deque()
        self.print_cnt = True
        self.mv_cnt = 0
        self.timer = rospy.Rate(5)


    def bboxes_callback(self, bboxes_msg):
        if self.mv_cnt == 0: 
            detected_objects = bboxes_msg.bounding_boxes
            detected_classes = {obj.Class for obj in detected_objects if obj.Class in {'moving_obstacle', 'machine_tower', 'roof'}}
            now = time.time()
            if detected_classes:
                self.detections_queue.append(now)

            # Check if the time difference of the first time stamp in the queue and now is greater than the time_period
            while self.detections_queue and now - self.detections_queue[0] > self.time_period:
                self.detections_queue.popleft()

            if len(self.detections_queue) >= self.frequency_threshold:
                # rospy.loginfo('Moving object detected')
                if self.print_cnt:
                    
                    rospy.set_param('mv_obs_detect_mode', True)
                    self.mv_cnt=1
                    self.speed_pub.publish(0)
                    print("[장애물 미션] 동적 장애물 발견! (정지 및 재출발 기능 미완)")
                    print("[장애물 미션] ((일단 5초 쉬기))")
                    rospy.Timer(rospy.Duration(7), self.reset_last_stop_detected, oneshot=True)
                    self.print_cnt = False
                self.speed_pub.publish(0)
        else: 
            rospy.Timer(rospy.Duration(30), self.mv_delay, oneshot=True)

    def mv_delay(self, event):
        self.mv_cnt = 0

    def reset_last_stop_detected(self,event):
        rospy.set_param('mv_obs_detect_mode', False)
        print("[장애물 미션] ((출발))")
        self.print_cnt = True


if __name__ == '__main__':
    rospy.init_node('mv_obstacle')
    node = ObjectDetectionNode()
    rospy.spin()
