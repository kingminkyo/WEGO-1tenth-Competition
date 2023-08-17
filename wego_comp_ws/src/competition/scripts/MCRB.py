#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float64
from math import *

############################################################
# range 90보다 높여보는 중 





############################################################


class LIDAR_RB():
    def __init__(self):
        self.myXY = [0,0]
        self.FILTER_VALUE = 10.0
        # self.now_hd = 0
        
        self.pub_angle = rospy.Publisher("/control/angle/mcmcmc", Float64, queue_size=1)
        self.lidar_rb = rospy.Publisher("/lidar2D_RB", LaserScan, queue_size=1)
        self.point_rb = rospy.Publisher('/Point_RB', Int32, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

    def get_dist(self, data, angle, deg=True):
        if deg:
            angle = np.deg2rad(angle)
        dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
        if dis < data.range_min or dis > data.range_max:
            dis = self.FILTER_VALUE
        return dis
        
    def lidar_callback(self, data):
        # lidar original data
        # org_ranges = list(data.ranges)
        dis = []
        for angle in range(-180,180):
            dis.append(self.get_dist(data,angle))
        modified_ranges = dis
                
        # constant number
        angle_increment = 0.01745329238474369 ############### deg로 따지면 1도임
        
        # define search angle
        sa = 150
        # convert angle for searching
        ranges = modified_ranges[360-sa:360] + modified_ranges[0:sa+1] ############### 0~360 이었구나! 
        
        ALL_XY = [] ############### '라이다 좌표계'에서 라이다 중심(0,0)에서 해당 range까지의 상대 좌표차이
        ROI_TXY = []
        for i in range(len(ranges)):
            x = ranges[i]*np.cos(angle_increment*(i-60))
            y = ranges[i]*np.sin(angle_increment*(i-60))
            ALL_XY.append([x,y])
            
            # set ROI 5m around the car
            if self.distance([0,0], [x,y])<2:
                ROI_TXY.append([i,x,y]) 
                
        # exception process
        try:
            RB_TXY = [ROI_TXY[0]]
        except:
            self.lidar_rb.publish(data) ############### "내가 추가함"
            return
        
        Angle_Relation = {}
        Angle_Relation[ROI_TXY[0][0]]=[] ############### value가 list임
        for i in range(len(ROI_TXY)):
            fac=True
            for j in range(len(RB_TXY)): ############### len(RB_TXY)는 1 아닌가???? (X) --- 처음에는 1이지만 다른 라바콘이 탐지될 때마다 증가함.
                # use given information that rabacon diameter is under 0.1m
                if self.distance([RB_TXY[j][1], RB_TXY[j][2]], [ROI_TXY[i][1],ROI_TXY[i][2]])<0.1:
                    fac=False
            if fac: ############### 다른 라바콘 포인트의 경우 RB_TXY에 추가해줌. 
                RB_TXY.append(ROI_TXY[i])
                Angle_Relation[ROI_TXY[i][0]] = []


        ############### 모든 라바콘 포인트를 찾았으면, 0.3m 이내의 라바콘들끼리 관계성 나타내기 
        for i in range(len(RB_TXY)):
            for j in range(i+1,len(RB_TXY)):
                # use given information that each rabacon locate 0.3 meters away from each other
                # if self.distance([RB_TXY[i][1],RB_TXY[i][2]], [RB_TXY[j][1],RB_TXY[j][2]])<0.3

                ############### 낮으면 벽 안이어짐. 높으면 벽 아닌 라바콘들끼리도 이어짐 :
                # if self.distance([RB_TXY[i][1],RB_TXY[i][2]], [RB_TXY[j][1],RB_TXY[j][2]])<0.65: 
                if self.distance([RB_TXY[i][1],RB_TXY[i][2]], [RB_TXY[j][1],RB_TXY[j][2]])<0.4: 
                    # make relation graph of each rabacon
                    Angle_Relation[RB_TXY[i][0]].append(RB_TXY[j][0])
                    Angle_Relation[RB_TXY[j][0]].append(RB_TXY[i][0])


        # use depth first search to make tree of rabacon relation
        self.Angle_Relation = Angle_Relation
        self.mc_visited = [False for _ in range(sa*2+1)]
        # self.mc_visited = [False] * len(ranges)
        Lidar_Correction = []
        for i in range(len(RB_TXY)):
            if self.mc_visited[RB_TXY[i][0]]==False:
                temp = self.dfs(RB_TXY[i][0],[]) ############### temp는 연결된 라바콘의 인덱스 리스트 
                # ignore short tree and re-dfs at the end for make correction list
                if len(temp)>2:
                    Lidar_Correction.append(self.dfs(temp[-1],[]))
        
        
        min_distance_angle = [] # [distance, angle]
        Temp_D= {}
        
        CORRECTION_TXY = []
        for relation_list in Lidar_Correction: ############### (?) relation_list 오름차순으로 되어 있나??
            temp = []
            for i in range(len(relation_list)-1):
                # range correction by using inner branch
                ang_s, x_s, y_s = relation_list[i], ALL_XY[relation_list[i]][0], ALL_XY[relation_list[i]][1]
                ang_e, x_e, y_e = relation_list[i+1], ALL_XY[relation_list[i+1]][0], ALL_XY[relation_list[i+1]][1]
                if ang_s>ang_e:
                    ang_e, x_e, y_e = relation_list[i], ALL_XY[relation_list[i]][0], ALL_XY[relation_list[i]][1]
                    ang_s, x_s, y_s = relation_list[i+1], ALL_XY[relation_list[i+1]][0], ALL_XY[relation_list[i+1]][1]
                # use for maneuver car in front of rabacon
                Temp_D[ang_s] = self.distance([0,0], [x_s, y_s]) ############### 라이다 중심(원점)과 특정 인덱스 포인트 간의 거리
                Temp_D[ang_e] = self.distance([0,0], [x_e, y_e])
                for j in range(ang_s+1,ang_e):
                    corr_x = x_s + (x_e-x_s)*((j-ang_s)/(ang_e-ang_s)) ############### 라바콘 사이 벽을 위한 좌표 계산
                    corr_y = y_s + (y_e-y_s)*((j-ang_s)/(ang_e-ang_s))
                    
                    # calculate minimum distance with each tree
                    if 60<j<120:                                        ############### (?) 정면에서 양쪽 30도 뜻함! 왜냐면 +x 방향에서 -x 방향으로 180도이므로
                        temp.append([self.distance([0,0], [corr_x, corr_y]), j]) 
                    try:
                        if Temp_D[j]>self.distance([0,0], [corr_x, corr_y]):
                            Temp_D[j] = self.distance([0,0], [corr_x, corr_y])
                            CORRECTION_TXY.append([j, corr_x, corr_y])
                    except:
                        Temp_D[j] = self.distance([0,0], [corr_x, corr_y])
                        CORRECTION_TXY.append([j, corr_x, corr_y]) ############### 라바콘 사이 벽 좌표들
            # sort tree by distance with car
            temp.sort()                                         ############### (?)(?) temp 어떤건지 파악 못함 + 이중리스트인데 어떤 것을 기준으로 sort?
            # exception process
            if temp:
                min_distance_angle.append(temp[0])
        try:
            # sort angle information by distance with car
            min_distance_angle.sort()
            
            # two angle that car will pursuit
            a = min_distance_angle[0][0]
            b = min_distance_angle[1][0] 
            
            # correction control angle by adding weight of distance
            johanggack = -(min_distance_angle[0][1]*(a/(a+b)) + min_distance_angle[1][1]*(b/(a+b))) + 90
            # print(johanggack)
            offset = 19.4799995422
            steer = Float64()
            steer.data = (johanggack+offset)/(2*offset)
            # print(steer.data)
            
            if steer.data>1.0:
                steer.data=1.0
            elif steer.data<0:
                steer.data=0
            # self.pub_angle.publish(steer)
            # print('\n')
        except: pass
                
        # new_ranges = list(data.ranges)
        new_ranges = modified_ranges
        for i in range(len(CORRECTION_TXY)):
            angle = CORRECTION_TXY[i][0]
            # angle_rad = np.deg2rad(angle)
            x = CORRECTION_TXY[i][1]
            y = CORRECTION_TXY[i][2]
            # reconvert angle for lidar data
            if angle<sa:
                new_ranges[360-sa+angle] = self.distance([0,0], [x,y])
            else:
                new_ranges[angle-sa] = self.distance([0,0], [x,y])
            


        # count number of angle that modified
        cnt = 0
        for i in range(len(new_ranges)):
            if new_ranges[i]<10 and new_ranges[i]!=modified_ranges[i]:
                cnt += 1

        data.ranges = new_ranges
        data.angle_increment = 0.01745329238474369

        self.lidar_rb.publish(data)
        self.point_rb.publish(cnt)
        # print(len(new_ranges))

    def distance(self, myXY, wayXY):
        '''return distance btw 2pt
        '''
        return ((myXY[0]-wayXY[0])**2+(myXY[1]-wayXY[1])**2)**0.5

    def dfs(self, start, visited=[]):
        '''deep first search
        
        search relationship btw rabacons
        '''
        global Angle_Relation, mc_visited ############### 필요 없는듯?
        visited.append(start)
        self.mc_visited[start] = True
        for node in self.Angle_Relation[start]:
            if node not in visited:
                self.mc_visited[start] = True
                self.dfs(node, visited)
        return visited
def run():
    rospy.init_node("lidarRB")
    LIDAR_RB()
    rospy.spin()


if __name__ == '__main__':
    run()