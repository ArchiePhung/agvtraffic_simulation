#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    - Điều khiển các AGV chạy trên rviz 
"""

import roslib
roslib.load_manifest('agvtraffic_simulation')
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import *
# from simulate_agvs.msg import *
from agvtraffic_simulation.msg import *
import math
import time 

class Fake_stiClient():
    def __init__(self):
        # -- init node -- 
        rospy.init_node('stiClient_fake', anonymous=False)
        
        # -- node publish -- 
        # self.agv_name = rospy.get_param('~agv_name')
        self.pub_agv1_requestMove = rospy.Publisher('/agv1/request_move_fake', Move_request, queue_size = 10)
        self.data_agv1_requestMove = Move_request()

        self.pub_agv2_requestMove = rospy.Publisher('/agv2/request_move_fake', Move_request, queue_size = 10)
        self.data_agv2_requestMove = Move_request()

        self.pub_agv3_requestMove = rospy.Publisher('/agv3/request_move_fake', Move_request, queue_size = 10)
        self.data_agv3_requestMove = Move_request()

        self.pub_agv4_requestMove = rospy.Publisher('/agv4/request_move_fake', Move_request, queue_size = 10)
        self.data_agv4_requestMove = Move_request()

        self.rate = rospy.Rate(50.0)
        
        # -- node subcribe -- 
        rospy.Subscriber('/Detect_collision', Collision_info, self.handle_DetectCollision)
        self.msg_detectCollision = Collision_info()

        # -- biến lộ trình cho AGV 
        self.agv1_list_index = 0            # agv1 
        self.agv1_list_pub_x = []
        self.agv1_list_pub_y = []
        self.agv1_list_len = 5              

        self.agv2_list_index = 0            # agv2 
        self.agv2_list_pub_x = []
        self.agv2_list_pub_y = []
        self.agv2_list_len = 5              

        self.agv3_list_index = 0            # agv3
        self.agv3_list_pub_x = []
        self.agv3_list_pub_y = []
        self.agv3_list_len = 5   

        self.agv4_list_index = 0            # agv3
        self.agv4_list_pub_x = []
        self.agv4_list_pub_y = []
        self.agv4_list_len = 5 

        # -- biến hệ thống --
        self.DieTime_pub = time.time()           # thời gian chờ pub
        self.mode = 0                            # chế độ hoạt động 
    
    # # -- hàm nhận dữ liệu từ stiDebug
    def handle_DetectCollision(self, data):
        self.msg_detectCollision = data
        # if self.msg_detectCollision 

    def run(self):
        while not rospy.is_shutdown():
            if self.mode == 0:                                           # khi các AGV chưa chạy 
                if time.time() - self.DieTime_pub < 3:                   # chờ 2s rồi pub lộ trình tiếp theo
                    # -- lộ trình AGV1 --
                    self.agv1_list_pub_x = [0,3,3,4,4]
                    self.agv1_list_pub_y = [2,2,4,4,5]
                    # self.agv1_list_pub_x = [0,2,2,4,5]
                    # self.agv1_list_pub_y = [2,2,0,0,0]

                    # -- lộ trình AGV2 --
                    self.agv2_list_pub_x = [3,3,4,4,5]
                    self.agv2_list_pub_y = [1,2,2,4,4]

                    # -- lộ trình AGV3 --
                    self.agv3_list_pub_x = [0,-1,-1,4,4]
                    self.agv3_list_pub_y = [1,1,2,2,5]
                    # self.agv3_list_pub_x = [-1,-1,4,4,4]
                    # self.agv3_list_pub_y = [1,2,2,4,5]

                    # -- lộ trình AGV4
                    # self.agv4_list_pub_x = [0,-1,-1,4,4]
                    # self.agv4_list_pub_y = [2,1,2,2,5]
                    self.agv4_list_pub_x = [0,2,2,4,5]
                    self.agv4_list_pub_y = [-2,-2,0,0,0]
                else:

                    # gửi lộ trình cho các AGV 
                    self.data_agv1_requestMove.enable = 1
                    self.data_agv1_requestMove.list_x = self.agv1_list_pub_x
                    self.data_agv1_requestMove.list_y = self.agv1_list_pub_y
                    self.pub_agv1_requestMove.publish(self.data_agv1_requestMove)

                    self.data_agv2_requestMove.enable = 1
                    self.data_agv2_requestMove.list_x = self.agv2_list_pub_x
                    self.data_agv2_requestMove.list_y = self.agv2_list_pub_y
                    self.pub_agv2_requestMove.publish(self.data_agv2_requestMove)

                    self.data_agv3_requestMove.enable = 1
                    self.data_agv3_requestMove.list_x = self.agv3_list_pub_x
                    self.data_agv3_requestMove.list_y = self.agv3_list_pub_y
                    self.pub_agv3_requestMove.publish(self.data_agv3_requestMove)

                    self.data_agv4_requestMove.enable = 1
                    self.data_agv4_requestMove.list_x = self.agv4_list_pub_x
                    self.data_agv4_requestMove.list_y = self.agv4_list_pub_y
                    self.pub_agv4_requestMove.publish(self.data_agv4_requestMove)

                    self.DieTime_pub = time.time()
                    self.mode = 1

            elif self.mode == 1:

                if self.msg_detectCollision.agv_pairs == "13" or self.msg_detectCollision.agv_pairs== "31":
                    self.data_agv1_requestMove.enable = 0
                    self.data_agv3_requestMove.enable = 0

                elif self.msg_detectCollision.agv_pairs == "23" or self.msg_detectCollision.agv_pairs == "32":
                    self.data_agv2_requestMove.enable = 0
                    self.data_agv3_requestMove.enable = 0 

                elif self.msg_detectCollision.agv_pairs == "12" or self.msg_detectCollision.agv_pairs == "21":
                    self.data_agv1_requestMove.enable = 0
                    self.data_agv2_requestMove.enable = 0 
                
                elif self.msg_detectCollision.agv_pairs == "14" or self.msg_detectCollision.agv_pairs == "41":
                    self.data_agv1_requestMove.enable = 0
                    self.data_agv4_requestMove.enable = 0 

                elif self.msg_detectCollision.agv_pairs == "24" or self.msg_detectCollision.agv_pairs == "42":
                    self.data_agv2_requestMove.enable = 0
                    self.data_agv4_requestMove.enable = 0

                elif self.msg_detectCollision.agv_pairs == "34" or self.msg_detectCollision.agv_pairs == "43":
                    self.data_agv3_requestMove.enable = 0
                    self.data_agv4_requestMove.enable = 0

                self.pub_agv1_requestMove.publish(self.data_agv1_requestMove)
                self.pub_agv2_requestMove.publish(self.data_agv2_requestMove)
                self.pub_agv3_requestMove.publish(self.data_agv3_requestMove)
                self.pub_agv4_requestMove.publish(self.data_agv4_requestMove)

            self.rate.sleep()

def main():
	print('Program starting')
	program = Fake_stiClient()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()