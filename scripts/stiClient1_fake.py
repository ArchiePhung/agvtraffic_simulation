#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    - Gửi lộ trình cho AGV 
    - Dừng AGV nếu va chạm với AGV khác
"""

import roslib
roslib.load_manifest('agvtraffic_simulation')
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import *
from agvtraffic_simulation.msg import *
import math
import time 

class Fake_stiClient1():
    def __init__(self):
        # -- init node -- 
        rospy.init_node('stiClient1_fake', anonymous=False)
        
        # -- node publish -- 
        # self.agv_name = rospy.get_param('~agv_name')
        self.pub_agv1_requestMove = rospy.Publisher('/agv1/request_move_fake', Move_request, queue_size = 10)
        self.data_agv1_requestMove = Move_request()

        self.rate = rospy.Rate(10.0)
        
        # -- node subcribe -- 
        # rospy.Subscriber('/Detect_collision', Collision_info, self.handle_DetectCollision)
        # self.msg_detectCollision = Collision_info()

        rospy.Subscriber('/Process_collision', process_agv_collision, self.handle_ProcessCollision)
        self.msg_processCollision = process_agv_collision()

        rospy.Subscriber('/agv1/NN_infoRespond', NN_infoRespond, self.handle_DetectErr)
        self.msg_detectErr = NN_infoRespond()        

        # -- biến lộ trình cho AGV 
        self.agv1_list_index = 0            # agv1 
        self.agv1_list_pub_x = []
        self.agv1_list_pub_y = []
        self.agv1_list_len = 5              

        # -- biến hệ thống --
        self.DieTime_pub = time.time()           # thời gian chờ pub
        self.mode = 0                            # chế độ hoạt động 
    
    # # -- hàm nhận dữ liệu từ stiDebug
    # def handle_DetectCollision(self, data):
    #     self.msg_detectCollision = data

    def handle_ProcessCollision(self, data):
        self.msg_processCollision = data

    def handle_DetectErr(self, data):
        self.msg_detectErr = data

    def run(self):
        while not rospy.is_shutdown():
            if self.mode == 0:                                           # khi các AGV chưa chạy 
                if time.time() - self.DieTime_pub < 3:                   # chờ 2s rồi pub lộ trình tiếp theo
                    # -- lộ trình AGV1 --
                    self.agv1_list_pub_x = [0,3,3,4,4]
                    self.agv1_list_pub_y = [2,2,4,4,5]
                    # self.agv1_list_pub_x = [0,2,2,4,5]
                    # self.agv1_list_pub_y = [2,2,0,0,0]

                else:

                    # gửi lộ trình cho các AGV 
                    self.data_agv1_requestMove.enable = 1
                    self.data_agv1_requestMove.list_x = self.agv1_list_pub_x
                    self.data_agv1_requestMove.list_y = self.agv1_list_pub_y
                    self.pub_agv1_requestMove.publish(self.data_agv1_requestMove)

                    self.DieTime_pub = time.time()
                    self.mode = 1

            elif self.mode == 1:
                if self.msg_processCollision.is_agv1_collide == 0:
                    if len(self.msg_detectErr.listError) > 0:
                        self.data_agv1_requestMove.enable = 0
                    else:
                        self.data_agv1_requestMove.enable = 1
                else:
                    self.data_agv1_requestMove.enable = 0

                # if (self.msg_detectCollision.agv_pairs == "12" or self.msg_detectCollision.agv_pairs == "21" or
                #         self.msg_detectCollision.agv_pairs == "13" or self.msg_detectCollision.agv_pairs == "31" or
                #             self.msg_detectCollision.agv_pairs == "14" or self.msg_detectCollision.agv_pairs == "41" or
                #                 len(self.msg_detectErr.listError) > 0) :
                #     self.data_agv1_requestMove.enable = 0

                # elif (self.msg_detectCollision.agv_pairs != "12" and self.msg_detectCollision.agv_pairs != "21" and
                #         self.msg_detectCollision.agv_pairs != "13" and self.msg_detectCollision.agv_pairs != "23" and
                #             self.msg_detectCollision.agv_pairs != "14" and self.msg_detectCollision.agv_pairs != "41" and
                #                 len(self.msg_detectErr.listError) == 0) :
                #     self.data_agv1_requestMove.enable = 1

                self.pub_agv1_requestMove.publish(self.data_agv1_requestMove)

            self.rate.sleep()

def main():
	print('Program starting')
	program = Fake_stiClient1()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()