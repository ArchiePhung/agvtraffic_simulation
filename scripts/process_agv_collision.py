#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    - Xử lý khi có xảy ra va chạm giữa các AGV. 
"""

import roslib
roslib.load_manifest('agvtraffic_simulation')
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import *
from agvtraffic_simulation.msg import *
import math
import time 

class Process_Collide():
    def __init__(self):
        # -- init node -- 
        rospy.init_node('process_AGVcollision', anonymous=False)
        
        # -- node publish -- 
        # self.agv_name = rospy.get_param('~agv_name')
        self.pub_agv_collision = rospy.Publisher('/Process_collision', process_agv_collision, queue_size = 10)
        self.data_agv_collision = process_agv_collision()

        self.rate = rospy.Rate(30.0)
        
        # -- node subcribe -- 
        rospy.Subscriber('/Detect_collision', Collision_info, self.handle_DetectCollision)
        self.msg_detectCollision = Collision_info()      

        # -- GLOBAL VARIABLES -- 

    # # -- hàm nhận dữ liệu từ stiDebug
    def handle_DetectCollision(self, data):
        self.msg_detectCollision = data

    def run(self):
        while not rospy.is_shutdown():
            if self.msg_detectCollision.agv_pairs == "12" or self.msg_detectCollision.agv_pairs == "21":
                self.data_agv_collision.is_agv1_collide = 1
                self.data_agv_collision.is_agv2_collide = 1

            elif self.msg_detectCollision.agv_pairs == "13" or self.msg_detectCollision.agv_pairs == "31":
                self.data_agv_collision.is_agv1_collide = 1
                self.data_agv_collision.is_agv3_collide = 1
            
            elif self.msg_detectCollision.agv_pairs == "14" or self.msg_detectCollision.agv_pairs == "41":
                self.data_agv_collision.is_agv1_collide = 1
                self.data_agv_collision.is_agv4_collide = 1

            elif self.msg_detectCollision.agv_pairs == "23" or self.msg_detectCollision.agv_pairs == "32":
                self.data_agv_collision.is_agv2_collide = 1
                self.data_agv_collision.is_agv3_collide = 1

            elif self.msg_detectCollision.agv_pairs == "24" or self.msg_detectCollision.agv_pairs == "42":
                self.data_agv_collision.is_agv2_collide = 1
                self.data_agv_collision.is_agv4_collide = 1
            
            elif self.msg_detectCollision.agv_pairs == "34" or self.msg_detectCollision.agv_pairs == "43":
                self.data_agv_collision.is_agv3_collide = 1
                self.data_agv_collision.is_agv4_collide = 1                

            self.pub_agv_collision.publish(self.data_agv_collision)
            self.rate.sleep()

def main():
	print('Program starting')
	program = Process_Collide()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()