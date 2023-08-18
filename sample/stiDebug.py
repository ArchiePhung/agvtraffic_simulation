#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    - Phát hiện va chạm giữa các AGV
    - Ghi lại thời điểm, vị trí va chạm 
    - >> Gửi cảnh báo va chạm tới điện thoại 
"""

import roslib
roslib.load_manifest('simulate_agvs')
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import *
from simulate_agvs.msg import *
import math
import time 

class sti_Debug():
     def __init__(self):
          # -- init node -- 
          rospy.init_node('stiDebug', anonymous=False)
        
          # -- node publish -- 
          self.pub_stiDebug_info = rospy.Publisher('/stiDebug_info', stiDebug_respond, queue_size = 10)
          self.data_stiDebug_info = stiDebug_respond()
          self.rate = rospy.Rate(10.0)

          # -- số lượng AGV --
          self.number_agv = rospy.get_param('number_agv', '')

          # -- node subcribe -- 
          rospy.Subscriber('/agv1/infoRespond' , agv_infoRespond, self.handle_agv1_infoRespond)
          self.msg_agv1_infoRespond = agv_infoRespond()
          rospy.Subscriber('/agv2/infoRespond' , agv_infoRespond, self.handle_agv2_infoRespond)
          self.msg_agv2_infoRespond = agv_infoRespond()
          rospy.Subscriber('/agv3/infoRespond' , agv_infoRespond, self.handle_agv3_infoRespond)
          self.msg_agv3_infoRespond = agv_infoRespond()
          
          # -- thông số của AGV -- 
          self.agv_radius = rospy.get_param('agv_radius', 0.0)
          self.agv_diameter = 2*self.agv_radius
          self.agv_safety_zone = rospy.get_param('safety_zone', 0.0)

          # -- xác thực AGV có va chạm --
          self.is_agv12_collide = False
          self.is_agv13_collide = False
          self.is_agv23_collide = False
          
          self.error_id = 0
          # self.case_collision = 0

     def handle_agv1_infoRespond(self, data):
          self.msg_agv1_infoRespond = data

     def handle_agv2_infoRespond(self, data):
          self.msg_agv2_infoRespond = data

     def handle_agv3_infoRespond(self, data):
          self.msg_agv3_infoRespond = data 
     
     def set_value_pub(self, X1_, Y1_, X2_, Y2_):
          if X1_ == self.msg_agv1_infoRespond.x and Y1_ == self.msg_agv1_infoRespond.y and X2_ == self.msg_agv2_infoRespond.x and Y2_ == self.msg_agv2_infoRespond.y:
               self.data_stiDebug_info.is_agv12_collision = True
               self.data_stiDebug_info.collision_pose12_x1 = X1_
               self.data_stiDebug_info.collision_pose12_y1 = Y1_
               self.data_stiDebug_info.collision_pose12_x2 = X2_
               self.data_stiDebug_info.collision_pose12_y2 = Y2_

          elif X1_ == self.msg_agv1_infoRespond.x and Y1_ == self.msg_agv1_infoRespond.y and X2_ == self.msg_agv3_infoRespond.x and Y2_ == self.msg_agv3_infoRespond.y:
               self.data_stiDebug_info.is_agv13_collision = True
               self.data_stiDebug_info.collision_pose13_x1 = X1_
               self.data_stiDebug_info.collision_pose13_y1 = Y1_
               self.data_stiDebug_info.collision_pose13_x3 = X2_
               self.data_stiDebug_info.collision_pose13_y3 = Y2_

          elif X1_ == self.msg_agv2_infoRespond.x and Y1_ == self.msg_agv2_infoRespond.y and X2_ == self.msg_agv3_infoRespond.x and Y2_ == self.msg_agv3_infoRespond.y:
               self.data_stiDebug_info.is_agv23_collision = True
               self.data_stiDebug_info.collision_pose23_x2 = X1_
               self.data_stiDebug_info.collision_pose23_y2 = Y1_
               self.data_stiDebug_info.collision_pose23_x3 = X2_
               self.data_stiDebug_info.collision_pose23_y3 = Y2_

     def reset_value_pub(self, X1_,Y1_, X2_, Y2_):
          if X1_ == self.msg_agv1_infoRespond.x and Y1_ == self.msg_agv1_infoRespond.y and X2_ == self.msg_agv2_infoRespond.x and Y2_ == self.msg_agv2_infoRespond.y:
              self.data_stiDebug_info.is_agv12_collision = False
          elif X1_ == self.msg_agv1_infoRespond.x and Y1_ == self.msg_agv1_infoRespond.y and X2_ == self.msg_agv3_infoRespond.x and Y2_ == self.msg_agv3_infoRespond.y:
              self.data_stiDebug_info.is_agv13_collision = False 
          elif X1_ == self.msg_agv2_infoRespond.x and Y1_ == self.msg_agv2_infoRespond.y and X2_ == self.msg_agv3_infoRespond.x and Y2_ == self.msg_agv3_infoRespond.y:
              self.data_stiDebug_info.is_agv23_collision = False

     def detect_agv_collision(self, x1, y1, x2, y2):
          if x1 == x2 and y1 == y2: 
               self.set_value_pub(x1, y1, x2, y2)
               self.error_id = 1

          elif x1 == x2 and y1 != y2:
               if abs(y2 - y1) <= (self.agv_diameter + self.agv_safety_zone):
                    self.set_value_pub(x1, y1, x2, y2)
                    self.error_id = 2
               else:
                    self.reset_value_pub(x1, y1, x2, y2)
                    self.error_id = 0

          elif x1 != x2 and y1 == y2:
               if abs(x2 - x1) <= (self.agv_diameter + self.agv_safety_zone):
                    # print("Tao bị kẹt ở đây này ", "x2-x1 = ", abs(x2 - x1), "y1 =", y1, "y2 = ", y2)
                    self.set_value_pub(x1, y1, x2, y2)
                    self.error_id = 3
               else: 
                    self.reset_value_pub(x1, y1, x2, y2)
                    self.error_id = 0

          elif x1 != x2 and y1 != y2:
               self.a = 0.5*(x2 + x1) + 0.5*(pow(y2, 2) - pow(y1, 2))/(x2 - x1)
               self.b = (y1 - y2)/(x2 - x1)
               self.c = pow(self.b, 2) + 1
               self.d = 2*(self.a - x1)*self.b - 2*y1
               self.e = pow((self.a - x1), 2) + pow(y1, 2) - pow((self.agv_radius + self.agv_safety_zone), 2)
               self.delta = pow(self.d, 2) - 4*self.c*self.e
               
               if self.delta >= 0:
                    self.set_value_pub(x1, y1, x2, y2)
                    self.error_id = 4
               else:
                    self.reset_value_pub(x1, y1, x2, y2)
                    self.error_id = 0

     def display_status_agv_collision(self):
          if self.data_stiDebug_info.is_agv12_collision == True:
               if self.error_id == 1:
                    self.data_stiDebug_info.time12_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV1 và AGV2 va chạm trong trường hợp 1 lúc", self.data_stiDebug_info.time12_error)
               elif self.error_id == 2:
                    self.data_stiDebug_info.time12_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV1 và AGV2 va chạm trong trường hợp 2 lúc", self.data_stiDebug_info.time12_error)
               elif self.error_id == 3:
                    self.data_stiDebug_info.time12_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV1 và AGV2 va chạm trong trường hợp 3 lúc", self.data_stiDebug_info.time12_error)
               elif self.error_id == 4:
                    self.data_stiDebug_info.time12_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV1 và AGV2 va chạm trong trường hơp 4 lúc", self.data_stiDebug_info.time12_error)
          
          elif self.data_stiDebug_info.is_agv13_collision == True:
               if self.error_id == 1:
                    self.data_stiDebug_info.time13_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV1 và AGV3 va chạm trong trường hợp 1 lúc", self.data_stiDebug_info.time13_error)
               elif self.error_id == 2:
                    self.data_stiDebug_info.time13_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV1 và AGV3 va chạm trong trường hợp 2 lúc", self.data_stiDebug_info.time13_error)
               elif self.error_id == 3:
                    self.data_stiDebug_info.time13_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV1 và AGV3 va chạm trong trường hợp 3 lúc", self.data_stiDebug_info.time13_error)
               elif self.error_id == 4:
                    self.data_stiDebug_info.time13_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV1 và AGV3 va chạm trong trường hơp 4 lúc", self.data_stiDebug_info.time13_error)
          
          elif self.data_stiDebug_info.is_agv23_collision == True:
               if self.error_id == 1:
                    self.data_stiDebug_info.time23_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV2 và AGV3 va chạm trong trường hợp 1 lúc", self.data_stiDebug_info.time23_error)
               elif self.error_id == 2:
                    self.data_stiDebug_info.time23_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV2 và AGV3 va chạm trong trường hợp 2 lúc", self.data_stiDebug_info.time23_error)
               elif self.error_id == 3:
                    self.data_stiDebug_info.time23_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV2 và AGV3 va chạm trong trường hợp 3 lúc", self.data_stiDebug_info.time23_error)
               elif self.error_id == 4:
                    self.data_stiDebug_info.time23_error = time.strftime("%H:%M:%S", time.localtime())
                    # print("AGV2 và AGV3 va chạm trong trường hơp 4 lúc", self.data_stiDebug_info.time23_error)

          self.pub_stiDebug_info.publish(self.data_stiDebug_info)
          
     def run(self):
          while not rospy.is_shutdown():
               if self.msg_agv1_infoRespond.process == 1 or self.msg_agv2_infoRespond.process == 1 or self.msg_agv3_infoRespond.process == 1:
                    self.detect_agv_collision(self.msg_agv1_infoRespond.x, self.msg_agv1_infoRespond.y, self.msg_agv2_infoRespond.x, self.msg_agv2_infoRespond.y)
                    self.display_status_agv_collision()
                    self.detect_agv_collision(self.msg_agv1_infoRespond.x, self.msg_agv1_infoRespond.y, self.msg_agv3_infoRespond.x, self.msg_agv3_infoRespond.y)
                    self.display_status_agv_collision()
                    self.detect_agv_collision(self.msg_agv2_infoRespond.x, self.msg_agv2_infoRespond.y, self.msg_agv3_infoRespond.x, self.msg_agv3_infoRespond.y)
                    self.display_status_agv_collision()

               self.rate.sleep()

def main():
	print('Program starting')
	program = sti_Debug()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()