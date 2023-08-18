#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    - Phát hiện va chạm giữa các AGV
    - Ghi lại thời điểm, vị trí va chạm 
"""

import roslib
roslib.load_manifest('agvtraffic_simulation')
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import *
from agvtraffic_simulation.msg import *
import math
import time 

class Detect_collision():
     def __init__(self):
          # -- init node -- 
          rospy.init_node('detect_agv_collision', anonymous=False)
        
          # -- node publish -- 
          self.pub_DetectCollision = rospy.Publisher('/Detect_collision', Collision_info, queue_size = 10)
          self.data_DetectCollision = Collision_info()
          self.rate = rospy.Rate(30.0)

          # -- số lượng AGV --
          self.number_agv = rospy.get_param('number_agv', '')
          self.indexString = ""
          self.nameString = "agv"
          self.ListOfagv = []
          self.ListOfagv.append("")
          for i in range(1, (self.number_agv + 1)):
               self.indexString = str(i)
               self.ListOfagv.append(self.nameString + self.indexString)
         
          self.msg_agv = []
          self.msg_agv.append(0)

          # -- node subcribe -- 
          for i in range(1, (1 + self.number_agv)):
               rospy.Subscriber('/%s/agv_pose' % self.ListOfagv[i], Pose, self.agvPose_Callback, self.ListOfagv[i])
               self.msg_agv.append(Pose())
          
          # -- thông số của AGV -- 
          self.agv_radius = rospy.get_param('agv_radius', 0.2)
          self.agv_diameter = 2*self.agv_radius
          self.agv_safety_zone = rospy.get_param('safety_zone', 0.1)

          # -- xác thực AGV có va chạm --
          self.is_agv12_collide = False
          self.is_agv13_collide = False
          self.is_agv23_collide = False
          self.is_agv_collide = 0
          
          # -- mã lỗi -- 
          self.error_id = 0
          self.t = ""

          # -- biến trung gian để so sánh các tọa độ với nhau 
          self.res1 = []
          self.res2 = []
          self.is_receivedData = False

     def agvPose_Callback(self, data, agv_name):                         
          for i in range(1, (1 + self.number_agv)):
               if self.ListOfagv[i] == agv_name:
                    self.msg_agv[i] = data

          # if agv_name == self.ListOfagv[1]:
          #      self.msg_agv[1] = data
          # elif agv_name == self.ListOfagv[2]:
          #      self.msg_agv[2] = data
          # elif agv_name == self.ListOfagv[3]:
          #      self.msg_agv[3] = data
          
          self.res1 = self.msg_agv
          self.res2 = self.msg_agv
          self.is_receivedData = True

     def detect_agv_collision(self, x1, y1, x2, y2):
          if x1 == x2 and y1 != y2:
               if math.fabs(y2 - y1) <= (self.agv_diameter + self.agv_safety_zone):
                    self.t = time.strftime("%H:%M:%S", time.localtime())
                    self.is_agv_collide = 1
                    self.error_id = 1
               else:
                    self.is_agv_collide = 0
                    self.error_id = 0

          elif x1 != x2 and y1 == y2:
               if math.fabs(x2 - x1) <= (self.agv_diameter + self.agv_safety_zone):
                    self.t = time.strftime("%H:%M:%S", time.localtime())
                    self.is_agv_collide = 1
                    self.error_id = 2
               else: 
                    self.is_agv_collide = 0
                    self.error_id = 0

          elif x1 != x2 and y1 != y2 :
               TS = A = B = C = D = E = DELTA = 0
               TS = x2**2 + y2**2 - x1**2 - y1**2
               A = TS/2/(y2 - y1)
               B = (x1 - x2)/(y2 - y1)
               C = 1 + B**2
               D = 2*A*B - 2*x1 - 2*y1*B
               E = A**2 - 2*y1*A + x1**2 + y1**2 - (self.agv_radius + self.agv_safety_zone)**2
               DELTA = D**2 - 4*C*E

               if DELTA >= 0:
                    self.t = time.strftime("%H:%M:%S", time.localtime())
                    self.is_agv_collide = 1
                    self.error_id = 3
               else: 
                    self.is_agv_collide = 0
                    self.error_id = 0

          return self.is_agv_collide, self.error_id, self.t
     
     def run(self):
          while not rospy.is_shutdown(): 
               if self.is_receivedData == True:
                    for i in range(1, len(self.res1)):
                         for j in range(1, len(self.res2)):
                              if j != i:
                                   (is_collide, error_id, time_collide) = self.detect_agv_collision(self.res1[i].position.x, self.res1[i].position.y, self.res2[j].position.x, self.res2[j].position.y)
                                   if is_collide == 1:
                                        self.data_DetectCollision.agv_pairs = str(i) + str(j)
                                        self.data_DetectCollision.error_id = error_id
                                        self.data_DetectCollision.time_collision = time_collide
                                        print("AGV va chạm là %s và %s với trường hợp %s tại thời điểm %s" %(i,j,error_id, time_collide))
                                        self.pub_DetectCollision.publish(self.data_DetectCollision)
                                   else:
                                        self.data_DetectCollision.agv_pairs = ""
                                        self.data_DetectCollision.error_id = 0
                                        self.data_DetectCollision.time_collision = "" 
                 
               self.rate.sleep()

def main():
	print('Program starting')
	program = Detect_collision()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()