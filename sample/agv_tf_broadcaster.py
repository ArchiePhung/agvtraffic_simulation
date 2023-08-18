#!/usr/bin/env python3  
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('simulate_agvs')
import rospy

import time
import math
import tf
from simulate_agvs.msg import *
from sti_msgs.msg import NN_cmdRequest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class AGV_TF():
    def __init__(self):

        # -- ros init --
        rospy.init_node('agv_tf_broadcaster')
        self.rate = rospy.Rate(60.0)
        
        self.agv_name = rospy.get_param('~agv_name')
        # -- ros subcriber
        rospy.Subscriber('/%s/NN_cmdRequest' % self.agv_name, NN_cmdRequest, self.handle_NN_cmdRequest)
        self.msg_traffic = NN_cmdRequest()
        
        rospy.Subscriber('/%s/permission_running' % self.agv_name, stiClient_respond, self.handle_permission_running)
        self.msg_permission_running = stiClient_respond()

        rospy.Subscriber('/%s/ProcessError_info' % self.agv_name, agv_infoError, self.handle_ProcessError_info)
        self.msg_ProcessError_info = agv_infoError()

        # -- ros publish 
        # self.pub_path_local = rospy.Publisher('path_plan_local', Path, queue_size= 20)
        self.pub_path_global = rospy.Publisher('/%s/path_plan_global' % self.agv_name, Path, queue_size= 20)
        self.pub_agv_pose = rospy.Publisher('/%s/infoRespond' % self.agv_name, agv_infoRespond, queue_size=20)
        self.agv_info = agv_infoRespond()

        # -- Mô tả vị trí ban đầu của AGV hiển thị trên tf rviz   -- 
        self.agv_start_pose_x = rospy.get_param('~agv_start_x')
        self.agv_start_pose_y = rospy.get_param('~agv_start_y')
        self.agv_start_angle = rospy.get_param('~agv_start_angle')

        self.translation = (self.agv_start_pose_x, self.agv_start_pose_y, 0.0)
        
        # -- Khai báo tf -- 
        self.br = tf.TransformBroadcaster()
        self.agv_frame =  self.agv_name
        self.origin_frame = "world"

        # -- journey info 
        self.path_plan = Path()
        self.path_plan.header.frame_id = self.origin_frame
        self.path_plan.header.stamp = rospy.Time.now()
        self.path_plan.poses.append(self.point_path(self.agv_start_pose_x, self.agv_start_pose_y))

        # -- global variable -- 
        # -- vị trí trước đó của agv 
        self.agv_pre_pose_x = self.agv_start_pose_x
        self.agv_pre_pose_y = self.agv_start_pose_y
        self.agv_pre_angle = self.agv_start_angle
        # -- khoảng di chuyển theo chiều x và y 
        self.move_distance_x = 0
        self.move_distance_y = 0
        # -- agv bắt đầu chạy nếu có lộ trình dược gửi đến
        # self.alow_to_run = 0
        #-- agv đã thực hiện việc  quay xong 
        self.agv_finish_spin = 0

        # -- vị trí điểm đích tiếp theo của agv theo lộ trình gửi xuống  
        self.agv_target_pose_x = 0
        self.agv_target_pose_y = 0
        self.agv_target_angle = 0
        self.agv_target_index = 0

        # -- AGV thực hiện di chuyển trên rviz 
        self.x = self.agv_pre_pose_x
        self.y = self.agv_pre_pose_y
        self.theta = self.agv_pre_angle
        self.time_move = rospy.get_param('~agv_time_linear')           
        self.time_spin = rospy.get_param('~agv_time_angular')            # 0.02
        self.time_count_move = time.time()
        self.time_count_spin = time.time()

        self.vel_count_move = rospy.get_param('~agv_vel_linear')
        self.vel_count_spin = rospy.get_param('~agv_vel_angular')

        # -- Thông số điện của AGV -- 
        self.max_voltage = rospy.get_param('max_voltage', '')
        self.min_voltage = rospy.get_param('min_voltage', '')
        self.current_voltage = self.max_voltage
        self.time_count_voltage = time.time()

        # -- Thời gian dừng chờ -- 
        self.time_count_waitStop = time.time()

    def handle_ProcessError_info(self, data):
        # print(data)
        self.msg_ProcessError_info = data
        self.time_count_voltage = time.time()
        self.time_count_waitStop = time.time()

        if self.msg_ProcessError_info.agv_common_error == 1 or self.msg_ProcessError_info.agv_common_error == 2 or self.msg_ProcessError_info.agv_common_error == 3:
                print(self.agv_name, " đã dừng do AGV gặp lỗi ")
                self.msg_permission_running.allow_to_run = False                     # AGV ko chạy với lỗi bất kì
                self.agv_info.process = 0 

    def handle_permission_running(self, data):
        self.msg_permission_running = data

    def point_path(self, x, y):
        point = PoseStamped()
        point.header.frame_id = self.origin_frame
        point.header.stamp = rospy.Time.now()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = 0
        point.pose.orientation.w = 1.0
        return point
    
    def handle_NN_cmdRequest(self, data):
        self.msg_traffic = data
        self.time_count_move = time.time()
        self.time_count_spin = time.time()

        self.agv_target_pose_x = self.msg_traffic.list_x
        self.agv_target_pose_y = self.msg_traffic.list_y

        self.move_distance_x = self.agv_target_pose_x[self.agv_target_index] - self.agv_pre_pose_x
        self.move_distance_y = self.agv_target_pose_y[self.agv_target_index] - self.agv_pre_pose_y
        
        if self.move_distance_x == 0 and self.move_distance_y > 0:
            self.agv_target_angle = 1.5708
            self.is_orient = 2          # agv di chuyển lên 
        elif self.move_distance_x == 0 and self.move_distance_y < 0:
            self.agv_target_angle = -1.5708
            self.is_orient = 4          # agv di chuyển dưới 
        elif self.move_distance_x < 0 and self.move_distance_y == 0:
            self.agv_target_angle = 3.1416
            self.is_orient = 3          # agv di chuyển trái 
        elif self.move_distance_x > 0 and self.move_distance_y == 0:
            self.agv_target_angle = 0
            self.is_orient = 1          # agv di chuyển phải

        # self.alow_to_run = 1
        
        self.agv_info.is_reach_target_point = 0
        for i in range(len(self.msg_traffic.list_x)):
            point = PoseStamped()
            point.header.frame_id = self.origin_frame
            point.header.stamp = rospy.Time.now()
            point.pose.position.x = self.msg_traffic.list_x[i]
            point.pose.position.y = self.msg_traffic.list_y[i]
            point.pose.position.z = 0.
            point.pose.orientation.w = 1.0
            # print(point)
            self.path_plan.poses.append(point)
        # print(self.path_plan.poses)
        self.pub_path_global.publish(self.path_plan)

        # cho các node khác biết là AGV đã bắt đầu di chuyển
        self.agv_info.process = 1
        self.pub_agv_pose.publish(self.agv_info)
        print(self.agv_name," đã bắt đầu chạy!")

    def run(self):
        while not rospy.is_shutdown():
            self.br.sendTransform(self.translation, tf.transformations.quaternion_from_euler(0, 0, self.theta), rospy.Time.now(), self.agv_frame, self.origin_frame)
            # -- AGV Xử lý điện áp thấp -- 
            if self.current_voltage >= self.min_voltage:
                # print("AGV đang đủ điện áp V= ", self.current_voltage)
                if time.time() - self.time_count_voltage >= self.msg_ProcessError_info.agv_time_decrease_voltage:
                    self.current_voltage = self.current_voltage - self.msg_ProcessError_info.agv_delta_voltage
                    self.time_count_voltage = time.time()
            else:
                print(self.agv_name," đã dừng do điện áp thấp")
                self.msg_permission_running.allow_to_run = False                     # điện áp < điện áp min thì ko cho AGV chạy nữa

            # -- AGV xử lý dừng chờ AGV khác-- 
            if self.msg_ProcessError_info.agv_waitStop == 1:
                if time.time() - self.time_count_waitStop <= self.msg_ProcessError_info.agv_time_waitStop:
                    # print("AGV đang dừng tránh AGV khác ")
                    self.msg_permission_running.allow_to_run = False                 # AGV dừng tránh AGV khác 
                else:
                    # print("AGV tiếp tục chạy sau dừng tránh" ,self.msg_ProcessError_info.agv_waitStop)
                    self.msg_permission_running.allow_to_run = True
                    self.msg_ProcessError_info.agv_waitStop = 0

            # -- AGV với lỗi bất kì -- 
            # ví dụ: id = 1: bắt vật cản, id = 2: ko phát hiện gương, id = 3: mất kết nối mạch  
            if self.msg_ProcessError_info.agv_common_error == 1 or self.msg_ProcessError_info.agv_common_error == 2 or self.msg_ProcessError_info.agv_common_error == 3:
                print(self.agv_name," đã dừng do AGV gặp lỗi ")
                self.msg_permission_running.allow_to_run = False                     # AGV ko chạy với lỗi bất kì 

            # -- AGV di chuyển -- 
            if self.msg_permission_running.allow_to_run == True: 
                # -- agv di chuyển tiến -- 
                if self.move_distance_x > 0 and self.move_distance_y == 0:           #agv di chuyển theo trục x tăng 
                    if self.agv_finish_spin == 1:
                        if time.time() - self.time_count_move >= self.time_move:
                            self.x += self.vel_count_move
                            if self.x >= self.agv_target_pose_x[self.agv_target_index]:
                                self.move_distance_x = 0
                                self.x = self.agv_target_pose_x[self.agv_target_index]
                            
                            self.translation = (self.x, self.agv_target_pose_y[self.agv_target_index], 0)
                            self.time_count_move = time.time()
                        # print(self.translation)
                        # time.sleep(self.time_move)

                elif self.move_distance_x < 0 and self.move_distance_y == 0:         #agv di chuyển theo trục x giảm 
                    if self.agv_finish_spin == 1:
                        if time.time() - self.time_count_move >= self.time_move:
                            self.x -= self.vel_count_move
                            if self.x <= self.agv_target_pose_x[self.agv_target_index]:
                                self.move_distance_x = 0
                                self.x = self.agv_target_pose_x[self.agv_target_index]

                            self.translation = (self.x, self.agv_target_pose_y[self.agv_target_index], 0)
                            self.time_count_move = time.time()
                        # print(self.translation)
                        # time.sleep(self.time_move)

                elif self.move_distance_x == 0 and self.move_distance_y > 0:         #agv di chuyển theo trục y tăng 
                    if self.agv_finish_spin == 1:
                        if time.time() - self.time_count_move >= self.time_move:
                            self.y += self.vel_count_move
                            if self.y >= self.agv_target_pose_y[self.agv_target_index]:
                                self.move_distance_y = 0
                                self.y = self.agv_target_pose_y[self.agv_target_index]
                            
                            self.translation = (self.agv_target_pose_x[self.agv_target_index], self.y, 0)
                            self.time_count_move = time.time()
                        # print(self.translation)
                        # time.sleep(self.time_move)  
                
                elif self.move_distance_x == 0 and self.move_distance_y < 0:          #agv di chuyển theo trục y giảm
                    if self.agv_finish_spin == 1:
                        if time.time() - self.time_count_move >= self.time_move:
                            self.y -= self.vel_count_move
                            if self.y <= self.agv_target_pose_y[self.agv_target_index]:
                                self.move_distance_y = 0
                                self.y = self.agv_target_pose_y[self.agv_target_index]
                            
                            self.translation = (self.agv_target_pose_x[self.agv_target_index], self.y, 0)
                            self.time_count_move = time.time()
                        # print(self.translation)
                        # time.sleep(self.time_move)

                elif self.move_distance_x == 0 and self.move_distance_y == 0:         #agv di chuyển hết một điểm từ lộ trình gửi xuống 
                    # print("AGV đã chạy tới điểm thứ ", self.agv_target_index + 1)
                    self.agv_finish_spin = 0
                    # self.agv_target_angle = 0
                    del self.path_plan.poses[0]
                    # self.pub_path_global.publish(self.path_plan)

                    self.agv_pre_pose_x = self.agv_target_pose_x[self.agv_target_index]
                    self.agv_pre_pose_y = self.agv_target_pose_y[self.agv_target_index]
                    
                    self.agv_pre_angle = self.agv_target_angle                  # -- 
                    self.theta = self.agv_pre_angle
                    # print("Goc trước đó là" , self.agv_pre_angle)

                    self.agv_target_index = self.agv_target_index + 1             # agv chạy hết 5 điểm 
                    if self.agv_target_index > 4:
                        # self.msg_permission_running.allow_to_run = False
                        self.agv_info.process = 0
                        self.agv_target_index = 0
                        self.agv_info.is_reach_target_point = True                   
                        # print("AGV đã dừng lại")

                    # time.sleep(2)
                    self.move_distance_x = self.agv_target_pose_x[self.agv_target_index] - self.agv_pre_pose_x
                    self.move_distance_y = self.agv_target_pose_y[self.agv_target_index] - self.agv_pre_pose_y

                    if self.move_distance_x == 0 and self.move_distance_y > 0:        
                        self.agv_target_angle = 1.5708
                    elif self.move_distance_x == 0 and self.move_distance_y < 0:
                        self.agv_target_angle = -1.5708
                    elif self.move_distance_x < 0 and self.move_distance_y == 0:
                        self.agv_target_angle = 3.1416
                    elif self.move_distance_x > 0 and self.move_distance_y == 0:
                        self.agv_target_angle = 0
                    
                if self.agv_target_angle > self.agv_pre_angle:
                    if time.time() - self.time_count_spin >= self.time_spin:  
                        self.theta += self.vel_count_spin
                        # time.sleep(self.time_spin)
                        if self.theta >= self.agv_target_angle:
                            self.theta = self.agv_target_angle
                            self.agv_finish_spin = 1
                        self.time_count_spin = time.time()

                elif self.agv_target_angle < self.agv_pre_angle:
                    if time.time() - self.time_count_spin >= self.time_spin:
                        self.theta -= self.vel_count_spin
                        # time.sleep(self.time_spin)
                        if self.theta <= self.agv_target_angle:
                            self.theta = self.agv_target_angle
                            self.agv_finish_spin = 1
                        self.time_count_spin = time.time()
                else:
                    self.agv_finish_spin = 1

            self.agv_info.x = round(self.x, 2)
            self.agv_info.y = round(self.y, 2)
            # print(self.agv_info.x)
            # print(self.agv_info.y)
            self.agv_info.angle = round(self.theta, 2)

            self.pub_agv_pose.publish(self.agv_info)
            self.pub_path_global.publish(self.path_plan)
            self.rate.sleep()

def main():
    print('Programmer started!')
    agv1 = AGV_TF()
    agv1.run()
    print('Programmer stopped!!')

if __name__ == '__main__':
    main()
