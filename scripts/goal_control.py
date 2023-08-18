#!/usr/bin/env python3  
# -*- coding: utf-8 -*-
# Dev: Phùng Quý Dương STI 19/5/2023

import roslib
roslib.load_manifest('agvtraffic_simulation')

import time
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sti_msgs.msg import *
from agvtraffic_simulation.msg import * 
from nav_msgs.msg import Path

class goalControl():
    def __init__(self):
        rospy.init_node('goal_control', anonymous=False)
        
        # -- tên của AGV -- 
        self.agv_name = rospy.get_param('~agv_name')

        # -- mô tả vị trí ban đầu của AGV trên rviz -- 
        self.agv_start_pose_x = rospy.get_param('~agv_start_x')
        self.agv_start_pose_y = rospy.get_param('~agv_start_y')
        self.agv_start_angle = rospy.get_param('~agv_start_angle')

        # -- mô tả vị trí trước đó của agv -- 
        self.agv_pre_pose_x = self.agv_start_pose_x
        self.agv_pre_pose_y = self.agv_start_pose_y
        self.agv_pre_angle = self.agv_start_angle

        # -- vị trí điểm đích tiếp theo của agv theo lộ trình gửi xuống  
        self.agv_target_pose_x = 0
        self.agv_target_pose_y = 0
        self.agv_target_angle = 0
        self.agv_target_index = 0

        # -- khoảng nhìn trước max --
        self.dist_ahead_max = rospy.get_param('~khoang_nhin_truoc_max')
        self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1')
        self.vel_rot_step1 = rospy.get_param('~vel_rot_step1')

        # -- node subcribe -- 
        rospy.Subscriber('/%s/agv_pose' % self.agv_name, Pose, self.getPose, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.theta_rb_ht = 0.0

        rospy.Subscriber('/%s/request_move_fake' % self.agv_name, Move_request, self.move_callback ,queue_size = 20)                # lộ trình nhận được từ sti_control 
        self.req_move = Move_request()                                                 
        self.is_request_move = False
        
        # -- node publish -- 
        self.rate_hz = 50
        self.pub_cmd_vel = rospy.Publisher('/%s/cmd_vel' % self.agv_name, Twist, queue_size = 20)
        self.pub_path_global = rospy.Publisher('/%s/path_plan_global' % self.agv_name, Path, queue_size = 20)

        self.rate = rospy.Rate(self.rate_hz)
        
    # -- biến từ node goal_Control mẫu, khi hoàn thành sẽ xóa những biến thừa đi
        self.process = 1
        self.pre_mess = ""      # print just one time.
        self.war_agv = 0 # 0: agv di chuyen binh thuong | 1: agv gap vat can

        self.coordinate_unknown = 500.0
        self.is_target_change = False

        self.target_x = self.coordinate_unknown
        self.target_y = self.coordinate_unknown
        self.target_z = 0.0
        self.tag = 0
        self.offset = 0.0
        self.list_x = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.list_y = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.list_id = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.mission = 0

        self.completed_simple = 0      # bao da den dich.
        self.completed_backward = 0     # bao da den aruco.
        self.completed_all = 0
        self.completed_list = 0
        self.completed_reset = 0
        self.stt_agv = 0
        self.error = 0

        self.end_of_list = False

        self.cur_goal_x = 0.0
        self.cur_goal_y = 0.0

        self.point_goal_start_x = 0.0
        self.point_goal_start_y = 0.0

        self.is_need_turn_step1 = 0
        self.is_need_pttt = 1
        self.is_pre_pttt = 0
        self.x_td_goal = 0.0 
        self.y_td_goal = 0.0
        self.dis_hc = 0.0

        self.cur_goal_is = 0 # 1: diem thuong, 2: diem dac biet, 3: diem dich

        self.distance_goal = 0.0

        self.tol_simple = 0.04      # do chinh xac theo truc x,y
        self.tol_target = 0.02
        self.ss_luivsnextGoal = 0.25

        self.vel_x_now = 0.0

        self.vel_x = 0.0            # toc do theo truc x
        self.theta_max = 0.6

        self.vel_x1 = 0.4  		    # level 1 slowless
        self.theta_max_1 = 0.4 

        self.vel_x2 = 0.5  		    # level 2 
        self.theta_max_2 = 0.5

        self.vel_x3 = 0.6    		# level 3 fastless
        self.theta_max_3 = 0.6

        self.min_vel_x = 0.04


        self.odom_x_ht = 0.0
        self.odom_y_ht = 0.0
        self.kc_backward = 0.0

        self.XRobotStart = 0.0
        self.YRobotStart = 0.0

        self.min_vel_x_gh = 0.3
        self.theta = 0.0

        self.angle_find_vel = 30.0*PI/180.0
        self.time_start_navi = rospy.Time.now().to_sec()

        self.angle_giam_toc = 45.0*PI/180.0

        self.dis_gt = 0.3
        self.kc_con_lai = 0.0
        self.kc_qd = 0.0
        self.is_over_goal = False

        self.X_n = 0.0
        self.Y_n = 0.0
        self.a_qd = 0.0
        self.b_qd = 0.0
        self.c_qd = 0.0

        self.id_fl = 0.0

        self.rate_cmdvel = 30
        self.time_tr = rospy.get_time()

        # self.path_plan = Path()
        # self.path_plan.header.frame_id = 'frame_map_nav350'
        # self.path_plan.header.stamp = rospy.Time.now()

        # -- journey info 
        self.agv_frame =  self.agv_name
        self.origin_frame = "world"

        self.path_plan = Path()
        self.path_plan.header.frame_id = self.origin_frame
        self.path_plan.header.stamp = rospy.Time.now()
        self.path_plan.poses.append(self.point_path(self.agv_start_pose_x, self.agv_start_pose_y))

        self.timeRecieveNAV = time.time()
        self.timeRecieveTIM = time.time()

        self.timeZone3TIM = 0
        self.timeZone2TIM = 0

        self.timeWaitTIM = 0.1
        self.timeWaitNAV = 0.5

        self.oldzone = 0
        # -- Duowng add
        self.path_index = 0
        self.v_th_send = 0

        self.is_first_pub = True
#-------------------------------------------------------------------------------------------------
    def getPose(self, data):
        self.is_pose_robot = True
        self.poseRbMa = data
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb_ht = euler[2]
    
    def move_callback(self, data):
        self.req_move = data
        self.is_request_move = True

        if self.is_first_pub == True:
            for i in range(len(self.req_move.list_x)):
                point = PoseStamped()
                point.header.frame_id = self.origin_frame
                point.header.stamp = rospy.Time.now()
                point.pose.position.x = self.req_move.list_x[i]
                point.pose.position.y = self.req_move.list_y[i]
                point.pose.position.z = 0.
                point.pose.orientation.w = 1.0
                self.path_plan.poses.append(point)

            self.pub_path_global.publish(self.path_plan)
            self.is_first_pub = False   

    def log_mess(self, typ, mess, val):
        if self.pre_mess != mess:
            if typ == "info":
                rospy.loginfo (mess + ": %s", val)
            elif typ == "warn":
                rospy.logwarn (mess + ": %s", val)
            else:
                rospy.logerr (mess + ": %s", val)
        self.pre_mess = mess

    def point_path(self, x, y):
        point = PoseStamped()
        point.header.frame_id = self.origin_frame
        point.header.stamp = rospy.Time.now()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = 0
        point.pose.orientation.w = 1.0
        return point
    
    def find_angle_between(self, a, b, angle_rb):
        angle_bt = 0.0
        angle_fn = 0.0
        if b == 0:
            if a < 0:
                angle_bt = PI/2.0
            elif a > 0:
                angle_bt = -PI/2.0
        elif a == 0:
            if -b < 0:
                angle_bt = 0.0
            elif -b > 0:
                angle_bt = PI
        else:

            angle_bt = acos(b/sqrt(b*b + a*a))
            if -a/b > 0:
                if fabs(angle_bt) > PI/2:
                    angle_bt = -angle_bt
                else:
                    angle_bt = angle_bt
            else:
                if fabs(angle_bt) > PI/2:
                    angle_bt = angle_bt
                else:
                    angle_bt = -angle_bt

        angle_fn = angle_bt - angle_rb
        # print(angle_bt, angle_fn)
        if fabs(angle_fn) >= PI:
            angle_fnt = (2*PI - fabs(angle_fn))
            if angle_fn > 0:
                angle_fn = -angle_fnt
            else:
                angle_fn = angle_fnt

        # print(angle_fn)
        return angle_fn
    
    def turn_ar(self, theta, tol_theta, vel_rot):
        if fabs(theta) > tol_theta: # +- 10 do
            if theta > 0: #quay trai
                # print "b"
                if fabs(theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhh')
                    vel_th = (fabs(theta)/self.angle_giam_toc)*vel_rot
                else:
                    vel_th = vel_rot

                if vel_th < 0.1:
                    vel_th = 0.1

                # vel_th = fabs(theta) + 0.1
                # if vel_th > vel_rot : vel_th = vel_rot
                return vel_th

            if theta < 0: #quay phai , vel_z < 0
                # print "a"
                if fabs(theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhhh')
                    vel_th = (fabs(theta)/self.angle_giam_toc)*(-vel_rot)
                else:
                    vel_th = -vel_rot

                if vel_th > -0.1:
                    vel_th = -0.1

                # vel_th = -fabs(theta) - 0.1
                # if vel_th < -vel_rot : vel_th = -vel_rot
                return vel_th
                # buoc = 1

        else : 
            return -10 

    # -- Hàm tìm tọa độ hình chiếu của AGV trên lộ trình     
    def find_hc(self, Xt, Yt, Xs, Ys):
        X_n = Y_n = 0.0
        kc_hinh_chieu = 0.0
        # pt duong thang quy dao
        a_qd = Ys - Yt
        b_qd = Xt - Xs 
        c_qd = Xs*Yt - Xt*Ys
        
        # pt duong thang hinh chieu
        a_hc = b_qd
        b_hc = -a_qd
        c_hc = -self.poseRbMa.position.x*a_hc - self.poseRbMa.position.y*b_hc

        # -- tính toán vị trí hình chiếu trên lộ trình --
        if a_qd == 0 and b_qd != 0:             # lộ trình của AGV là đường thẳng nằm ngang có dạng by + c = 0
            X_n = self.poseRbMa.position.x
            Y_n = -c_qd/b_qd
        
        elif a_qd != 0 and b_qd == 0:           # lộ trình của AGV là đường thẳng thẳng đứng có dạng ax + c = 0
            X_n = -c_qd/a_qd
            Y_n = self.poseRbMa.position.y
        
        elif a_qd != 0 and b_qd != 0:           # lộ trình của AGV là đường thẳng có dạng: ax + by + c = 0
            Y_n = (a_hc*c_qd - c_hc*a_qd)/(b_hc*a_qd - a_hc*b_qd)
            X_n = -c_qd/a_qd - b_qd*Y_n/a_qd

        elif a_qd == 0 and b_qd == 0:              # ko thay đổi lộ trình 
            pass     

        kc_hinh_chieu = sqrt((X_n - self.poseRbMa.position.x)**2 + (Y_n - self.poseRbMa.position.y)**2)

        return X_n, Y_n, a_qd, b_qd, c_qd, kc_hinh_chieu
    
    # -- Hàm tìm vị trí điểm đích trên lộ trình -- 
    def find_point_goal(self, Xt, Yt, Xs, Ys, a_qd, b_qd, c_qd, X_n, Y_n, dis_ahead):

        X_g = Y_g =  X_g1 = Y_g1 = X_g2 = Y_g2 = 0.0
        x_cv = y_cv = 0.0

        kc_ns = sqrt((X_n - Xs)**2 + (Y_n - Ys)**2)
        if a_qd == 0 and b_qd != 0:               # lộ trình của AGV là đường thẳng nằm ngang có dạng by + c = 0
            Y_g1 = Y_g2 = -c_qd/b_qd
            X_g1 = -sqrt(dis_ahead*dis_ahead - (Y_g1 - Y_n)*(Y_g1 - Y_n)) + X_n
            X_g2 = sqrt(dis_ahead*dis_ahead - (Y_g2 - Y_n)*(Y_g2 - Y_n)) + X_n
            
        elif a_qd !=0 and b_qd == 0:              # lộ trình của AGV là đường thẳng thẳng đứng có dạng ax + c = 0
            X_g1 = X_g2 = -c_qd/a_qd
            Y_g1 = -sqrt(dis_ahead*dis_ahead - (X_g1 - X_n)*(X_g1 - X_n)) + Y_n
            Y_g2 = sqrt(dis_ahead*dis_ahead - (X_g2 - X_n)*(X_g2 - X_n)) + Y_n

        elif a_qd != 0 and b_qd !=0:              # lộ trình của AGV là đường thẳng có dạng: ax + by + c = 0
            la = (1.0 + (a_qd/b_qd)*(a_qd/b_qd))
            lb = -2.0*(X_n - (a_qd/b_qd)*((c_qd/b_qd) + Y_n))
            lc = X_n*X_n + ((c_qd/b_qd) + Y_n)*((c_qd/b_qd) + Y_n) - dis_ahead*dis_ahead
            denlta = lb*lb - 4.0*la*lc
            # print(la,lb,lc,denlta)

            X_g1 = (-lb + sqrt(denlta))/(2.0*la)
            X_g2 = (-lb - sqrt(denlta))/(2.0*la)

            Y_g1 = (-c_qd - a_qd*X_g1)/b_qd
            Y_g2 = (-c_qd - a_qd*X_g2)/b_qd

        elif a_qd == 0 and b_qd == 0:              # ko thay đổi lộ trình 
            pass 
        
        # print(a_qd, b_qd, c_qd, X_n, Y_n)

        # -- Chọn vị trí goal mà có hướng cùng chiều với hướng di chuyển -- 
       # loai nghiem bang vector
        vector_qd_x = Xt - Xs
        vector_qd_y = Yt - Ys

        vector_point1_x = X_n - X_g1
        vector_point1_y = Y_n - Y_g1

        if vector_qd_x == 0.0:
            if vector_qd_y*vector_point1_y > 0.0:
                X_g = X_g1
                Y_g = Y_g1
            else:
                X_g = X_g2
                Y_g = Y_g2
        elif vector_qd_y == 0.0:
            if vector_qd_x*vector_point1_x > 0.0:
                X_g = X_g1
                Y_g = Y_g1
            else:
                X_g = X_g2
                Y_g = Y_g2

        else:
            v_a = vector_qd_x/vector_point1_x
            v_b = vector_qd_y/vector_point1_y
            if v_a*v_b > 0.0 and v_a > 0.0:
                X_g = X_g1
                Y_g = Y_g1
            else:
                X_g = X_g2
                Y_g = Y_g2

        # print(X_g, Y_g)

        x_cv, y_cv = self.convert_relative_coordinates(X_g, Y_g)              
        return x_cv, y_cv, kc_ns
    
    # -- Hàm chuyển đổi hệ tọa độ của AGV làm gốc: 
    def convert_relative_coordinates(self, X_cv, Y_cv):
        angle = -self.theta_rb_ht
        _X_cv = (X_cv - self.poseRbMa.position.x)*cos(angle) - (Y_cv - self.poseRbMa.position.y)*sin(angle)
        _Y_cv = (X_cv - self.poseRbMa.position.x)*sin(angle) + (Y_cv - self.poseRbMa.position.y)*cos(angle)
        return _X_cv, _Y_cv
    
    # -- Hàm đưa ra  vận tốc góc của AGV (by myself)
    # def control_navigation(self, X_point_goal, Y_point_goal, vel_x):
    #     omega_th = 0.0
    #     l = sqrt(X_point_goal**2 + Y_point_goal**2)
    #     if Y_point_goal == 0:
    #         # print(Y_point_goal)
    #         Y_point_goal = 0.0001

    #     phi = atan2(X_point_goal,Y_point_goal)
    #     rest_phi = PI - 2*phi
    #     r = sin(phi)*l/sin(rest_phi)
    #     omega = vel_x/r        # vel : omega 

    #     if Y_point_goal > 0:
    #         omega_th = omega
    #     else:
    #         omega_th = -omega

    #     return omega_th

    def control_navigation(self, X_point_goal, Y_point_goal, vel_x):
        vel_th = 0.0
        l = (X_point_goal*X_point_goal) + (Y_point_goal*Y_point_goal)
        if Y_point_goal == 0:
            print(Y_point_goal)
            Y_point_goal = 0.0001

        r = l/(2*fabs(Y_point_goal))
        vel = vel_x/r        # vel : omega 

        if Y_point_goal > 0:
            vel_th = vel
        else:
            vel_th = -vel

        return vel_th
    
    def fnCalcDistPoints(self, x1, x2, y1, y2):                     # tính khoảng cách giữa 2 điểm trên hệ trục tọa độ 
        return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)
    
    def ptgt(self, denlta_time, time_s, v_s, v_f):
        v_re = 0.0
        denlta_time_now = rospy.Time.now().to_sec() - time_s
        if denlta_time_now <= denlta_time :
            v_re = v_s + (v_f-v_s)*denlta_time_now

        else:
            v_re = v_f

        return v_re
    
    def pub_cmdVel(self, twist , rate):

        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass
    
    def stop(self):
        for i in range(2):
            self.pub_cmd_vel.publish(Twist())
# -------------------------------------------------------------------------------------------------------------------------------
    
    def run(self):
        while not rospy.is_shutdown():
            # kiểm tra dữ liệu đầu vào
            if self.process == 1:
                c_k = 0
                if self.is_request_move == True:
                    c_k = c_k + 1

                else:
                    pass
                    # self.log_mess("warn","Wait command from STI_Control", c_k)

                if self.is_pose_robot == True:
                    c_k = c_k + 1
                else:
                    pass
                    # self.log_mess("warn","Wait data from STI_Getpose", c_k)

                if c_k == 2:
                    rospy.loginfo("Completed wakeup ('_')")
                    self.process = 2

            elif self.process == 2:                       # kiểm tra lộ trình di chuyển 
                if self.path_index == 0:
                    self.point_goal_start_x = self.poseRbMa.position.x
                    self.point_goal_start_y = self.poseRbMa.position.y
                    self.cur_goal_x = self.req_move.list_x[self.path_index]
                    self.cur_goal_y = self.req_move.list_y[self.path_index]
                else:
                    self.point_goal_start_x = self.cur_goal_x
                    self.point_goal_start_y = self.cur_goal_y
                    self.cur_goal_x = self.req_move.list_x[self.path_index]
                    self.cur_goal_y = self.req_move.list_y[self.path_index]
                    # print("Tao đã vô đây rồi nè !")

                self.is_need_turn_step1 = 1
                self.process = 3    
            
            # -- Quay AGV về đường mục tiêu -- 
            elif self.process == 3: 
                if self.is_need_turn_step1 == 1:
                    a = self.poseRbMa.position.y - self.cur_goal_y                   # khoảng cách giữa điểm mục tiêu và vị trí hiện tại của AGV. 
                    b = self.cur_goal_x - self.poseRbMa.position.x
                    theta = self.find_angle_between(a, b, self.theta_rb_ht)          # trả về góc lệch giữa AGV và đường lộ trình 

                    gt = self.turn_ar(theta,self.tolerance_rot_step1,self.vel_rot_step1)
                    
                    # print("Góc lệch giữa AGV và lộ trình là:", theta)
                    # print("Tốc độ để robot quay về hướng lộ trình là", gt)

                    if gt == -10:
                        # print("Không thực hiện quay")
                        self.stop()
                        # self.stt_agv = 1
                        self.is_need_turn_step1 = 0
                        # self.time_start_navi = rospy.Time.now().to_sec()

                    else:
                        # print("Đang thực hiện quay")
                        twist = Twist()
                        twist.angular.z = gt
                        self.pub_cmdVel(twist, self.rate_cmdvel)
                else:
                    self.stop()
                    self.process = 4

            # -- Điều hướng AGV -- 
            elif self.process == 4:

                self.X_n, self.Y_n, self.a_qd, self.b_qd, self.c_qd, self.dis_hc = self.find_hc(self.point_goal_start_x,\
                                                                                                self.point_goal_start_y,\
                                                                                                self.cur_goal_x,\
                                                                                                self.cur_goal_y)
                
                self.theta = self.find_angle_between(self.a_qd, self.b_qd, self.theta_rb_ht)

                dist_ahead = self.dist_ahead_max
                self.x_td_goal, self.y_td_goal, self.kc_con_lai = self.find_point_goal(self.point_goal_start_x,\
                                                                        self.point_goal_start_y,\
                                                                        self.cur_goal_x,\
                                                                        self.cur_goal_y,\
                                                                        self.a_qd,self.b_qd,self.c_qd,\
                                                                        self.X_n,self.Y_n,\
                                                                        dist_ahead)

                self.distance_goal = self.fnCalcDistPoints(self.poseRbMa.position.x,\
                                                                    self.cur_goal_x,\
                                                                    self.poseRbMa.position.y,\
                                                                    self.cur_goal_y)
                
                # print("dis_hc= %s , x_now= %s, y_now= %s, distance_goal= %s, kc_conlai= %s" %(self.dis_hc, self.poseRbMa.position.x, self.poseRbMa.position.y, self.distance_goal, self.kc_con_lai))

                self.vel_x = self.vel_x2        # = 0.5

                v_x = 0.0                                                      
                if fabs(self.theta) > self.angle_find_vel:                      # nếu self.theta > PI/6, v_x = 0.3
                    v_x = self.min_vel_x_gh                                   

                elif round(fabs(self.theta),2) == 0.0:                          # nếu theta = 0, v_x = self.vel_x =0.6  trong trường hơp điện áp đầy 
                    v_x = self.vel_x

                else:
                    v_x = self.min_vel_x_gh + ((self.angle_find_vel - fabs(self.theta))/self.angle_find_vel)*(self.vel_x - self.min_vel_x_gh)
                
                if self.distance_goal <= self.tol_simple or self.kc_con_lai <= self.tol_simple:    # kiểm tra AGV gần tới điểm cuối và chuyển lộ trình tiếp theo 
                    self.stop()
                    self.process = 2                    # chuyển về process 2 để kiểm tra lộ trình cần di chuyển.
                    self.path_index += 1
                    del self.path_plan.poses[0]                     # xóa lộ trình cũ đi 
                    self.pub_path_global.publish(self.path_plan)
                    
                    if self.path_index > 4:             # AGV đã hoàn thành lộ trình cần di chuyển 
                        self.process = 5
                        self.path_index = 0          

                else:
                    self.v_th_send = self.control_navigation(self.x_td_goal, self.y_td_goal, v_x) 

                twist = Twist()
                twist.linear.x = v_x
                twist.angular.z = self.v_th_send
                self.pub_cmdVel(twist,self.rate_cmdvel)

            # AGV đã hoàn thành lộ trình cần di chuyển 
            elif self.process == 5:    
                self.stop()
                print(self.agv_name,"đã hoàn thành lộ trình !")

        self.rate.sleep()

def main():
	print('Program starting')
	program = goalControl()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()
