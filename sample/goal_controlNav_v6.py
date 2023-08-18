#!/usr/bin/python3
# Author : PhucHoang 25/6/2021

import threading
import time
import rospy
from std_msgs.msg import String, Bool, Int8

import sys
import struct
import string
import roslib
import serial
import signal

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sti_msgs.msg import Move_request, Move_respond, Zone_lidar_2head, POWER_info, Status_goalControl, Velocities, Status_goal_control, HC_info

import os 
from nav_msgs.msg import Path

class goalControl():
    def __init__(self):

        rospy.init_node('goal_control_v2', anonymous=True)

        #get param 
        self.rate = rospy.get_param('~rate',50) 
        self.vel_x_max = rospy.get_param('~vel_x_max',0.8)
        self.vel_x_min = rospy.get_param('~vel_x_min',0.1)
        self.vel_theta_max = rospy.get_param('~vel_theta_max',0.7)
        self.vel_theta_min = rospy.get_param('~vel_theta_min',0.1)
        self.tolerance_xy_max = rospy.get_param('~tolerance_xy_max',0.2)
        self.tolerance_xy_min = rospy.get_param('~tolerance_xy_min',0.02)
        self.tolerance_theta_max = rospy.get_param('~tolerance_theta_max',0.2)
        self.tolerance_theta_min = rospy.get_param('~tolerance_theta_min',0.018)

        self.tolerance_theta = rospy.get_param('~tolerance_theta',0.015)
        self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1',0.02)

        self.vel_rot_step1 = rospy.get_param('~vel_rot_step1',0.45)
        self.vel_rot_step_f = rospy.get_param('~vel_rot_step_f',0.35)

        self.gioihan_lui  = rospy.get_param('~gioihan_lui',2.5)
        self.K_x = rospy.get_param('~he_so',0.6)
        self.dist_ahead_max = rospy.get_param('~khoang_nhin_truoc_max',1.4)
        self.dist_ahead_min = rospy.get_param('~khoang_nhin_truoc_min',0.05)
        self.khoang_offset = rospy.get_param('~khoang_offset', 1.)
        self.needRotatyFinish = rospy.get_param('~need_rotaty_finish', 1)

        self.rate = rospy.Rate(self.rate)

        rospy.Subscriber("/request_move", Move_request, self.move_callback)
        self.req_move = Move_request()    # list requirment get from UDP
        self.is_request_move = False

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.getPose, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.theta_rb_ht = 0.0

        rospy.Subscriber("/HC_info", HC_info, self.zone_callback)
        self.zone_lidar = HC_info()
        self.is_check_zone = False	

        rospy.Subscriber("/POWER_info", POWER_info, self.MAIN_callback) 
        self.power_info = POWER_info()
        self.is_readMain = False

        rospy.Subscriber("/raw_vel", TwistWithCovarianceStamped, self.rawvel_callback)	
        self.is_raw_vel = False
        self.vel_raw = TwistWithCovarianceStamped()

        rospy.Subscriber('/odometry', Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.is_odom_rb = False
        self.odom_rb = Odometry()

        rospy.Subscriber('/safety_NAV', Int8, self.cbZoneNAV, queue_size = 1)
        self.is_ZoneNav = False
        self.dataZoneNav = Int8()
        
        # -- node publish -- 
        self.pub_respond = rospy.Publisher("/respond_move", Move_respond, queue_size= 20)
        self.pub_move = Move_respond()

        self.pub_requestFields = rospy.Publisher("/HC_fieldRequest", Int8, queue_size= 20)
        self.oldselectfield = 0

        self.pub_stt_goal = rospy.Publisher('/status_goal_control', Status_goal_control, queue_size=10) 
        self.pub_path_local = rospy.Publisher('path_plan_local', Path, queue_size= 20)
        self.pub_path_global = rospy.Publisher('path_plan_global', Path, queue_size= 20)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)

        #fnShutDown
        rospy.on_shutdown(self.fnShutDown)
        self.auto_reset = 0
        self.is_slowly = False

        # cac bien khac
        self.process = 0
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
        self.is_need_pttt = 0
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

        self.dis_gt = 1.25
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

        self.path_plan = Path()
        self.path_plan.header.frame_id = 'frame_map_nav350'
        self.path_plan.header.stamp = rospy.Time.now()

        self.timeRecieveNAV = time.time()
        self.timeRecieveTIM = time.time()

        self.timeZone3TIM = 0
        self.timeZone2TIM = 0

        self.timeWaitTIM = 0.1
        self.timeWaitNAV = 0.5

        self.oldzone = 0

    def move_callback(self, data):
        self.req_move = data
        self.is_request_move = True

    def getPose(self, data):
        self.is_pose_robot = True
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb_ht = euler[2]

    def zone_callback(self, data):
        self.zone_lidar = data
        self.is_check_zone = True
        self.timeRecieveTIM = time.time()

    def MAIN_callback(self, data):
        self.power_info = data
        self.is_readMain = True

    def rawvel_callback(self, data):
        self.vel_raw = data
        self.is_raw_vel = True

    def cbGetRobotOdom(self, data):
        self.odom_rb = data
        self.is_odom_rb = True

    def cbZoneNAV(self, data):
        self.dataZoneNav = data
        self.is_ZoneNav = True
        self.timeRecieveNAV = time.time()

    def pub_status(self, misson, status_now, error, safety, complete_misson , id):
        stt_1 = 'AGV dang thuc hien nhiem vu - khong co loi :)'
        stt_2 = 'AGV dang thuc hien nhiem vu - loi co vat can khi di thang :('
        stt_3 = 'AGV dang thuc hien nhiem vu - loi co vat can khi di lui :('
        stt_4 = 'AGV dang thuc hien nhiem vu - loi co vat can khi quay tai cho :('
        stt_5 = 'AGV dang thuc hien nhiem vu - loi target khong hop le '
        stt_6 = 'AGV dang thuc hien nhiem vu - loi goal khong hop le :('
        stt_7 = 'AGV da hoan thanh nhiem vu - dang doi nhiem vu tiep theo :)'
        stt_8 = 'AGV khong co nhiem vu gi :)'
        stt_9 = 'AGV da hoan thanh het goal trong list - dang doi list tiep theo :)'
        stt_10 = 'AGV dang thuc hien nhiem vu - loi mat cam bien an toan :('
        status = Status_goal_control()
        status.misson = misson
        status.status_now = status_now
        status.ID_follow = int(id)
        status.error = error
        status.safety = safety
        if (misson == 1 or misson == 3) and complete_misson == 0 and self.end_of_list == True:
            status.complete_misson = 2
        else:
            status.complete_misson = complete_misson
        if complete_misson == 1:
            status.meaning = stt_7
        elif misson == 0:
            status.meaning = stt_8
        else:
            if status_now == 1 and safety == 1:
                status.meaning = stt_2
            elif status_now == 2 and safety == 1:
                status.meaning = stt_3
            elif status_now == 3 and safety == 1:
                status.meaning = stt_4
            elif error == 1:
                status.meaning = stt_5
            elif error == 2:
                status.meaning = stt_6
            elif error == 3:
                status.meaning = stt_10
            elif self.end_of_list == True:
                status.meaning = stt_9
            else:
                status.meaning = stt_1

        self.pub_stt_goal.publish(status)


    def point_path(self, x, y):
        point = PoseStamped()
        point.header.frame_id = 'frame_map_nav350'
        point.header.stamp = rospy.Time.now()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = -1.0
        point.pose.orientation.w = 1.0
        return point

    def fnCalcDistPoints(self, x1, x2, y1, y2):                     # tính khoảng cách giữa 2 điểm trên hệ trục tọa độ 
        return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def constrain(self,val, min_val, max_val):
        if val < min_val: return min_val
        if val > max_val: return max_val
        return val


    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def check_goal(self,pose1, goal2, tolerance):
        if ( fabs(pose1.position.x) - fabs(goal2.target_pose.pose.position.x) ) < tolerance and \
            ( fabs(pose1.position.y) - fabs(goal2.target_pose.pose.position.y) ) < tolerance :
            return True  
        else : return False 

    def log_mess(self, typ, mess, val):
        if self.pre_mess != mess:
            if typ == "info":
                rospy.loginfo (mess + ": %s", val)
            elif typ == "warn":
                rospy.logwarn (mess + ": %s", val)
            else:
                rospy.logerr (mess + ": %s", val)
        self.pre_mess = mess

    def find(self, p_x, p_y, lis_x, lis_y):
        for i in range(len(lis_x)):
            if round(p_x, 3) == round(lis_x[i], 3):
                if round(p_y, 3) == round(lis_y[i], 3):
                    return i
        return -1

    def find_point_special(self, x_c_goal, y_c_goal, x_n_goal, y_n_goal):
        dx1 = self.poseRbMa.position.x - x_c_goal
        dy1 = self.poseRbMa.position.y - y_c_goal
        dx2 = x_n_goal - x_c_goal
        dy2 = y_n_goal - y_c_goal
        c_goc = (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-12)
        goc = acos(c_goc)

        if goc >= 160.0*PI/180.0:
            rospy.loginfo('x = %lf, y = %lf khong la diem dac biet',x_c_goal,y_c_goal)
            return False
        else:
            rospy.loginfo('x = %lf, y = %lf la diem dac biet',x_c_goal,y_c_goal)
            return True

    def update_all(self):
        # self.cur_goal = PoseStamped() # --
        if len(self.req_move.list_x) != 0 and len(self.req_move.list_y) != 0 and self.req_move.list_id[0] != 0.0:
            self.target_x = round(self.req_move.target_x, 3)
            self.target_y = round(self.req_move.target_y, 3)
            self.target_z = round(self.req_move.target_z, 3)
            self.cur_goal_x = self.req_move.list_x[0]
            self.cur_goal_y = self.req_move.list_y[0]
            self.id_fl = self.req_move.list_id[0]
            self.stt_agv = 0
            self.completed_all = 0
            self.completed_simple = 0
            self.completed_backward = 0
            self.end_of_list = False
            self.completed_reset = 0
            self.mission = self.req_move.mission
            self.is_target_change = True
            self.is_need_turn_step1 = 0
            self.is_need_pttt = 0
            self.is_pre_pttt = 0
            self.cur_goal_is = 0
            self.timeZone3TIM = 0
            self.timeZone2TIM = 0
            self.selectfield(1)
            return 1
        
        else:
            return 2

    def reset_all(self):

        self.completed_all = 0
        self.completed_simple = 0
        self.completed_backward = 0
        self.end_of_list = False
        self.completed_reset = 0
        self.stt_agv = 0
        self.is_need_turn_step1 = 0
        self.is_need_pttt = 0
        self.is_pre_pttt = 0
        self.error = 0
        self.cur_goal_is = 0
        self.target_x = self.coordinate_unknown
        self.target_y = self.coordinate_unknown
        self.id_fl = 0.0
        self.timeZone3TIM = 0
        self.timeZone2TIM = 0
        self.selectfield(1)


    def stop(self):
        for i in range(2):
            self.pub_cmd_vel.publish(Twist())

    def pub_cmdVel(self, twist , rate):

        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass

    def find_hc(self, X_s, Y_s, X_f, Y_f):
        X_n = Y_n = 0.0
        kc_hinh_chieu = 0.0
        # pt duong thang quy dao
        a_qd = Y_s - Y_f
        b_qd = X_f - X_s
        c_qd = -X_s*a_qd -Y_s*b_qd      #  ????? sai 

        # pt tu RB to Goal
        a_rg = self.poseRbMa.position.y - Y_f
        b_rg = X_f - self.poseRbMa.position.x
        #pt duong thang hinh chieu
        # a_hc = b_qd
        # b_hc = -a_qd
        # c_hc = -self.poseRbMa.position.x*a_hc -self.poseRbMa.position.y*b_hc
        # diem hinh chieu
        if self.poseRbMa.position.x == X_s and self.poseRbMa.position.y == Y_s :
            # print('VAO DAY ROI')
            X_n = X_s
            Y_n = Y_s
        else:
            #pt duong thang hinh chieu
            a_hc = b_qd
            b_hc = -a_qd
            c_hc = -self.poseRbMa.position.x*a_hc -self.poseRbMa.position.y*b_hc

            X_n = ((c_hc*b_qd)-(c_qd*b_hc))/((a_qd*b_hc)-(b_qd*a_hc))
            Y_n = ((c_hc*a_qd)-(c_qd*a_hc))/((a_hc*b_qd)-(b_hc*a_qd))

        kc_hinh_chieu = sqrt((X_n - self.poseRbMa.position.x)*(X_n - self.poseRbMa.position.x) + (Y_n - self.poseRbMa.position.y)*(Y_n - self.poseRbMa.position.y))

        return X_n, Y_n, a_qd, b_qd, c_qd, kc_hinh_chieu, a_rg, b_rg

    def find_point_goal(self, X_s, Y_s, X_f, Y_f, a_qd, b_qd, c_qd, X_n, Y_n, dis_ahead):

        X_g = Y_g =  X_g1 = Y_g1 = X_g2 = Y_g2 = 0.0
        vector_point1_x = vector_point1_y = vector_point2_x = vector_point2_x =0.0
        v_a = v_b = 0.0

        vector_qd_x = X_s - X_f
        vector_qd_y = Y_s - Y_f
        
        x_cv = y_cv = 0.0
        kc_g1 = kc_g2 = 0.0
        kc_ns = kc_nf = 0.0
        is_over = False
        kc_sf = sqrt((X_f - X_s)*(X_f - X_s) + (Y_f - Y_s)*(Y_f - Y_s))
        # pt duong thang quy dao

        kc_nf = sqrt((X_n - X_f)*(X_n - X_f) + (Y_n - Y_f)*(Y_n - Y_f))
        kc_ns = sqrt((X_n - X_s)*(X_n - X_s) + (Y_n - Y_s)*(Y_n - Y_s))

        if kc_ns >= kc_sf and kc_nf <= kc_sf:
            is_over = True
        else:
            is_over = False

        if  b_qd == 0.0:
            X_g1 = X_g2 = -c_qd/a_qd
            Y_g1 = -sqrt(dis_ahead*dis_ahead - (X_g1 - X_n)*(X_g1 - X_n)) + Y_n
            Y_g2 = sqrt(dis_ahead*dis_ahead - (X_g2 - X_n)*(X_g2 - X_n)) + Y_n
        else:
            la = (1.0 + (a_qd/b_qd)*(a_qd/b_qd))
            lb = -2.0*(X_n - (a_qd/b_qd)*((c_qd/b_qd) + Y_n))
            lc = X_n*X_n + ((c_qd/b_qd) + Y_n)*((c_qd/b_qd) + Y_n) - dis_ahead*dis_ahead
            denlta = lb*lb - 4.0*la*lc
            # print(la,lb,lc,denlta)

            X_g1 = (-lb + sqrt(denlta))/(2.0*la)
            X_g2 = (-lb - sqrt(denlta))/(2.0*la)

            Y_g1 = (-c_qd - a_qd*X_g1)/b_qd
            Y_g2 = (-c_qd - a_qd*X_g2)/b_qd

        #loai nghiem bang khoang cach
        # kc_g1 = sqrt((X_g1 - X_f)*(X_g1 - X_f) + (Y_g1 - Y_f)*(Y_g1 - Y_f))
        # kc_g2 = sqrt((X_g2 - X_f)*(X_g2 - X_f) + (Y_g2 - Y_f)*(Y_g2 - Y_f))

        # if kc_g1 < kc_g2:
        #     X_g = X_g1
        #     Y_g = Y_g1

        # else:
        #     X_g = X_g2
        #     Y_g = Y_g2


        # loai nghiem bang vector
        vector_qd_x = X_s - X_f
        vector_qd_y = Y_s - Y_f

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
        return x_cv, y_cv, kc_nf, kc_sf, is_over


    def convert_relative_coordinates(self, X_cv, Y_cv):
        angle = -self.theta_rb_ht
        _X_cv = (X_cv - self.poseRbMa.position.x)*cos(angle) - (Y_cv - self.poseRbMa.position.y)*sin(angle)
        _Y_cv = (X_cv - self.poseRbMa.position.x)*sin(angle) + (Y_cv - self.poseRbMa.position.y)*cos(angle)
        return _X_cv, _Y_cv


    def ptgt(self, denlta_time, time_s, v_s, v_f):
        v_re = 0.0
        denlta_time_now = rospy.Time.now().to_sec() - time_s
        if denlta_time_now <= denlta_time :
            v_re = v_s + (v_f-v_s)*denlta_time_now

        else:
            v_re = v_f

        return v_re

    def control_navigation(self, X_point_goal, Y_point_goal, vel_x, theta, dis):
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

    def find_angle_between(self, a, b, angle_rb):             # đưa ra góc cần quay của AGV khi muốn tới điểm tiếp theo. 
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

    def turn_ar(self, theta, tol_theta, vel_rot):              # ham thực hiện việc quay của AGV 
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

    #-----------------------------------------------------------------------------------------------
    def turn_around(self, theta, tol_theta, vel_rot):
        # print(theta)

        if fabs(theta) > tol_theta: # +- 10 do
            if (theta > PI) or (theta > -PI and theta < 0 ): #quay phai , vel_z < 0
                # print "a"
                if fabs(theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhhh')
                    vel_th = (fabs(theta)/self.angle_giam_toc)*(-vel_rot)
                else:
                    vel_th = -vel_rot

                if vel_th > -0.15:
                    vel_th = -0.15

                # vel_th = -fabs(theta) - 0.1
                # if vel_th < -vel_rot : vel_th = -vel_rot
                return vel_th
                # buoc = 1

            if (theta < -PI) or (theta < PI and theta > 0 ): #quay trai , vel_z > 0
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
                # buoc = 1
        else : 
            return -10

    def check_safetyNAV(self, timeRecieve, timeCheck):
        t = time.time() - timeRecieve
        if t >= timeCheck:
            return -1

        else:
            return self.dataZoneNav.data


    # def check_safetyTIM(self, timeRecieve, timeCheck):
    #     t = time.time() - timeRecieve
    #     if t >= timeCheck:
    #         self.timeZone3TIM = 0
    #         self.timeZone2TIM = 0
    #         return -1

    #     else:
    #         # return self.zone_lidar.zone_sick_ahead
    #         if self.zone_lidar.zone_sick_ahead == 1:
    #             self.timeZone3TIM = 0
    #             self.timeZone2TIM = 0
    #             self.oldzone = 1
    #             return 1
    #         elif self.zone_lidar.zone_sick_ahead == 0:
    #             self.timeZone3TIM = 0
    #             self.timeZone2TIM = 0
    #             self.oldzone = 0
    #             return 0

    #         if self.is_check_zone == True:
    #             self.is_check_zone = False

    #             if self.zone_lidar.zone_sick_ahead == 3:
    #                 self.timeZone2TIM = 0
    #                 self.timeZone3TIM = self.timeZone3TIM + 1

    #             elif self.zone_lidar.zone_sick_ahead == 2:
    #                 self.timeZone3TIM = 0
    #                 self.timeZone2TIM = self.timeZone2TIM + 1

    #             if self.timeZone3TIM > 3:
    #                 self.oldzone = 3
    #                 return 3
    #             elif self.timeZone2TIM > 3:
    #                 self.oldzone = 2
    #                 return 2
    #             else:
    #                 return self.oldzone
    #         else:
    #             return self.oldzone
    
    def check_safetyTIM(self, timeRecieve, timeCheck):
        t = time.time() - timeRecieve
        if t >= timeCheck:
            return -1

        else:
            # return self.zone_lidar.zone_sick_ahead
            if self.zone_lidar.zone_sick_ahead == 1:
                return 1
            elif self.zone_lidar.zone_sick_ahead == 2:
                return 2
            elif self.zone_lidar.zone_sick_ahead == 3:
                return 3
            else:
                return 0

    def selectfield(self, field):
        if self.oldselectfield != field:
            dfield = Int8()
            dfield.data = field
            for i in range(2):
                self.pub_requestFields.publish(dfield)

            self.oldselectfield = field

    def run(self):
        # khoi tao vung an toan nho
        self.selectfield(1)
        # print('---start navigation---')
        while not rospy.is_shutdown():
            # print('hhh')
            # khoi tao
            if self.process == 0:
                rospy.loginfo(".......... Start STI Navigation .........")
                # zonetim = self.check_safetyTIM(self.timeRecieveTIM, self.timeWaitTIM)
                # print(zonetim)
                self.process = 1
                self.reset_all()

            # kiem tra du lieu dau vao
            elif self.process == 1:
                c_k = 0
                if self.is_request_move == True:
                    c_k = c_k + 1
                else:
                    self.log_mess("warn","Wait command from STI_Control", c_k)

                if self.is_pose_robot == True:
                    c_k = c_k + 1
                else:
                    self.log_mess("warn","Wait data from STI_Getpose", c_k)

                if self.is_check_zone == True:
                    c_k = c_k + 1
                else:
                    self.log_mess("warn","Wait data from STI_Zone_lidar", c_k)

                if self.is_ZoneNav == True:
                    c_k = c_k + 1
                else:
                    self.log_mess("warn","Wait data from STI_Zone_Nav", c_k)

                if c_k == 4:
                    rospy.loginfo("Completed wakeup ('_')")
                    self.process = 2
                # self.process = 2

            # -- RUN main
            elif self.process == 2:  # kiem tra udp client co cho phep di chuyen.
                if self.req_move.enable == 0: # reset all
                    self.process = -3
                    self.mission = 0

                elif self.req_move.enable == 1 or self.req_move.enable == 3: # run
                    if self.req_move.enable == 1:
                        self.selectfield(1)
                        self.mission = 1
                    else:
                        self.selectfield(0)
                        self.mission = 3
                        
                    self.process = 3

                elif self.req_move.enable == 2: #yeu cau lui
                    self.process = 50
                    self.mission = 2

            elif self.process == 3:
                if  self.req_move.target_x < 500.0 and self.req_move.target_y < 500.0:
                    if (round(self.req_move.target_x, 3) != round(self.target_x, 3) ) or ( round(self.req_move.target_y, 3) != round(self.target_y, 3) ): # neu co thay doi diem dich
                        self.stop()
                        dk = self.update_all()
                        if dk == 1:
                            rospy.logwarn("Change target from: X = %s |Y = %s to: X = %s |Y = %s ", self.target_x, self.target_y ,self.req_move.target_x, self.req_move.target_y)
                            self.process = 4
                        else:
                            rospy.logwarn("Something Wrong!, wait the update True Target")
                            self.process = 2

                    else:
                        self.is_target_change = False
                        self.process = 4
                    
                else:
                    if self.stt_agv == 0:
                        self.error = 1
                        rospy.logwarn("Something Wrong!, wait the update True Target")
                        self.process = 2

                    else:
                        self.process = 4

            elif self.process == 4:
                if self.completed_all != 1: # agv da di den dich cuoi.
                    self.error = 0
                    if self.completed_simple != 1:   # chua hoan thanh di chuyen diem thuong
                        self.process = 5      # simple point

                    else:
                        self.process = 8
                        self.log_mess("info", "hoan thanh di chuyen diem thuong, dap ung goc cuoi", 0)

                else:
                    self.error = 0
                    self.stt_agv = 0
                    self.log_mess("info", "Wait new target",0)
                    self.process = 2

            # tim ra current goal, next goal
            elif self.process == 5:
                # update list x,y
                try:
                    if ((self.list_x != self.req_move.list_x) or (self.list_y != self.req_move.list_y) or (self.list_id != self.req_move.list_id)) and self.req_move.list_id[0] != 0.0:
                        self.list_x = self.req_move.list_x
                        self.list_y = self.req_move.list_y
                        # print(self.list_x)
                        self.list_id = self.req_move.list_id
                        self.end_of_list = False

                        del self.path_plan.poses[:]
                        self.path_plan.poses.append(self.point_path(self.poseRbMa.position.x, self.poseRbMa.position.y))

                        for i in range(len(self.list_x)):
                            if self.list_id[i] != 0.0:
                                point = PoseStamped()
                                point.header.frame_id = 'frame_map_nav350'
                                point.header.stamp = rospy.Time.now()
                                point.pose.position.x = self.list_x[i]
                                point.pose.position.y = self.list_y[i]
                                point.pose.position.z = -1.0
                                point.pose.orientation.w = 1.0
                                # print(point)
                                self.path_plan.poses.append(point)
                            else:
                                break

                        self.pub_path_global.publish(self.path_plan)
                except Exception as e:
                    print("Update list Unknow!!!")

                if self.stt_agv == 0 : # AGV tinh goal tiep theo sau khi dung
                    # print("vao day nay")

                    if self.end_of_list == True:
                        # print("aaaaaaaaaaaaaaaaaaaaaa")
                        self.process = 2

                    else:
                        if self.is_target_change == True:
                            self.is_target_change = False
                            rospy.logwarn('kiem tra bat dau')
                            if self.id_fl != 0.0:
                                if self.fnCalcDistPoints(self.poseRbMa.position.x, self.cur_goal_x, self.poseRbMa.position.y, self.cur_goal_y) > self.ss_luivsnextGoal:
                                    self.is_need_turn_step1 = 1
                                    self.is_need_pttt = 1
                                    self.point_goal_start_x = self.poseRbMa.position.x
                                    self.point_goal_start_y = self.poseRbMa.position.y
                                    self.process = 6

                                else:
                                    if (round(self.cur_goal_x, 3) == round(self.target_x, 3)) and (round(self.cur_goal_y, 3) == round(self.target_y, 3)):
                                        self.is_over_goal = False
                                        self.is_need_pttt = 0
                                        self.is_pre_pttt = 0
                                        rospy.logwarn('da den goal cuoi roi roi!!!!!!')
                                        self.stop()
                                        self.completed_simple = 1
                                        self.process = 8

                                    else:
                                        rospy.logwarn('chon goal tiep theo lam curgoal')		

                                        if self.list_id[1] != 0.0:
                                            self.is_need_turn_step1 = 1
                                            self.is_need_pttt = 1
                                            self.point_goal_start_x = self.poseRbMa.position.x
                                            self.point_goal_start_y = self.poseRbMa.position.y
                                            self.cur_goal_x = self.list_x[1]
                                            self.cur_goal_y = self.list_y[1]
                                            self.id_fl = self.list_id[1]
                                            self.process = 6
                                        
                                        else:
                                            self.end_of_list = True
                                            self.process = 2

                            else:
                                self.end_of_list = True
                                self.process = 2
                                                    
                        else:
                            f = self.find(self.cur_goal_x, self.cur_goal_y, self.list_x, self.list_y)
                            # print(f,self.cur_goal_x,self.cur_goal_y,self.list_x,self.list_y)   
                            if f == -1:
                                # print('aaa')
                                if self.list_id[0] != 0.0:
                                    self.is_need_turn_step1 = 1
                                    self.is_need_pttt = 1
                                    self.point_goal_start_x = self.poseRbMa.position.x
                                    self.point_goal_start_y = self.poseRbMa.position.y 

                                    self.cur_goal_x = self.list_x[0]
                                    self.cur_goal_y = self.list_y[0]
                                    self.id_fl = self.list_id[0]
                                    self.process = 6

                                else:
                                    self.end_of_list = True
                                    self.process = 2

                            else:
                                f_next = f + 1 # diem tiep theo
                                if f >= 4:
                                    self.cur_goal_is = 2
                                    self.end_of_list = True
                                
                                else:
                                    if self.list_id[f_next] != 0.0:
                                        self.is_need_turn_step1 = 1
                                        self.is_need_pttt = 1
                                        self.point_goal_start_x = self.poseRbMa.position.x
                                        self.point_goal_start_y = self.poseRbMa.position.y 

                                        self.cur_goal_x = self.list_x[f_next]
                                        self.cur_goal_y = self.list_y[f_next]
                                        self.id_fl = self.list_id[f_next]
                                        self.process = 6

                                    else:
                                        self.end_of_list = True
                                        self.process = 2

                elif self.stt_agv == 1:

                    # print('aaa')
                    if self.fnCalcDistPoints(self.poseRbMa.position.x,self.cur_goal_x,self.poseRbMa.position.y,self.cur_goal_y) < self.khoang_offset and self.cur_goal_is != 2 :
                        if (round(self.cur_goal_x, 3) == round(self.target_x, 3)) and (round(self.cur_goal_y, 3) == round(self.target_y, 3)):
                            self.cur_goal_is = 3

                        else:
                            # print('aaa')
                            f = self.find(self.cur_goal_x, self.cur_goal_y, self.list_x, self.list_y)
                            if f == -1:

                                if self.list_id[0] != 0.0:
                                    if self.find_point_special(self.cur_goal_x,self.cur_goal_y,self.list_x[0],self.list_y[0]):
                                        self.cur_goal_is = 2
                                    else:
                                        self.cur_goal_is = 1
                                        self.point_goal_start_x = self.cur_goal_x
                                        self.point_goal_start_y = self.cur_goal_y

                                        self.cur_goal_x = self.list_x[0]
                                        self.cur_goal_y = self.list_y[0]
                                        self.id_fl = self.list_id[0]
                                else:
                                    self.cur_goal_is = 2
                                    self.end_of_list = True

                            else:
                                if f >= 4:
                                    self.cur_goal_is = 2
                                    self.end_of_list = True

                                else:
                                    f_next = f + 1 # diem tiep theo
                                    # if f_next > 4:
                                    #     f_next = 4
                                    if self.list_id[f_next] != 0.0:
                                        if self.find_point_special(self.cur_goal_x,self.cur_goal_y,self.list_x[f_next],self.list_y[f_next]):
                                            self.cur_goal_is = 2
                                        else:
                                            self.cur_goal_is = 1
                                            self.point_goal_start_x = self.cur_goal_x
                                            self.point_goal_start_y = self.cur_goal_y

                                            self.cur_goal_x = self.list_x[f_next]
                                            self.cur_goal_y = self.list_y[f_next]
                                            self.id_fl = self.list_id[f_next]

                                    else:
                                        # print("phat cuoi vao day")
                                        self.cur_goal_is = 2
                                        self.end_of_list = True
                    
                    self.process = 6

                elif self.stt_agv == 3 :
                    self.process = 6

            elif self.process == 6:
                if self.id_fl != 0.0:
                    self.error = 0
                    if self.is_need_turn_step1 == 1:
                        self.stt_agv = 3
                        # quay agv toi goal
                        theta_poin = atan2(self.cur_goal_y - self.poseRbMa.position.y, \
                                            self.cur_goal_x - self.poseRbMa.position.x )

                        a = self.poseRbMa.position.y - self.cur_goal_y                   # khoảng cách giữa điểm mục tiêu và vị trí hiện tại của AGV. 
                        b = self.cur_goal_x - self.poseRbMa.position.x

                        theta = self.find_angle_between(a, b, self.theta_rb_ht)          # self.theta_rb_ht: góc hiện tại 

                        if self.zone_lidar.zone_sick_ahead == 100:
                            self.war_agv = 1
                            rospy.logwarn('co vat can o vung tron truoc sau')
                            self.stop()
                        else:
                            self.war_agv = 0
                            gt = self.turn_ar(theta,self.tolerance_rot_step1,self.vel_rot_step1)
                            if gt == -10:
                                self.stop()
                                self.stt_agv = 1
                                self.is_need_turn_step1 = 0
                                self.time_start_navi = rospy.Time.now().to_sec()

                            else:
                                twist = Twist()
                                twist.angular.z = gt
                                self.pub_cmdVel(twist, self.rate_cmdvel)

                        self.process = 2
                            
                    else:
                        self.process = 7

                else:
                    self.stop()
                    # self.log_mess("warn", "Stop send Goal because Now Goal Unknown", self.cur_goal_x)	
                    self.error = 2
                    self.process = 2

            #----- dieu huong AGV
            elif self.process == 7:

                self.path_plan.poses = [self.point_path(self.poseRbMa.position.x,self.poseRbMa.position.y),\
                        self.point_path(self.cur_goal_x,self.cur_goal_y)]
                self.pub_path_local.publish(self.path_plan)

                self.X_n, self.Y_n, self.a_qd, self.b_qd, self.c_qd, self.dis_hc, a_rg, b_rg = self.find_hc(self.point_goal_start_x,\
                                                                                                self.point_goal_start_y,\
                                                                                                self.cur_goal_x,\
                                                                                                self.cur_goal_y)

                self.theta = self.find_angle_between(self.a_qd, self.b_qd, self.theta_rb_ht)
                dist_ahead = self.dist_ahead_max

                self.x_td_goal, self.y_td_goal, self.kc_con_lai, self.kc_qd, self.is_over_goal = self.find_point_goal(self.point_goal_start_x,\
                                                                                                                self.point_goal_start_y,\
                                                                                                                self.cur_goal_x,\
                                                                                                                self.cur_goal_y,\
                                                                                                                self.a_qd,self.b_qd,self.c_qd,\
                                                                                                                self.X_n,self.Y_n,\
                                                                                                                dist_ahead)
                # angle_Mode = atan2(self.y_td_goal,self.x_td_goal)
                # if fabs(angle_Mode) > PI/2.0:

                self.distance_goal = self.fnCalcDistPoints(self.poseRbMa.position.x,\
                                                                    self.cur_goal_x,\
                                                                    self.poseRbMa.position.y,\
                                                                    self.cur_goal_y)

                print("dis_hc= %s , x_now= %s, y_now= %s, distance_goal= %s, kc_conlai= %s" %(self.dis_hc, self.poseRbMa.position.x, self.poseRbMa.position.y, self.distance_goal, self.kc_con_lai))
                # self.theta = theta_poin - self.theta_rb_ht

                #co topic POWER thi bat lai
                # self.vel_x = 0.4

# tính self.vel_x -> v_x 
                bat = round(self.power_info.voltages, 1)
                # print(bat)
                if bat < 24.5:
                	self.vel_x = self.vel_x1        # = 0.4

                elif 24.5 <= bat and bat < 25:
                    self.vel_x = self.vel_x2        # = 0.5

                elif bat >= 25:
                	self.vel_x = self.vel_x3        # = 0.6
                        
# tính v_x -> v_x_send
                v_x = 0.0                                                      
                if fabs(self.theta) > self.angle_find_vel:                      # nếu self.theta > PI/6, v_x = 0.3
                    v_x = self.min_vel_x_gh                                   ?????# 1114: self.theta = self.find_angle_between(self.a_qd, self.b_qd, self.theta_rb_ht)

                elif round(fabs(self.theta),2) == 0.0:                          # nếu theta = 0, v_x = self.vel_x =0.6  trong trường hơp điện áp đầy 
                    v_x = self.vel_x

                else:
                    v_x = self.min_vel_x_gh + ((self.angle_find_vel - fabs(self.theta))/self.angle_find_vel)*(self.vel_x - self.min_vel_x_gh)
                    # v_x = 0.3 + (pi/6 - self.theta)/(pi/6) * (0.6-0.3)

# Tình v_x_send -> vel_x -> twist.linear.x
                v_x_send = 0.0                                                    
                if self.is_need_pttt == 1 and self.distance_goal > self.dis_gt:        ????# tính self.distance_goal ở dòng 1127 ???? self.dis_gt = 1.25
                    self.is_pre_pttt = 1
                    self.vel_x_now = self.ptgt(4.0,self.time_start_navi,0.0, v_x)      # tính ra v_x_send từ v_x 
                    v_x_send = self.vel_x_now

                    if v_x_send >= v_x:                                                ??? # nếu v_x_send >= v_x thì thực hiện điều kiện tiếp theo
                        self.is_need_pttt = 0
                        v_x_send = v_x

                else:
                    self.is_need_pttt = 0
                    if self.is_pre_pttt == 1 and self.vel_x_now >= 0.3:              # cái này ko bao giờ xảy ra 
                        v_x_send = self.vel_x_now*(self.distance_goal/self.dis_gt)
                        if v_x_send > self.vel_x_now:
                            v_x_send = self.vel_x_now

                    else:
                        self.is_pre_pttt = 0
                        v_x_send = v_x*(self.distance_goal/self.dis_gt)
                        if v_x_send > v_x:
                            v_x_send = v_x

                if self.cur_goal_is == 2 and (self.distance_goal <= self.tol_simple or self.kc_con_lai <= self.tol_simple or self.is_over_goal == True) :
                    self.cur_goal_is = 0
                    self.is_over_goal = False
                    self.is_need_pttt = 0
                    self.is_pre_pttt = 0
                    rospy.loginfo('da den goal trung gian roi!!!!!!')
                    self.stop()

                    self.stt_agv = 0
                    self.process = 2

                elif self.cur_goal_is == 3 and (self.distance_goal <= self.tol_target or self.kc_con_lai <= self.tol_target or self.is_over_goal == True) :
                    #----- them xoay dap ung goc cuoi
                    self.is_over_goal = False
                    self.is_need_pttt = 0
                    self.is_pre_pttt = 0
                    rospy.loginfo('da den goal cuoi roi roi!!!!!!')
                    self.stop()
                    self.completed_simple = 1
                    self.process = 8
                    time.sleep(0.6)

                else:
                    self.stt_agv = 1

                    zonetim = self.check_safetyTIM(self.timeRecieveTIM, self.timeWaitTIM)
                    zonenav = self.check_safetyNAV(self.timeRecieveNAV, self.timeWaitNAV)
                    # print(zonetim)
                    if zonetim == -1 or zonenav == -1:
                        self.stop()
                        # rospy.logwarn('khong nhan duoc du lieu laser')
                        self.error = 3
                        self.process = 2

                    else:
                        self.error = 0
                        if zonetim == 1 or zonenav == 1:
                            self.stop()
                            self.is_need_pttt = 1
                            self.is_pre_pttt = 0
                            self.time_start_navi = rospy.Time.now().to_sec()
                            self.war_agv = 1

                        else :
                            vel_x = 0.0
                            if zonetim == 3:
                                self.war_agv = 2
                                vel_x = v_x_send*0.7

                            elif zonetim == 2:
                                self.war_agv = 2
                                vel_x = v_x_send*0.4

                            else:
                                self.war_agv = 0
                                vel_x = v_x_send

                            if vel_x >= self.vel_x_max:
                                vel_x = self.vel_x_max

                            if vel_x <= self.min_vel_x :
                                vel_x = self.min_vel_x

                            v_th_send = self.control_navigation(self.x_td_goal, self.y_td_goal,vel_x,self.theta, self.dis_hc) 
                            # print('v_dai = %f, v_goc = %f' %(vel_x,v_th_send))
                            twist = Twist()
                            twist.linear.x = vel_x
                            twist.angular.z = v_th_send
                            self.pub_cmdVel(twist,self.rate_cmdvel)

                        self.process = 2


            elif self.process == 8:               # hoàn thành di chuyển điểm đường, đáp ứng góc cuối 
                    
                if self.needRotatyFinish == 1:
                    theta = self.target_z - self.theta_rb_ht
                    # print(angle_bt, angle_fn)
                    if fabs(theta) >= PI:
                        theta_t = (2*PI - fabs(theta))
                        if theta > 0:
                            theta = -theta_t
                        else:
                            theta = theta_t

                    if self.zone_lidar.zone_sick_ahead == 100 :
                        self.war_agv = 1
                        # rospy.logwarn('co vat can o vung tron')
                        self.stop()

                    else:
                        self.war_agv = 0
                        gt = self.turn_ar(theta,self.tolerance_theta,self.vel_rot_step_f)
                        if gt == -10:
                            self.stop()
                            self.stt_agv = 0
                            self.completed_all = 1
                            del self.path_plan.poses[:]
                            self.pub_path_global.publish(self.path_plan)
                            self.process = 2
                        else:
                            twist = Twist()
                            twist.angular.z = gt
                            self.pub_cmdVel(twist,self.rate_cmdvel)
                            self.stt_agv = 3
                else:
                        
                    self.war_agv = 0
                    self.stop()
                    self.stt_agv = 0
                    self.completed_all = 1
                    del self.path_plan.poses[:]
                    self.pub_path_global.publish(self.path_plan)
                    self.process = 2

                self.process = 2
            
            elif self.process == -3:   # RESET: khong cho phep di chuyen -reset all.
                if self.completed_reset == 0:
                    self.stop()
                    self.reset_all()
                    self.completed_reset = 1
                    self.process = 2

                else:
                    self.stt_agv = 0
                    # self.log_mess("info", "Wait new misson",0)
                    self.process = 2
                    
            elif self.process == 50:
                if self.completed_backward == 0:
                    self.completed_reset = 0
                    if self.stt_agv == 0: # agv chua di chuyen
                        if self.req_move.target_x < 500 and self.req_move.target_y < 500:
                            self.error = 0
                            self.selectfield(1)

                            self.kc_backward = self.fnCalcDistPoints(self.poseRbMa.position.x,\
                                                                    self.req_move.target_x,\
                                                                    self.poseRbMa.position.y,\
                                                                    self.req_move.target_y )

                            if fabs(self.kc_backward) > self.gioihan_lui :
                                self.kc_backward = self.gioihan_lui 

                            self.odom_x_ht = self.odom_rb.pose.pose.position.x
                            self.odom_y_ht = self.odom_rb.pose.pose.position.y

                            self.XRobotStart = self.poseRbMa.position.x
                            self.YRobotStart = self.poseRbMa.position.y

                            self.process = 51

                        else:
                            self.stop()
                            # rospy.logwarn('target khong hop le')
                            self.error = 1 # loi target ko hop le
                            self.process = 2

                    elif self.stt_agv == 2: # agv dang lui
                        self.process = 51

                    elif self.stt_agv == -1:
                        self.process = 2

                else:
                    self.stt_agv = 0
                    # self.log_mess("info", "Wait new target",0)
                    self.process = 2

            elif self.process == 51:
                twist = Twist()
                self.error = 0
                s = self.fnCalcDistPoints(self.odom_rb.pose.pose.position.x,self.odom_x_ht,self.odom_rb.pose.pose.position.y,self.odom_y_ht)
                s_nav = self.fnCalcDistPoints(self.poseRbMa.position.x,self.XRobotStart,self.poseRbMa.position.y,self.YRobotStart)
                if s < fabs(self.kc_backward) or s_nav < fabs(self.kc_backward):
                    self.stt_agv = 2
                    zonetim = self.check_safetyTIM(self.timeRecieveTIM, self.timeWaitTIM)
                    zonenav = self.check_safetyNAV(self.timeRecieveNAV, self.timeWaitNAV)
                    # print(zonetim)
                    if zonetim == -1 or zonenav == -1:
                        self.stop()
                        self.error = 3

                    else:
                        if zonetim == 1 or zonenav == 1:
                            print("co vat can tren dau")
                            self.war_agv = 1
                            self.stop()

                        else:
                            self.war_agv = 0
                            twist.linear.x = 0.16
                            self.pub_cmdVel(twist,self.rate_cmdvel)

                else :
                    self.stop()
                    self.stt_agv = 0
                    self.completed_backward = 1

                self.process = 2

            if self.mission == 0:
                self.pub_status(self.mission,0,0,0,0,0)

            elif self.mission == 1 or self.mission == 3:
                self.pub_status(self.mission,self.stt_agv,self.error, self.war_agv,self.completed_all,self.id_fl)

            elif self.mission == 2:
                self.pub_status(self.mission,self.stt_agv,self.error,self.war_agv,self.completed_backward,self.id_fl)          

            # rospy.loginfo("process: %s", self.process)
            self.rate.sleep()

    
def main():

    program = goalControl()
    program.run()
        #pass

if __name__ == '__main__':
    main()

