#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Phùng Quý Dương
Company: STI Viet Nam
Date: 19/6/2023

Function: 
	+ Phù hợp giữa node stiClient và stiControl 
	+ Xử lý, tổng hợp các biến từ các node khác => tín hiệu điều khiển, lỗi và mức độ lỗi. 

Line of code: 
	+ 1714: 

"""

import roslib

import sys
import time
from decimal import *
import math
import rospy

# -- add 19/01/2022
import subprocess
import re
import os

from sti_msgs.msg import *

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int8
from message_pkg.msg import *
from sensor_msgs.msg import Imu
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

import json
#--------------------------------------------------------------------------------- ROS
class ros_control():
	def __init__(self):
		rospy.init_node('stiControl', anonymous=False)
		self.rate = rospy.Rate(30)

		# get name of AGV
		self.agv_name = rospy.get_param('~agv_name')

		# -- cancel mission
		rospy.Subscriber("/cancelMission_control", Int16, self.callback_cancelMission)
		self.cancelMission_control = Int16()		
		self.flag_cancelMission = 0
		self.status_cancel = 0

		self.pub_cancelMission = rospy.Publisher("/cancelMission_status", Int16, queue_size=100)	
		self.cancelMission_status = Int16()		
		# -------------- Cac ket noi ngoai vi:
		# -- Reconnect
		rospy.Subscriber("/status_reconnect", Status_reconnect, self.callback_reconnect)
		self.status_reconnect = Status_reconnect()

		# -- MAIN - POWER
		rospy.Subscriber("/%s/POWER_info" % self.agv_name, POWER_info, self.callback_main) 
		self.pub_requestMain = rospy.Publisher("/POWER_request", POWER_request, queue_size=100)	
		self.main_info = POWER_info()
		self.power_request = POWER_request()
		self.timeStampe_main = rospy.Time.now()
		self.voltage = rospy.get_param('max_voltage', 25.5)

		self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.velPs2 = Twist()

		# -- HC 82
		rospy.Subscriber("/%s/HC_info" % self.agv_name, HC_info, self.callback_hc) # lay thong tin trang thai cua node va cam bien sick an toan.
		self.pub_HC = rospy.Publisher("/HC_request", HC_request, queue_size=100)	# dieu khien den bao va den ho tro camera
		self.HC_info = HC_info()
		self.HC_request = HC_request()
		self.timeStampe_HC = rospy.Time.now()

		# -- OC 82
		rospy.Subscriber("/lift_status" , Lift_status, self.callback_oc) # lay thong tin trang thai mach dieu khien ban nang.
		self.lift_status = Lift_status()
		self.timeStampe_OC = rospy.Time.now()

		self.pub_OC = rospy.Publisher("/lift_control", Lift_control, queue_size=100)	# Dieu khien ban nang.
		self.lift_control = Lift_control()

		# -- App
		rospy.Subscriber("/%s/app_button" % self.agv_name, App_button, self.callback_appButton) # lay thong tin trang thai nut nhan tren man hinh HMI.
		self.app_button = App_button()

		# -- data safety NAV
		rospy.Subscriber("/safety_NAV" , Int8, self.safety_NAV_callback) 
		self.safety_NAV = Int8()

		# -- data nav
		rospy.Subscriber("/nav350_data", Nav350_data, self.nav350_callback) 
		self.nav350_data = Nav350_data()

		# -------------- Cac node thuat toan dieu khien.
		self.pub_moveReq = rospy.Publisher("/%s/request_move" % self.agv_name, Move_request, queue_size=100)
		# self.request_move = Move_request()

		# -- Communicate with Server
		rospy.Subscriber("/%s/NN_cmdRequest" % self.agv_name, NN_cmdRequest, self.callback_cmdRequest)
		self.NN_cmdRequest = NN_cmdRequest()

		self.pub_infoRespond = rospy.Publisher("/%s/NN_infoRespond" % self.agv_name, NN_infoRespond, queue_size = 100)
		self.NN_infoRespond = NN_infoRespond()

		# # -- IMU
		rospy.Subscriber("/imu/data", Imu, self.callback_imu)
		self.imu_data = Imu()

		# -- Safety Zone
		rospy.Subscriber("/safety_zone", Zone_lidar_2head, self.callback_safetyZone)
		self.zoneRobot = Zone_lidar_2head()
		self.is_readZone = 0

		# -- Odometry - POSE
		# rospy.Subscriber('/robotPose_nav', PoseStamped, self.callback_getPose, queue_size = 100)
		# self.robotPose_nav = PoseStamped()
		rospy.Subscriber('/%s/agv_pose' % self.agv_name, Pose, self.callback_getPose, queue_size = 100)
		self.robotPose_nav = Pose()

		# -- Port physical
		rospy.Subscriber("/status_port", Status_port, self.callback_port)
		self.status_port = Status_port()

		# -- parking
		rospy.Subscriber("/%s/parking_respond" % self.agv_name, Parking_respond, self.callback_parking)
		self.parking_status = Parking_respond()
		self.parking_offset = 0.0
		self.parking_poseBefore = Pose()
		self.parking_poseTarget = Pose()
		self.is_parking_status = 0
		self.flag_requirResetparking = 0 # do truoc do co loi Parking: ko nhin thay Tag

		self.pub_parking = rospy.Publisher("/%s/parking_request" % self.agv_name, Parking_request, queue_size= 20)
		self.enb_parking = 0
		self.parking_poseBefore = Pose()
		self.parking_poseAfter = Pose()
		self.parking_offset = 0.0
		# -- backward
		self.flag_requirBackward = 0
		self.completed_backward = 0
		self.backward_x = 0.0
		self.backward_y = 0.0
		self.backward_z = 0.0

		rospy.Subscriber("/%s/NN_infoRequest" % self.agv_name, NN_infoRequest, self.callback_infoRequest)
		self.NN_infoRequest = NN_infoRequest()
		self.timeStampe_server = rospy.Time.now()

		rospy.Subscriber("/%s/status_goal_control" % self.agv_name, Status_goal_control, self.callback_goalControl)
		self.status_goalControl = Status_goal_control() # sub from move_base
		self.timeStampe_statusGoalControl = rospy.Time.now()
		
		rospy.Subscriber("/driver1_respond", Driver_respond, self.callback_driver1)
		self.driver1_respond = Driver_respond()
		
		rospy.Subscriber("/driver2_respond", Driver_respond, self.callback_driver2)
		self.driver2_respond = Driver_respond()
		self.timeStampe_driver = rospy.Time.now()

		# -- loadCell
		rospy.Subscriber("/loadcell_respond", Loadcell_respond, self.callback_loadCell)
		self.loadcell_respond = Loadcell_respond()
		# -- 
		self.pub_weightNow = rospy.Publisher("/weight_now", Int16, queue_size= 20)
		self.weightNow = Int16()
		# -- 
		self.pub_taskDriver = rospy.Publisher("/task_driver", Int16, queue_size= 20)
		self.task_driver = Int16()
		# -- task driver
		self.taskDriver_nothing = 0
		self.taskDriver_resetRead = 1
		self.taskDriver_Read = 2
		self.task_driver.data = self.taskDriver_Read

		# debug
		# self.pub_debug = rospy.Publisher("/debug_control", Debug_control, queue_size=100)
		# self.debug_control = Debug_control()	
		# -- HZ	
		self.FrequencePubBoard = 10.
		self.pre_timeBoard = 0
		# -- Mode operate
		self.md_by_hand = 1
		self.md_auto = 2
		self.mode_operate = self.md_auto    # Lưu chế độ hoạt động.

		# -- Target
		self.target_x = 0.		 # lưu tọa độ điểm đích hiện tại.			
		self.target_y = 0.
		self.target_z = 0.
		self.target_tag = 0.

		self.process = -1        # tiến trình đang xử lý
		self.before_mission = 0  # nhiệm vụ trước khi di chuyển.
		self.after_mission = 0   # nhiệm vụ sau khi di chuyển.

		self.completed_before_mission = 0	 # Báo nhiệm vụ trước đã hoàn thành.
		self.completed_after_mission = 0	 # Báo nhiệm vụ sau đã hoàn thành.
		self.completed_move = 0			 	 # Báo di chuyển đã hoàn thành.
		self.completed_moveSimple = 0        # bao da den dich.
		self.completed_moveSpecial = 0       # bao da den aruco.
		self.completed_setpose = 1           # Báo di setpose đã hoàn thành.
		self.completed_reset = 0             # hoan thanh reset.
		self.completed_MissionSetpose = 0 	 # 
		self.completed_checkLift = 0 	     # kiem tra ke co hay ko sau khi nang. 

		# -- Flag
		self.flag_afterChager = 0
		self.flag_checkLiftError = 0         # cờ báo ko có kệ khi nâng. 
		self.flag_Auto_to_Byhand = 0
		self.enb_move = 0                    # cho phep navi di chuyen
		self.flag_read_client = 0
		self.flag_error = 0
		self.flag_warning = 0
		self.move_req = Move_request()
		self.pre_mess = ""               # lưu tin nhắn hiện tại.

		# -- Check:
		self.is_get_pose = 0
		self.is_mission = 0
		self.is_read_prepheral = 0       # ps2 - sti_read
		self.is_request_client = 0
		self.is_set_pose = 0
		self.is_zone_lidar = 0
		self.completed_wakeup = 0        # khi bật nguồn báo 1 sau khi setpose xong.

		# -- Status to server:
		self.statusU300L = 0
		self.statusU300L_ok = 0
		self.statusU300L_warning = 1
		self.statusU300L_error = 2	
		self.statusU300L_cancelMission = 5	

		# -- Status to detail to follow:
		self.stf = 0
		self.stf_wakeup = 0
		self.stf_running_simple = 1
		self.stf_stop_obstacle = 2
		self.stf_running_speial = 3
		self.stf_running_backward = 4
		self.stf_performUp = 5
		self.stf_performDown = 6

		# -- EMC reset
		self.EMC_resetOn = 1
		self.EMC_resetOff = 0
		self.EMC_reset = self.EMC_resetOff

		# -- EMC write
		self.EMC_writeOn = 1
		self.EMC_writeOff = 0
		self.EMC_write = self.EMC_writeOff

		# -- Led
		self.led = 0
		self.led_error = 1 			# 1
		self.led_simpleRun = 2 		# 2
		self.led_specialRun = 3 	# 3
		self.led_perform = 4 		# 4
		self.led_completed = 5  	# 5	
		self.led_stopBarrier = 6  	# 6

		# -- Mission server
		self.statusTask_liftError = 64 # trang thái nâng kệ nhueng ko có kệ.
		self.serverMission_liftUp = 65
		self.serverMission_liftDown = 66
		self.serverMission_charger = 5
		self.serverMission_unknown = 0
		self.serverMission_liftDown_charger = 6

		# -- Lift task.
		self.liftTask = 0 
		self.liftUp = 2
		self.liftDown = 1
		self.liftStop = 0
		self.liftResetOn = 1
		self.liftResetOff = 0
		self.liftReset = self.liftStop
		self.flag_commandLift = 0
		self.liftTask_byHand = self.liftStop

		# -- Speaker
		self.speaker = 0
		self.speaker_requir = 0 # luu trang thai cua loa
		self.spk_error = 3
		self.spk_move = 1
		self.spk_warn = 2
		self.spk_not = 4		
		self.spk_off = 0
		self.enb_spk = 1

		# -- Charger
		self.charger_on = 1
		self.charger_off = 0		
		self.charger_requir = self.charger_off
		self.charger_write = self.charger_requir
		self.charger_valueOrigin = 0.1

		# -- Voltage
		self.timeCheckVoltage_charger = 1800 # s => 30 minutes.
		self.timeCheckVoltage_normal = 60     # s
		self.pre_timeVoltage = 0   # s
		self.valueVoltage = 0
		self.step_readVoltage = 0

		# -- Cmd_vel
		self.vel_ps2 = Twist()       # gui toc do

		# -- ps2 
		self.time_ht = rospy.get_time()
		self.time_tr = rospy.get_time()
		self.rate_cmdvel = 10. 

		# -- Error Type
		self.error_move = 0
		self.error_perform = 0
		self.error_device = 0  # camera(1) - MC(2) - Main(3) - SC(4)
		self.numberError = 0
		self.lastTime_checkLift = 0.0

		# -- add new
		self.enb_debug = 0
		# --
		self.listError = []
		self.job_doing = 0

		# -- -- -- Su dung cho truong hop khi AGV chuyen Che do bang tay, bi keo ra khoi vi tri => AGV se chay lai.
		# -- Pose tai vi tri Ke, sac
		self.poseWait = Pose()
		self.distance_resetMission = 0.1
		self.flag_resetFramework = 0
		# --
		self.flag_stopMove_byHand = 0
		self.flag_overLoad = 0
		self.weight_origin = 78
		# --
		self.timeStampe_reflectors = rospy.Time.now()
		print ("launch")
		# -- 
		self.enb_check_overLoad = 1

		# -- 15/01
		self.timeStampe_reflectors = rospy.Time.now()
		
		self.pose_parkingRuning = Pose()
		print ("launch")
		# --
		self.cancelbackward_pose = Pose()
		self.cancelbackward_offset = 0.0

		# -- add 18/01/2022 : sua loi di lai cac diem cu khi mat ket noi server.
		self.list_id_unknown = [0, 0, 0, 0, 0]
		self.flag_listPoint_ok = 0
		
		# -- add 19/01/2022 : Check error lost server.
		self.name_card = "wlp3s0b1"
		self.address = "172.21.16.224" # "172.21.15.224"
		self.saveTime_checkServer = rospy.Time.now()
		self.saveStatus_server = 0
		# -- add 22/01/2022
		self.flag_notCharger = 0
		# -- add 21/03/2022
		self.saveTime_checkLift_whenMove = rospy.Time.now()
		# -- add 15/04/2022
		self.flag_listPointEmpty = 0

	def callback_loadCell(self, data):
		self.loadcell_respond = data

	def callback_imu(self, data):
		self.imu_data = data

	def nav350_callback(self, data):
		self.nav350_data = data

	def safety_NAV_callback(self, data):
		self.safety_NAV = data

	def callback_goalControl(self, data):
		self.status_goalControl = data
		self.timeStampe_statusGoalControl = rospy.Time.now()

	def callback_cancelMission(self, dat):
		self.cancelMission_control = dat

	def callback_infoRequest(self, dat):
		self.NN_infoRequest = dat
		self.timeStampe_server = rospy.Time.now()

	def callback_parking(self, dat):
		self.parking_status = dat
		self.is_parking_status = 1

	def callback_reconnect(self, dat):
		self.status_reconnect = dat

	def callback_main(self, dat):
		self.main_info = dat
		self.voltage = round(self.main_info.voltages, 2)
		self.timeStampe_main = rospy.Time.now()

	def callback_hc(self, dat):
		self.HC_info = dat
		self.timeStampe_HC = rospy.Time.now()

	def callback_oc(self, dat):
		self.lift_status = dat
		self.timeStampe_OC = rospy.Time.now()

	def callback_appButton(self, dat):
		self.app_button = dat
		if (self.app_button.cb_check_load == 1):
			self.enb_check_overLoad = 1
		else:
			self.enb_check_overLoad = 0

	def callback_cmdRequest(self, dat):
		self.NN_cmdRequest = dat
		self.timeStampe_server = rospy.Time.now()
		self.flag_listPoint_ok = 0

	# def callback_infoRequest(self, dat):
	# 	self.NN_infoRequest = dat

	def callback_safetyZone(self, dat):
		self.zoneRobot = dat
		self.is_readZone = 1

	def callback_port(self, dat):
		self.status_port = dat

	def callback_driver1(self, dat):
		self.driver1_respond = dat
		self.timeStampe_driver = rospy.Time.now()

	def callback_driver2(self, dat):
		self.driver2_respond = dat
		self.timeStampe_driver = rospy.Time.now()

	def callback_getPose(self, dat):
		self.robotPose_nav = dat
		# doi quaternion -> rad    
		quaternion1 = (self.robotPose_nav.orientation.x, self.robotPose_nav.orientation.y,\
					self.robotPose_nav.orientation.z, self.robotPose_nav.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion1)

		self.NN_infoRespond.x = round(self.robotPose_nav.position.x, 3)
		self.NN_infoRespond.y = round(self.robotPose_nav.position.y, 3)
		self.NN_infoRespond.z = round(euler[2], 3)

	# -- add 15/04/2022
	def check_listPoints(self, list_id):
		count = 0
		try: 
			for i in range(5):
				if (list_id[i] != 0):
					count += 1

			if count != 0:
				return 1
			else: 
				return 0
		except:
			return 0

	def log_mess(self, typ, mess, val):
		# -- add new
		if (self.enb_debug):
			if self.pre_mess != mess:
				if typ == "info":
					rospy.loginfo (mess + ": %s", val)
				elif typ == "warn":
					rospy.logwarn (mess + ": %s", val)
				else:
					rospy.logerr (mess + ": %s", val)
			self.pre_mess = mess

	def pub_move_req(self, ena, ls):
		req_move = Move_request()

		req_move.enable = ena
		req_move.target_x = ls.target_x
		req_move.target_y = ls.target_y
		req_move.target_z = ls.target_z
		req_move.tag = ls.tag
		req_move.offset = ls.offset
		req_move.list_id = ls.list_id
		req_move.list_x = ls.list_x
		req_move.list_y = ls.list_y
		req_move.list_speed = ls.list_speed
		req_move.mission = ls.mission

		self.pub_moveReq.publish(req_move)

	def pub_cmdVel(self, twist , rate , time):
		self.time_ht = time 
		# print self.time_ht - self.time_tr
		# print 1/float(rate)
		if self.time_ht - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = self.time_ht
			self.pub_vel.publish(twist)
		# else :
			# rospy.logwarn("Hz /cmd_vel OVER !! - %f", 1/float(self.time_ht - self.time_tr) )  

	def Main_pub(self, charge, sound, EMC_write, EMC_reset):
		# charge, sound_on, sound_type, EMC_write, EMC_reset, OFF_5v , OFF_22v, led_button1, led_button2, a_coefficient , b_coefficient 
		mai = POWER_request()
		mai.charge = charge
		if sound == 0:
			mai.sound_on = 0	
		else:
			mai.sound_on = 1
			mai.sound_type = sound
		mai.EMC_write = EMC_write
		mai.EMC_reset = EMC_reset
		
		self.pub_requestMain.publish(mai)

# 	def do_cancelMisssion(self):
# 		if self.flag_cancelMission == 0:
# 			self.status_cancel = 0
# 		else:
# 			self.status_cancel = 1

	# -- add 19/01/2020
	def get_ipAuto(self, name_card): # name_card : str()
		try:
			address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
			# print ("address: ", address)
			return 1
		except Exception:
			return 0

	def pingServer(self, address):
			# - ping server
			try:
				ping = subprocess.check_output("ping -{} 1 {}".format('c',address), shell=True)
				# print(ping)
				vitri = str(ping).find("time")
				time_ping = str(ping)[(vitri+5):(vitri+9)]
				return float(time_ping)
			except Exception:
				# print("no ping")			
				return -1

	def check_server(self):
		is_ip = self.get_ipAuto(self.name_card)
		# is_ip = 1
		# time_ping = self.pingServer(self.address)
		time_ping = 0
		if (is_ip == 1):
			if (time_ping == -1):
				return 1 # khong Ping dc server
			else:
				return 0 # oki
		else:
			return 2 # khong lay dc IP

	def point_same_point(self, x1, y1, z1, x2, y2, z2):
		# tọa độ
		x = x2 - x1
		y = y2 - y1
		d = math.sqrt(x*x + y*y)
		# góc 
		if z2*z1 >= 0:
			z = z2 - z1
		else:
			z = z2 + z1
		if d > 0.2 or abs(z) > 0.14:  # 20 cm - ~ 20*C
			return 1
		else:
			return 0

	def pub_park(self, modeRun, poseBefore, poseTarget, offset):
		park = Parking_request()
		park.modeRun = modeRun
		park.poseBefore = poseBefore
		park.poseTarget = poseTarget
		park.offset = offset

		self.pub_parking.publish(park)

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def readbatteryVoltage(self): # 
		time_curr = rospy.get_time()
		delta_time = (time_curr - self.pre_timeVoltage)
		if self.charger_requir == self.charger_on:
			self.flag_afterChager = 1
			if self.step_readVoltage == 0:  # bat sac.
				self.charger_write = self.charger_on
				if (delta_time > self.timeCheckVoltage_charger):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 1

			elif self.step_readVoltage == 1: # tat sac va doi.
				self.charger_write = self.charger_off
				if (delta_time > self.timeCheckVoltage_normal*3):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 2

			elif self.step_readVoltage == 2: # do pin.	
				bat = round(self.main_info.voltages, 1)*10
				# print "charger --"
				if  bat > 255:
					self.valueVoltage = 255
				elif bat < 0:
					self.valueVoltage = 0
				else:
					self.valueVoltage = bat

				self.pre_timeVoltage = time_curr
				self.step_readVoltage = 3

			elif self.step_readVoltage == 3: # doi.
				if (delta_time > 2):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 0

		elif self.charger_requir == self.charger_off:
			if self.flag_afterChager == 1:   # sau khi tat sac doi T s roi moi do dien ap.
				self.pre_timeVoltage = time_curr
				self.flag_afterChager = 0
				self.charger_write = self.charger_off
				self.step_readVoltage = 0
			else:
				if (delta_time > self.timeCheckVoltage_normal):
					self.pre_timeVoltage = time_curr
					bat = round(self.main_info.voltages, 1)*10
					# print "normal --"
					if  bat > 255:
						self.valueVoltage = 255
					elif bat < 0:
						self.valueVoltage = 0
					else:
						self.valueVoltage = int(bat)

	def resetOriginWeight(self):
		if (self.app_button.bt_resetOrigin_load == True):
			if (self.loadcell_respond.status != -1):
				try:
					# Opening JSON file
					file_para = open('/home/stivietnam/catkin_ws/src/sti_control/data/parameter.json')
					data = json.load(file_para)
					# Closing file
					file_para.close()
					# --
					file_para = open('/home/stivietnam/catkin_ws/src/sti_control/data/parameter.json', 'w')
					data["originWeight"] = int(self.loadcell_respond.weight)
					# print (self.weight_origin)

					json.dump(data, file_para, indent = 4)

					# Closing file
					file_para.close()
				except:
					print ("Set Origin Weight ERROR!")

				self.loadOriginWeight()

	def loadOriginWeight(self):
		try:
			# Opening JSON file
			file_para = open('/home/stivietnam/catkin_ws/src/sti_control/data/parameter.json')
			 
			data = json.load(file_para)
			self.weight_origin = data["originWeight"]
			# print (self.weight_origin)

			# Closing file
			file_para.close()
		except:
			print ("Load Origin Weight ERROR!")

	def run_maunal(self):
		cmd_vel = Twist()
		sts = 0
		if (self.app_button.bt_forwards == True):
			if (self.HC_info.zone_sick_ahead == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0
			else:
				cmd_vel.linear.x = 0.25
				cmd_vel.angular.z = 0.0

		if (self.app_button.bt_backwards == True):
			if (self.HC_info.zone_sick_behind == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0
			else:
				cmd_vel.linear.x = -0.25
				cmd_vel.angular.z = 0.0

		if (self.app_button.bt_rotation_left == True):
			if (self.HC_info.zone_sick_ahead == 1 or self.HC_info.zone_sick_behind == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0				
			else:
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.2

		if (self.app_button.bt_rotation_right == True):
			if (self.HC_info.zone_sick_ahead == 1 or self.HC_info.zone_sick_behind == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0				
			else:
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = -0.2

		if (self.app_button.bt_stop == True):
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = 0.0

		if (self.safety_NAV.data == 1):
			cmd_vel = Twist()

		return cmd_vel

	def calculate_angle(self, qua1, qua2): # p1, p2 |
		euler1 = self.quaternion_to_euler(qua1)
		euler2 = self.quaternion_to_euler(qua2)

		delta_angle = euler2 - euler1
		if (abs(delta_angle) >= pi):
			if (delta_angle >= 0):
				delta_angle = (pi*2 - abs(delta_angle))*(-1)
			else:
				delta_angle = pi*2 - abs(delta_angle)
		return delta_angle

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def getPose_from_offset(self, pose_in, offset):
		pose_out = Pose()
		angle = self.quaternion_to_euler(pose_in.orientation)

		if (angle >= 0):
			angle_target = angle - pi
		else:
			angle_target = pi + angle

		pose_out.position.x = pose_in.position.x + cos(angle_target)*offset
		pose_out.position.y = pose_in.position.y + sin(angle_target)*offset

		pose_out.orientation = self.euler_to_quaternion(angle_target)
		return pose_out

	def detectLost_goalControl(self):
		delta_t = rospy.Time.now() - self.timeStampe_statusGoalControl
		if (delta_t.to_sec() > 1.4):
			return 1
		return 0

	def detectLost_driver(self):
		delta_t = rospy.Time.now() - self.timeStampe_driver
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_hc(self):
		delta_t = rospy.Time.now() - self.timeStampe_HC
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_oc(self):
		delta_t = rospy.Time.now() - self.timeStampe_OC
		if (delta_t.to_sec() > 0.8):
			return 1
		return 0

	def detectLost_main(self):
		delta_t = rospy.Time.now() - self.timeStampe_main
		if (delta_t.to_sec() > 1.0):
			return 1
		return 0

	def detectLost_nav(self):
		delta_t = rospy.Time.now() - self.nav350_data.header.stamp
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_poseRobot(self):
		delta_t = rospy.Time.now() - self.robotPose_nav.header.stamp
		if (delta_t.to_sec() > 0.5):
			return 1
		return 0

	def detectLost_Imu(self):
		delta_t = rospy.Time.now() - self.imu_data.header.stamp
		if (delta_t.to_sec() > 0.2):
			return 1
		return 0

	# -- add 19/01/2022
	def detectLost_server(self):
		delta_t = rospy.Time.now() - self.timeStampe_server
		if (delta_t.to_sec() > 15):
			delta_s = rospy.Time.now() - self.saveTime_checkServer
			if (delta_s.to_sec() > 5):
				self.saveTime_checkServer = rospy.Time.now()
				self.saveStatus_server = self.check_server()

			if (self.saveStatus_server == 1):
				return 2
			elif (self.saveStatus_server == 2):
				return 3
			return 1
		return 0

	def detectLost_reflectors(self):
		# -- so luong guong
		if (self.nav350_data.number_reflectors >= 3): # loi mat guong
			self.timeStampe_reflectors = rospy.Time.now()

		delta_t = rospy.Time.now() - self.timeStampe_reflectors
		if (delta_t.to_sec() > 1.2):
			return 1
		return 0

	def detectLost_loadCell(self):
		delta_t = rospy.Time.now() - self.loadcell_respond.header.stamp
		if (delta_t.to_sec() > 2):
			return 1
		return 0

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def find_element(self, value_find, list_in):
		lenght = len(list_in)
		for i in range(lenght):
			if (value_find == list_in[i]):
				return 1
		return 0

	# -- add 21/03/2022
	def check_lostLift_whenMove(self):
		if self.completed_before_mission == 1 and self.before_mission == self.serverMission_liftUp and self.completed_checkLift == 1 and self.completed_moveSpecial == 0 and self.mode_operate == self.md_auto:
			if self.lift_status.sensorLift.data == 1:	
				self.saveTime_checkLift_whenMove = rospy.Time.now()

			delta_s = rospy.Time.now() - self.saveTime_checkLift_whenMove
			if (delta_s.to_sec() > 3):
				return 1
		return 0

	def synthetic_error(self):
		listError_now = []
		# -- EMG
		if self.main_info.EMC_status == 1:
			listError_now.append(121)

		# -- Va cham
		if self.HC_info.vacham == 1:
			listError_now.append(122)

		# -- OC-CAN - 29/06/2022
		if (self.lift_status.status.data == -2):
			listError_now.append(343)

		# -- Error Driver 1
		summation1 = self.driver1_respond.alarm_all + self.driver1_respond.alarm_overload + self.driver1_respond.warning
		if (summation1 != 0):
			listError_now.append(252)

		# -- Error Driver 2
		summation2 = self.driver2_respond.alarm_all + self.driver2_respond.alarm_overload + self.driver2_respond.warning
		if (summation2 != 0):
			listError_now.append(262)

		# -- Ban Nang khong bat duoc cam bien
		if (self.lift_status.status.data == -1 or self.lift_status.status.data == 238):
			listError_now.append(141)

		# -- Loi Code Parking:
		if (self.parking_status.warning == 2):
			listError_now.append(281)

		# -- load cell - Loi dau noi
		if (self.loadcell_respond.status == -1):
			listError_now.append(182)
	
		# -- Loi qua tai
		if (self.enb_check_overLoad == 1):
			if (self.loadcell_respond.weight > (700 + self.weight_origin) ):
				listError_now.append(184)

		# -- Nâng kệ nhưng không có kệ.
		if self.flag_checkLiftError == 1:
			listError_now.append(471)

		# -- 19/01/2022 - Mat giao tiep voi Server 
		# sts_sr = self.detectLost_server()
		# if sts_sr == 1: # lost server
		# 	listError_now.append(431)
		# elif sts_sr == 2: # lost server: Ping
		# 	listError_now.append(432)
		# elif sts_sr == 3: # lost server: IP
		# 	listError_now.append(433)

		# --- Low battery
		if self.voltage < 23:
			listError_now.append(451)

		# -- Co vat can
		if (self.status_goalControl.safety == 1):
			listError_now.append(411)

		# -- Co vat can khi di chuyen parking
		if (self.parking_status.warning == 1):
			listError_now.append(412)
			
		# -- AGV dung do da di het danh sach diem.
		if self.status_goalControl.complete_misson == 2:
			if self.status_goalControl.misson == 1 or self.status_goalControl.misson == 3:
				listError_now.append(441)
				self.flag_listPoint_ok = 1

		# -- add 22/01/2022 - Loi khong sac duowc pin
		if self.flag_notCharger == 1:
			listError_now.append(452)

		return listError_now

	def resetAll_variable(self):
		self.enb_move = 0

		# if self.parking_status.status == 11:  # khi doi lenh. no reset truoc khi parking nhan ra no da hoan thanh.
		# 	self.flag_requirBackward = 1

		self.enb_parking = 0
		
		# -- add 02/08/2023
		# self.completed_before_mission = 0
		self.completed_after_mission = 0
		self.completed_move = 0
		self.completed_moveSimple = 0
		self.completed_moveSpecial = 0
		self.completed_backward = 0
		self.completed_MissionSetpose = 0

		self.completed_checkLift = 0
		self.flag_checkLiftError = 0

		self.enb_mission = 0
		self.mission = 0
		self.lifttable = 0
		self.conveyor = 0
		self.flag_listPoint_ok = 0

		# rospy.logwarn("Update new target from: X= %s | Y= %s to X= %s| Y= %s", self.target_x, self.target_y, self.NN_cmdRequest.target_x, self.NN_cmdRequest.target_y)
		self.log_mess("info", "Update new target: X_new = ", self.NN_cmdRequest.target_x)
		self.target_x = self.NN_cmdRequest.target_x
		self.target_y = self.NN_cmdRequest.target_y
		self.target_z = self.NN_cmdRequest.target_z
		self.target_tag = self.NN_cmdRequest.tag
		self.before_mission = self.NN_cmdRequest.before_mission
		self.after_mission = self.NN_cmdRequest.after_mission
		# -- add 12/11/2021
		self.flag_resetFramework = 0
		self.flag_Auto_to_Byhand = 0
		# -- add 22/01/2022
		self.flag_notCharger = 0
		
	def run(self):
		if self.process == -1: # khi moi khoi dong len
			# mode_operate = mode_hand
			self.loadOriginWeight()
			time.sleep(0.01)
			self.resetOriginWeight()
			time.sleep(0.01)

			# self.mode_operate = self.md_by_hand
			self.mode_operate = self.md_auto	
			self.led = 0
			self.speaker_requir = self.spk_warn
			self.process = 0
			self.enb_parking = 0

		elif self.process == 0:	# chờ cac node khoi dong xong, # do node này khởi động sau cùng nên không phải chờ. 
			ct = 8
			if ct == 8:
				self.process = 1

		elif self.process == 1: # reset toan bo: Main - MC - OC.
			self.flag_error = 0
			self.completed_before_mission = 0
			self.completed_after_mission = 0
			self.completed_move = 0
			self.completed_checkLift = 0
			self.liftTask = 0
			self.error_device = 0
			self.error_move = 0
			self.error_perform = 0
			self.process = 2

		elif self.process == 2: # kiem tra toan bo thiet bi ok, xem lỗi này là cảnh báo hay lỗi cần dừng khẩn 
			self.listError =  self.synthetic_error()
			self.numberError = len(self.listError)
			lenght = len(self.listError)
			
			count_error = 0
			count_warning = 0
			for i in range(lenght):
				if (self.listError[i] < 400):
					count_error += 1
				else:
					count_warning += 1

			if count_error == 0 and count_warning == 0:
				self.flag_error = 0
				self.flag_warning = 0

			elif count_error == 0 and count_warning > 0:
				self.flag_error = 0
				self.flag_warning = 1

			else:
				self.flag_error = 1
				self.flag_warning = 0

			if self.flag_cancelMission == 0:
				# -- edit
				if (self.flag_error == 1):
					self.statusU300L = self.statusU300L_error      # = 2
				else:
					if (self.flag_warning == 1):
						self.statusU300L = self.statusU300L_warning  # = 1
					else:
						self.statusU300L = self.statusU300L_ok       # = 0
			else:
				self.statusU300L = self.statusU300L_cancelMission    # = 5

			# -- ERROR, Khi lỗi thì làm gì, khí nhấn nút xóa lỗi thì làm gì 
			if self.app_button.bt_clearError == 1 or self.main_info.stsButton_reset == 1:
				if self.flag_requirResetparking == 1:
					self.enb_parking = 0
					self.flag_requirResetparking = 0

				# -- dung parking neu co loi. - thay doi
				# self.enb_parking = 2

				self.EMC_write = self.EMC_writeOff
				self.EMC_reset = self.EMC_resetOn
				# -- sent clear error
				self.flag_error = 0
				self.error_device = 0
				self.error_move = 0
				self.error_perform = 0

				self.flag_checkLiftError = 0
				self.task_driver.data = self.taskDriver_resetRead
				
				# -- xoa loi ban nang.
				if (self.lift_status.status.data == -1):
					self.liftReset = self.liftResetOn
				else:
					self.liftReset = self.liftResetOff

				# -- add 22/01/2022
				self.flag_notCharger = 0

			else:
				self.task_driver.data = self.taskDriver_Read
				self.EMC_reset = self.EMC_resetOff

				# -- Add new: 23/12: Khi mat ket Driver, EMG duoc keo len.
				if (self.flag_error == 1):
					if (self.find_element(251, self.listError) == 1 or self.find_element(261, self.listError) == 1):
						self.EMC_write = self.EMC_writeOn
					else:
						self.EMC_write = self.EMC_writeOff

			# if (self.error_device != 0 or self.error_perform != 0 or self.error_move != 0):
			# 	self.flag_error = 1	

			self.process = 3

		elif self.process == 3: # read app, chuyển chế độ điều khiển 
			if self.app_button.bt_passHand == 1:
				if self.mode_operate == self.md_auto: # keo co bao dang o tu dong -> chuyen sang bang tay.
					self.flag_Auto_to_Byhand = 1				
				self.mode_operate = self.md_by_hand
				
			if self.app_button.bt_passAuto == 1:
				if self.completed_setpose == 1:
					self.mode_operate = self.md_auto
				else:
					self.log_mess("info", "Make sure AGV Run in Map", 0)

			if self.mode_operate == self.md_by_hand:
				self.process = 30
			elif self.mode_operate == self.md_auto:
				self.process = 40
	# ------------------------------------------------------------------------------------
	# -- BY HAND:
		elif self.process == 30:
			self.enb_move = 0
			self.job_doing = 20

			if self.parking_status.status != 0:
				self.enb_parking = 0

			if self.flag_error == 0:
			  # -- Send vel
				if self.lift_status.status.data == 0 or self.lift_status.status.data >= 3:  # Đang thực hiện nhiệm vụ ở chế độ auto -> ko cho phép di chuyển.
					# -- Move
					self.velPs2 = self.run_maunal()
					self.pub_cmdVel(self.velPs2, self.rate_cmdvel, rospy.get_time())
				else:
					self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

			else: # -- Has error
				# pass
				self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

			# ------------------------------------------------------------ HMI
			# -- Lift
			if self.app_button.bt_lift_up == True:
				self.liftTask_byHand = self.liftUp
				self.flag_commandLift = 1

			elif self.app_button.bt_lift_down == True:
				self.liftTask_byHand = self.liftDown
				self.flag_commandLift = 1

			if self.flag_commandLift == 1:
				self.liftTask = self.liftTask_byHand
				if self.lift_status.status.data >= 3: # Hoàn thành
					self.flag_commandLift = 0
			else:
				self.liftTask = self.liftStop

			# -- Speaker
			if self.app_button.bt_spk_on == True:
				self.enb_spk = 1
			elif self.app_button.bt_spk_off == True:
				self.enb_spk = 0

			# -- Charger
			if self.app_button.bt_chg_on == True:
				self.charger_requir = self.charger_on
			elif self.app_button.bt_chg_off == True:
				self.charger_requir = self.charger_off

			self.process = 2

	# -- RUN AUTO:
		elif self.process == 40: # -- kiem tra loi, từ chế độ bằng tay chuyển sang chế độ tự động thì cần check lại lỗi . 
			if self.flag_error == 1: # thay doi != 0
				self.enb_move = 0
				self.enb_mission = 0
				self.process = 2

			else:
				self.process = 41

		elif self.process == 41:    # kiem tra muc tieu thay doi, ví dụ đang trong quá trình thực hiện lệnh(đang nâng hạ, đang lùi kệ,...) mà gặp lệnh mới thi các trạng thái của AGV sẽ được update ở đây. 
			if ( self.target_x != self.NN_cmdRequest.target_x ) or ( self.target_y != self.NN_cmdRequest.target_y) or ( self.target_tag != self.NN_cmdRequest.tag):
				if (self.NN_cmdRequest.target_x < 500) and (self.NN_cmdRequest.target_y < 500):
					# self.move_req = Move_request()

					# Khong che phep doi len khi dang:
					# - 1, Nang hoac Ha.
					# - 2, Dang di vao ke.
					# - 3, Dang di ra khoi ke.
					a1 = 0
					a2 = 0
					if self.lift_status.status == -1 or self.lift_status.status == 1 or self.lift_status.status == 2:  # Ban nang: Dung hoac Hoan thanh.
						a1 = 1
					else:
						a1 = 0

					if self.parking_status.status > 8 and self.parking_status.status <= 11: # Dang di vao trong ke -> ko cho doi lenh.
						a2 = 1 # thay doi
						# if (self.flag_error == 1 and self.numberError == 121): # EMC
						# 	a2 = 0
							# self.enb_parking = 0
						# else:
						# 	a2 = 1
					else:
						a2 = 0

					if a1 == 1 or a2 == 1:
						self.log_mess("warn", "Have new target but must Waiting perform done ....", 0)
						self.process = 42
					else:
						self.resetAll_variable()
						self.process = 42
				else:
					self.log_mess("info", "Have new target but Not fit: X= ", self.NN_cmdRequest.target_x)
					self.log_mess("info", "Have new target but Not fit: y= ", self.NN_cmdRequest.target_y)
					self.process = 2
				self.NN_infoRespond.offset = 0 # --
			else:
				# Sử dụng trong trường hợp Lỗi vẫn hành: đã thực hiện xong nhiệm vụ di chuyển -> lái tay sang vị trí khác -> AGV đứng im (lẽ ra phải di chuyển đến đích)
				# if self.completed_move == 1:
				# 	if self.target_x < 500 and self.target_y < 500:
				# 		if self.point_same_point(self.target_x, self.target_y, self.target_z, self.NN_infoRespond.x, self.NN_infoRespond.y, self.NN_infoRespond.z) == 1:
				# 			self.completed_move = 0

				# Nếu target ko đổi mà nhiện vụ muốn thay đổi (lấy hoặc trả hàng luôn tại đó).
				if self.completed_after_mission == 1:
					if self.after_mission != self.NN_cmdRequest.after_mission:
						self.completed_after_mission = 0
						self.log_mess("info", "After mission change to ", self.NN_cmdRequest.after_mission)
						self.after_mission = self.NN_cmdRequest.after_mission

				self.process = 42

		elif self.process == 42:   # AGV chạy tiếp lệnh tự động đang dở, khi người dùng chuyển từ chệ độ tự động >> bằng tay >> tự động 
			if self.flag_Auto_to_Byhand == 1: 
				# -- add 12/11/2021 : Chay lai quy trinh Vao Sac khi Chuuyen che do.
				if self.completed_after_mission == 1:
					if self.after_mission == self.serverMission_liftDown_charger or self.after_mission == self.serverMission_charger: #
						delta_distance = self.calculate_distance(self.robotPose_nav.pose.position, self.poseWait.position)
						# print ("poseWait: " + str(self.poseWait.position.x) + " | " + str(self.poseWait.position.y))
						# print ("robotPose_nav: " + str(self.robotPose_nav.pose.position.x) + " | " + str(self.robotPose_nav.pose.position.y))
						# print ("delta_distance: ", delta_distance)
						if (delta_distance > self.distance_resetMission):
							self.resetAll_variable()
							self.completed_backward = 1
				else:
					# -- add 23/12/2021: Xu ly loi Dang Parking thi bi chuyen che do -> AGV cu parking.
					if self.completed_before_mission == 1 and self.completed_moveSimple == 1 and self.completed_moveSpecial == 0:
						delta_distance = self.calculate_distance(self.robotPose_nav.pose.position, self.pose_parkingRuning.position)
						# -- phien ban 1: 
						# delta_angle = self.calculate_angle(self.robotPose_nav.pose.orientation, self.pose_parkingRuning.orientation)
						# -- phien ban 2: Xac dinh do lech giua goc cua AGV voi goc cua diem vao Tag
						delta_angle = self.calculate_angle(self.robotPose_nav.pose.orientation, self.parking_poseTarget.orientation)
						# print ("--------------------")
						# print ("delta_distance: ", delta_distance)
						# print ("delta_angle: ", degrees(delta_angle))

						if (delta_distance > 0.2 or abs(delta_angle) > radians(20)):
							self.completed_moveSimple = 0

				# -- add 27/12/2021: Sua loi cu di thang ra sau khi parking
				if self.completed_before_mission == 1 and self.completed_checkLift == 1 and self.completed_backward == 0:
					delta_distance = self.calculate_distance(self.robotPose_nav.pose.position, self.cancelbackward_pose.position)
					delta_angle = self.calculate_angle(self.robotPose_nav.pose.orientation, self.cancelbackward_pose.orientation)

					if delta_distance > self.cancelbackward_offset*0.85 or abs(delta_angle) > radians(20) :
						self.completed_backward = 1

				# -- add 18/01/2022
				if (self.flag_listPoint_ok == 1):
					self.NN_cmdRequest.list_id = self.list_id_unknown
					self.flag_listPoint_ok = 0

				self.job_doing = 1
				# thuc hien lai nhiem vu nang, ha, sac sau khi chuyen che do tu tu dong sang bang tay.
				if self.completed_after_mission == 0 and self.completed_before_mission == 0:
					self.flag_Auto_to_Byhand = 0

				elif self.completed_after_mission == 0 and self.completed_before_mission == 1:
					if self.before_mission == self.serverMission_unknown:
						self.flag_Auto_to_Byhand = 0

						self.completed_checkLift = 1
						self.NN_infoRespond.task_status = self.serverMission_unknown

					elif self.before_mission == self.serverMission_liftDown: # Hạ
						self.liftTask = self.liftDown	

						# -- add 14/11
						self.flag_overLoad = 0

						if self.lift_status.status.data == 3:  # Hoàn thành
							self.liftTask = self.liftStop
							self.flag_Auto_to_Byhand = 0

							self.completed_checkLift = 1
							self.NN_infoRespond.task_status = self.serverMission_liftDown

					elif self.before_mission == self.serverMission_liftUp: # Nâng
						self.liftTask = self.liftUp				
						if self.lift_status.status.data == 4:  # Hoàn thành
							self.liftTask = self.liftStop
							self.flag_Auto_to_Byhand = 0

							self.completed_checkLift = 0
							self.NN_infoRespond.task_status = self.statusTask_liftError

					else: 
						self.flag_Auto_to_Byhand = 0

						self.completed_checkLift = 1
						self.NN_infoRespond.task_status = self.before_mission

				elif self.completed_after_mission == 1 and self.completed_before_mission == 1:
					if self.after_mission == self.serverMission_unknown:
						self.flag_Auto_to_Byhand = 0

						self.completed_checkLift = 1
						self.NN_infoRespond.task_status = self.serverMission_unknown

					elif self.after_mission == self.serverMission_liftDown: # Hạ
						self.liftTask = self.liftDown	
						# -- add 14/11
						self.flag_overLoad = 0

						if self.lift_status.status.data == 3:  # Hoàn thành
							self.liftTask = self.liftStop
							self.flag_Auto_to_Byhand = 0

							self.completed_checkLift = 1
							self.NN_infoRespond.task_status = self.serverMission_liftDown


					elif self.after_mission == self.serverMission_liftUp: # Nâng
						self.liftTask = self.liftUp				
						if self.lift_status.status.data == 4:  # Hoàn thành
							self.liftTask = self.liftStop
							self.flag_Auto_to_Byhand = 0

							self.completed_checkLift = 0
							self.NN_infoRespond.task_status = self.statusTask_liftError

					elif self.after_mission == self.serverMission_liftDown_charger: # Hạ
						self.liftTask = self.liftDown				
						if self.lift_status.status.data == 3:  # Hoàn thành
							self.charger_requir = self.charger_on
							self.liftTask = self.liftStop
							self.flag_Auto_to_Byhand = 0
		
							self.completed_checkLift = 1
							self.NN_infoRespond.task_status = self.serverMission_liftDown_charger

					elif self.after_mission == self.serverMission_charger: # - Sac
						self.charger_requir = self.charger_on
						self.flag_Auto_to_Byhand = 0
						self.completed_checkLift = 1
						self.NN_infoRespond.task_status = self.serverMission_charger

					else: 
						self.flag_Auto_to_Byhand = 0
						self.completed_checkLift = 1
						self.NN_infoRespond.task_status = self.after_mission
				# -- add 18/11
				else:
					self.flag_Auto_to_Byhand = 0

				self.process = 2
			else:
				self.process = 43
		
		elif self.process == 43: 	# Thực hiện nhiệm vụ trước. Kiểm tra nhiệm vụ trước( nâng, hạ, lấy hàng, trả hàng ) đã hoàn thành chưa 
			if self.completed_before_mission == 0: # chua thuc hien
				
				self.job_doing = 2 
				if self.before_mission == 0:
					self.charger_requir = self.charger_off
					self.log_mess("info", "Before mission Not have", self.before_mission)
					self.completed_before_mission = 1
					# -- add new
					self.completed_checkLift = 1
					self.NN_infoRespond.task_status = 0

				elif self.before_mission == self.serverMission_liftDown: # Hạ
					self.charger_requir = self.charger_off
					self.liftTask = self.liftDown
					
					# -- add 14/11
					self.flag_overLoad = 0

					if self.lift_status.status.data == 3:  # Hoàn thành
						self.log_mess("info", "Before mission completed", self.serverMission_liftDown)
						self.liftTask = self.liftStop
						self.completed_before_mission = 1

						self.completed_checkLift = 1
						self.NN_infoRespond.task_status = self.serverMission_liftDown


				elif self.before_mission == self.serverMission_liftUp: # Nâng
					# -- add 14/11 Them phan kiem tra tai
					self.charger_requir = self.charger_off
					self.liftTask = self.liftUp		
					if self.lift_status.status.data == 4:  # Hoàn thành
						self.log_mess("info", "Before mission completed", self.serverMission_liftUp)
						self.liftTask = self.liftStop
						self.completed_before_mission = 1

						self.completed_checkLift = 0
						self.NN_infoRespond.task_status = self.statusTask_liftError

				# -- add 20/01/2022
				else:
					self.completed_before_mission = 1

					self.completed_checkLift = 1
					self.NN_infoRespond.task_status = self.before_mission

				self.process = 2

			else:
				self.process = 44

		elif self.process == 44:	# Thuc hien kiểm tra kệ có trên bàn nâng ko. >> Chỉ dùng cho dòng nâng, dòng băng tải ko cần 
			if self.completed_checkLift == 0:
				self.job_doing = 3
				if self.before_mission == self.serverMission_liftUp or self.after_mission == self.serverMission_liftUp: # Nâng
					if self.lift_status.sensorLift.data == 0:
						self.lastTime_checkLift = time.time()

					t = (time.time() - self.lastTime_checkLift)%60
					if (t > 2): # 2 s						
						self.flag_checkLiftError = 0
						self.completed_checkLift = 1
						self.NN_infoRespond.task_status = self.serverMission_liftUp
					else:
						self.flag_checkLiftError = 1 # báo không có kệ.
				else:
					self.completed_checkLift = 1
				self.process = 2
			else:
				self.process = 45

		elif self.process == 45:	# Thuc hien di chuyen tiến >> Đi tiến ra khỏi vị trí trả hàng đến điểm di chuyển. 
			if self.completed_backward == 1:
				self.process = 46
			else:
				self.job_doing = 4
				# -- add 15/04/2022
				if self.check_listPoints(self.NN_cmdRequest.list_id) == 1:
					if self.flag_requirBackward == 1:
						self.move_req.target_x = self.backward_x
						self.move_req.target_y = self.backward_y
						self.move_req.target_z = self.backward_z
						if self.status_goalControl.misson == 2 and self.status_goalControl.complete_misson == 1:  # Hoan thanh di chuyen lui.
							self.flag_requirBackward = 0
							self.enb_move = 0
							self.completed_backward = 1	
						else:
							self.enb_move = 2
					else:
						self.completed_backward = 1	
						self.enb_move = 0
					# -- add 15/04/2022
					self.flag_listPointEmpty = 0
					self.process = 2
				else:
					self.enb_move = 0
					self.flag_listPointEmpty = 1
					self.log_mess("info", "process = 43 - Buoc lui ra, Danh sach diem Null -> ko chay", 0)
					self.process = 2

		elif self.process == 46:	# Thuc hien di chuyen diem thuong >> Di chuyển điểm thường đến điểm cuối của lộ trình 
			if self.completed_moveSimple == 1:      # 
				self.process = 47
				self.enb_move = 0
			else:
				self.job_doing = 5
				if (len(self.NN_cmdRequest.list_x) != 0) and (len(self.NN_cmdRequest.list_y) != 0) and (self.target_x < 500) and (self.target_y < 500):
					self.move_req.target_x = self.target_x
					self.move_req.target_y = self.target_y
					self.move_req.target_z = self.target_z
					
					self.parking_offset = self.NN_cmdRequest.offset

					self.move_req.tag = self.NN_cmdRequest.tag
					self.move_req.offset = self.NN_cmdRequest.offset

					self.move_req.list_id = self.NN_cmdRequest.list_id
					self.move_req.list_x = self.NN_cmdRequest.list_x
					self.move_req.list_y = self.NN_cmdRequest.list_y
					self.move_req.list_speed = self.NN_cmdRequest.list_speed

					if self.before_mission == self.serverMission_liftUp: # Nâng
						self.move_req.mission = 1
					else:
						self.move_req.mission = 0
					
					# -- add 19/01/2022 : chuyen vung sick.
					if self.before_mission == self.serverMission_liftUp: # Nâng
						self.enb_move = 1 # -- vung To
					else:
						self.enb_move = 3 # -- vung Nho

					# self.enb_move = 1
				else:
					self.log_mess("warn", "ERROR: Target of List point wrong !!!", 0)

				if self.status_goalControl.complete_misson == 1 :  # Hoan thanh di chuyen.
					if self.status_goalControl.misson == 1 or self.status_goalControl.misson == 3:
						self.completed_moveSimple = 1
						self.enb_move = 0
						self.log_mess("info", "Move completed", 0)
				self.process = 2

	 	# -- Parking	
		elif self.process == 47:    #  Kiểm tra vị trí này đã có kệ hay chưa, nếu có thì ko mang vào nữa, đứng báo lỗi 
		  	# print "46 --"
			if self.completed_moveSpecial == 1:
				self.completed_move = 1
				self.process = 34
			else:
				self.job_doing = 6
				self.process = 48 # kiem tra diem vao ke

		elif self.process == 48:   # Parking, AGV lùi vào trả kệ 
			if self.completed_moveSpecial == 0: # chua hoan thanh di chuyen
				# sau sẽ thêm phần khi đổi mã tag thì tự động reset paking.
				if self.parking_status.status == 1: # Free
					self.process = 50
					# self.log_mess("info", "Special point: readly", self.parking_status.status)

				elif self.parking_status.status == 51: # -- Completed Run
					# self.log_mess("info", "Special point: Completed to point: ", self.parking_status.status)
					self.enb_parking = 0
					self.completed_moveSpecial = 1
					self.flag_requirBackward = 1   # 1 - yeu cau 
					self.backward_x = self.target_x
					self.backward_y = self.target_y
					self.backward_z = self.target_z
					self.process = 2

					# -- add 12/11/2021
					self.poseWait = self.robotPose_nav.pose

					# -- add 27/12/2021
					self.cancelbackward_pose = self.robotPose_nav.pose
					self.cancelbackward_offset = self.parking_offset
				else:
					self.process = 2

				# -- add 23/12/2021:
				self.pose_parkingRuning = self.robotPose_nav.pose
			else:
				self.process = 2

		elif self.process == 50: 	# Yêu cầu di chuyển. và check hoàn thành , process 49 và 50 có thể gộp hoặc tách 
			if self.after_mission == self.serverMission_liftDown_charger or self.after_mission == self.serverMission_charger:
				self.enb_parking = 2
			elif self.after_mission == self.serverMission_liftDown:
				self.enb_parking = 1
			else:
				self.enb_parking = 3

			self.parking_offset = self.NN_cmdRequest.offset
			# -
			self.parking_poseBefore.position.x = self.NN_cmdRequest.target_x
			self.parking_poseBefore.position.y = self.NN_cmdRequest.target_y
			self.parking_poseBefore.orientation = self.euler_to_quaternion(self.NN_cmdRequest.target_z)
			self.parking_poseTarget = self.getPose_from_offset(self.parking_poseBefore, self.parking_offset)

			self.log_mess("info", "Tag offset requir: ", self.parking_offset)
			self.process = 2
	# ------------------------------------------------------------------------------------
		elif self.process == 34:	# -- Thực hiện nhiệm vụ sau. ( nâng , hạ )
			if self.completed_after_mission == 0: # chua thuc hien
				self.job_doing = 7
				if self.after_mission == 0:
					self.log_mess("info", "Last mission Not have Suf: ", self.after_mission)
					self.completed_after_mission = 1

					self.completed_checkLift = 1
					self.NN_infoRespond.task_status = self.after_mission
					
				elif self.after_mission == self.serverMission_liftDown: # Hạ

					self.liftTask = self.liftDown
					# -- add 14/11
					self.flag_overLoad = 0

					if self.lift_status.status.data == 3: # Hoàn thành
						self.log_mess("info", "Last mission completed: ", self.serverMission_liftDown)
						self.liftTask = self.liftStop
						self.completed_after_mission = 1

						self.completed_checkLift = 1
						self.NN_infoRespond.task_status = self.serverMission_liftDown

				elif self.after_mission == self.serverMission_liftUp: # Nâng

					self.liftTask = self.liftUp				
					if self.lift_status.status.data == 4: # Hoàn thành
						self.log_mess("info", "Last mission completed: ", self.serverMission_liftUp)
						self.liftTask = self.liftStop
						self.completed_after_mission = 1
						
						self.completed_checkLift = 0
						self.NN_infoRespond.task_status = self.statusTask_liftError

				elif self.after_mission == self.serverMission_charger: # sac
					self.charger_requir = self.charger_on
					self.log_mess("info", "Last mission completed: ", self.serverMission_charger)
					self.completed_after_mission = 1

					self.completed_checkLift = 1
					self.NN_infoRespond.task_status = self.serverMission_charger
					
				elif self.after_mission == self.serverMission_liftDown_charger: # sac

					self.liftTask = self.liftDown
					if self.lift_status.status.data == 3: # Hoàn thành
						self.log_mess("info", "Last mission completed", self.serverMission_liftDown_charger)
						self.liftTask = self.liftStop
						self.completed_after_mission = 1
						self.charger_requir = self.charger_on # turn on charger	

						self.completed_checkLift = 1
						self.NN_infoRespond.task_status = self.serverMission_liftDown_charger

				# -- add 20/01/2022			
				else:
					self.completed_after_mission = 1

					self.completed_checkLift = 1
					self.NN_infoRespond.task_status = self.after_mission
					
				self.process = 2
			else:
				self.process = 35

		elif self.process == 35:    # hoàn thành hết rồi, chờ nhiệm vụ mới. 
			self.job_doing = 8
			self.process = 2
			self.log_mess("warn", "Wating new Target ...", 0)
			
		# -- Tag + Offset:
		if self.mode_operate == self.md_auto:    # khi AGV ở trạng thái nào thì gửi lên cái gì cho Traffic 
			if self.completed_move == 1:
				self.NN_infoRespond.tag = self.NN_cmdRequest.tag
				self.NN_infoRespond.offset = self.NN_cmdRequest.offset
			else:
				self.NN_infoRespond.tag = 0
				self.NN_infoRespond.offset = 0

			# if self.completed_before_mission == 1 and self.completed_after_mission == 0 and self.completed_checkLift == 1:
			# 	self.NN_infoRespond.task_status = self.before_mission
			# elif self.completed_before_mission == 1 and self.completed_after_mission == 0 and self.completed_checkLift == 0:
			# 	self.NN_infoRespond.task_status = self.statusTask_liftError
			# if self.completed_before_mission == 1 and self.completed_after_mission == 1:
			# 	self.NN_infoRespond.task_status = self.after_mission

		self.NN_infoRespond.status = self.statusU300L    # Status: Error
		self.NN_infoRespond.error_perform = self.process
		self.NN_infoRespond.error_moving = self.flag_error
		self.NN_infoRespond.error_device = self.numberError
		self.NN_infoRespond.listError = self.listError
		self.NN_infoRespond.process = self.job_doing

		# -- Battery - ok
		self.readbatteryVoltage()
		self.NN_infoRespond.battery = int(round(self.voltage, 1)*10)
		# self.NN_infoRespond.battery = 255

		# -- mode respond server
		if self.mode_operate == self.md_by_hand:         # Che do by Hand
			self.NN_infoRespond.mode = 1

		elif self.mode_operate == self.md_auto:          # Che do Auto
			self.NN_infoRespond.mode = 2

		# -- Respond Client
		self.pub_infoRespond.publish(self.NN_infoRespond)    # Pub Client

		# -- Request Navigation
		self.pub_move_req(self.enb_move, self.move_req)  # Pub Navigation

		# -- Speaker    Điều khiển loa khi có tín hiệu báo lỗi 
		if self.flag_error == 1 and self.flag_warning == 1:
			self.speaker_requir = self.spk_error
		elif self.flag_error == 1 and self.flag_warning == 0:
			self.speaker_requir = self.spk_error
		elif self.flag_error == 0 and self.flag_warning == 1:
			self.speaker_requir = self.spk_warn			
		else:
			if self.completed_backward == 0 and self.completed_before_mission == 0 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 0 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 1 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:	
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 0:	
				self.speaker_requir = self.spk_move
			elif self.completed_backward == 1 and self.completed_before_mission == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 1:
				self.speaker_requir = self.spk_move
			else:
				self.speaker_requir = self.spk_move

		# -- LED           
		if self.flag_error == 1:
			self.led = self.led_error
		else:
			if self.completed_before_mission == 0 and self.completed_backward == 0 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				self.led = self.led_perform

			elif self.completed_before_mission == 1 and self.completed_backward == 0 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				if (self.status_goalControl.safety == 1):
					self.led = self.led_stopBarrier
				else:
					self.led = self.led_simpleRun

			elif self.completed_before_mission == 1 and self.completed_backward == 1 and self.completed_moveSimple == 0 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:
				if (self.status_goalControl.safety == 1):
					self.led = self.led_stopBarrier
				else:
					self.led = self.led_simpleRun

			elif self.completed_before_mission == 1 and self.completed_backward == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 0 and self.completed_after_mission == 0:	
				# self.led = self.led_specialRun
				if (self.parking_status.warning == 1):
					self.led = self.led_stopBarrier
				else:
					self.led = self.led_specialRun

			elif self.completed_before_mission == 1 and self.completed_backward == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 0:	
				self.led = self.led_perform

			elif self.completed_before_mission == 1 and self.completed_backward == 1  and self.completed_moveSimple == 1 and self.completed_moveSpecial == 1 and self.completed_after_mission == 1:
				self.led = self.led_completed

			else:
				self.led = self.led_simpleRun
				
		# -- -- -- pub Board
		time_curr = rospy.get_time()
		d = (time_curr - self.pre_timeBoard)
		if (d > float(1/self.FrequencePubBoard)): # < 20hz 
			self.pre_timeBoard = time_curr
			# -- Request OC:
			self.lift_control.control.data = self.liftTask
			self.lift_control.reset.data = self.liftReset
			self.pub_OC.publish(self.lift_control)

			# -- Request Main: charger, sound, EMC_write, EMC_reset
			# -- modiify 22/01/2022
			if self.flag_error == 0:
				# tat Loa khi sac thanh cong!
				if self.completed_after_mission == 1 and self.mode_operate == self.md_auto and (self.after_mission == self.serverMission_charger or self.after_mission == self.serverMission_liftDown_charger):
					if self.charger_write == self.charger_on:
						if self.main_info.charge_current >= self.charger_valueOrigin :
							self.speaker = self.spk_off
							self.flag_notCharger = 0
						else:
							self.flag_notCharger = 1
							if self.enb_spk == 1:
								self.speaker = self.speaker_requir
							else:
								self.speaker = self.spk_off	
					else:
						self.speaker = self.spk_off
						self.flag_notCharger = 0
				else:
					self.flag_notCharger = 0
					if self.enb_spk == 1:
						self.speaker = self.speaker_requir
					else:
						self.speaker = self.spk_off	
			else:
				self.flag_notCharger = 0
				if self.enb_spk == 1:
					self.speaker = self.speaker_requir
				else:
					self.speaker = self.spk_off

			self.Main_pub(self.charger_write, self.speaker, self.EMC_write, self.EMC_reset)  # MISSION

			# -- Request HC:
			self.HC_request.RBG1 = self.led 
			# if self.charger_write == self.charger_on and self.main_info.charge_current >= 0.5: 
			# 	self.HC_request.RBG1 = 0
			# else:
				# self.HC_request.RBG1 = self.led

			self.HC_request.RBG2 = 0
			self.pub_HC.publish(self.HC_request)

			# -- Request task Driver:
			self.pub_taskDriver.publish(self.task_driver)

			# -- weight
			weight = int(self.loadcell_respond.weight) - self.weight_origin
			if (weight < 0):
				self.weightNow.data = 0
			else:
				self.weightNow.data = weight

			self.resetOriginWeight()
			self.pub_weightNow.publish(self.weightNow)
			
	  	# -- cancel mission

		if self.cancelMission_control.data == 1:
			self.flag_cancelMission = 1

		if self.NN_cmdRequest.id_command == 0:
			self.flag_cancelMission = 0

		self.cancelMission_status = self.cancelMission_control
		self.pub_cancelMission.publish(self.cancelMission_status)
		
		self.pub_park(self.enb_parking, self.parking_poseBefore, self.parking_poseTarget, self.parking_offset)
		self.rate.sleep()


def main():
	# Start the job threads
	class_1 = ros_control()
	# Keep the main thread running, otherwise signals are ignored.
	while not rospy.is_shutdown():
		class_1.run()

if __name__ == '__main__':
	main()

"""
Stt :
0: chờ đủ dữ liệu để parking
1: chờ tín hiệu parking
21:  tính khoảng cách tiến lùi
-21: thực hiện di chuyen tiến lùi
-210: thực hiện quay trước nếu gặp TH AGV bị lệch góc lớn
31: tính góc quay để lùi vào kệ
-31: thực hiện quay
41: Parking
51: completed - Đợi Reset
52: error: bien doi tf loi

"""
