#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 01/11/2021

Include: 3 Threads
	1, Main:
	2, Read and respond server
	3, Print to check error

Move:
	status: 
		1 - run
		0 - stop
"""
import socket
import roslib

# get ip
import os                                                                                                                                                           
import re 

import sys
import struct
import time
from decimal import *
import math
import rospy
from datetime import datetime

from sti_msgs.msg import NN_cmdRequest     # request to client udp
from sti_msgs.msg import NN_infoRequest     # request to client udp
from sti_msgs.msg import NN_infoRespond     # respond from client udp

from sti_msgs.msg import FL_infoRespond
from sti_msgs.msg import FL_cmdRespond
from sti_msgs.msg import FL_infoRequest
from sti_msgs.msg import FL_cmdRequest

# import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

#--------------------------------------------------------------------------------- Variable
# --------------------------------------------------------- For UDP
# Symbolic name meaning all available interfaces


# Arbitrary non-privileged port
# number_recieve = 0 # 0: ko nhan dc gi - 1: frame info - 2: frame load - 3: frame info
# pre_self.data_receive ""
# frequency_build_frame = 2  # Liên quan đến tần số lấy dữ liệu từ Node: Modbus + Nav

#----------------------------------------------------------- For thread Read and respond
# mode

# quan ly tat ca gia tri dung cho Luồng xử lý: đọc dữ liệu và phản hồi từ server
# luu tat ca gia tri dung cho Luồng xử lý chính: xử lý tiến trình, gửi dữ liệu đi các Node khác

""" luu trang thai AGV: Nhan tu node Move 
Status: run or stop
Localization : x,y,x
Error: ....
"""

""" luu Yeu cau gui cho AGV: 
Status: run or stop
Localization : x,y, 5 points
Target
Direction
Accury___: ....
"""

#---------------------------------------------------------------------------------------- Class
#----------------------------------------------------------------- Frame

""" Quy định tất cả dữ liệu được gắn trực tiếp vào biến trong class frame phải ở dạng của server:
 	VD: 
  	1, Tọa độ x, y theo mm
		x,y (ros) = 3,12 m -> x(server) = 3.12*1000 = 3120 mm 
 	2, Góc: từ rad -> 360 độ 
		z(ros) = 1.047 (rad) -> z(server) = 59.9 (degree) *100 = 5990 

"""

#------------------------------------------------- Info
class frame_info_rx:
	"""
	NUMBER:  0  |   1   |   2    |    3
	name:	Sum |   U   | Id AGV | name AGV
	SIZE:	 1  |   1   |   4    |   n  
	TYPE:    b  |   s   |   b    |   s  
	"""
	Sum = 0
	Id = 'U'
	Agv_id = 0
	Agv_name = ""

class frame_info_tx:
	"""
	NUMBER:  0  |  1  |    2   |     3    |  4  |  5  |      6    |    7   |     8   |    9   |  10  |  11 
	name:	Sum |  U  | AGV ID | Pos name |  x  |  y  | direction | Offset | Battery | Status | Mode | Lift
	SIZE:	 1  |  1  |    4   |    2     |  4  |  4  |     2     |   2    |    1    |    1   |   1  |  2  
	TYPE:    b  |  s  |    b   |    s     |  b  |  b  |     b     |   o    |    b    |    0   |   b  |  o  
	"""
	Sum = '\x1D' 	# 1
	Id = 'U'		# 1
	Agv_id = 0		# 4
	x = 0.0			# 4
	y = 0.0			# 4
	direction = 0.0	# 4
	Pos_name = 0	# 2
	Battery = 0		# 1
	Status = 0		# 1
	Mode = 0		# 1
	Status = 1		
	Task_status = 0
	Device_error = 0
	Moving_error = 0
	Perform_error = 0

#------------------------------------------------- Command
class frame_command_rx:
	"""
	NUMBER:  1  | 2 |    3    |    4    |     5    |    6     |     7     |    8   |    9   |  10  |  11  |  12  |  13  |  14  |  15  |  16  |  17  |  18  | 19   |    20   |    21   |    22
	name:	Sum | C | Command | Process | Target_x | Target_y | direction |   Tag  | Offset | P1 X | P1 Y | P2 X | P2 Y | P3 X | P3 Y | P4 X | P4 Y | P5 X | P5 Y | PreCode | SufCode | Target name
	SIZE:	 1  | 1 |    4    |    1    |     4    |     4    |     2     |    2   |    2   |  4   |   4  |   4  |   4  |   4  |   4  |   4  |   4  |   4  |   4  |    1    |    1    |      1 + ....
	"""
	Sum = 0 		# 1
	Id = 'C' 		# 1
	Command = 0 	# 4
	Process = 0 	# 1
	Target_name = 0 # 1
	Target_x = 0.0 	# 4
	Target_y = 0.0 	# 4
	Direction = 0.0 # 2
	Offset = 0 		# 2
	Point_name = [0, 0, 0, 0, 0, 0] 		# 1
	Point_x = [0.0, 0.0, 0.0, 0.0, 0.0] 	# 4
	Point_y = [0.0, 0.0, 0.0, 0.0, 0.0] 	# 4
	Point_speed = [0.0, 0.0, 0.0, 0.0, 0.0] # 1
	Pre_code = 0 	# 1
	Suf_code = 0 	# 1
	Message = ''	# n

class frame_command_tx:
	"""
	name:	Sum |   C   |  Command
	SIZE:	 1  |   1   |    4   
	TYPE:    b  |   s   |    s   
	"""
	Sum = '\x06'
	Id = 'C'
	Command = 0

"""----------------------------------------------------------------------------------------------------------------------------------------"""
#------------------------------------------------------------- SUB

""" MODBUS: Nhan tu modbus: thông tin
1, % pin
2, Trạng thái hoat động
3, Cảm biến an toàn
4, ......
"""
# Datagram (udp) socket
class read_and_respond_UDP():
	def __init__(self):
		rospy.init_node('stiClient_nav', anonymous=False)
		self.rate = rospy.Rate(100)

		self.agv_name = rospy.get_param("~agv_name")
		# -- add 22/01/2022
		# self.name_card = rospy.get_param("name_card", "wlp3s0b1")
		self.name_card = "wlp3s0b1"
		
		rospy.Subscriber("/%s/NN_infoRespond" % self.agv_name, NN_infoRespond, self.NN_infoCallback)	
		self.NN_infoRespond = NN_infoRespond()
		self.NN_is_infoReceived = 0 # 0

		self.NN_cmdPub = rospy.Publisher("/%s/NN_cmdRequest" % self.agv_name, NN_cmdRequest, queue_size=50)
		self.NN_cmdRequest = NN_cmdRequest()

		self.NN_infoRequestPub = rospy.Publisher("/%s/NN_infoRequest" % self.agv_name, NN_infoRequest, queue_size=50)
		self.NN_infoRequest = NN_infoRequest()		

		self.NN_cmdRequest.list_id = [0, 0, 0 ,0 , 0]
		self.NN_cmdRequest.list_x = [0.0, 0.0, 0.0, 0.0, 0.0]
		self.NN_cmdRequest.list_y = [0.0, 0.0, 0.0, 0.0, 0.0]
		self.NN_cmdRequest.list_speed = [0.0, 0.0, 0.0, 0.0, 0.0]
		self.before_mission = 0  # nhiem vu can thuc hien truoc khi di chuyen
		self.after_mission = 0  # nhiem vu can thuc hien sau khi di chuyen den dich

		# self.HOST_receive = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen('ip addr show wlp3s0').read()).groups()[0]
		# self.HOST_receive = '192.168.1.69'
		self.HOST_receive = '192.168.1.210'

		# self.HOST_receive = '172.21.84.213'
		# -- add 22/01/2022
		# self.HOST_receive = self.get_ipAuto(self.name_card)
		self.HOST_receive = rospy.get_param("~agv_ip")

		self.PORT_receive = 8888 # 8888
		self.PORT_sento = 8000   # 8000
		self.time_wait_respond = 0.01   # thoi gian cho phan hoi
		self.process = 0
		self.data_raw_info = ''
		self.pre_mess = ""
		self.data_received = ''

		self.frequency_pub_info = 16.
		self.frequency_pub_cmd = 16.
		self.pre_time_info = 0.0
		self.pre_time_cmd = 0.0

		# Header Frame info NN
		self.NN_infoSum = ''
		self.NN_infoId = 'U'
		self.NN_infoNameU300L = ['','','','']

		# Header Frame info FL
		self.FL_infoSum = ''

		#-- Connect to Server UDP
		try :
			self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			print ('Socket created')
		except socket.error: #, msg:
			print ('Failed to create socket. Error Code : ' ) # + str(msg[0]) + ' Message ' + msg[1])
			sys.exit()

		# Bind socket to local host and port
		try:
			self.udp.bind((self.HOST_receive, self.PORT_receive))
			print ('Socket bind: ' + self.HOST_receive + ': ' + str(self.PORT_receive) + ' completed')
		except socket.error: # , msg:
			print ('Bind failed. Error Code : ') # + str(msg[0]) + ' Message ' + msg[1])
			sys.exit()

	def NN_infoCallback(self, dat):
		self.NN_infoRespond = dat
		self.NN_is_infoReceived = 1

 #--------------------------------------------------------------------------------- manipulation with point
	#-------------------------------------------------------------- CONVERT
	#-- list bytes to int (256)
	def bytes_list_to_coor(self, list_byte, pos_byte): # 4 byte
		n = 0
		x = 0
		v = pos_byte + 1
		d = int(list_byte[pos_byte])
		for t in range(4):
			x = int(list_byte[t+v])
			n += x*pow(256, 3 - t - 1)
		
		if d == 1:    # am
			return (n/1000.)*(-1)   # đảm bảo khác 0
		elif d == 2:  # duong:
			return (n/1000.)
		elif d == 0:  # ko xac dinh:
			return 1000
		else:
			print ("d: ", d)
			return 5000

	def bytes_list_to_coor_vs1(self, list_byte, pos_byte): # 4 byte
		n = 0
		x = 0
		v = pos_byte + 1

		for t in range(4):
			x = int(list_byte[pos_byte + t])
			n += x*pow(256, 3 - t)

		val = 0
		if (n > pow(256, 3)):
			val = (n - pow(256, 4))/1000.
		else:
			val  = n/1000.
		return val

	def bytes_list_to_corner(self, list_byte, pos_byte): # 2 byte
		n = 0
		x = 0
		for t in range(0,2):
			x = int(list_byte[pos_byte + t])
			n += x*pow(256, 2 - t - 1)
		if n == 40000:
			return 4
		else:
			c = n/100. # độ
			if c > 180:
				c = 360 - c
				return (c/180)*math.pi*(-1)
			else:
				return (c/180)*math.pi

	def coordinates_to_bytes_vs1(self, co):
		ss = b''
		if (co >= 0):
			ss += self.int_to_byte(0)
		else:
			ss += self.int_to_byte(1)
		x = 0
		t = 0
		co *= 1000
		val = 0
		# value
		val = int(abs(co))
		# print ("val:", val)
		ss += self.int_to_bytes(val, 4)
		return ss

	def coordinates_to_bytes_vs2(self, co):
		ss = b''
		# if (co >= 0):
		ss += self.int_to_byte(0)
		# else:
			# ss += self.int_to_byte(1)

		x = 0
		t = 0
		co *= 1000
		val = 0
		# value
		val = int(co)
		# print ("val:", val)
		ss += self.intA_to_bytes(val, 4)
		return ss

	def coordinates_to_bytes(self, co): # ????????????
		ss = b''
		x = 0
		t = 0
		co *= 1000
		val = 0
		# unknown/-/+
		if co > 500:
			ss += self.int_to_byte(0)
		elif co < 0:
			ss += self.int_to_byte(1)
		else:
			ss += self.int_to_byte(2)
		# value
		val = int(abs(co))
		for i in range(0, 4):
			t += pow(256, 4 - i)*x
			x = (val - t)/pow(256, 3 - i) 
			# print(x)
			ss += self.int_to_byte(int(x))
		return ss
			
	def direction_to_bytes(self, dir): # chuyen toa do tu Rad {-pi,pi} -> {0,360} -> scale: 100 - 2 byte
		if dir >= 4:
			return self.int_to_bytes(40000, 4)
		elif dir >= 0 and dir < 4:
			d = math.degrees(dir)*100
			# print(d)
			return self.int_to_bytes(int(d), 4)
		else:
			d = (360 + math.degrees(dir))*100
			return self.int_to_bytes(int(d), 4)

	def bytes_list_to_int(self, list_byte, pos_byte, n_byte): # int (256)
		n = 0
		x = 0
		for t in range(0, n_byte):
			x = int(list_byte[t + pos_byte])
			n += x*pow(256, n_byte - t - 1)
		return int(n)

	def byte_to_int(self, byt): # byte to int (standard)
		x = 0
		x = int(byt)
		# print ("byt: ", byt)
		# foo.encode('utf-8').strip()
		# x = int(byt.encode('hex'), 16)
		return x

	def bytes_to_int(self, bytes_): # bytes (multiple) to int (256)
		v = 0
		for t in range(0,len(bytes_)):
			x = int(list_byte[t+pos_byte].encode('hex'), 16)
			v =+ x*pow(256, n_byte - t - 1)
		return v

	def int_to_bytes(self, val, n): # int to n bytes
		ss = b''
		x = 0
		t = 0
		for i in range(0, n):
			t += pow(256, n - i)*x
			x = int((val - t)/pow(256, n - i - 1) )
			ss += self.int_to_byte(x)
		return ss

	def intA_to_bytes(self, val, n): # int to n bytes
		ss = b''
		x = 0
		t = 0

		if (val >= 0):
			val_1 = val
		else:
			val_1 = pow(256, 4) + val

		for i in range(0, n):
			t += pow(256, n - i)*x
			x = int((val_1 - t)/pow(256, n - i - 1) )
			ss += self.int_to_byte(x)
		return ss

	def offset_to_bytes(self, val): # int to 2 bytes - val mét
		ss = b''

		x = 0
		t = 0
		val = val*1000 # chuyen sang 'mm'
		for i in range(2):
			t += pow(256, 2 - i)*x
			x = (val - t)/pow(256, 2 - i - 1) 
			# print(x)
			ss += self.int_to_byte(x)
		return ss

	def tag_to_bytes(self, val): # int to 2 bytes - int 0 -> 255
		ss = b''
		x = 0
		t = 0
		val = val
		
		for i in range(2):
			t += pow(256, 2 - i)*x
			x = (val - t)/pow(256, 2 - i - 1) 
			# print(x)
			ss += self.int_to_byte(x)
		return ss

	def bytes_list_to_offset(self, list_byte, pos_byte): # 4 byte
		n = 0
		X = 0
		for t in range(0, 4):
			x = int(list_byte[t + pos_byte])
			n += x*pow(256, 4 - t - 1)
		return n/1000.

	def bytes_list_to_tag(self, list_byte, pos_byte): # 2 byte
		n = 0
		X = 0
		for t in range(0,2):
			x = int(list_byte[t+pos_byte].encode('hex'), 16)
			n += x*pow(256, 2 - t - 1)
		return n

	def int_to_byte(self, val): # int to a bytes
		if val > 255:
			rospy.logerr("int_to_byte: Val error: %s", val)
			val = 255
		elif val < 0:
			rospy.logerr("int_to_byte: Val error: %s", val)
			val = 0
		return struct.pack("B", int(val)) # bytes(chr(int(val)), 'ascii')
	
	def build_stringByte(self, list_byte, pos_byte, n_byte): # list | start position read | number byte  -> return array string
		ss = b''
		for t in range(0, n_byte):
			ss += bytes(chr(list_byte[pos_byte + t]), 'ascii')
		return ss

	def build_string(self, list_byte, pos_byte, n_byte): # list | start position read | number byte  -> return array string
		ss = ''
		for t in range(0, n_byte):
			ss += chr(list_byte[pos_byte + t])
		return ss

	#-- chuyen toa do tu float -> chuoi 4 byte dang '\x' : byte 0: +/- byte {1:3}: gia tri toa do theo mm.
	#----------- Check a point in a Circle ? : point | center circle(point) | radius
	def check_point_in_list(self, po, ls):
		pass

	def find_location(self, name_, list_): # find_location of a point in list
		pass	

	#--------------------------------------------------------------------------------- analysis data receive
	def NN_infoAnalysis(self, data_receive):
		self.NN_infoSum = self.byte_to_int(data_receive[0])
		self.NN_infoNameU300L = self.bytes_list_to_int(data_receive,2,4)

	def NN_cmdAnalysis(self, data_receive):
		# print len(data_receive)
		ll = 0
		ll = len(data_receive)
		# -- add new
		self.NN_cmdRequest.id_command = self.bytes_list_to_int(data_receive, 2, 4)

		self.NN_cmdRequest.process = self.byte_to_int(data_receive[6])

		self.NN_cmdRequest.tag =  self.bytes_list_to_int(data_receive, 7, 2)

		self.NN_cmdRequest.target_x = round(self.bytes_list_to_coor_vs1(data_receive, 9) , 3)
		self.NN_cmdRequest.target_y = round(self.bytes_list_to_coor_vs1(data_receive, 13) , 3)
			
		self.NN_cmdRequest.target_z = round(self.bytes_list_to_corner(data_receive, 17), 3)
		
		# self.NN_cmdRequest.tag = 
		self.NN_cmdRequest.offset = self.bytes_list_to_offset(data_receive, 19) # 4 bytes

		for l in range(5):
			self.NN_cmdRequest.list_id[l] = self.bytes_list_to_int(data_receive, 23 + l*11, 2)
			self.NN_cmdRequest.list_x[l] = round(self.bytes_list_to_coor_vs1(data_receive, 25 + l*11), 3)
			self.NN_cmdRequest.list_y[l] = round(self.bytes_list_to_coor_vs1(data_receive, 29 + l*11), 3)
			self.NN_cmdRequest.list_speed[l] = int(data_receive[33 + l*11])

		self.NN_cmdRequest.before_mission = self.byte_to_int(data_receive[78])
		self.NN_cmdRequest.after_mission = self.byte_to_int(data_receive[79])

		if ll > 80:
			self.NN_cmdRequest.command = self.build_string(data_receive, 80, ll - 80)
		else:
			self.NN_cmdRequest.command = ""
	
	def NN_cmdBuild(self, data_receive):
		data_raw = ''
		data_raw = '\x06' + 'R' + data_receive[2] + data_receive[4] + data_receive[4] + data_receive[5]
		return data_raw 	

	def convert_error(self, x):
		switcher={
		  # NN
			0:0,   # ALL RIGHT
			111:1,  # Va vào Blsock.
			121:2, # Ấn EMG.
			131:3, # Ra khỏi đường từ.
			141:4, # Bàn nâng.
			211:5, # Camera:Mất kết nối vật lý.
			212:6, # Camera: Không giao tiếp truyền thông.
			221:7, # Lidar. Phía trước.
			222:8, # Lidar. Phía sau.
			223:9, # Lidar. Cả 2.
			231:10, # IMU Không giao tiếp truyền thông.
			241:11, # PS2 Không giao tiếp truyền thông.
			251:12, # DRIVER 1 (trái) NN – Natual Navigatiroson.
			261:13, # DRIVER 2 (phải) NN – Natual Navigation.
			271:14, # Mangnetic line: NN - RS232 – PC lỗi (phía trước).
			272:15, # Mangnetic line: NN - RS232 – PC lỗi (phía sau).
			311:16, # Mạch MC - NN.	Mất kết nối vật lý Serial.
			312:17, # Mạch MC - NN. Không giao tiếp truyền thông Serial.
			321:18, # Mạch Main - NN:	Mất kết nối vật lý Serial.
			322:19, # Mạch Main - NN: Không giao tiếp truyền thông Serial.
			331:20, # Mạch SC: Mất kết nối vật lý Serial.
			332:21, # Mạch SC: Không giao tiếp truyền thông Serial.		
			341:22,	# Mạch OC: Mất kết nối vật lý Serial.
			342:23,	# Mạch OC: Không giao tiếp truyền thông Serial.
			351:24,	# Mạch HC: Mất kết nối vật lý Serial.
			352:25,	# Mạch HC: Không giao tiếp truyền thông Serial.
			411:201, # Di chuyển NN (không vạch từ)- Không thể đến được đích.
			421:202, # Có vật cản.
			431:203, # Mất kết nối server.
			441:204, # Không thể thấy Tag.
			451:205, # Điện Áp Thấp.
			461:206, # Có vật cản khi vào kệ.
			471:207  # không có kệ hoặc lệch kệ khi nâng.
		}
		return switcher.get(x, 0)		

	def NN_infoBuild(self):
		data_raw = b''

		data_raw += self.coordinates_to_bytes_vs2(self.NN_infoRespond.x)
		# data_raw += self.coordinates_to_bytes_vs2(123) 
		data_raw += self.coordinates_to_bytes_vs2(self.NN_infoRespond.y)
		# data_raw += self.coordinates_to_bytes_vs2(-70.0)

		data_raw += self.direction_to_bytes(self.NN_infoRespond.z) 
		# data_raw += self.direction_to_bytes(-2.141)

		data_raw += self.int_to_bytes(self.NN_infoRespond.tag, 2)
		# print ("tag hex: ", self.int_to_bytes(self.NN_infoRespond.tag, 2) )
		
		data_raw += self.int_to_byte(self.NN_infoRespond.battery)
		# data_raw += self.int_to_byte(254)
		
		data_raw += self.int_to_byte(self.NN_infoRespond.status)

		data_raw += self.int_to_byte(self.NN_infoRespond.mode)
		data_raw += self.int_to_byte(0)
		data_raw += self.int_to_byte(self.NN_infoRespond.task_status)
		return data_raw

	def log_mess(self, typ, mess, val):
		if self.pre_mess != mess:
			if typ == "info":
				rospy.loginfo (mess + ": %s", val)
			elif typ == "warn":
				rospy.logwarn (mess + ": %s", val)
			else:
				rospy.logerr (mess + ": %s", val)
		self.pre_mess = mess

	# -- add 22/01/2022
	# def get_ipAuto(self, name_card): # name_card : str()
	# 	try:
	# 		address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
	# 		print ("address: ", address)
	# 		return address
	# 	except Exception:
	# 		return "-1"

 #--------------------------------------------------------------------------------- Read and respond
	def run(self):	
		
		if self.process == 0:
			self.process = 1

		elif self.process == 1: # recivce data raw
			try:			
				self.data_received, addr = self.udp.recvfrom(1024)
				self.HOST_sento = addr[0]
				# print ("rec: ",self.data_received)
				# rospy.loginfo ("lenght frame: %s - Frame: %s", self.byte_to_int(self.data_received[0]), self.data_received[1])
			except socket.error:
				self.log_mess("err", "Error -- recv() --!", 0)
			# print len(self.data_received)
			if len(self.data_received) != 0:
				# pass
				self.process = 2
			else:
				self.process = 1
				self.log_mess("warn", "not data receive", 0)

		elif self.process == 2: # kiem tra kich thuoc cua data
			if self.byte_to_int(self.data_received[0]) != len(self.data_received):
				rospy.logerr ("Error lenght frame: %s - want: %s - current: %s", self.data_received[1], self.byte_to_int(self.data_received[0]), len(self.data_received))
			else:
				self.process = 3

		elif self.process == 3: # nhan dien frame

			if (self.byte_to_int(self.data_received[1]) == 85): # info NN 'U'
				self.process = 4
			elif self.data_received[1] == 67: # command NN 'C'
				self.process = 5
			else:
				rospy.logwarn("Recivce dif frame")
				rospy.logwarn(self.data_received[1])
				self.process = 1

		elif self.process == 4:  # info NN - analysis and respond
			try:
				if self.NN_is_infoReceived == 1: # 1:
					data_raw_info = self.NN_infoBuild()
					mm = b''

					mm += self.int_to_byte(27) + self.int_to_byte(self.data_received[1]) + self.build_stringByte(self.data_received, 2, 4) + data_raw_info 
					# print ("len(data_raw_info): ", len(data_raw_info))
					# abc = []
					# for i in range(len(mm)):
						# abc.append(self.byte_to_int(mm[i]))
					# print ("sent:", abc)

					self.udp.sendto(mm, (self.HOST_sento, self.PORT_sento))
				else:
					rospy.logwarn("Wait respond from Sti_control")

			except socket.error:
				rospy.logerr ("NN: Can not send Info!")

			# pub info request by Server, nhận được thông tin từ server và hiển thị ra 
			self.NN_infoRequest.id_agv = self.bytes_list_to_int(self.data_received, 2, 4)

			lenght = len(self.data_received)
			if (lenght > 6):
				self.NN_infoRequest.name_agv = self.build_string(self.data_received, 6, lenght - 6 )
			else:
				self.NN_infoRequest.name_agv = ''

			self.NN_infoRequestPub.publish(self.NN_infoRequest)
			self.process = 1

		elif self.process == 5:  # command NN - analysis and respond
			rr = self.int_to_byte(6) + self.int_to_byte(self.data_received[1]) + self.int_to_byte(self.data_received[2] + self.data_received[3]) + self.int_to_byte(self.data_received[4]) + self.int_to_byte(self.data_received[5])
			time.sleep(self.time_wait_respond)
			try:
				self.udp.sendto(rr, (self.HOST_sento, self.PORT_sento))
			except socket.error:
				rospy.logerr ("NN: Can not send Command!")

			if (len(self.data_received) >= 80):
				self.NN_cmdAnalysis(self.data_received)
				self.NN_cmdPub.publish(self.NN_cmdRequest)
				
			else:
				rospy.logerr("NN: Error size of Frame command: %s", len(self.data_received))
			self.process = 1

		self.rate.sleep()

def main():

	print('Starting main program')
    # Start the job threads
	class_1 = read_and_respond_UDP()		
	# Keep the main thread running, otherwise signals are ignored.
	while not rospy.is_shutdown():
		class_1.run()
	class_1.udp.close()

if __name__ == '__main__':
    main()
