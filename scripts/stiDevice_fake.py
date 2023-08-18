#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    - Giả tín hiệu lỗi và gửi tới AGV:
    - Có 3 hạng mục lỗi:
        + Lỗi pin yếu: Pin giảm dần theo thời gian, nếu nhỏ hơn mức cho phép thì không di chuyển nữa
        + Lỗi dừng tránh, di chuyển hết điểm: Sau 1 thời gian lỗi sẽ tự hết
        + Lỗi khác: EMC, va chạm: AGV không di chuyển, cần người thao tác đến xử lý
"""

import roslib
roslib.load_manifest('agvtraffic_simulation')
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import *
from agvtraffic_simulation.msg import *
import time

import random

class errorProcess():
    def __init__(self):
        # -- init node -- 
        rospy.init_node('stiDevice_fake', anonymous=False)

        self.agv_name = rospy.get_param('~agv_name')
        # -- node publish -- 
        # self.pub_errTempStop = rospy.Publisher('/%s/status_goal_control' % self.agv_name, Status_goal_control, queue_size = 10)
        # self.data_errTempStop = Status_goal_control()
        # self.data_errTempStop.complete_misson = 2
        
        self.pub_errVolt = rospy.Publisher('/%s/POWER_info' % self.agv_name, POWER_info, queue_size = 10)
        self.data_errVolt = POWER_info()

        self.rate = rospy.Rate(50.0)
        # -- global variables -- 
        # for error of Stopping temporary
        # self.list_TempStop = []
        # self.max_TempStop = 60
        # for i in range(1, self.max_TempStop):
        #     self.list_TempStop.append(i)

        # self.time_TempStop = rospy.get_param('time_TempStop', 4)
        # self.count_time_TempStop = time.time()
        # self.count_timeOfRD_TempStop = time.time()
        # self.flag_TempStop = 0
        
        # for errr of low voltage
        self.max_voltage = rospy.get_param('max_voltage', 25.5)
        self.min_voltage = rospy.get_param('min_voltage', 23.0)
        self.time_downVolt = rospy.get_param('~time_downVolt', 10)
        self.count_time_downVolt = time.time()
        self.delta_voltage = rospy.get_param('~delta_voltage', 0.01)
        self.data_errVolt.voltages = self.max_voltage

    def run(self):
        while not rospy.is_shutdown():
            # if self.flag_TempStop == 0:
            #     if time.time() - self.count_timeOfRD_TempStop >= 1:
            #         self.data_errTempStop.misson = random.choice(self.list_TempStop)
            #         self.count_timeOfRD_TempStop = time.time()

            #     if self.data_errTempStop.misson == 1 or self.data_errTempStop.misson == 3:
            #         self.flag_TempStop = 1
            #         self.count_time_TempStop = time.time()
            # else:
            #     if time.time() - self.count_time_TempStop > self.time_TempStop:
            #         self.data_errTempStop.misson = 0
            #         self.flag_TempStop = 0
            #         self.count_timeOfRD_TempStop = time.time()
            
            if (time.time() - self.count_time_downVolt)*1000 >= self.time_downVolt:
                self.data_errVolt.voltages = self.data_errVolt.voltages - self.delta_voltage
                self.count_time_downVolt = time.time()

                if self.data_errVolt.voltages < self.min_voltage:
                    self.data_errVolt.voltages = self.min_voltage - 0.2

            # self.pub_errTempStop.publish(self.data_errTempStop)
            self.pub_errVolt.publish(self.data_errVolt)
            self.rate.sleep()

def main():
	print('Program starting')
	program = errorProcess()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()