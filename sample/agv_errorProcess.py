#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    - Gửi và kiểm soát lỗi của AGV
"""

import roslib
roslib.load_manifest('simulate_agvs')
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import *
from simulate_agvs.msg import *
import time

class errorProcess():
    def __init__(self):
        # -- init node -- 
        rospy.init_node('stiClient_fake', anonymous=False)

        # -- node publish -- 
        self.pub_agv1_errorProcess = rospy.Publisher('/agv1/ProcessError_info', agv_infoError, queue_size = 10)
        self.data_agv1_errorProcess = agv_infoError()

        self.pub_agv2_errorProcess = rospy.Publisher('/agv2/ProcessError_info', agv_infoError, queue_size = 10)
        self.data_agv2_errorProcess = agv_infoError()

        self.pub_agv3_errorProcess = rospy.Publisher('/agv3/ProcessError_info', agv_infoError, queue_size = 10)
        self.data_agv3_errorProcess = agv_infoError()
        
        self.rate = rospy.Rate(50.0)

        # -- biến sử dụng -- 
        self.is_first_pub = 0
        self.time_count_pub = time.time()

    def run(self):
        while not rospy.is_shutdown():
            if self.is_first_pub == 0:
                if time.time() - self.time_count_pub < 2:
                    self.data_agv1_errorProcess.agv_delta_voltage = 0.1
                    self.data_agv1_errorProcess.agv_time_decrease_voltage = 2
                    self.data_agv1_errorProcess.agv_waitStop = 0
                    self.data_agv1_errorProcess.agv_time_waitStop = 0
                    self.data_agv1_errorProcess.agv_common_error = 0

                    self.data_agv2_errorProcess.agv_delta_voltage = 0.15
                    self.data_agv2_errorProcess.agv_time_decrease_voltage = 3
                    self.data_agv2_errorProcess.agv_waitStop = 0
                    self.data_agv2_errorProcess.agv_time_waitStop = 0
                    self.data_agv2_errorProcess.agv_common_error = 0

                    self.data_agv3_errorProcess.agv_delta_voltage = 0.2
                    self.data_agv3_errorProcess.agv_time_decrease_voltage = 4
                    self.data_agv3_errorProcess.agv_waitStop = 0
                    self.data_agv3_errorProcess.agv_time_waitStop = 0
                    self.data_agv3_errorProcess.agv_common_error = 0
                else: 
                    self.pub_agv1_errorProcess.publish(self.data_agv1_errorProcess)
                    self.pub_agv2_errorProcess.publish(self.data_agv2_errorProcess)
                    self.pub_agv3_errorProcess.publish(self.data_agv3_errorProcess)
                    self.is_first_pub = 1
            self.rate.sleep()


def main():
	print('Program starting')
	program = errorProcess()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()