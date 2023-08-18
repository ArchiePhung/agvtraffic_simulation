#!/usr/bin/env python3

from tkinter import *
import time
import math

import roslib
roslib.load_manifest('simulate_agvs')
import rospy

from geometry_msgs.msg import *
from simulate_agvs.msg import *
from sti_msgs.msg import NN_cmdRequest

class Enviroment():
    def __init__(self, background_color_, width_, height_) :
        self.root = Tk()
        self.root.title("Giao diện mô phỏng AGV")
        self.root.resizable(0,0)

        self.can = Canvas(self.root, background = background_color_, width = width_, height= height_)
        self.can.pack()

        self.CANVAS_MID_X = width_/2
        self.CANVAS_MID_Y = height_/2
        self.SIDE = width_/4

class AGV():
    def __init__(self, color) :
        self.enviroment1 = Enviroment("#00BFFF", 500, 500)
        self.canvas = self.enviroment1.can
        
        # -- specify parameters of AGV -- 
        self.agv_dimension_1 = 0
        self.agv_dimension_2 = 20

        self.agv_circle_shape =[                                    #items is circle with R = 10
            [self.agv_dimension_1, self.agv_dimension_1],
            [self.agv_dimension_2, self.agv_dimension_2],
        ]
        self.agv_radius = (self.agv_dimension_2 - self.agv_dimension_1) / 2

        # -- specify the first position of AGV on display screen -- 
        self.agv_start_pose_x = self.enviroment1.CANVAS_MID_X - self.agv_radius
        self.agv_start_pose_y = self.enviroment1.CANVAS_MID_Y - self.agv_radius
        self.agv_start_angle = 0

        self.id = self.canvas.create_oval(self.agv_circle_shape, fill = color)

        self.canvas.move(self.id, self.agv_start_pose_x,  self.agv_start_pose_y)
        
        # -- ros init --
        rospy.init_node('tkinter_agv')
        
        # -- rosnode subcribe --
        # rospy.Subscriber('/turtle1/pose',turtlesim.msg.Pose, self.handle_turtle_pose)
        rospy.Subscriber('/agv_vel',Twist, self.cal_odom)
        rospy.Subscriber('/NN_cmdRequest', NN_cmdRequest, self.handle_NN_cmdRequest)
        # self.agv_vel = Twist()
        self.msg_traffic = NN_cmdRequest()

        # -- rosnode publish --
        self.agv_pub_info_feedback = rospy.Publisher('/agv_infoRespond', agv_infoRespond, queue_size = 10)
        self.agv_info_feedback = agv_infoRespond()

        self.rate = rospy.Rate(60.0)

        # -- global variable -- 
        self.time_now = time.time()
        self.target_time_agv_move = 1.0 
        self.agv_pre_pose_x = self.agv_start_pose_x
        self.agv_pre_pose_y = self.agv_start_pose_y
        self.agv_pre_angle = self.agv_start_angle
        self.move_distance_x = 0
        self.move_distance_y = 0
        self.ready_to_run = 0                       # trạng thái di chuyển của agv 
        self.agv_target_pose_x = 0
        self.agv_target_pose_y = 0
        self.agv_target_index = 0 
        
        self.agv_pos = self.canvas.coords(self.id)
        print(self.agv_pos)

    def handle_NN_cmdRequest(self, data):
        self.msg_traffic = data
        self.agv_target_pose_x = self.msg_traffic.list_x
        self.agv_target_pose_y = self.msg_traffic.list_y

        self.move_distance_x = self.agv_target_pose_x[self.agv_target_index] - self.agv_pre_pose_x
        self.move_distance_y = self.agv_target_pose_y[self.agv_target_index] - self.agv_pre_pose_y
        self.ready_to_run = 1
        print("AGV đã bắt đầu chạy rồi nha!")

    def cal_odom(self, data):
        self.agv_vel = data
        self.agv_angle = self.agv_pre_angle + self.agv_vel.angular.z * self.target_time_agv_move
        # print(self.agv_angle)

        self.agv_pose_x = self.agv_pre_pose_x + self.agv_vel.linear.x * math.cos(self.agv_angle) * self.target_time_agv_move
        # print(self.agv_pose_x)

        self.agv_pose_y = self.agv_pre_pose_y + self.agv_vel.linear.x * math.sin(self.agv_angle) * self.target_time_agv_move
        # print(self.agv_pose_y)

    def draw(self):
        self.canvas.move(self.id, 0, -1)
    
    def run(self):
        while not rospy.is_shutdown():
            if self.ready_to_run == 1:
                # self.agv_pos = self.canvas.coords(self.id)
                # print(self.agv_pos)  
                
                if self.move_distance_x > 0 and self.move_distance_y == 0:          #agv di chuyển ngang phai 
                    self.canvas.move(self.id, 1, 0)
                    # self.enviroment1.root.update_idletasks()
                    # self.enviroment1.root.update()
                    time.sleep(0.01)
                    self.move_distance_x -= 1
                    if self.move_distance_x < 0:
                        self.move_distance_x = 0
                    # print(self.move_distance_x, self.move_distance_y)
                    
                elif self.move_distance_x < 0 and self.move_distance_y == 0:       # agv di chuyển ngang sang trai
                    self.canvas.move(self.id, -1, 0)
                    time.sleep(0.01)
                    self.move_distance_x += 1
                    if self.move_distance_x > 0:
                        self.move_distance_x = 0
                    # print(self.move_distance_x, self.move_distance_y)

                elif self.move_distance_x == 0 and self.move_distance_y > 0:       # agv di chuyển doc xuống dưới 
                    self.canvas.move(self.id, 0, 1)
                    time.sleep(0.01)
                    self.move_distance_y -= 1
                    if self.move_distance_y < 0:
                        self.move_distance_y = 0
                    # print(self.move_distance_x, self.move_distance_y)

                elif self.move_distance_x == 0 and self.move_distance_y < 0:      # agv di chuyển dọc lên trên
                    self.canvas.move(self.id, 0, -1)
                    time.sleep(0.01)
                    self.move_distance_y += 1
                    if self.move_distance_y > 0:
                        self.move_distance_y = 0
                    # print(self.move_distance_x, self.move_distance_y)

                elif self.move_distance_x == 0 and self.move_distance_y == 0:
                    print("AGV đã chạy tới điểm thứ ", self.agv_target_index + 1)
                    self.agv_pre_pose_x = self.agv_target_pose_x[self.agv_target_index]
                    self.agv_pre_pose_y = self.agv_target_pose_y[self.agv_target_index]

                    self.agv_target_index = self.agv_target_index + 1
                    if self.agv_target_index > 4:
                        self.ready_to_run = 0
                        self.agv_target_index = 0
                        print("AGV đã dừng lại")

                    time.sleep(3)
                    self.move_distance_x = self.agv_target_pose_x[self.agv_target_index] - self.agv_pre_pose_x
                    self.move_distance_y = self.agv_target_pose_y[self.agv_target_index] - self.agv_pre_pose_y

            self.enviroment1.root.update_idletasks()
            self.enviroment1.root.update()

            self.agv_pos = self.canvas.coords(self.id)              # hàm trả về tọa độ hiện tại của AGV
            self.agv_info_feedback.x = self.agv_pos[0]
            self.agv_info_feedback.y = self.agv_pos[1]

            self.agv_pub_info_feedback.publish(self.agv_info_feedback)
            # self.agv_info_feedback.x = self.canvas.coords(self.id)
            self.rate.sleep()
            # self.enviroment1.root.mainloop()

class Ball: 
    def __init__(self, canvas, color):
        self.canvas = canvas
        self.id = canvas.create_oval(10, 10, 25, 25, fill = color)
        self.canvas.move(self.id, 100, 200)
    def draw(self):
        self.canvas.move(self.id, 0, -1)

def main():
    # tk = Tk()
    # tk.title("a")
    # tk.resizable(0, 0)
    # can = Canvas(tk, width=400, height=400)
    # can.pack()

    # bong = Ball(can, "#FF4500")
    # while 1:
    #     bong.draw()
    #     tk.update_idletasks()
    #     tk.update()
    #     time.sleep(0.01)
    print('Programmer started!')
    agv1 = AGV("#FF4500")
    agv1.run()
    print('Programmer stopped!!')

if __name__ == '__main__':
    main()
