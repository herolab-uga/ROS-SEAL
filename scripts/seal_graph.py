#!/usr/bin/env python3
# license removed for brevity
__author__ = 'Ehsan Latif'
__version__ = '1.0'


import math
from turtle import circle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from matplotlib import pyplot as pb
import random
from datetime import datetime
import time
import g2o 
import os

import argparse
from nav_msgs.msg import Odometry


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
#from robot_msgs.msg import Robot_Pos, StringList
#from robot_msgs.srv import GetCharger, GetChargerResponse, ReleaseCharger, ReleaseChargerResponse 

areaSize=(10, 10)
No_Of_Robots = 3

total_overall_rss=[]
total_original_tragectory=[]

colors = ['b','r','c','y','k','m', 'g', 'b','r','c','b','g','c','y','k','m', 'g', 'b','r','c']

def dist(pos1, pos2):
    return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)

initial_poses = {"tb3_0":(0.5,1.7),"tb3_1":(0.5,2.5),"tb3_2":(0.5,3.3)}
colors = {"tb3_0":'b',"tb3_1":'r',"tb3_2":'c'} #,'y','k','m', 'g', 'b','r','c','b','g','c','y','k','m', 'g', 'b','r','c']


# axis[0].set_title("Trajectory",fontsize=24)
# # manager = plt.get_current_fig_manager()
# # manager.full_screen_toggle()

# axis[0].set_ylim(0,areaSize[1])
# axis[0].set_xlim(0,areaSize[0])

# axis[0].set_xlabel('X-axis', fontsize=22)
# axis[0].set_ylabel('Y-axis', fontsize=22) 

# axis[0].tick_params(axis='x', labelsize=20)
# axis[0].tick_params(axis='y', labelsize=20)
graph = {}
step = 0
figure, axis = plt.subplots(1, 2)

axis[0].cla()
axis[0].set_title("Trajectory",fontsize=24)
# manager = plt.get_current_fig_manager()
# manager.full_screen_toggle()

axis[0].set_ylim(-10,10)
axis[0].set_xlim(-10,10)

axis[0].set_xlabel('X-axis', fontsize=18)
axis[0].set_ylabel('Y-axis', fontsize=18) 

axis[0].tick_params(axis='x', labelsize=16)
axis[0].tick_params(axis='y', labelsize=16)
axis[0].set_xticks(np.arange(-10,10, 1))
axis[0].set_yticks(np.arange(-10,10, 1))

axis[1].cla()
axis[1].set_title("Graph",fontsize=24)
plt.axis("equal")

axis[1].set_xlabel('X-axis', fontsize=18)
axis[1].set_ylabel('Y-axis', fontsize=18) 

axis[1].tick_params(axis='x', labelsize=16)
axis[1].tick_params(axis='y', labelsize=16)
axis[1].set_ylim(-11,11)
axis[1].set_xlim(-10,10)
axis[1].set_xticks(np.arange(-11,11, 1))
axis[1].set_yticks(np.arange(-10,10, 1))

mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())

now = datetime.now()

robots = ["tb3_0","tb3_1","tb3_2"]

prev_time = now.strftime("%H:%M:%S")
def position_callback(odom_positions,positions):
    # print(positions)
    global graph
    global step
    global total_original_tragectory
    global figure
    global axis
    global prev_time


    circles = []
    texts = axis[1].plot()
    lines =  []
    
    current_trajectory = {}
    for robot_i in robots:
        
        # robot_poses[robot_i] = positions[robot_i]
        robot_i_frame = robot_i
        # print("robot:"+ robot_i_frame)
        # print(robot_i_frame)
        robot_i_position = positions[robot_i]
        current_trajectory[robot_i+'_pos']=robot_i_position
        current_trajectory[robot_i+'_odom']=odom_positions[robot_i]

        graph[robot_i_frame]={}
        for robot_j in robots:
            robot_j_frame = robot_j
            robot_j_position = positions[robot_j]
            if robot_j_frame!=robot_i_frame:    
                robot_distance = dist(robot_i_position,robot_j_position)
                if robot_distance < 5:
                    graph [robot_i_frame][robot_j_frame] = robot_distance
                circles.append(axis[1].add_artist(plt.Circle((robot_i_position[0],robot_i_position[1] ), radius=0.2, color=colors[robot_i_frame] )))
                texts.append(axis[1].text(robot_i_position[0]-0.02,robot_i_position[1], str(robot_i_frame.split('_')[-1]), fontsize = 10, color='w'))
                lines.append(axis[1].plot([robot_i_position[0],robot_j_position[0]],[robot_i_position[1],robot_j_position[1]],colors[robot_i_frame]+'-',linewidth=2,clip_on=False))
        # print(step)s
        if step > 0:
            axis[0].plot([total_original_tragectory[step-1][robot_i+'_odom'][0],current_trajectory[robot_i+'_odom'][0]],[total_original_tragectory[step-1][robot_i+'_odom'][1],current_trajectory[robot_i+'_odom'][1]],colors[robot_i_frame]+'-',linewidth=2,clip_on=False)
            # axis[0].plot([total_original_tragectory[step-1][robot_i+'_pos'][0],current_trajectory[robot_i+'_pos'][0]],[total_original_tragectory[step-1][robot_i+'_pos'][1],current_trajectory[robot_i+'_pos'][1]],colors[robot_i_frame]+'--',linewidth=2,clip_on=False)

        # else:
        #     axis[0].plot([total_original_tragectory[ste][robot_i_frame].x,total_original_tragectory[step][robot_i_frame].x],[total_original_tragectory[step-1][robot_i_frame].y,total_original_tragectory[step][robot_i_frame].y],colors[int(robot_i_frame)-3]+'-',linewidth=2,clip_on=False)

    total_original_tragectory.append(current_trajectory)
    step+=1


    plt.draw()
    plt.pause(0.000001)
    for line in lines:
        line.pop(0).remove()
    for circle in circles:
        circle.remove()
    for text in texts:
        text.remove()



# plt.show(block=False)
# plt.pause(3)
# plt.savefig('Dist_graph_optm_localization.png')
# plt.clf()
# plt.close()

def active_robots_callback(robots):
    global No_Of_Robots
    No_Of_Robots = len(robots.data)

positions = {"tb3_0":(0,0),"tb3_1":(0,0),"tb3_2":(0,0)}
odom_positions = {"tb3_0":(0,0),"tb3_1":(0,0),"tb3_2":(0,0)}

def tb3_0_odom_callback(msg):
    global positions
    global odom_positions
    odom_x,odom_y= msg.pose.pose.position.x,msg.pose.pose.position.y
    odom_positions["tb3_0"]=(odom_x,odom_y)
    pos_x,pos_y=odom_x+random.uniform(0.01,0.20),odom_y+random.uniform(0.01,0.20)
    positions["tb3_0"] = (pos_x,pos_y)
    if odom_positions["tb3_1"]!=(0,0) and odom_positions["tb3_2"]!=(0,0):
        position_callback(odom_positions,positions)

def tb3_1_odom_callback(msg):
    global positions
    global odom_positions
    odom_x,odom_y= msg.pose.pose.position.x,msg.pose.pose.position.y
    odom_positions["tb3_1"]=(odom_x,odom_y)
    pos_x,pos_y=odom_x+random.uniform(0.01,0.20),odom_y+random.uniform(0.01,0.20)
    positions["tb3_1"] = (pos_x,pos_y)
    if odom_positions["tb3_0"]!=(0,0) and odom_positions["tb3_2"]!=(0,0):
        position_callback(odom_positions,positions)


def tb3_2_odom_callback(msg):
    global positions
    global odom_positions
    odom_x,odom_y= msg.pose.pose.position.x,msg.pose.pose.position.y
    odom_positions["tb3_2"]=(odom_x,odom_y)
    pos_x,pos_y=odom_x+random.uniform(0.01,0.20),odom_y+random.uniform(0.01,0.20)
    positions["tb3_2"] = (pos_x,pos_y)
    if odom_positions["tb3_1"]!=(0,0) and odom_positions["tb3_0"]!=(0,0):
        position_callback(odom_positions,positions)





if __name__ == '__main__':
    try:
        rospy.init_node('graph_node', anonymous=True)
        # interfacename = rospy.get_param('~INTERFACE_NAME', 'wlan0')
        # update_rate = rospy.get_param('~update_rate_wireless_quality', 10)	
        pub_position = rospy.Publisher('position', PoseStamped, queue_size=10)
        rate = rospy.Rate(1)

        # rospy.Subscriber("/positions", Robot_Pos, position_callback,queue_size=1)
        # rospy.Subscriber("/active_robots", StringList, active_robots_callback,queue_size=1)
        rospy.Subscriber('/tb3_0/odom',Odometry,tb3_0_odom_callback,queue_size=1)
        rospy.Subscriber('/tb3_1/odom',Odometry,tb3_1_odom_callback,queue_size=1)
        rospy.Subscriber('/tb3_2/odom',Odometry,tb3_2_odom_callback,queue_size=1)



        plt.ion()

        plt.show(block=True)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
