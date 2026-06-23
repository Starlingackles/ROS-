#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import math
import time
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry

def calc_distance(p_1, p_2):
    return math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)

def calculate_angle(p_1, p_2, p_3):        
    '''
    p_1 is the point it is coming from 

    p_2 is the point it is currently at

    p_3 is the point it is going to
    '''
    
    angle_1 = math.atan((p_2[1]-p_1[1])/(p_2[0]-p_1[0]))
    angle_2 = math.atan((p_3[1]-p_2[1])/(p_3[0]-p_2[0]))

    # return (angle_1 - angle_2)*180/math.pi
    return angle_2 - angle_1

def quat_2_euler(ox, oy, oz, ow):
    t3 = +2.0 * (ow * oz + ox * oy)
    t4 = +1.0 - 2.0 * (oy * oy + oz * oz)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def get_sign(current, goal, dir):
    
    vect1 = np.array([goal[0] - current[0], goal[1] - current[1]])/ np.linalg.norm([goal[0] - current[0], goal[1] - current[1]])
    vect2 = np.array(dir)/np.linalg.norm(dir)

    if round(np.dot(vect1, vect2), 2) < 0:
            return -1
    else:
        return 1


    


class controller():

    def __init__(self, path) -> None:
        
        self.path = path
        self.max_speed = 1
        self.translating = False
        self.rotating = False
        self.num_of_points = len(path)

        self.vec = 0

        self.prev_orient = 0
        self.current_pos = (0,0)
        self.current_orient = 0
        self.trans_speed = 0.1
        self.rot_speed = 1
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.get_pos)

        self.kp_lin = 0.05
        self.ki_lin = 0
        self.kd_lin = 0.0

        self.kp_ang = 0.01
        self.ki_ang = 0
        self.kd_ang = 0

        self.error = 0
        self.error_last = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.output = 0

    def get_pos(self, data):
        self.current_pos = (data.pose.pose.position.x, data.pose.pose.position.y)
        ox, oy, oz, ow = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
        self.current_orient = quat_2_euler(ox, oy, oz, ow)*180/math.pi

    def mover(self):
        print('number of nodes: ', len(self.path))
        #To move from (0,0) to first point         
        #Traversing rest of the path
        reached = False
        point_idx = 0
        
        self.rotate = True
        self.translate = False
        self.correction_protocol = False
        
        

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        move_cmd = Twist()
        
        rate = rospy.Rate(100)

        while not reached:

            # ...todo

            # send control instructions to robot
            pub.publish(move_cmd)
        


def callback(data):
    path_nodes = []
    published_data = data.data
    
    for i in range(len(published_data)):
        if i%2 == 0:
            path_nodes.append((round(published_data[i], 3), round(published_data[i+1], 3)))
        else:
            continue
    print(path_nodes)
    control = controller(path_nodes)
    control.mover()

def listener():
    rospy.init_node('controller', anonymous=True)
    data = rospy.wait_for_message('path', Float32MultiArray, timeout=None)
    callback(data)

    rospy.spin()
    
if __name__ == '__main__':
    listener()
