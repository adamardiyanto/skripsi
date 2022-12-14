#!/usr/bin/env python

import math
import rospy
import numpy as np
import time
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose
from JPS import *
#from Astar import *

cmd_vel = Twist()

x= 0.0
y= 0.0

precision = 0.1

kp_x = 0.5
kd_x = 0.05

kp_y = 0.5
kd_y = 0.05

grid = [[0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0]]

def get_odom(msg):
    global pos_msg
    global x,y
    x = round(msg.pose.pose.position.x, 3)
    y = round(msg.pose.pose.position.y, 3)

    rotation = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])

def waypoint_reached(next):
    error_x = math.fabs(round(next[0] - x, 2))
    error_y = math.fabs(round(next[1] - y, 2))

    if error_x < precision and error_y < precision:
        reached = True
    else:
        reached = False
    
    return reached

def move(speedX = 0, speedY = 0):
    cmd_vel.linear.y = speedX
    cmd_vel.linear.x = -speedY

    pub.publish(cmd_vel)


def diagonal(next):
    err_prev_x = 0
    err_prev_y = 0
    while not waypoint_reached(next):
        err_x = next[0] - x
        err_y = next[1] - y
        err_diff_x = err_x - err_prev_x
        err_diff_y = err_y - err_prev_y

        adj_vel_x = kp_x * err_x + kd_x * err_diff_x
        adj_vel_y = kp_x * err_y + kd_y * err_diff_y

        if(adj_vel_x >= 0):
            vel_x = np.clip(adj_vel_x, 0.15, 0.4)
        else:
            vel_x = np.clip(adj_vel_x,-0.4, -0.15)

        if(adj_vel_y >= 0):
            vel_y = np.clip(adj_vel_y, 0.15, 0.4)
        else:
            vel_y = np.clip(adj_vel_y,-0.4, -0.15)

        # vel_x = min(adj_vel_x, 0.2)
        # vel_y = min(adj_vel_y, 0.2)

        move(vel_x, vel_y)
        r.sleep()
    move() 


def horizontal(next):
    err_prev = 0
    while not waypoint_reached(next):
        if math.fabs(next[0] - x) > precision:
            error = next[0] - x
            err_diff = error - err_prev
            adj_vel = kp_x * error + kd_x * err_diff
            if(adj_vel >= 0):
                vel = np.clip(adj_vel, 0.15, 0.4)
            else:
                vel = np.clip(adj_vel,-0.4, -0.15)
            #vel = min(adj_vel, 0.2)
            move(vel, 0)
        elif math.fabs(next[1] - y) > precision :
            error = next[1] - y
            err_diff = error - err_prev
            adj_vel = kp_y * error + kd_y * err_diff
            if(adj_vel >= 0):
                vel = np.clip(adj_vel, 0.15, 0.4)
            else:
                vel = np.clip(adj_vel,-0.4, -0.15)
            #vel = min(adj_vel, 0.2)
            move(0, vel)
        else:
            break
        
        r.sleep()
    move()


def main():
    rospy.init_node('moving_drone')
    rospy.Subscriber("/run_slam/camera_pose", Odometry, get_odom)
    global pub 
    pub = rospy.Publisher("/tello/cmd_vel", Twist, queue_size = 00)
    global r    
    r = rospy.Rate(4)
    start = tuple([0,0])
    goal = tuple([4,0])
    route = [(0.5,0.0)]
    ### JPS
    (jps_path, jps_time) = search_jps(grid, start, goal) 
    for i in range(len(jps_path)):
        wp_x, wp_y = jps_path[i]
        wp_x = wp_x/4.0
        wp_y = wp_y/4.0
        jps_path[i] = (wp_x,wp_y)
    #route = jps_path
    start_time_flight = time.time()
    print("mission started")
    print("route = {}".format(route))

    ### Astar
    # (A_path, A_time) = search(grid, start, goal) 

    # for i in range(len(A_path)):
    #     wp_x, wp_y = A_path[i]
    #     wp_x = wp_x/4.0
    #     wp_y = wp_y/4.0
    #     jps_path[i] = (wp_x,wp_y)
    # route = A_path
    
    nextWP = route.pop(0)
    print("next waypoint = {}".format(nextWP))
    while not rospy.is_shutdown():
        if (math.fabs(nextWP[0] - x) > precision 
            and 
            math.fabs(nextWP[1] - y) > precision ):
            # jalan diagonal
            diagonal(nextWP)
        else:
            #jalan horizontal
            horizontal(nextWP)

        if waypoint_reached(nextWP):
            print("waypoint reached")
            print("x = {}, y = {}".format(x, y))
            if len(route) != 0:
                nextWP = route.pop(0)
                print("next waypoint = {}".format(nextWP))
            else :
                endtime_flight = time.time()
                print("finished")
                move()
                flight_time = round(endtime_flight - start_time_flight, 4)
                print("searching time = ".format(jps_time)) #jps

                #print("searching time = ".format(A_time)) #Astar

                print("flight time = ".format(flight_time))

                break
        r.sleep()
  
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        move()
    finally:
        move()
        rospy.logerr("end")
        
