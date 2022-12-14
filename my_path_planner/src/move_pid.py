
import math
from turtle import distance
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist

cmd_vel = Twist()

kp_dist = 1
kd_dist = 0.5

route = []
nextwp = Point()
wp = 1
yaw_precision = math.pi / 30 
dist_precision = 0.1

def move(speed_y, speed_z):
    cmd_vel.linear.y = speed_y
    cmd_vel.angular.z = speed_z

    pub.publish(cmd_vel)
    

def get_odom(msg):
    global x
    global y
    global theta

    x = round(msg.pose.pose.position.x ,3)
    y = round(msg.pose.pose.position.y ,3)

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def get_distance(goal):
    ## get distance from current pos to goal pos 
    distance = math.sqrt(pow(goal.y - y , 2) + pow(goal.x - x , 2))
    return distance

def forward(goal):
    wp_reached = True
    ## go forward to goal pos
    prev_error = 0
    while(wp_reached):
        if(get_distance(goal) > dist_precision ):
            error = get_distance(goal)
            diff_error = error - prev_error
            adjust_speed = kp_dist * error + kd_dist * diff_error
            speed_x = min(adjust_speed,0.1)
            move(speed_x, 0)
            prev_error = error
            wp_reached = True
        else:
            move(0 , 0)
            wp_reached = False
        


def fix_heading(goal_angle):
    heading_fixed = True
    ## fix heading to goal angle pos
    while(heading_fixed):
        if(goal_angle - theta <= yaw_precision):
            move(0, 0.2)
            heading_fixed = False
        elif(goal_angle - theta >= yaw_precision):
            move(0, -0.2)
            heading_fixed = False
        else :
            move(0, 0)
            heading_fixed = True
        
def get_angle(goal):
    ## get angle from current pos to goal pos
    goal_angle = math.atan2(goal.y - y , goal.x - x)
    return goal_angle
    



def main():
    global pub
    rospy.init_node('movement control with PID')
    rospy.Subscriber("/run_slam/camera_pose",Odometry,get_odom)
    pub = rospy.Publisher("/tello/cmd_vel", Twist,queue_size=10)
    rospy.Rate(20)
    print("mission started")

    route = [(1,1),(2,2)]
    while(route.count(1)>0):
        nextwp = route.pop(0)
        if(get_angle(nextwp) >= yaw_precision or get_angle(nextwp) <= yaw_precision):
            fix_heading(get_angle(nextwp))
        elif(get_distance(nextwp)> dist_precision):
            forward(nextwp)
        else:
            move(0,0)
            print("an error may appear")
        
        rospy.loginfo("waypoint reached")
    
    rospy.loginfo("all waypoint has reached")
    rospy.loginfo("mission finished")
    


    

if __name__ == '__main__':
 
    try:
        main()
    except KeyboardInterrupt:
        rospy.logerr("exit")
    finally :
        move(0,0)
        rospy.logerr("end")

