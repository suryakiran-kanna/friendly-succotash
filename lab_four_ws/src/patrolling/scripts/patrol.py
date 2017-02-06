#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import roslaunch
from random import randint
import time

spawn_x=[-7.57,-7.75,-2.33,-1.71,-1.47,3.76,6.45,1.57,-3.39,6.67]
spawn_y=[-4.27,4.82,-1.19,3.69,7.73,4.1,-0.19,-3.89,-5.03,-8.65]

def moveToGoal(xGoal,yGoal):

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")


    goal = MoveBaseGoal()

    #set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal*/

    goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)
    
    ac.wait_for_result(rospy.Duration(10))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True

    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False

def move():
   
    one_x = 8.17984202602
    one_y = -8.49176794236
    two_x = 1.79868924906
    two_y = -4.23748139042
    three_x = -4.60369452677
    three_y = -1.10880111438
  
    moveToGoal(two_x,two_y)
    moveToGoal(one_x,one_y)
    moveToGoal(three_x,three_y)

def spawn():
    
#<!-- Spawn Jackal -->
    package ='gazebo_ros'
    executable ='spawn_model'

    x = randint(0,9)
   
    node = roslaunch.core.Node(package, executable, args="-urdf -model jackal -param robot_description -x "+str(spawn_x[x])+" -y "+str(spawn_y[x])+" -z 1.0")
    
	
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    #while process.is_alive():
      #  print process.is_alive()
    
    return x 


if __name__ == '__main__':
    try:
        rospy.init_node('map_navigation', anonymous=False)
	spawn()
        #move()
	time.sleep(20)   
	#rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
