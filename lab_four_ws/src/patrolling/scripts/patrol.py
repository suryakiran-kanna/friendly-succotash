#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Twist, Vector3, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path
import roslaunch
from random import randint
import time
import math

# Spawn points to choose from
spawn_x=[-7.57,-7.75,-2.33,-1.71,-1.47,3.76,6.45,1.57,-3.39,6.67]
spawn_y=[-4.27,4.82,-1.19,3.69,7.73,4.1,-0.19,-3.89,-5.03,-8.65]

# Predefined waypoints
NumGoals = 7
goal_x=[-4.3806, 6.515, 3.8655, 6.515, -4.1195, -7.9609, 5.9347]
goal_y=[-1.2384, 6.6707, -2.8412, 6.6707, 5.2126,-4.1242, -8.4489]

# Rotation Parameters
rotTime = 100
rotLin = Vector3(x=0, y=0, z=0)
rotAng = Vector3(x=0, y=0, z=1)
rotMsg = Twist(linear=rotLin, angular=rotAng)

# Path Variables
navPathX = 0
navPathY = 0
navPathLength = 0
matchMulti = 10

# Callback for a path messages
def path_callback(myPath):
    global navPathX, navPathY, navPathLength
    numPoints = len(myPath.poses)
    pIndex = 0
    length = 0
    
    if(numPoints > 1):
        while(pIndex < (numPoints-1)):
            dx = myPath.poses[pIndex + 1].pose.position.x - myPath.poses[pIndex].pose.position.x
            dy = myPath.poses[pIndex + 1].pose.position.y - myPath.poses[pIndex].pose.position.y
            length = length + math.sqrt(math.pow(dx,2) + math.pow(dy,2))
            pIndex = pIndex + 1
            
        navPathX = myPath.poses[numPoints-1].pose.position.x
        navPathY = myPath.poses[numPoints-1].pose.position.y
        navPathLength = length

# Move a new goal
def moveToGoal(xGoal,yGoal,tLimit):
    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
        time.sleep(1) 

    goal = MoveBaseGoal()

    # set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal
    goal.target_pose.pose.position = Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)
    
    ac.wait_for_result(rospy.Duration(tLimit))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True

    else:
        ac.cancel_goal() # Remove old goal that wasnt reached
        rospy.loginfo("The robot failed to reach the destination")
        return False
        
    
# Rotate in place based on rotate parameters
def rotate():
    tIndex = 0
    pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    while(tIndex < rotTime):
        pub.publish(rotMsg)
        time.sleep(0.1) 
        tIndex = tIndex + 1
    
# Move loop
def move():
    index = 0
    
    while(index < NumGoals):
        moveToGoal(goal_x[index],goal_y[index], 30)
        rotate()
        index = index + 1

# Spawn a new Jackel at a random location
def spawn():
    # Setup the URDF spawn node and launch it with random spawn point
    package ='gazebo_ros'
    executable ='spawn_model'
    x = randint(0,9)
    node = roslaunch.core.Node(package, executable, args="-urdf -model jackal -param robot_description -x "+str(spawn_x[x])+" -y "+str(spawn_y[x])+" -z 1.0")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    
    # Setup and publish an initail pose to AMCL to coincide with the random spawn
    initialPose = PoseWithCovarianceStamped()
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    # Only set up for random x,y. Does not handle random spawn rotation
    initialPose.header.frame_id = "map"
    initialPose.pose.pose.position = Point(spawn_x[x],spawn_y[x],0)
    initialPose.pose.pose.orientation = Quaternion(0,0,0,1)
    initialPose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    
    # Send the message several times to be sure AMCL got the point
    index = 0
    while(index < 10):
        initialPose.header.stamp = rospy.Time.now()
        pub.publish(initialPose)
        time.sleep(0.5) 
        index = index + 1
    
    return x 

# Find the closest waypoint via path
def findClosest():
    index = 0
    shortestIndex = 0;
    shortestDistance = 0;
    
    # Cycle through waypoints to see which is closest
    while(index < NumGoals):
        moveToGoal(goal_x[index],goal_y[index],0.01)
        
        # varaibles for checking if path is valid
        px = int(navPathX * matchMulti)
        py = int(navPathY * matchMulti)
        gx = int(goal_x[index] * matchMulti)
        gy = int(goal_y[index] * matchMulti)
        
        rospy.loginfo("X: [%d ? %d]   Y: [%d ? %d]", px, gx, py, gy)
        
        # Check if path is valid, must match within matchMulti-ith of a unit
        # I.E. matchMulti = 10, path end and waypoint must be within 1/10th of a unit
        if (px == gx) and (py == gy):
            # Handle first cycle case
            if(index == 0):
                shortestDistance = navPathLength
                
            # Track shortest path
            elif(navPathLength < shortestDistance):
                shortestIndex = index
                shortestDistance = navPathLength
            rospy.loginfo("Distance to [%2.3f, %2.3f] is %2.3f", navPathX, navPathY, navPathLength)
            index = index + 1
            
        # The proposed path was not a match for the waypoint, sleep and try again
        else:
            rospy.loginfo("Failed to find valid path, trying again")
            time.sleep(1)
    
    rospy.loginfo("Closest waypoint is [%2.3f, %2.3f]", goal_x[shortestIndex], goal_y[shortestIndex])
    
    return shortestIndex
        
# Initialize the patrol, setup callback for path messages, spawn the robot randomly
def init():
    rospy.init_node('map_navigation', anonymous=False)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, path_callback) 
    spawn()

# Main. Initialize, spawn, plan path, execute path, and quit
if __name__ == '__main__':
    try:
        time.sleep(10)
        init()
        startIndex = findClosest()
        #move()
        time.sleep(20)   
        #rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
        
        
        
        
        
