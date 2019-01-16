#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import *
import math
import rospy

__REQUIRED_API_VERSION__ = "1"  # API version
__ROBOT_VELOCITY__ = 0.5        # velocity of the robot

# main program
def start_program():

    # define the positions:
    joint_goal = [1.49, -0.54, 1.09, 0.05, 0.91,-1.67]  # Use joint values for the first position
    cartesian_goal=Pose(position=Point(0, 0, 0.1), orientation=from_euler(0, math.radians(180), math.radians(0))) # Use cartesian coordinates for another position
    
    # Move to start point with joint values to avoid random trajectory
    r.move(Ptp(goal=joint_goal, vel_scale=__ROBOT_VELOCITY__))

    #Move to the second position
    r.move(Ptp(goal=cartesian_goal, vel_scale=__ROBOT_VELOCITY__))
    
    
if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program()
