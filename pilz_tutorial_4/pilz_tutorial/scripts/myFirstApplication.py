#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import *
import math
import rospy

__REQUIRED_API_VERSION__ = "1"    # API version
__ROBOT_VELOCITY__ = 0.5          # velocity of the robot

# main program
def start_program():

    rospy.loginfo("Program started") # log

    # important positions
    start_pos = [1.49, -0.54, 1.09, 0.05, 0.91,-1.67]   # joint values

    pick_pose = Pose(position=Point (0, -0.5, 0.25), orientation=from_euler(0, math.radians(180), math.radians(0))) # cartesian coordinates
    work_station_pose = Pose(position=Point(-0.5, 0.1, 0.2) , orientation=from_euler(0, math.radians(-135), math.radians(90)))  # cartesian coordinates
    place_pose = Pose(position=Point(-0.1,0.4,0.25) ,from_euler(0, math.radians(180),  math.radians(90))) # cartesian coordinates
    
  
    # move to start point with joint values to avoid random trajectory
    r.move(Ptp(goal=start_pos, vel_scale=__ROBOT_VELOCITY__))

    rospy.loginfo("Start loop") # log
    while(True):
        # do infinite loop

        # pick the PNOZ
        rospy.loginfo("Move to pick position") # log
        r.move(Ptp(goal=pick_pose, vel_scale = __ROBOT_VELOCITY__, relative=False))
        rospy.loginfo("Pick movement") # log
        pick_and_place()

        # put the PNOZ in a "machine"
        rospy.loginfo("Move to virtual machine") # log
        r.move(Ptp(goal=work_station_pose,vel_scale = __ROBOT_VELOCITY__, relative=False))
        rospy.loginfo("Place PNOZ in machine") # log
        pick_and_place()
        rospy.sleep(1)      # Sleeps for 1 sec (wait until work is finished)
        rospy.loginfo("Pick PNOZ from machine") # log
        pick_and_place()

        # place the PNOZ
        rospy.loginfo("Move to place position") # log
        r.move(Ptp(goal=place_pose, vel_scale = __ROBOT_VELOCITY__, relative=False))
        rospy.loginfo("Place movement") # log
        pick_and_place()

def pick_and_place():
    """pick and place function"""

    # a static velocity of 0.2 is used
    # the position is given relative to the TCP.
    r.move(Lin(goal=Pose(position=Point(0, 0, 0.1)), reference_frame="prbt_tcp", vel_scale=0.2))
    rospy.loginfo("Open/Close the gripper") # log
    rospy.sleep(0.2)    # pick or Place the PNOZ (close or open the gripper)
    r.move(Lin(goal=Pose(position=Point(0, 0, -0.1)), reference_frame="prbt_tcp", vel_scale=0.2))


if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program()
