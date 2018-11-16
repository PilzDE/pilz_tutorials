#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming_v2 import 4
import math
import rospy

__REQUIRED_API_VERSION__ = "0"  #API version
__ROBOT_VELOCITY__ = 1          #velocity of the robot

#main program
def start_program():

    #important positions
    start_pos = [1.49, -0.54, 1.09, 0.05, 0.91,-1.67]   #joint values
    
    pos_pick = Point (0, -0.5, 0.25)            #cartesian coordinates
    pos_work_station = Point(-0.5, 0.1, 0.2)    #cartesian coordinates
    pos_place = Point(-0.1,0.4,0.25)            #cartesian coordinates
    
    #Spherical coordinates
    orientation_pick = from_euler(0, math.radians(180), math.radians(0))
    orientation_work_station = from_euler(0, math.radians(-135), math.radians(90))
    orientation_place = from_euler(0, math.radians(180),  math.radians(90))

    #move to start point with joint values to avoid random trajectory
    r.move(Ptp(goal=start_pos, vel_scale=__ROBOT_VELOCITY__))
    
    while(True):
        #Do infinite loop
        #Pick the PNOZ
        r.move(Ptp(goal=Pose(position=pos_pick, orientation=orientation_pick),
            vel_scale = __ROBOT_VELOCITY__, relative=False))
        pick_and_place()

        #Put the PNOZ in a "machine"
        r.move(Ptp(goal=Pose(position=pos_work_station,
            orientation=orientation_work_station),vel_scale = __ROBOT_VELOCITY__, relative=False))
        pick_and_place()
        rospy.sleep(1)      # Sleeps for 1 sec (wait until work is finished)
        pick_and_place()

        #Place the PNOZ
        r.move(Ptp(goal=Pose(position=pos_place, orientation=orientation_place),
            vel_scale = __ROBOT_VELOCITY__, relative=False))
        pick_and_place()

def pick_and_place():
    """pick and place function"""

    #A static velocity from 0.2 is used
    #The position is given relative to the TCP. 
    r.move(Ptp(goal=Pose(position=Point(0, 0, 0.1)), reference_frame="prbt_tcp", vel_scale=0.2))
    rospy.sleep(0.2)    #Pick or Place the PNOZ (close or open the gripper)
    r.move(Ptp(goal=Pose(position=Point(0, 0, -0.1)), reference_frame="prbt_tcp", vel_scale=0.2))


if __name__ == "__main__":
    # Init a rosnode
    rospy.init_node('robot_program_node')

    #initialisation
    r = Robot(__REQUIRED_API_VERSION__)  #instance of the robot
   
    #start the main program
    start_program()