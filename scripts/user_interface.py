#! /usr/bin/env python

import rospy
import time
from rt2_assignment1.srv import Command
import actionlib
import rt2_assignment1.msg
from geometry_msgs.msg import Twist

## The main function.
#
#  This node allows the user to interact with the robot.
#
#
# @var ui_client defines a service client of user_interface type.
# It taskes as argument the Command service to activate/deactivate robot 
# @var client defines the action client  of the go_to_point Action. 
# As argument it takes the action of type GoalReaching 
# @var pub It defines the publsiher of the cmd_vel toic.
# Used to stop the robot
# @var x it stores the input value inserted by the user

def main():
    # Initialising the user_interface node
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        # If entered value is equal to 1 
        if (x == 1):
            # a request is sent to the go_to_point server to activate the robot
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))            
            
        else:
            # the ui_client ' argument is settled to stp in order to deactivate the behaviour 
            ui_client("stop")
            # asking for another input
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()


