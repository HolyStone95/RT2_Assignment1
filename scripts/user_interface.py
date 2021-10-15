#! /usr/bin/env python


"""
.. module:: user_interface
  :platform: Unix
  :synopsis: Python module used by the user to interact with the robot
.. moduleauthor:: Iacopo Pietrasanta iacopo.pietrsanta@gmail.com

This node allows the user to interact with the robot.

"""

import rospy
import time
from rt2_assignment1.srv import Command
import actionlib
import rt2_assignment1.msg
from geometry_msgs.msg import Twist


def main():
    """
    This function initializes the needed variables and then wait to see if
      the user want to start using the robot.
    Defines a service client of ``user_interface`` type
      it taskes as argument the Command service to activate/deactivate robot 
    
    Args:None
      
    Returns:None
    """
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


