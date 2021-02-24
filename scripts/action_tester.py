#!/usr/bin/env python3
""" This script publishes ROS messages containing the 3D coordinates of a single point """
import rospy
from q_learning_project.msg import RobotMoveDBToBlock


class Test_Sender:



    def __init__(self):
        rospy.init_node('send_robot_action')    # initialize our ROS node
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size = 10)
    def run(self):
        # rospy.Rate specifies the rate of the loop (in this case 2 Hz)
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            action = RobotMoveDBToBlock()
            action.robot_db = input("Please input a dumbbell color (red, green, blue): ")
            action.block_id = int(input("Please input a block number (1, 2, 3): "))
            self.action_pub.publish(action)
            print ("Done!")
            r.sleep()

if (__name__ == '__main__'):
    ROS = Test_Sender()
    ROS.run();
