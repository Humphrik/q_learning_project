#!/usr/bin/env python3

import rospy

import moveit_commander
import moveit_msgs.msg

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock

#from object_recognizer import ObjectRecognizer

from tf.transformations import quaternion_from_euler, euler_from_quaternion

#STEPS for ease of copy pasting

# roscore
# roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch
# roslaunch turtlebot3_manipulation_moveit_config move_group.launch * remember to press play!
# rosrun q_learning_project motion.py
# roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


class Motion(object):


    def __init__(self):

        # initialize this node
        rospy.init_node('motion')

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")


        default_pos =[0,.2, .8, -.95]

        self.move_group_arm.go(default_pos, wait=True)
        self.move_group_arm.stop()

        rospy.sleep(0.8)

        self.pickup()

        rospy.sleep(0.8)

        self.drop()

# actions:

    # def moveto(self, destination):
    ##orient self in front of dumbbell
    #
    def pickup(self):

        grab_pos = [0,.2,.6,-.7]
        lifted_pos = [0,-.10,.6,-.7]
        default_pos =[0,.2, .8, -.95]

        open_grip = [0.004, 0.004]
        closed_grip = [-.005, -0.005]

        self.move_group_gripper.go(open_grip, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(0.8)

        self.move_group_arm.go(grab_pos, wait=True)
        self.move_group_arm.stop()

        rospy.sleep(0.8)

        self.move_group_gripper.go(closed_grip, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(0.8)

        self.move_group_arm.go(lifted_pos, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(2)

    def drop(self):

        grab_pos = [0,.2,.6,-.7]
        lifted_pos = [0,-.10,.6,-.7]
        default_pos =[0,.2, .8, -.95]

        open_grip = [0.009, 0.009]
        closed_grip = [-.005, -0.005]

        self.move_group_arm.go(grab_pos, wait=True)
        self.move_group_arm.stop()

        rospy.sleep(0.8)

        self.move_group_gripper.go(open_grip, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(0.8)

        self.move_group_arm.go(default_pos, wait=True)
        self.move_group_arm.stop()

        rospy.sleep(0.8)



    def run(self):
        rospy.spin()


if __name__=="__main__":

    node = Motion()
    node.run()
