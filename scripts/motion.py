#!/usr/bin/env python3

import rospy
import math

import moveit_commander
import moveit_msgs.msg

from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Image, LaserScans
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from q_learning_project.msg import RobotMoveDBToBlock

from object_recognizer import ObjectRecognizer

from tf.transformations import quaternion_from_euler, euler_from_quaternion

# terminal steps for ease of copy pasting

# roscore
# roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch
# roslaunch turtlebot3_manipulation_moveit_config move_group.launch * remember to press play!
# rosrun q_learning_project motion.py
# roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

global db_order
global block_order

global db_pos
global block_pos

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def displacement(x1, y1, x2, y2):
    d = math.sqrt(math.pow((x2-x1),2) + math.pow((y2-y1),2))
    return d

class Motion(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('motion')

        # arm controls
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # The lidar subscriber - delete later?
        # self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.scan_recieved)

        # The odometry subscriber
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.update_odometry)

        self.action_sub = rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.get_action)

        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


        # set the arm into into default position
        default_pos =[0,.2, .8, -.95]
        self.move_group_arm.go(default_pos, wait=True)
        self.move_group_arm.stop()

        # get the location of given db and block
        #self.location = ObjectRecognizer()
        #self.location.run()

        # wait to
        #while (not self.location.finished):
        #    pass



        # command the bot to perform the given move db to block action
        self.db_to_b()

    def get_action(self):
        db_dex = 0
        b_index = 0

        for x in range(2):
            if (self.action.robot_db == db_order[x]):
                db_index = x
                break

        for y in range(2):
            if (self.action.block_id == block_order[y]):
                b_index = y
                break

        self.db_pose = db_pos[db_index]
        self.b_pose = block_pos[b_index]

        print("Location of goal dumbbell:" + self.db_pose)
        print("Location of goal block:" + self.b_pose)

    def update_odometry(self, data):
        self.odom_pose = data.pose.pose

    # entire moving dumbbell to block command
    def db_to_b(self):

        db_r = self.db_pose[0]
        db_theta = self.db_pose[1]

        self.move_to(db_r, db_theta)
        self.pickup()

        b_r  = self.b_pose[0]
        b_theta = self.b_pose[0]

        self.move_to(b_r, b_theta)
        self.drop()

    # orient self in front of dumbbell
    def move_to(r, theta):

        #move dist meters in the current direction
        dist = r - 0.05 #how close you get to dumbbell, may have to change this for block?

        self.turn_to(theta)

        #move dist meters in the current direction

        #starting pose before the loop
        start_x = self.odom_pose.pose.position.x
        start_y = self.odom_pose.pose.position.y

        print("starting location (" + start_x + ", "+ start_y + ")")

        #move forward until robot travels dist meters
        while ((displacement(start_x, start_y,
            self.odom_pose.pose.position.x, self.odom_pose.pose.position.y)) < dist):
            self.velocity_pub.publish(Vector3(.1, 0, 0), Vector3(0, 0, 0))
         # stop the robot
        self.velocity_pub.publish(Vector3(0, 0, 0), Vector3(0, 0, 0))

    # helper function to turn towards the target object

    def turn_to(self, theta):
        # Note: Only turns CCW for now!
        print ("Turning to yaw: ", theta)
        try:
            current_yaw = get_yaw_from_pose(self.odom_pose)
        except AttributeError:
            print ("Careful! No odom pose exists yet!")
            current_yaw = 0
        while (abs(current_yaw - theta) > 0.05):
            try:
                current_yaw = get_yaw_from_pose(self.odom_pose)
            except AttributeError:
                current_yaw = 0
            # Let's just turn in one direction to account for discontinuity.
            # Using sqrt error to make it turn a little faster at small errors.
            error = math.sqrt(abs(current_yaw - theta))
            self.pub.publish(Vector3(0, 0, 0), Vector3(0, 0, 0.2 * error + 0.05))
        self.pub.publish(Vector3(0, 0, 0), Vector3(0, 0, 0))
        return

    def pickup(self):

        print("Picking up")

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
        print("Holding")

    def drop(self):

        print("Putting down")

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

        print("Dropped")

    def run(self):
        rospy.spin()


if __name__=="__main__":

    node1 = ObjectRecognizer()
    node1.run()

    db_order = node1.db_order
    block_order = node1.block_order

    db_pos = node1.db_pos
    block_pos = node1.block_pos

    print("Objects identified and located!")

    node1.shutdown()

    node = Motion()
    node.run()
