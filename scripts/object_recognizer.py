#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy, math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3, Point, Pose

import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion


import matplotlib.pyplot as plt

# Credit to https://pypi.org/project/keras-ocr/
# pip install keras-ocr
# pip install tensorflow
import keras_ocr


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw




class ObjectRecognizer(object):

    def __init__(self):

        self.initalized = False

        rospy.init_node('object_recognizer')

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)
        # The odometry subscriber.
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.update_odometry)
        # The lidar sub, being used as a glorified timer.
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.laser_scan)


        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        self.seen_dumbbell = False
        self.seen_block = False
        

        # The order of the objects from left to right (relative to the robot facing them.)
        self.db_order = ["na", "na", "na"]
        self.block_order = [0, 0, 0]


        # The actual positons:
        self.db_pos = [(0,0)] * 3
        self.block_pos = [(0,0)] * 3


        # Has everything been found?
        self.finished = False

        self.initalized = True


    def update_odometry(self, data):
        self.odom_pose = data.pose.pose
        
    def image_callback(self, data):
        if (not self.initalized):
            return
        self.img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')


    def detect_dumbbell(self, data):
        # take the ROS message with the image and turn it into a format cv2 can use
        #self.img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # Define the upper and lower bounds for each dumbbell color. (R, G, and B).
        colors = []
        colors.append((numpy.array([0, 10, 10]), numpy.array([10, 255, 255])))
        colors.append((numpy.array([55, 10, 10]), numpy.array([65, 255, 255])))
        colors.append((numpy.array([110, 10, 10]), numpy.array([130, 255, 255])))
             
        h, w, _ = self.img.shape
        
        db_count = 0
        db_locations = []
        
        for i in range(0, 3):

            name = ""
            if (i == 0):
                name = "red"
            elif (i == 1):
                name = "green"
            else:
                name = "blue"

            # using moments() function, the center of the yellow pixels is determined
            low, high = colors[i]
            mask = cv2.inRange(hsv, low, high)
            # Remove incorrect pixels from the mask.
            #search_top = int(3*h/4)
            #search_bot = int(3*h/4 + 20)
            #mask[0:search_top, 0:w] = 0
            #mask[search_bot:h, 0:w] = 0
            
            M = cv2.moments(mask)
            if M['m00'] > 0:
                db_count += 1
                # center of the pixels.
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # a black circle is visualized in the debugging window to indicate
                # the center point of this dumbell's pixels.
                cv2.circle(self.img, (cx, cy), 20, (0,0,0), -1)   
                db_locations.append((cx, cy, name)) 

        # We didn't find all 3 dumbbells
        if (db_count != 3):
            return

        self.seen_dumbbell = True
        # Sort the colors from left to right (cx val).
        db_locations.sort(key = lambda x: x[0])

        # Print the order of the colors from left to right.
        print ("The dumbbells from left to right are: ")
        for i in range(0, 3):
            x, y, name = db_locations[i]
            self.db_order[i] = name
        print (self.db_order)

        # Visualize the location of the dumbbells.
        #cv2.imshow('img', self.img)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()


    def detect_block(self, data):
        # Start sweeping.

        # The right-most block.
        self.turn_to (3*math.pi/4)
        cv2.imwrite("rblock.png", self.img)
        # The middle block.
        self.turn_to (math.pi)
        cv2.imwrite("mblock.png", self.img)
        # The left-most block.
        self.turn_to (-3*math.pi/4)
        cv2.imwrite("lblock.png", self.img)

        # Officially passing off the the keras_ocr model.

        pipeline = keras_ocr.pipeline.Pipeline()

        # Get a set of three example images
        images = [
            keras_ocr.tools.read(url) for url in [
                "lblock.png",
                "mblock.png",
                "rblock.png"
            ]
        ]

        # Each list of predictions in prediction_groups is a list of
        # (word, box) tuples.
        prediction_groups = pipeline.recognize(images)
        # print (prediction_groups)
        rospy.sleep(1)
        for i in range(0, 3):
            # Get the word result from each image.
           
            wordarr, _  = prediction_groups[i]
            word, _ = wordarr
            # Nobody's perfect.
            if (word == 'l'):
                word = '1'
            print (word)
            self.block_order[i] = int(word)

        # Return to start.
        self.turn_to (0)
        print ("Block detection done!")
        self.seen_block = True
        return



    # Turn to the given yaw value.
    def turn_to (self, theta):
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


    def begin_processing(self, data):

        # Find the dumbbells.
        if (not self.seen_dumbbell):
            print ("Looking for dumbbells...")
            self.turn_to(0)
            self.detect_dumbbell(data)

        # Now find the blocks.
        # The robot will need to spin to do a full sweep of the blocks.

        if (not self.seen_block):
            print ("Looking for blocks...")
            self.detect_block(data)

        print ("-------------------------------------")
        print ("Processing complete!")
        print ("Final db order: ", self.db_order)
        print ("Final block order: ", self.block_order)



    def get_locations(self, data):
        """ The goal of this method is to sweep through the lidar scan, which should (hopefully)
        Find 6 clusters of objects, representing the 3 dumbells and blocks.
        Starting from 90 degrees, we find lblock, mblock, rblock, db1, db2, db3 in that order."""

        # Is the lidar finding an object?
        scanning = False
        # When did the robot start finding an object?
        scan_val = 90
        # How many objects have we found so far?
        counter = 0
        # Save the ranges
        distances = []

        for i in range (90, 450):
            j = i%360
            curr_range = data.ranges[j]
            if (curr_range <= data.range_max):
                # Case 1: Still scanning object...
                if (scanning):
                    continue
                # Case 2: Start scanning object...
                else:
                    scanning = True
                    scan_val = i
            else:
                # Case 3: No more object...
                if (scanning):
                    #Note that i is used, not j.
                    index = ((i+scan_val)/2)%360
                    distances.append ((index, data.ranges[int(index)]))
                    scanning = False
                    counter += 1
                # Case 4: Still no object...
                else:
                    continue


        for i in range(0, 3):
            self.block_pos[i] = distances[i]
        for i in range(3, 6):
            self.db_pos[i-3] = distances[i]
        
        # The final positions
        print ("Block positions: ", self.block_pos)
        print ("Dumbell position: ", self.db_pos)




        return


    def laser_scan(self, data):
        if (self.finished or not self.initalized):
            return
        self.begin_processing(data)
        self.get_locations(data)
        self.finished = True
        





    def run(self):
        rospy.spin()


    def shutdown(self):
        rospy.signal_shutdown("Processing complete!")


if __name__ == '__main__':
    node = ObjectRecognizer()
    node.run()
