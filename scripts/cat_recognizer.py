#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3



class CatRecognizer(object):

    def __init__(self):

        self.initalized = False

        rospy.init_node('cat_recognizer')

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.robot_scan_recieved)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # load the opencv2 XML classifier for cat faces
        # obtained from https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalcatface_extended.xml
        self.catface_cascade = cv2.CascadeClassifier('catface_detector.xml') 

        self.seen_first_image = False

        self.angle_error = 0.5
        self.dist_error = 1 
        self.initalized = True


    def image_callback(self, data):

        if (not self.initalized):
            return

        if (not self.seen_first_image):

            # we have now seen the first image
            if (self.angle_error < 0.05):
                self.seen_first_image = True

            # take the ROS message with the image and turn it into a format cv2 can use
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # turn the image into a grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # using the XML classifier, we now detect cat faces in the image
            cat_faces = self.catface_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
            height, width, _ = img.shape
            

            for (x,y,w,h) in cat_faces:
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                self.angle_error = (width/2 - (x+w/2))/width


            # visualize the cat face location in the image
            cv2.imshow('img',img)
            cv2.waitKey(3)
            #cv2.destroyAllWindows()


    def robot_scan_recieved(self, data):
        print (self.angle_error)
        orientation_val = self.angle_error
        forward_val = 0
        if (orientation_val < 0.05):
            dist = data.ranges[0]
            print (dist)
            if (dist > data.range_max):
                dist = 0.5
            self.dist_error = (dist - 0.5)
            forward_val = 0.15*self.dist_error
        self.pub.publish(Vector3(forward_val,0,0), Vector3(0,0,orientation_val))
                


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = CatRecognizer()
    node.run()
