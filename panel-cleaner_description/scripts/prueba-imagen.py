#!/usr/bin/env python
from __future__ import print_function

from numpy.lib.type_check import imag
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
import rospy
import sys
from geometry_msgs.msg import Twist
#from pid_control import PID

import roslib

roslib.load_manifest('panel-cleaner_description')


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/out_image_raw", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/robot/camera1/image_raw", Image, self.callback)
        # self.pid_object = PID()

        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape

        #descentrate = 120
        #offset = 220
        #crop_img = cv_image[int(rows/2)+descentrate:rows -
                            # offset, int(cols/4):int(cols*3/4)]

        cv_image_g = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(cv_image_g, 127, 255, 0)
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        image_contours = cv2.drawContours(
            cv_image_g, contours, -1, (0, 255, 0), 2)

        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
          # calculate x,y coordinate of center
            try:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            except ZeroDivisionError:
                cY, cX = int(rows/2), int(cols/2)

            cv2.circle(cv_image, (cX, cY), 5, (255, 0, 255), -1)
            # cv2.putText(crop_img, "centroid", (cX - 25, cY - 25),
            # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        #setPoint_value = cols/2

        twist_object = Twist()
        twist_object.linear.x = 0.2
        twist_object.angular.z = 0

        # self.publisher.publish(Twist)

        cv2.imshow("Image window", image_contours)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(
                image_contours, "passthrough"))
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
