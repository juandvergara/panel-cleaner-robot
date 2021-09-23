#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveKobuki
from pid_control import PID


class LineFollower(object):

    def __init__(self):
        rospy.logwarn("Init line Follower")
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/robot/camera1/image_raw", Image, self.camera_callback)
        self.movekobuki_object = MoveKobuki()
        self.pid_object = PID()

    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 20
        crop_img = cv_image[(height)/2+descentre:(height) /
                            2+(descentre+rows_to_watch)][1:width]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV
        # [[[30 196 235]]]

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([50, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        # Draw the centroid in the resultut image
        cv2.circle(res, (int(cx), int(cy)), 10, (0, 0, 255), -1)

        cv2.imshow("RES", res)

        cv2.waitKey(1)

        # Move teh robot , center it in the middle of the witdth 640 -> 320:

        setPoint_value = width/2
        self.pid_object.setpoint_update(value=setPoint_value)

        twist_object = Twist()
        twist_object.linear.x = 0.2

        # Make it start turning
        self.pid_object.state_update(value=cx)
        effort_value = self.pid_object.get_control_effort()
        # We divide the effort to map it to the normal values for angular speed in the turtlebot
        rospy.logwarn("Set Value=="+str(setPoint_value))
        rospy.logwarn("State Value=="+str(cx))
        rospy.logwarn("Effort Value=="+str(effort_value))
        angular_effort_value = effort_value/200.0

        twist_object.angular.z = angular_effort_value
        rospy.logwarn("Twist =="+str(twist_object.angular.z))
        self.movekobuki_object.move_robot(twist_object)

    def clean_up(self):
        self.movekobuki_object.clean_class()
        cv2.destroyAllWindows()


def main(args):
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
    main()
