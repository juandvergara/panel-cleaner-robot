#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import message_filters
from numpy.core.fromnumeric import shape
from numpy.lib.polynomial import poly
from numpy.lib.type_check import imag
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rospy.core import loginfo
from sensor_msgs.msg import Image, Range, Imu
from pid_control import PID


class robot_cleaner(object):

    def __init__(self):
        rospy.logwarn("Init line Follower")

        self.cx_n1 = 400.0
        self.cx_n2 = 400.0
        self.cx_n3 = 400.0

        self.bridge_object = CvBridge()
        self.image_sub = message_filters.Subscriber(
            "/robot/camera1/image_raw", Image)
        self.movement_cleaner = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=1)
        self.pid_object = PID()

        self.sonar_1_sub = message_filters.Subscriber(
            "/sensor/sonar1_scan", Range)
        self.sonar_2_sub = message_filters.Subscriber(
            "/sensor/sonar2_scan", Range)
        self.imu_sub = message_filters.Subscriber("/imu", Imu)

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.sonar_1_sub, self.sonar_2_sub, self.imu_sub], queue_size=3, slop=0.1)
        ts.registerCallback(self.cleaner_callback)

        self.condition = -1
        self.counter = 0
        self.to_stop = 0

    def cleaner_callback(self, camera, sonar1, sonar2, imu):

        # rospy.loginfo("Data sonar1: "+str(sonar1.range))
        # rospy.loginfo("Data sonar2: "+str(sonar2.range))
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(camera, "bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape

        image_g = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        image_g = cv2.GaussianBlur(image_g, (5, 5), 0)
        roi_image = self.region_of_interest(image_g)
        bw = cv2.adaptiveThreshold(roi_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY, 15, -2)

        vertical = self.find_vertical_lines(bw)

        centers, cv_image = self.find_contours(bw, cv_image)

        if centers:
            # Encontrar todos los puntos que encierren los centros
            hull = cv2.convexHull(np.array(centers))

            poly_result = np.zeros_like(image_g)

            result = cv2.fillPoly(poly_result, [hull], 255)

            m = cv2.moments(result, False)
            try:
                cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            except ZeroDivisionError:
                cy, cx = height/2, width/2

            cv_image = cv2.circle(
                cv_image, (int(cx), int(cy)), 10, (0, 0, 255), -1)
        else:
            cx = width/2

        # cv2.imshow("Vertical", vertical)
        cv2.imshow("RES2", cv_image)
        cv2.waitKey(1)

        # Move teh robot , center it in the middle of the witdth 800 -> 400:
        setPoint_value = width/2
        self.pid_object.setpoint_update(value=setPoint_value)

        # Make it start turning
        self.pid_object.state_update(value=cx)
        effort_value = self.pid_object.get_control_effort()

        # We divide the effort to map it to the normal values for angular speed in the turtlebot
        rospy.logwarn("Set Value=="+str(setPoint_value))
        rospy.logwarn("State Value=="+str(cx))
        rospy.logwarn("Effort Value=="+str(effort_value))
        angular_effort_value = effort_value/20000.0
        rospy.logwarn("Effort Value Real=="+str(angular_effort_value))

        twist_object = Twist()

        if ((sonar1.range < 0.3 and sonar2.range < 0.3) and ((0.45 < imu.orientation.x < 0.55) or ((-0.45) > imu.orientation.x > (-0.55)))):
            twist_object.linear.x = 0.1
            twist_object.angular.z = angular_effort_value
            rospy.logwarn("Twist =="+str(twist_object.angular.z))
            self.counter = 0
        else:
            self.counter = self.counter + 1
            if (self.counter == 1):
                self.condition = self.condition*(-1)
                self.to_stop = self.to_stop + 1
            twist_object.angular.z = 0.24 * self.condition
            twist_object.linear.x = 0.14
            if(self.to_stop == 7):
                twist_object.angular.z = 0
                twist_object.linear.x = 0
                self.clean_up()

        # self.movement_cleaner.publish(twist_object)

    def region_of_interest(self, image_g):
        # Coordenadas del polígono de interés
        height, width = image_g.shape
        # ROI = np.array([[(25, 580), (25, 550), (300, 450), (500, 450), (width-25, 550),(width-25, 580)]], dtype=np.int32)
        ROI = np.array([[(200, 600), (375, 400), (425, 400), (600, 600)]], dtype=np.int32)
        blank = np.zeros_like(image_g)
        region_of_interest = cv2.fillPoly(blank, ROI, 255)
        # Filtrar la zona de interés
        roi_image = cv2.bitwise_and(image_g, region_of_interest)
        # cv2.imshow("ROI", roi_image)
        return roi_image

    def find_vertical_lines(self, thresh):
        vertical = np.copy(thresh)

        verticalStructure = cv2.getStructuringElement(
            cv2.MORPH_RECT, (1, vertical.shape[1]//20))
        # Apply morphology operations
        vertical = cv2.erode(vertical, verticalStructure)
        vertical = cv2.dilate(vertical, verticalStructure)
        return vertical

    def find_contours(self, thresh, cv_image):
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cv_image = cv2.drawContours(cv_image, contours, -1, (0, 0, 0), 2)
        centers = []
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
            # calculate x,y coordinate of center
            try:
                centers.append((int(M["m10"] / M["m00"]),
                               int(M["m01"] / M["m00"])))
                # cv2.circle(cv_image, centers[-1], 5, (255, 0, 255), -1)
            except ZeroDivisionError:
                centers.append(
                    (int(thresh.shape[0]/2), int(thresh.shape[1]/4)))
        return centers, cv_image

    def clean_up(self):
        twist_object = Twist()
        twist_object.linear.x = 0.0
        twist_object.angular.z = 0.0
        self.movement_cleaner.publish(twist_object)
        cv2.destroyAllWindows()


def main():
    rospy.init_node('cleaner_following_node', anonymous=True)
    cleaner_follower_object = robot_cleaner()

    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        cleaner_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
    main()
