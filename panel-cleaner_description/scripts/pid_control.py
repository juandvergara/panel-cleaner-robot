#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64


class PID(object):

    def __init__(self):
        self._setpoint_pub = rospy.Publisher(
            "/setpoint", Float64, queue_size=1)
        self._state_pub = rospy.Publisher("/state", Float64, queue_size=1)
        self._control_effort_sub = rospy.Subscriber(
            "/control_effort", Float64, self.control_effort_callback)
        self._control_effort_value = Float64()

    def control_effort_callback(self, data):
        self._control_effort_value.data = data.data

    def setpoint_update(self, value):
        value_object = Float64()
        value_object.data = value
        self._setpoint_pub.publish(value_object)

    def state_update(self, value):
        value_object = Float64()
        value_object.data = value
        self._state_pub.publish(value_object)

    def get_control_effort(self):
        return self._control_effort_value.data
