
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range

def callback(data):
    print(data)

rospy.init_node('range_values')
sub = rospy.Subscriber('/sensor/sonar1_scan', Range, callback)
rospy.spin()