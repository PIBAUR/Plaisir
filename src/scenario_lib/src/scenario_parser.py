#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from scenario_msgs.msg import *


medias_pub = rospy.Publisher('media', MediaArray)
bpath_pub = rospy.Publisher('bezier_path', BezierPath)

def scenarioCB(data):
    bcurves_msg = data.bezier_paths
    medias_msg = data.medias    
    bpath_pub.publish(bcurves_msg)
    medias_pub.publish(medias_msg)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('scenario', Scenario, scenarioCB)
    rospy.spin()
