#!/usr/bin/env python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Point
from scenario.msg import Scenario
from bezier_curve import bezierCurve
from rospy 
from geometry_msgs.msg import PoseArray


def ScenarioCB(data):
    path_pub = rospy.Publisher("path", PoseArray)
    step = 0.01
    
    path = PoseArray()
    p = Pose()
    
    for curve in data.curves:
        for i in range(0,1,step) :
            p.position = bezierCurve(i,curve)
            theta = bezierCurveTangente(i,curve)
            p.orientation.z = sin(theta/2)
            p.orientation.w = cos(theta/2)
            path.append(p)
    path.header = data.header
    path_pub.publish(path)


if __name__ == "__main__":
    
    rospy.init_node('bezier_interpolate', anonymous=True)

    rospy.Subscriber("scenario", Scenario, ScenarioCB)

    rospy.spin()
