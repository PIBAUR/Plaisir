#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from scenario_msgs.msg import Scenario
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from src.bezier_curve.src import getBezierCurveResult, getBezierCurveTangentResult

def ScenarioCB(data):
    pathPublisher = rospy.Publisher("path", PoseArray)
    step = 0.01
    
    path = PoseArray()
    p = Pose()
    
    for curve in data.curves:
        for i in range(0, 1, step) :
            p.position = getBezierCurveResult(i, curve)
            theta = getBezierCurveTangentResult(i, curve)
            p.orientation.z = math.sin(theta / 2)
            p.orientation.w = math.cos(theta / 2)
            path.append(p)
    path.header = data.header
    pathPublisher.publish(path)


if __name__ == "__main__":
    rospy.init_node('bezier_interpolate', anonymous=True)
    rospy.Subscriber("scenario", Scenario, ScenarioCB)
    rospy.spin()
