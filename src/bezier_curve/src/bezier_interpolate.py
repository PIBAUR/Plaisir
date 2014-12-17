#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
from scenario_msgs.msg import Scenario
from geometry_msgs.msg import PoseArray, Pose, Point

def scenarioCallback(data):
    pathPublisher = rospy.Publisher("path", PoseArray)
    step = 0.1
    
    path = PoseArray()
    path.poses = []
    path.header = data.header
    
    for curve in data.bezier_paths.curves:
        step = .1
        i = -step
        while i <= 1:
            i += step
            
            p = Pose()
            p.position = getBezierCurveResult(i, curve)
            theta = getBezierCurveTangentResult(i, curve)
            p.orientation.z = math.sin(theta / 2)
            p.orientation.w = math.cos(theta / 2)
            path.poses.append(p)

    pathPublisher.publish(path)

""" method to calculate the value of a point of a bezier curve 

:param u: position of the curve which we want to calculate, from 0 to 1
:type u: float
:param bezierCurve: coords of the start point, end point and tangent for each of them for the curve
:type bezierCurve: bezier_curve.msg.BezierCurve
:returns: result point
:rtype: geometry_msgs.msg.Point
"""
def getBezierCurveResult(u, bezierCurve):
    result = Point()
    result.x = pow(u, 3) * (bezierCurve.anchor_2.x + 3 * (bezierCurve.control_1.x - bezierCurve.control_2.x) - bezierCurve.anchor_1.x) \
               + 3 * pow(u, 2) * (bezierCurve.anchor_1.x - 2 * bezierCurve.control_1.x + bezierCurve.control_2.x) \
               + 3 * u * (bezierCurve.control_1.x - bezierCurve.anchor_1.x) + bezierCurve.anchor_1.x
 
    result.y = pow(u, 3) * (bezierCurve.anchor_2.y + 3 * (bezierCurve.control_1.y - bezierCurve.control_2.y) - bezierCurve.anchor_1.y) \
               + 3 * pow(u, 2) * (bezierCurve.anchor_1.y - 2 * bezierCurve.control_1.y + bezierCurve.control_2.y) \
               + 3 * u * (bezierCurve.control_1.y - bezierCurve.anchor_1.y) + bezierCurve.anchor_1.y
    
    return result


""" method to calculate the tangent of a point of a bezier curve 

:param u: position of the curve which we want to calculate, from 0 to 1
:type u: float
:param bezierCurve: coords of the start point, end point and tangent for each of them for the curve
:type bezierCurve: bezier_curve.msg.BezierCurve
:returns: result tangente
:rtype: float
"""
def getBezierCurveTangentResult(u, bezierCurve):
    p = Point()
    alphaX = (bezierCurve.anchor_2.x + 3 * (bezierCurve.control_1.x - bezierCurve.control_2.x) - bezierCurve.anchor_1.x)
    alphaY = (bezierCurve.anchor_2.y + 3 * (bezierCurve.control_1.y - bezierCurve.control_2.y) - bezierCurve.anchor_1.y)
    betaX = (bezierCurve.anchor_1.x - 2 * bezierCurve.control_1.x + bezierCurve.control_2.x)
    betaY = (bezierCurve.anchor_1.y - 2 * bezierCurve.control_1.y + bezierCurve.control_2.y)
    gammaX = (bezierCurve.control_1.x - bezierCurve.anchor_1.x)
    gammaY = (bezierCurve.control_1.y - bezierCurve.anchor_1.y)
    
    p.x = 3 * pow(u, 2) * alphaX + 6 * u * betaX + 3 * gammaX
    p.y = 3 * pow(u, 2) * alphaY + 6 * u * betaY + 3 * gammaY
    tangent = math.atan2(p.y, p.x)
    
    return tangent


if __name__ == "__main__":
    rospy.init_node('bezier_interpolate', anonymous = True)
    rospy.Subscriber("scenario", Scenario, scenarioCallback)
    rospy.spin()
