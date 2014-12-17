#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from geometry_msgs.msg import Point
from scenario_msgs.msg import BezierCurve


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
    result.x = pow(u, 3) * (bezierCurve.anchor2.x + 3 * (bezierCurve.control1.x - bezierCurve.control2.x) - bezierCurve.anchor1.x)
    + 3 * pow(u, 2) * (bezierCurve.anchor1.x - 2 * bezierCurve.control1.x + bezierCurve.control2.x)
    + 3 * u * (bezierCurve.control1.x - bezierCurve.anchor1.x) + bezierCurve.anchor1.x
 
    result.y = pow(u, 3) * (bezierCurve.anchor2.y + 3 * (bezierCurve.control1.y - bezierCurve.control2.y) - bezierCurve.anchor1.y)
    + 3 * pow(u, 2) * (bezierCurve.anchor1.y - 2 * bezierCurve.control1.y + bezierCurve.control2.y)
    + 3 * u * (bezierCurve.control1.y - bezierCurve.anchor1.y) + bezierCurve.anchor1.y
    
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
    alphaX = (bezierCurve.anchor2.x + 3 * (bezierCurve.control1.x - bezierCurve.control2.x) - bezierCurve.anchor1.x)
    alphaY = (bezierCurve.anchor2.y + 3 * (bezierCurve.control1.y - bezierCurve.control2.y) - bezierCurve.anchor1.y)
    betaX = (bezierCurve.anchor1.x - 2 * bezierCurve.control1.x + bezierCurve.control2.x)
    betaY = (bezierCurve.anchor1.y - 2 * bezierCurve.control1.y + bezierCurve.control2.y)
    gammaX = (bezierCurve.control1.x - bezierCurve.anchor1.x)
    gammaY = (bezierCurve.control1.y - bezierCurve.anchor1.y)
    
    p.x = 3 * pow(u, 2) * alphaX + 6 * u * betaX + 3 * gammaX
    p.y = 3 * pow(u, 2) * alphaY + 6 * u * betaY + 3 * gammaY
    tangent = math.atan2(p.y, p.x)
    
    return tangent

