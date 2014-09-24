#!/usr/bin/env python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Point
from bezier_curve.msg import BezierCurve

""" method to calculate the value of a point of a bezier curve 

:param u: position of the curve which we want to calculate, from 0 to 1
:type u: float
:param anchor1: coords of the start point of the curve
:type anchor1: geometry_msgs.msg.Point
:param anchor2: coords of the end point of the curve
:type anchor2: geometry_msgs.msg.Point
:param control1: coords of the tangent of the start point
:type control1: geometry_msgs.msg.Point
:param control2: coords of the tangent of the end point
:type control2: geometry_msgs.msg.Point
:returns: result point
"""
def bezierCurve(u, bezierCurve):
    result = Point()
    result.x = pow(u, 3) * (bezierCurve.anchor2.x + 3 * (bezierCurve.control1.x - bezierCurve.control2.x) - bezierCurve.anchor1.x)
    + 3 * pow(u, 2) * (bezierCurve.anchor1.x - 2 * bezierCurve.control1.x + bezierCurve.control2.x)
    + 3 * u * (bezierCurve.control1.x - bezierCurve.anchor1.x) + bezierCurve.anchor1.x
 
    result.y = pow(u, 3) * (bezierCurve.anchor2.y + 3 * (bezierCurve.control1.y - bezierCurve.control2.y) - bezierCurve.anchor1.y)
    + 3 * pow(u, 2) * (bezierCurve.anchor1.y - 2 * bezierCurve.control1.y + bezierCurve.control2.y)
    + 3 * u * (bezierCurve.control1.y - bezierCurve.anchor1.y) + bezierCurve.anchor1.y
    
    return result

