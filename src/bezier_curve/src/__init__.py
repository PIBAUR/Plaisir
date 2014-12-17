#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
def bezierCurve(u, bezierCurve):
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
def bezierCurveTangente(u, bezierCurve):
    p = Point()
    alpha_x = (bezierCurve.anchor2.x + 3 * (bezierCurve.control1.x - bezierCurve.control2.x) - bezierCurve.anchor1.x)
    alpha_y = (bezierCurve.anchor2.y + 3 * (bezierCurve.control1.y - bezierCurve.control2.y) - bezierCurve.anchor1.y)
    beta_x = (bezierCurve.anchor1.x - 2 * bezierCurve.control1.x + bezierCurve.control2.x)
    beta_y = (bezierCurve.anchor1.y - 2 * bezierCurve.control1.y + bezierCurve.control2.y)
    gamma_x = (bezierCurve.control1.x - bezierCurve.anchor1.x)
    gamma_y = (bezierCurve.control1.y - bezierCurve.anchor1.y)
    
    
    
    p.x = 3*pow(u, 2)*alpha_x + 6*u*beta_x + 3*gamma_x
    p.y = 3*pow(u, 2)*alpha_y + 6*u*beta_y + 3*gamma_y
    tangente = atan2(y,x)
    
    return tangente

