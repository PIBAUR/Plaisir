import math

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from geometry_msgs.msg import Point as PointMsg
from scenario_msgs.msg import BezierCurve as BezierCurveMsg

from src.bezier_curve.src import bezier_interpolate
from src.scenario_lib.src.items.point import Point


class CurvePoint():
    ANCHOR_SIZE = 5.0
    CONTROL_SIZE = 4.0
    
    blue = QColor(79, 128, 255)
    
    anchorPen = QPen(blue)
    controlPen = QPen(blue)
    linePen = QPen(blue)
    curvePen = QPen(blue)
    timePositionBracketPen = QPen(blue)
    timePositionPointPen = QPen(blue)
    
    
    def __init__(self, anchor, control1 = None, control2 = None):
        self.anchor = anchor
        self.control1 = control1
        self.control2 = control2
        
        if self.control1 is None: 
            self.control1 = anchor.clone()
        if self.control2 is None: 
            self.control2 = anchor.clone()
        
        # set graphics params
        CurvePoint.anchorPen.setCapStyle(Qt.SquareCap)
        CurvePoint.anchorPen.setWidth(1)
        
        CurvePoint.controlPen.setCapStyle(Qt.RoundCap)
        CurvePoint.controlPen.setWidth(CurvePoint.ANCHOR_SIZE)
        
        CurvePoint.linePen.setCapStyle(Qt.RoundCap)
        CurvePoint.linePen.setWidth(1)
        
        CurvePoint.curvePen.setCapStyle(Qt.RoundCap)
        CurvePoint.curvePen.setWidth(2)
        
        CurvePoint.timePositionBracketPen.setCapStyle(Qt.RoundCap)
        CurvePoint.timePositionBracketPen.setWidth(2)
        
        CurvePoint.timePositionPointPen.setCapStyle(Qt.RoundCap)
        CurvePoint.timePositionPointPen.setWidth(10)
    
    
    def save(self):
        result = {}
        result["anchor"] = (self.anchor._x, self.anchor._y)
        result["control1"] = (self.control1._x, self.control1._y)
        result["control2"] = (self.control2._x, self.control2._y)
        
        return result
        
        
    def drawKnobs(self, painter):
        # disable antialiasing, and enable at the end
        painter.setRenderHint(QPainter.Antialiasing, False)
        
        # draw control
        painter.setPen(CurvePoint.controlPen)
        painter.drawPoint(self.control1._x, self.control1._y)
        painter.drawPoint(self.control2._x, self.control2._y)
        
        # draw line
        painter.setPen(CurvePoint.linePen)
        painter.drawLine(self.control1._x, self.control1._y, self.anchor._x, self.anchor._y)
        painter.drawLine(self.control2._x, self.control2._y, self.anchor._x, self.anchor._y)
        
        # draw anchor
        painter.setPen(CurvePoint.anchorPen)
        rect = QRectF(self.anchor._x - CurvePoint.CONTROL_SIZE / 2, self.anchor._y - CurvePoint.CONTROL_SIZE / 2, CurvePoint.CONTROL_SIZE, CurvePoint.CONTROL_SIZE)
        painter.fillRect(rect, QColor(255, 255, 255))
        painter.drawRect(rect)
        
        painter.setRenderHint(QPainter.Antialiasing, True)
    
    
    def drawCurve(self, painter, nextPoint, color):
        curveToDraw = self.getBezierCurveWithNextPoint(nextPoint)
        
        previousBezierPoint = None
        numPoints = 70
        for i in range(0, numPoints + 1):
            u = float(i) / numPoints
            bezierPoint = bezier_interpolate.getBezierCurveResult(u, curveToDraw)
            
            if previousBezierPoint is not None:
                # draw line
                CurvePoint.curvePen.setColor(color)
                painter.setPen(CurvePoint.curvePen)
                painter.drawLine(previousBezierPoint.x, previousBezierPoint.y, bezierPoint.x, bezierPoint.y)
            
            previousBezierPoint = bezierPoint
    
    
    def drawTimePosition(self, painter, nextPoint, u, color, drawType = "bracket"):
        result = self.getPositionAndAngle(painter, nextPoint, u)
        timeCursorPoint = result[0]
        tangentAngle = result[1]
        
        if drawType == "bracket" or drawType == "pipe":
            CurvePoint.timePositionBracketPen.setColor(color)
            painter.setPen(CurvePoint.timePositionBracketPen)
            topTimeCursorPoint = Point()
            topTimeCursorPoint.setX(timeCursorPoint._x + 10 * math.cos(tangentAngle))
            topTimeCursorPoint.setY(timeCursorPoint._y + 10 * math.sin(tangentAngle))
            if drawType == "bracket":
                topRightTimeCursorPoint = Point()
                topRightTimeCursorPoint.setX(topTimeCursorPoint._x + 5 * math.cos(tangentAngle - math.pi / 2))
                topRightTimeCursorPoint.setY(topTimeCursorPoint._y + 5 * math.sin(tangentAngle - math.pi / 2))
            bottomTimeCursorPoint = Point()
            bottomTimeCursorPoint.setX(timeCursorPoint._x + 10 * math.cos(math.pi + tangentAngle))
            bottomTimeCursorPoint.setY(timeCursorPoint._y + 10 * math.sin(math.pi + tangentAngle))
            if drawType == "bracket":
                bottomRightTimeCursorPoint = Point()
                bottomRightTimeCursorPoint.setX(bottomTimeCursorPoint._x + 5 * math.cos(tangentAngle - math.pi / 2))
                bottomRightTimeCursorPoint.setY(bottomTimeCursorPoint._y + 5 * math.sin(tangentAngle - math.pi / 2))
            painter.drawLine(timeCursorPoint._x, timeCursorPoint._y, topTimeCursorPoint._x, topTimeCursorPoint._y)
            if drawType == "bracket":
                painter.drawLine(topTimeCursorPoint._x, topTimeCursorPoint._y, topRightTimeCursorPoint._x, topRightTimeCursorPoint._y)
            painter.drawLine(timeCursorPoint._x, timeCursorPoint._y, bottomTimeCursorPoint._x, bottomTimeCursorPoint._y)
            if drawType == "bracket":
                painter.drawLine(bottomTimeCursorPoint._x, bottomTimeCursorPoint._y, bottomRightTimeCursorPoint._x, bottomRightTimeCursorPoint._y)
        elif drawType == "point":
            CurvePoint.timePositionPointPen.setColor(color)
            painter.setPen(CurvePoint.timePositionPointPen)
            painter.drawPoint(timeCursorPoint._x, timeCursorPoint._y)
    
    
    def getPositionAndAngle(self, painter, nextPoint, u):
        curveToDraw = self.getBezierCurveWithNextPoint(nextPoint)
        
        bezierPoint = bezier_interpolate.getBezierCurveResult(u, curveToDraw)
        tangentAngle = bezier_interpolate.getBezierCurveTangentResult(u + (.01 if u == 0 else 0), curveToDraw)
        tangentAngle += math.pi / 2
        
        return (Point(bezierPoint.x, bezierPoint.y), tangentAngle)
            
    
    def getBezierCurveWithNextPoint(self, nextPoint, yRatio = 1, pointToSubstract = None):
        if pointToSubstract is None:
            pointToSubstract = Point()
            
        result = BezierCurveMsg()
        result.anchor_1 = PointMsg((self.anchor._x - pointToSubstract._x), (self.anchor._y - pointToSubstract._y) * yRatio, self.anchor._theta)
        result.anchor_2 = PointMsg((nextPoint.anchor._x - pointToSubstract._x), (nextPoint.anchor._y - pointToSubstract._y) * yRatio, 0)
        result.control_1 = PointMsg((self.control2._x - pointToSubstract._x), (self.control2._y - pointToSubstract._y) * yRatio, 0)
        result.control_2 = PointMsg((nextPoint.control1._x - pointToSubstract._x), (nextPoint.control1._y - pointToSubstract._y) * yRatio, 0)
        
        return result
        
    
    def getItemUnderMouse(self, x, y):
        if self.isControlUnderPoint(self.control1, x, y):
            return (self.control1, self)
        elif self.isControlUnderPoint(self.control2, x, y):
            return (self.control2, self)
        elif self.isAnchorUnderPoint(self.anchor, x, y):
            return (self.anchor, self)
        else:
            return None
    
    
    def isControlUnderPoint(self, control, x, y):
        return (math.sqrt(math.pow(control._x - x, 2) + math.pow(control._y - y, 2))) <= (CurvePoint.CONTROL_SIZE)

    
    def isAnchorUnderPoint(self, anchor, x, y):
        return anchor._x - CurvePoint.ANCHOR_SIZE <= x and anchor._x + CurvePoint.ANCHOR_SIZE >= x and anchor._y - CurvePoint.ANCHOR_SIZE <= y and anchor._y + CurvePoint.ANCHOR_SIZE >= y
    