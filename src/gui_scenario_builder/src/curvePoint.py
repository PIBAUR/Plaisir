import math

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from geometry_msgs.msg import Point
from bezier_curve.msg import BezierCurve

from src.bezier_curve.src import bezier_interpolate


class CurvePoint():
    ANCHOR_SIZE = 5.0
    CONTROL_SIZE = 4.0
    
    blue = QColor(79, 128, 255)
    
    anchorPen = QPen(blue);
    controlPen = QPen(blue);
    linePen = QPen(blue);
    curvePen = QPen(blue);
    timePositionPen = QPen(blue);
    
    
    def __init__(self, anchor, control1 = None, control2 = None):
        self.anchor = anchor
        self.control1 = control1
        self.control2 = control2
        
        if self.control1 is None: 
            self.control1 = QPoint(anchor.x(), anchor.y())
        if self.control2 is None: 
            self.control2 = QPoint(anchor.x(), anchor.y())
        
        # set graphics params
        CurvePoint.anchorPen.setCapStyle(Qt.SquareCap);
        CurvePoint.anchorPen.setWidth(1);
        
        CurvePoint.controlPen.setCapStyle(Qt.RoundCap);
        CurvePoint.controlPen.setWidth(CurvePoint.ANCHOR_SIZE);
        
        CurvePoint.linePen.setCapStyle(Qt.RoundCap);
        CurvePoint.linePen.setWidth(1);
        
        CurvePoint.curvePen.setCapStyle(Qt.RoundCap);
        CurvePoint.curvePen.setWidth(2);
        
        CurvePoint.timePositionPen.setCapStyle(Qt.RoundCap);
        CurvePoint.timePositionPen.setWidth(3);
        
        
    def drawKnobs(self, painter):
        # disable antialiasing, and enable at the end
        painter.setRenderHint(QPainter.Antialiasing, False)
        
        # draw control
        painter.setPen(CurvePoint.controlPen)
        painter.drawPoint(self.control1)
        painter.drawPoint(self.control2)
        
        # draw line
        painter.setPen(CurvePoint.linePen)
        painter.drawLine(self.control1, self.anchor)
        painter.drawLine(self.control2, self.anchor)
        
        # draw anchor
        painter.setPen(CurvePoint.anchorPen);
        rect = QRectF(self.anchor.x() - CurvePoint.CONTROL_SIZE / 2, self.anchor.y() - CurvePoint.CONTROL_SIZE / 2, CurvePoint.CONTROL_SIZE, CurvePoint.CONTROL_SIZE)
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
                painter.drawLine(QPoint(previousBezierPoint.x, previousBezierPoint.y), QPoint(bezierPoint.x, bezierPoint.y))
            
            previousBezierPoint = bezierPoint
    
    
    def drawTimePosition(self, painter, nextPoint, u, color):
        curveToDraw = self.getBezierCurveWithNextPoint(nextPoint)
        
        bezierPoint = bezier_interpolate.getBezierCurveResult(u, curveToDraw)
        tangentAngle = bezier_interpolate.getBezierCurveTangentResult(u + (.01 if u == 0 else 0), curveToDraw)
        tangentAngle += math.pi / 2
        
        # draw cursor
        CurvePoint.timePositionPen.setColor(color)
        painter.setPen(CurvePoint.timePositionPen)
        timeCursorPoint = QPoint(bezierPoint.x, bezierPoint.y)
        topTimeCursorPoint = QPoint()
        topTimeCursorPoint.setX(timeCursorPoint.x() + 10 * math.cos(tangentAngle))
        topTimeCursorPoint.setY(timeCursorPoint.y() + 10 * math.sin(tangentAngle))
        topRightTimeCursorPoint = QPoint()
        topRightTimeCursorPoint.setX(topTimeCursorPoint.x() + 5 * math.cos(tangentAngle - math.pi / 2))
        topRightTimeCursorPoint.setY(topTimeCursorPoint.y() + 5 * math.sin(tangentAngle - math.pi / 2))
        bottomTimeCursorPoint = QPoint()
        bottomTimeCursorPoint.setX(timeCursorPoint.x() + 10 * math.cos(math.pi + tangentAngle))
        bottomTimeCursorPoint.setY(timeCursorPoint.y() + 10 * math.sin(math.pi + tangentAngle))
        bottomRightTimeCursorPoint = QPoint()
        bottomRightTimeCursorPoint.setX(bottomTimeCursorPoint.x() + 5 * math.cos(tangentAngle - math.pi / 2))
        bottomRightTimeCursorPoint.setY(bottomTimeCursorPoint.y() + 5 * math.sin(tangentAngle - math.pi / 2))
        painter.drawLine(timeCursorPoint, topTimeCursorPoint)
        painter.drawLine(topTimeCursorPoint, topRightTimeCursorPoint)
        painter.drawLine(timeCursorPoint, bottomTimeCursorPoint)
        painter.drawLine(bottomTimeCursorPoint, bottomRightTimeCursorPoint)
            
    
    def getBezierCurveWithNextPoint(self, nextPoint):
        result = BezierCurve()
        result.anchor_1 = Point(self.anchor.x(), self.anchor.y(), 0)
        result.anchor_2 = Point(nextPoint.anchor.x(), nextPoint.anchor.y(), 0)
        result.control_1 = Point(self.control2.x(), self.control2.y(), 0)
        result.control_2 = Point(nextPoint.control1.x(), nextPoint.control1.y(), 0)
        
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
        return (math.sqrt(math.pow(control.x() - x, 2) + math.pow(control.y() - y, 2))) <= (CurvePoint.CONTROL_SIZE)

    
    def isAnchorUnderPoint(self, anchor, x, y):
        return anchor.x() - CurvePoint.ANCHOR_SIZE <= x and anchor.x() + CurvePoint.ANCHOR_SIZE >= x and anchor.y() - CurvePoint.ANCHOR_SIZE <= y and anchor.y() + CurvePoint.ANCHOR_SIZE >= y
    