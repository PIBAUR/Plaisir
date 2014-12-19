from PyQt4.QtGui import *
from PyQt4.QtCore import *

from geometry_msgs.msg import Point
from bezier_curve.msg import BezierCurve

from src.bezier_curve.src import bezier_interpolate

blue = QColor(79, 128 ,255)

anchorPen = QPen(blue);
anchorPen.setCapStyle(Qt.SquareCap);
anchorPen.setWidth(1);

controlPen = QPen(blue);
controlPen.setCapStyle(Qt.RoundCap);
controlPen.setWidth(5);

linePen = QPen(blue);
linePen.setCapStyle(Qt.RoundCap);
linePen.setWidth(1);


class CurvePoint():
    def __init__(self, anchor, control1 = None, control2 = None):
        self.anchor = anchor
        self.control1 = control1
        self.control2 = control2
        
        if self.control1 is None: 
            self.control1 = QPoint(anchor.x(), anchor.y())
        if self.control2 is None: 
            self.control2 = QPoint(anchor.x(), anchor.y())
        
        
    def drawKnobs(self, painter):
        # draw control
        painter.setPen(controlPen)
        painter.drawPoint(self.control1)
        painter.drawPoint(self.control2)
        
        # draw line
        painter.setPen(linePen)
        painter.drawLine(self.control1, self.control2)
        
        # draw anchor
        painter.setPen(anchorPen);
        rect = QRectF(self.anchor.x() - 2, self.anchor.y() - 2, 4, 4)
        painter.fillRect(rect, QColor(255, 255, 255))
        painter.drawRect(rect)
    
    
    def drawCurve(self, painter, nextPoint):
        anchor1 = Point(self.anchor.x(), self.anchor.y(), 0)
        anchor2 = Point(nextPoint.anchor.x(), nextPoint.anchor.y(), 0)
        control1 = Point(self.control2.x(), self.control2.y(), 0)
        control2 = Point(nextPoint.control1.x(), nextPoint.control1.y(), 0)
        
        curveToDraw = BezierCurve()
        curveToDraw.anchor_1 = anchor1
        curveToDraw.anchor_2 = anchor2
        curveToDraw.control_1 = control1
        curveToDraw.control_2 = control2
        
        previousBezierPoint = None
        numPoints = 100
        for i in range(0, numPoints):
            u = float(i) / numPoints
            bezierPoint = bezier_interpolate.getBezierCurveResult(u, curveToDraw)
            
            if previousBezierPoint is not None:
                # draw line
                painter.setPen(linePen)
                painter.drawLine(QPoint(previousBezierPoint.x, previousBezierPoint.y), QPoint(bezierPoint.x, bezierPoint.y))
            
            previousBezierPoint = bezierPoint
            
