from PyQt4.QtGui import *
from PyQt4.QtCore import *

from curvePoint import CurvePoint

class Canvas(QWidget):
    def __init__(self, x, y, widht, height):
        super(QWidget, self).__init__()
        
        self.setGeometry(x, y, widht, height)
        
        # vars
        self.points = []
        self.currentPoint = None
    
    
    def paintEvent(self, e):
        painter = QPainter(self)
        painter.begin(self)
        self.drawBackground(painter)
        self.drawPoints(painter)
        painter.end()
        
        
    def mousePressEvent(self, event):
        # set anchor
        self.currentPoint = CurvePoint(QPoint(event.x(), event.y()))
        self.points.append(self.currentPoint)
        self.update()
    
        
    def mouseReleaseEvent(self, event):
        self.update()
        
        
    def mouseMoveEvent(self, event):
        if self.currentPoint is not None:
            # set tangent
            self.currentPoint.control2.setX(event.x())
            self.currentPoint.control2.setY(event.y())
            # invert 2nd tangent
            self.currentPoint.control1.setX(2 * self.currentPoint.anchor.x() - self.currentPoint.control2.x())
            self.currentPoint.control1.setY(2 * self.currentPoint.anchor.y() - self.currentPoint.control2.y())
            
        self.update()
    
    
    def drawBackground(self, painter):
        painter.fillRect(QRectF(0, 0, self.width(), self.height()), QColor(200, 200, 200))
    
    
    def drawPoints(self, painter):
        for i in range(len(self.points)):
            point = self.points[i]
            point.drawKnobs(painter)
            
            if i + 1 < len(self.points):
                nextPoint = self.points[i + 1]
                point.drawCurve(painter, nextPoint)
