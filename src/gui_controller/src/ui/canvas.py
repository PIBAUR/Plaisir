from PyQt4.QtGui import *
from PyQt4.QtCore import *

class Canvas(QWidget):
    def __init__(self, callback, x, y, widht, height):
        super(QWidget, self).__init__()
        self.callback = callback
        
        self.setGeometry(x, y, widht, height)
        
        self.point = None
    
    
    def paintEvent(self, e):
        painter = QPainter(self)
        self.drawBackground(painter)
        self.drawPoints(painter)
        
        
    def mousePressEvent(self, event):
        self.point = (event.x(), event.y())
        self.updateMouse(event)
    
        
    def mouseReleaseEvent(self, event):
        self.point = None
        self.update()
        self.callback(0, 0)
        
        
    def mouseMoveEvent(self, event):
        self.updateMouse(event)
    
    
    def updateMouse(self, event):
        if self.point is not None:
            self.point = (event.x(), event.y())
            self.update()
            self.callback((float(event.x()) / self.width()) * 2 - 1, (float(event.y()) / self.height()) * 2 - 1)
    
    
    def drawBackground(self, painter):
        painter.fillRect(QRectF(0, 0, self.width(), self.height()), QColor(200, 200, 200))
    
    
    def drawPoints(self, painter):
        linePen = QPen(QColor(150, 150 ,150));
        linePen.setCapStyle(Qt.RoundCap);
        painter.setRenderHint(QPainter.Antialiasing, True);
        painter.setPen(linePen);
        linePen.setWidth(30);
        painter.setPen(linePen);
        
        if self.point is not None:
            painter.drawPoint(self.point[0], self.point[1])