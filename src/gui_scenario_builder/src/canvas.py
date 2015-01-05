from PyQt4.QtGui import *
from PyQt4.QtCore import *

from curvePoint import CurvePoint

class Canvas(QWidget):
    ADD_ACTION = 0
    REMOVE_ACTION = 1
    
    def __init__(self, x, y, widht, height):
        super(QWidget, self).__init__()
        
        self.setGeometry(x, y, widht, height)
        
        # vars
        self.currentAction = None
        self.showControls = False
        self.breakTangent = False
        
        self.points = []
        self.isEditing = False
        self.currentPoint = None
        self.currentItem = None
        self.currentAnchorOrigins = None
        self.currentControl1Origins = None
        self.currentControl2Origins = None
    
    
    def paintEvent(self, e):
        painter = QPainter(self)
        painter.begin(self)
        self.drawBackground(painter)
        self.drawPoints(painter)
        painter.end()
        
        
    def mousePressEvent(self, event):
        self.currentItem = None
        self.currentPoint = None
        itemUnderMouseResult = None
        
        # get item under the mouse (= clicked item)
        for point in self.points:
            itemUnderMouseResult = point.getItemUnderMouse(event.x(), event.y())
            if itemUnderMouseResult is not None:
                self.currentItem = itemUnderMouseResult[0]
                self.currentPoint = itemUnderMouseResult[1]
                break
        
        self.isEditing = self.currentAction != Canvas.REMOVE_ACTION and itemUnderMouseResult is not None
        
        if self.isEditing:
                self.currentItemOffset = QPoint(event.x() - self.currentItem.x(), event.y() - self.currentItem.y())
                self.currentAnchorOrigins = QPoint(self.currentPoint.anchor.x(), self.currentPoint.anchor.y())
                self.currentControl1Origins = QPoint(self.currentPoint.control1.x(), self.currentPoint.control1.y())
                self.currentControl2Origins = QPoint(self.currentPoint.control2.x(), self.currentPoint.control2.y())
        elif self.currentAction == Canvas.ADD_ACTION:
            # set anchor
            self.currentPoint = CurvePoint(QPoint(event.x(), event.y()))
            self.points.append(self.currentPoint)
        elif self.currentAction == Canvas.REMOVE_ACTION:
            if self.currentItem is not None and self.currentItem == self.currentPoint.anchor:
                self.points.remove(self.currentPoint)
            
        self.update()
    
        
    def mouseReleaseEvent(self, event):
        self.update()
        
        
    def mouseMoveEvent(self, event):
        if self.isEditing:
            if self.currentItem is not None:
                self.currentItem.setX(event.x() - self.currentItemOffset.x())
                self.currentItem.setY(event.y() - self.currentItemOffset.y())
                # move controls with anchor
                if self.currentItem == self.currentPoint.anchor:
                    self.currentPoint.control1.setX(self.currentControl1Origins.x() + (self.currentItem.x() - self.currentAnchorOrigins.x()))
                    self.currentPoint.control1.setY(self.currentControl1Origins.y() + (self.currentItem.y() - self.currentAnchorOrigins.y()))
                    self.currentPoint.control2.setX(self.currentControl2Origins.x() + (self.currentItem.x() - self.currentAnchorOrigins.x()))
                    self.currentPoint.control2.setY(self.currentControl2Origins.y() + (self.currentItem.y() - self.currentAnchorOrigins.y()))
                else:
                    # maybe control other if not broken tangent
                    if not self.breakTangent:
                        # get other tangent
                        otherTangent = self.currentPoint.control1 if self.currentPoint.control2 == self.currentItem else self.currentPoint.control2
                        # invert 2nd tangent
                        otherTangent.setX(2 * self.currentPoint.anchor.x() - self.currentItem.x())
                        otherTangent.setY(2 * self.currentPoint.anchor.y() - self.currentItem.y())
                
                self.update()
        elif self.currentAction == Canvas.ADD_ACTION:
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
            
            if self.showControls:
                point.drawKnobs(painter)
            
            if i + 1 < len(self.points):
                nextPoint = self.points[i + 1]
                point.drawCurve(painter, nextPoint)
