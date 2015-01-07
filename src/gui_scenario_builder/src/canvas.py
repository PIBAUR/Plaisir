import math

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from curvePoint import CurvePoint

class Canvas(QWidget):
    ADD_ACTION = 0
    REMOVE_ACTION = 1
    
    def __init__(self, saveCallback, x, y, widht, height):
        super(QWidget, self).__init__()
        
        self.setGeometry(x, y, widht, height)
        
        self.saveCallback = saveCallback
        
        # vars
        self.currentAction = None
        self.showControls = False
        self.showTemporization = True
        self.breakTangent = False
        
        self.currentRobot = None
        self.otherRobots = []
        self.isEditing = False
        self.currentPoint = None
        self.currentItem = None
        self.currentAnchorOrigins = None
        self.currentControl1Origins = None
        self.currentControl2Origins = None
    
    
    def paintEvent(self, e):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        
        self.drawBackground(painter)
        
        if self.showTemporization:
            self.drawTemporization(painter, self.currentRobot)
        
        self.drawPoints(painter, self.currentRobot, self.showControls)
        
        for otherRobot in self.otherRobots:
            self.drawPoints(painter, otherRobot, False)
        
        
    def mousePressEvent(self, event):
        self.currentItem = None
        self.currentPoint = None
        itemUnderMouseResult = None
        
        # get item under the mouse (= clicked item)
        for point in self.currentRobot.points:
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
            self.currentRobot.points.append(self.currentPoint)
        elif self.currentAction == Canvas.REMOVE_ACTION:
            if self.currentItem is not None and self.currentItem == self.currentPoint.anchor:
                self.currentRobot.points.remove(self.currentPoint)
            
        self.update()
    
        
    def mouseReleaseEvent(self, event):
        self.update()
        
        self.saveCallback()
        
        
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
    
    
    def drawPoints(self, painter, robot, showControls):
        for i in range(len(robot.points)):
            point = robot.points[i]
            
            if i + 1 < len(robot.points):
                nextPoint = robot.points[i + 1]
                point.drawCurve(painter, nextPoint, robot.color)
            
            if showControls:
                point.drawKnobs(painter)
                
                
    def drawTemporization(self, painter, robot):
        if len(robot.points) > 1:
            for video in robot.videos:
                startTimePosition = video.startTime * (len(robot.points) - 1)
                startPointIndex = int(math.floor(startTimePosition))
                startTimePositionRelative = startTimePosition - startPointIndex
                
                startTimeCurvePoint = robot.points[startPointIndex]
                startTimeCurvePoint.drawTimePosition(painter, robot.points[startPointIndex + 1], startTimePositionRelative, video.color)