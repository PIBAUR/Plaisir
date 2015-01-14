import math

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from curvePoint import CurvePoint

class Canvas(QWidget):
    ADD_ACTION = 0
    REMOVE_ACTION = 1
    
    gridColor = QColor(150, 150, 150)
    grey = QColor(200, 200, 200)
    gridPen = QPen(gridColor);
    
    def __init__(self, ui, saveCallback):
        super(QWidget, self).__init__()
        
        self.ui = ui
        self.saveCallback = saveCallback
        
        Canvas.gridPen.setCapStyle(Qt.SquareCap);
        Canvas.gridPen.setWidth(1);
        
        # vars
        self.currentAction = None
        self.showControls = False
        self.showTemporalization = False
        self.showMedia = False
        self.breakTangent = False
        
        self.currentRobot = None
        self.otherRobots = []
        self.isEditing = False
        self.currentPoint = None
        self.currentItem = None
        self.currentAnchorOrigins = None
        self.currentControl1Origins = None
        self.currentControl2Origins = None
        self.mediaPixmap = None
        self.currentTimelinePosition = 0
    
    
    def paintEvent(self, e):
        painter = QPainter(self)

        self.drawBackground(painter)
        
        painter.setRenderHint(QPainter.Antialiasing, True)
        
        if self.showTemporalization:
            self.drawTemporization(painter, self.currentRobot)
        
        self.drawPoints(painter, self.currentRobot, self.showControls)
        
        for otherRobot in self.otherRobots:
            self.drawPoints(painter, otherRobot, False)
            self.drawTimelineCursor(painter, otherRobot, self.currentTimelinePosition)
        
        if self.showMedia:
            self.drawMedia(painter, self.currentRobot, self.currentTimelinePosition)
        else:
            self.drawTimelineCursor(painter, self.currentRobot, self.currentTimelinePosition)
        
        
    def mousePressEvent(self, event):
        self.currentItem = None
        self.currentPoint = None
        itemUnderMouseResult = None
        
        mouseX, mouseY = self.addSnapToGrid(event.x(), event.y())
        
        # get item under the mouse (= clicked item)
        for point in self.currentRobot.points:
            itemUnderMouseResult = point.getItemUnderMouse(event.x(), event.y())
            if itemUnderMouseResult is not None:
                self.currentItem = itemUnderMouseResult[0]
                self.currentPoint = itemUnderMouseResult[1]
                break
        
        self.isEditing = self.currentAction != Canvas.REMOVE_ACTION and itemUnderMouseResult is not None
        
        if self.isEditing:
                currentItemOffsetX, currentItemOffsetY = self.addSnapToGrid(mouseX - self.currentItem.x(), mouseY - self.currentItem.y())
                self.currentItemOffset = QPoint(currentItemOffsetX, currentItemOffsetY)
                
                self.currentAnchorOrigins = QPoint(self.currentPoint.anchor.x(), self.currentPoint.anchor.y())
                self.currentControl1Origins = QPoint(self.currentPoint.control1.x(), self.currentPoint.control1.y())
                self.currentControl2Origins = QPoint(self.currentPoint.control2.x(), self.currentPoint.control2.y())
        elif self.currentAction == Canvas.ADD_ACTION:
            # set anchor
            self.currentPoint = CurvePoint(QPoint(mouseX, mouseY))
            self.currentRobot.points.append(self.currentPoint)
        elif self.currentAction == Canvas.REMOVE_ACTION:
            if self.currentItem is not None and self.currentItem == self.currentPoint.anchor:
                self.currentRobot.points.remove(self.currentPoint)
            
        self.update()
    
        
    def mouseReleaseEvent(self, event):
        self.update()
        
        self.saveCallback()
        
        
    def mouseMoveEvent(self, event):
        mouseX, mouseY = self.addSnapToGrid(event.x(), event.y())
        
        if self.isEditing:
            if self.currentItem is not None:
                self.currentItem.setX(mouseX - self.currentItemOffset.x())
                self.currentItem.setY(mouseY - self.currentItemOffset.y())
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
                self.currentPoint.control2.setX(mouseX)
                self.currentPoint.control2.setY(mouseY)
                # invert 2nd tangent
                self.currentPoint.control1.setX(2 * self.currentPoint.anchor.x() - self.currentPoint.control2.x())
                self.currentPoint.control1.setY(2 * self.currentPoint.anchor.y() - self.currentPoint.control2.y())
                
                self.update()
    
    
    def getGridSize(self):
        return 10 * self.ui.zoomCanvas_slider.value()
        
    
    def addSnapToGrid(self, mouseX, mouseY):
        if self.ui.snapToGrid_button.isChecked():
            gridSize = self.getGridSize()
            mouseX = round(mouseX / gridSize) * gridSize
            mouseY = round(mouseY / gridSize) * gridSize
        
        return mouseX, mouseY
        
        
    def drawBackground(self, painter):
        painter.fillRect(QRectF(0, 0, self.width(), self.height()), Canvas.grey)
        
        # grid
        painter.setPen(Canvas.gridPen)
        gridSize = self.getGridSize()
        numCols = int(math.floor(self.width() / gridSize)) + 1
        for i in range(numCols):
            col = i * gridSize
            painter.drawLine(QPoint(col, 0), QPoint(col, self.height()))
        numRows = int(math.floor(self.height() / gridSize)) + 1
        for i in range(numRows):
            row = i * gridSize
            painter.drawLine(QPoint(0, row), QPoint(self.width(), row))
    
    
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
            for media in robot.medias:
                startTimePosition = media.startTime * (len(robot.points) - 1)
                startPointIndex = int(math.floor(startTimePosition))
                startTimePositionRelative = startTimePosition - startPointIndex
                
                startTimeCurvePoint = robot.points[startPointIndex]
                startTimeCurvePoint.drawTimePosition(painter, robot.points[startPointIndex + 1], startTimePositionRelative, media.color)
                
                
    def drawTimelineCursor(self, painter, robot, timePosition):
        if len(robot.points) > 1:
            timePosition *= len(robot.points) - 1
            pointIndex = int(math.floor(timePosition))
            timePositionRelative = timePosition - pointIndex
            
            timeCurvePoint = robot.points[pointIndex]
            timeCurvePoint.drawTimePosition(painter, robot.points[pointIndex + 1], timePositionRelative, QColor(255, 0, 0), "point")
    
    
    def drawMedia(self, painter, robot, timePosition):
        if len(robot.points) > 1:
            timePosition *= len(robot.points) - 1
            pointIndex = int(math.floor(timePosition))
            timePositionRelative = timePosition - pointIndex
            
            timeCurvePoint = robot.points[pointIndex]
            result = timeCurvePoint.getPositionAndAngle(painter, robot.points[pointIndex + 1], timePositionRelative)
            position = result[0]
            angle = 180. * result[1] / math.pi
            
            if self.mediaPixmap is not None:
                transform = QTransform() 
                transform.translate(position.x(), position.y())
                transform.scale(.5, .5)
                transform.rotate(angle)
                transform.translate(-self.mediaPixmap.width() / 2, -self.mediaPixmap.height() / 2)
                painter.setTransform(transform)
                
                painter.drawPixmap(0, 0, self.mediaPixmap)#position.x(), position.y(), self.mediaPixmap)# - self.mediaPixmap.width() / 2, position.y() - self.mediaPixmap.height() / 2, self.mediaPixmap)