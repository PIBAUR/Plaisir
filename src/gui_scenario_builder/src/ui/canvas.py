import math

import rospy

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from src.scenario_lib.src.items.curvePoint import CurvePoint

class Canvas(QWidget):
    ADD_ACTION = 0
    REMOVE_ACTION = 1
    
    gridColor = QColor(150, 150, 150)
    grey = QColor(200, 200, 200)
    gridPen = QPen(gridColor);
    targetPen = QPen(QColor(0, 255, 0))
    targetOutlinePen = QPen(QColor(0, 100, 0))
    targetRadius = 10
    
    def __init__(self, main, ui, changeCallback):
        super(QWidget, self).__init__()
        
        self.main = main
        self.ui = ui
        self.changeCallback = changeCallback
        
        # get params
        try:
            self.mediaTimeBase = float(rospy.get_param("media_time_base"))
        except Exception:
            self.mediaTimeBase = 2.5
        
        Canvas.gridPen.setCapStyle(Qt.SquareCap)
        Canvas.gridPen.setWidth(1)
        
        Canvas.targetPen.setCapStyle(Qt.RoundCap)
        Canvas.targetPen.setWidth(Canvas.targetRadius * 2)
        Canvas.targetOutlinePen.setCapStyle(Qt.SquareCap)
        Canvas.targetOutlinePen.setWidth(1)
        
        self.ui.zoomCanvas_slider.valueChanged.connect(self.handleZoomSliderValueChanged)
        
        # vars
        self.currentAction = None
        self.showControls = False
        self.showTemporalization = False
        self.showMedia = False
        self.breakTangent = False
        
        self.currentRobot = None
        self.otherRobots = []
        self.isEditing = False
        self.targetDragging = False
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
        
        if self.showMedia:
            self.drawMedia(painter, self.currentRobot, self.currentTimelinePosition)
        else:
            self.drawTimelineCursor(painter, self.currentRobot, self.currentTimelinePosition)
        
        self.drawPoints(painter, self.currentRobot, self.showControls)
        self.drawTargetPoint(painter)
        
        for otherRobot in self.otherRobots:
            self.drawTimelineCursor(painter, otherRobot, self.currentTimelinePosition)
            self.drawPoints(painter, otherRobot, False)
        
        
        
    def mousePressEvent(self, event):
        self.currentItem = None
        self.currentPoint = None
        itemUnderMouseResult = None
        
        mouseX, mouseY = self.addSnapToGrid(event.x(), event.y())
        
        # get item under the mouse (= clicked item)
        distanceTargetMouse = math.sqrt(math.pow(mouseX - self.main.currentScenario.targetPosition[0], 2) + math.pow(mouseY - self.main.currentScenario.targetPosition[1], 2))
        if distanceTargetMouse <= Canvas.targetRadius:
            self.targetDragging = True
        else:
            for point in self.currentRobot.points:
                itemUnderMouseResult = point.getItemUnderMouse(event.x(), event.y())
                if itemUnderMouseResult is not None:
                    self.currentItem = itemUnderMouseResult[0]
                    self.currentPoint = itemUnderMouseResult[1]
                    break
        
        self.isEditing = self.currentAction != Canvas.REMOVE_ACTION and itemUnderMouseResult is not None
        
        if self.targetDragging:
            currentItemOffsetX, currentItemOffsetY = self.addSnapToGrid(mouseX - self.main.currentScenario.targetPosition[0], mouseY - self.main.currentScenario.targetPosition[1])
            self.currentItemOffset = QPoint(currentItemOffsetX, currentItemOffsetY)
        elif self.isEditing:
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
        self.targetDragging = False
        
        self.update()
        
        self.changeCallback()
        
        
    def mouseMoveEvent(self, event):
        mouseX, mouseY = self.addSnapToGrid(event.x(), event.y())
        
        if self.targetDragging:
            self.main.currentScenario.targetPosition[0] = mouseX - self.currentItemOffset.x()
            self.main.currentScenario.targetPosition[1] = mouseY - self.currentItemOffset.y()
        elif self.isEditing:
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
    
    
    def handleZoomSliderValueChanged(self, value):
        self.changeCallback()
        
        
    def getGridSize(self):
        return 10 * self.ui.zoomCanvas_slider.value()
        
    
    def addSnapToGrid(self, mouseX, mouseY):
        if self.ui.snapToGrid_button.isChecked():
            gridSize = self.getGridSize()
            mouseX = round(mouseX / gridSize) * gridSize
            mouseY = round(mouseY / gridSize) * gridSize
        
        if mouseX < 0:
            mouseX = 0
        elif mouseX > self.width():
            mouseX = self.width()
        if mouseY < 0:
            mouseY = 0
        elif mouseY > self.height():
            mouseY = self.height()
        
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
    
    
    def drawTargetPoint(self, painter):
        targetPoint = QPoint(self.main.currentScenario.targetPosition[0], self.main.currentScenario.targetPosition[1])
        
        painter.setPen(Canvas.targetPen)
        painter.drawPoint(targetPoint)
        painter.setPen(Canvas.targetOutlinePen)
        painter.drawEllipse(targetPoint, Canvas.targetRadius, Canvas.targetRadius)
        painter.drawLine(QPoint(targetPoint.x() - 3, targetPoint.y()), QPoint(targetPoint.x() - Canvas.targetRadius, targetPoint.y()))
        painter.drawLine(QPoint(targetPoint.x() + 3, targetPoint.y()), QPoint(targetPoint.x() + Canvas.targetRadius, targetPoint.y()))
        painter.drawLine(QPoint(targetPoint.x(), targetPoint.y() - 3), QPoint(targetPoint.x(), targetPoint.y() - Canvas.targetRadius))
        painter.drawLine(QPoint(targetPoint.x(), targetPoint.y() + 3), QPoint(targetPoint.x(), targetPoint.y() + Canvas.targetRadius))
        
    
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
            i = 0
            for media in robot.medias:
                numTimeBases = int(math.ceil(media.duration / self.mediaTimeBase))
                mediaTime = media.endTime - media.startTime
                # for each "2.5s"
                for j in range(numTimeBases):
                    lala = media.startTime + (mediaTime / numTimeBases) * j
                    timePosition = lala * (len(robot.points) - 1)
                    pointIndex = int(math.floor(timePosition))
                    timePositionRelative = timePosition - pointIndex
                    
                    timeCurvePoint = robot.points[pointIndex]
                    timeCurvePoint.drawTimePosition(painter, robot.points[pointIndex + 1], timePositionRelative, media.color, "bracket" if j == 0 else "pipe")
                
                i += 1
                
                
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