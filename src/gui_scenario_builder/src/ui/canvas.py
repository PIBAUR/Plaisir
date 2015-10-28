import math

import rospy

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from src.scenario_lib.src.items.curvePoint import CurvePoint
from src.scenario_lib.src.items.point import Point
from src.scenario_lib.src.items.sequence import Sequence

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
        
        self.sequences = None
        
        # get params
        self.canvasMoving = False
        self.canvasTranslateX = 0
        self.canvasTranslateY = 0
        self.canvasZoom = 10
        self.canvasMovingOriginX, self.canvasMovingOriginY = 0, 0
        
        try:
            self.mediaTimeBase = float(rospy.get_param("media_time_base"))
            self.monitorScreenWidth = rospy.get_param("monitor_screen_width")
            self.monitorScreenHeight = rospy.get_param("monitor_screen_height")
        except Exception:
            self.mediaTimeBase = 2
            self.monitorScreenWidth = 55
            self.monitorScreenHeight = 35
        
        Canvas.gridPen.setCapStyle(Qt.SquareCap)
        Canvas.gridPen.setWidth(1)
        
        Canvas.targetPen.setCapStyle(Qt.RoundCap)
        Canvas.targetPen.setWidth(Canvas.targetRadius * 2)
        Canvas.targetOutlinePen.setCapStyle(Qt.SquareCap)
        Canvas.targetOutlinePen.setWidth(1)
        
        self.updateBounds()
        
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
        
        self.drawPoints(painter, self.currentRobot, self.showControls)
        self.drawTargetPoint(painter)
    
        for otherRobot in self.otherRobots:
            if self.showTemporalization:
                self.drawSequences(painter, otherRobot)
            self.drawTimelineCursor(painter, otherRobot, self.currentTimelinePosition)
            self.drawPoints(painter, otherRobot, False)
        
        for otherRobot in self.otherRobots:
            if self.showMedia:
                if self.currentRobot == otherRobot:
                    self.drawMedia(painter, otherRobot, self.currentTimelinePosition)
                else:
                    self.drawTimelineWireframeMedia(painter, otherRobot, self.currentTimelinePosition)
            else:
                self.drawTimelineCursor(painter, otherRobot, self.currentTimelinePosition)
         
    
    def updateBounds(self):
        pass
        
        
    def mousePressEvent(self, event):
        if event.button() == 1:
            self.currentItem = None
            self.currentPoint = None
            itemUnderMouseResult = None
            
            mouseX = float(event.x())
            mouseY = float(event.y())
            
            # zoom transform
            mouseX -= self.canvasTranslateX
            mouseY -= self.canvasTranslateY
            mouseX /= self.canvasZoom
            mouseY /= self.canvasZoom
            
            # get item under the mouse (= clicked item)
            distanceTargetMouse = math.sqrt(math.pow(mouseX - self.main.currentScenario.targetPosition[0], 2) + math.pow(mouseY - self.main.currentScenario.targetPosition[1], 2))
            if (distanceTargetMouse * self.canvasZoom) <= Canvas.targetRadius:
                self.targetDragging = True
            else:
                for point in self.currentRobot.points:
                    itemUnderMouseResult = point.getItemUnderMouse(mouseX, mouseY, self.canvasZoom)
                    if itemUnderMouseResult is not None:
                        self.currentItem = itemUnderMouseResult[0]
                        self.currentPoint = itemUnderMouseResult[1]
                        break
            
            self.isEditing = self.currentAction != Canvas.REMOVE_ACTION and itemUnderMouseResult is not None
            
            if self.targetDragging:
                currentItemOffsetX = mouseX - self.main.currentScenario.targetPosition[0]
                currentItemOffsetY = mouseY - self.main.currentScenario.targetPosition[1]
                self.currentItemOffset = (currentItemOffsetX, currentItemOffsetY)
            elif self.isEditing:
                self.currentItemOffset = (mouseX - self.currentItem.x(), mouseY - self.currentItem.y())
                
                self.currentAnchorOrigins = (self.currentPoint.anchor.x(), self.currentPoint.anchor.y())
                self.currentControl1Origins = (self.currentPoint.control1.x(), self.currentPoint.control1.y())
                self.currentControl2Origins = (self.currentPoint.control2.x(), self.currentPoint.control2.y())
            elif self.currentAction == Canvas.ADD_ACTION:
                # set anchor
                self.currentPoint = CurvePoint(Point(mouseX, mouseY))
                # check if it is on the curve
                addPointAfter = -1
                for i in range(len(self.currentRobot.points) - 1):
                    point = self.currentRobot.points[i]
                    nextPoint = self.currentRobot.points[i + 1]
                    curveUPositionUnderPoint = point.isCurveUnderPoint(nextPoint, mouseX, mouseY, 5)
                    if curveUPositionUnderPoint is not None:
                        addPointAfter = i + 1
                
                # add the point
                if addPointAfter >= 0:
                    self.currentRobot.points.insert(addPointAfter, self.currentPoint)
                    
                    # offset the sequences
                    self.sequences.offsetTimesAfter(addPointAfter)
                else:
                    self.currentRobot.points.append(self.currentPoint)
                    
            elif self.currentAction == Canvas.REMOVE_ACTION:
                if self.currentItem is not None and self.currentItem == self.currentPoint.anchor:
                    self.currentRobot.points.remove(self.currentPoint)
        elif event.button() == 4:
            self.canvasMoving = True
            self.canvasMovingOriginX, self.canvasMovingOriginY = event.x(), event.y()
            
        self.update()
        
        self.updateBounds()
    
        
    def mouseReleaseEvent(self, event):
        self.targetDragging = False
        self.canvasMoving = False
        
        self.update()
        
        self.changeCallback()
        
        self.updateBounds()
        
        
    def mouseMoveEvent(self, event):
        mouseX = float(event.x())
        mouseY = float(event.y())
        
        # zoom transform
        mouseX -= self.canvasTranslateX
        mouseY -= self.canvasTranslateY
        mouseX /= self.canvasZoom
        mouseY /= self.canvasZoom
        
        if self.canvasMoving:
            self.canvasTranslateX += event.x() - self.canvasMovingOriginX
            self.canvasTranslateY += event.y() - self.canvasMovingOriginY
            self.canvasMovingOriginX = event.x()
            self.canvasMovingOriginY = event.y()
        elif self.targetDragging:
            self.main.currentScenario.targetPosition[0] = mouseX - self.currentItemOffset[0]
            self.main.currentScenario.targetPosition[1] = mouseY - self.currentItemOffset[1]
        elif self.isEditing:
            if self.currentItem is not None:
                self.currentItem.setX(mouseX - self.currentItemOffset[0])
                self.currentItem.setY(mouseY - self.currentItemOffset[1])
                # move controls with anchor
                if self.currentItem == self.currentPoint.anchor:
                    self.currentPoint.control1.setX(self.currentControl1Origins[0] + (self.currentItem.x() - self.currentAnchorOrigins[0]))
                    self.currentPoint.control1.setY(self.currentControl1Origins[1] + (self.currentItem.y() - self.currentAnchorOrigins[1]))
                    self.currentPoint.control2.setX(self.currentControl2Origins[0] + (self.currentItem.x() - self.currentAnchorOrigins[0]))
                    self.currentPoint.control2.setY(self.currentControl2Origins[1] + (self.currentItem.y() - self.currentAnchorOrigins[1]))
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
        
        self.updateBounds()
    
    
    def wheelEvent(self, event):
        self.canvasZoom += event.delta() * .01
        if self.canvasZoom > 80:
            self.canvasZoom = 80
        if self.canvasZoom < 1:
            self.canvasZoom = 1
        
    
    def getGridSize(self):
        return 10
        
    
    def drawBackground(self, painter):
        painter.fillRect(QRectF(0, 0, self.width(), self.height()), Canvas.grey)
        
        # grid
        painter.setPen(Canvas.gridPen)
        gridSize = self.getGridSize() * self.canvasZoom
        numCols = int(math.floor(self.width() / gridSize)) + 1
        for i in range(numCols):
            col = i * gridSize
            painter.drawLine(QPoint(col + self.canvasTranslateX % gridSize, 0), QPoint(col + self.canvasTranslateX % gridSize, self.height()))
        numRows = int(math.floor(self.height() / gridSize)) + 1
        for i in range(numRows):
            row = i * gridSize
            painter.drawLine(QPoint(0, row + self.canvasTranslateY % gridSize), QPoint(self.width(), row + self.canvasTranslateY % gridSize))
    
    
    def drawTargetPoint(self, painter):
        targetPoint = QPoint(self.main.currentScenario.targetPosition[0] * self.canvasZoom + self.canvasTranslateX, self.main.currentScenario.targetPosition[1] * self.canvasZoom + self.canvasTranslateY)
        
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
                point.drawCurve(painter, nextPoint, robot.color, self.canvasZoom, (self.canvasTranslateX,  + self.canvasTranslateY))
            
            if showControls:
                point.drawKnobs(painter, self.canvasZoom, (self.canvasTranslateX, self.canvasTranslateY))
                
                
    def drawSequences(self, painter, robot):
        if len(robot.points) > 1:
            i = 0
            for sequence in robot.sequences:
                pointIndex = sequence.getPositionPointIndex()
                
                if pointIndex < len(robot.points) - 1:
                    timeCurvePoint = robot.points[pointIndex]
                    timeCurvePoint.drawTimePosition(painter, robot.points[pointIndex + 1], 0, QColor(223, 103, 55) if sequence.focused else QColor(0, 0, 0), self.canvasZoom, (self.canvasTranslateX, self.canvasTranslateY), "pipe")
                
                i += 1
                
                
    def drawTimelineCursor(self, painter, robot, timePosition):
        result = self.getCurvePosition(robot, timePosition)
        
        if result is not None:
            result[0].drawTimePosition(painter, result[1], result[2], robot.color, self.canvasZoom, (self.canvasTranslateX, self.canvasTranslateY), "point")
    
    
    def drawTimelineWireframeMedia(self, painter, robot, timePosition):
        result = self.getCurvePosition(robot, timePosition)
        
        if result is not None:
            screenSize = self.getMonitorScreenSize()
            result[0].drawTimePosition(painter, result[1], result[2], robot.color, self.canvasZoom, (self.canvasTranslateX, self.canvasTranslateY), "wireframe_media", monitorScreenWidth = screenSize[0], monitorScreenHeight = screenSize[1])
    
    
    def drawMedia(self, painter, robot, timePosition):
        result = self.getCurvePosition(robot, timePosition)
        
        if result is not None:
            positionAndAngleResult = result[0].getPositionAndAngle(result[1], result[2])
            position = positionAndAngleResult[0]
            position.setX(position.x() * self.canvasZoom + self.canvasTranslateX)
            position.setY(position.y() * self.canvasZoom + self.canvasTranslateY)
            angle = 180. * positionAndAngleResult[1] / math.pi
            
            # if backward, turn over
            absoluteTimePosition = timePosition * self.sequences.temporalization.fullDuration
            if self.getSequencesIntervalOnTimePosition(robot, absoluteTimePosition)[0].backward:
                angle += 180.
            
            if self.mediaPixmap is not None:
                transform = QTransform() 
                transform.translate(position.x(), position.y())
                screenSize = [self.getMonitorScreenSize()[1], self.getMonitorScreenSize()[0]]
                transform.rotate(angle)
                transform.rotate(90)
                transform.scale(screenSize[0] / self.mediaPixmap.width(), screenSize[1] / self.mediaPixmap.height())
                transform.translate(-self.mediaPixmap.width() / 2, -self.mediaPixmap.height() / 2)
                painter.setTransform(transform)
                
                painter.drawPixmap(0, 0, self.mediaPixmap)#position.x(), position.y(), self.mediaPixmap)# - self.mediaPixmap.width() / 2, position.y() - self.mediaPixmap.height() / 2, self.mediaPixmap)
                painter.resetTransform()

    
    def getSequencesIntervalOnTimePosition(self, robot, timePosition):
        toDrawSequence = None
        toDrawNextSequence = None
        for sequenceId in range(-1, len(robot.sequences)):
            if sequenceId == -1: 
                currentSequence = Sequence(0, 0, False)
            else: 
                currentSequence = robot.sequences[sequenceId]
            
            if sequenceId == len(robot.sequences) - 1: 
                nextSequence = Sequence(robot.getDuration(), len(robot.points) - 1, False)
            else: 
                nextSequence = robot.sequences[sequenceId + 1]
            
            if timePosition >= currentSequence.timePosition and timePosition < nextSequence.timePosition:
                toDrawSequence = currentSequence
                toDrawNextSequence = nextSequence
                break
        
        return toDrawSequence, toDrawNextSequence
        
                
    def getCurvePosition(self, robot, timePosition):
        if len(robot.points) > 1 and len(robot.sequences) > 0:
            absoluteTimePosition = timePosition * self.sequences.temporalization.fullDuration
            
            # get the interval corresponding to the current time
            toDrawSequence, toDrawNextSequence = self.getSequencesIntervalOnTimePosition(robot, absoluteTimePosition)
            
            if toDrawSequence is not None and toDrawNextSequence is not None:
                pointIndex = toDrawSequence.getPositionPointIndex()
                nextPointIndex = toDrawNextSequence.getPositionPointIndex()
                
                if pointIndex < len(robot.points):
                    timeCurvePoint = robot.points[pointIndex]
                    nextTimeCurvePoint = robot.points[nextPointIndex]
                    
                    # due to a bug, cancel relative position if it the same point
                    if timeCurvePoint.anchor.isEqual(nextTimeCurvePoint.anchor):
                        timePositionRelative = 0
                    else:
                        timePositionRelative = (absoluteTimePosition - toDrawSequence.timePosition) / (toDrawNextSequence.timePosition - toDrawSequence.timePosition)
                    
                    # calculate intermediate points
                    deltaPointIndex = nextPointIndex - pointIndex
                    if deltaPointIndex > 1:
                        offsetPointIndex = int(math.floor(deltaPointIndex * timePositionRelative))
                        timeCurvePoint = robot.points[pointIndex + offsetPointIndex]
                        nextTimeCurvePoint = robot.points[pointIndex + offsetPointIndex + 1]
                        timePositionRelative = (timePositionRelative * deltaPointIndex) % 1
                    
                    return timeCurvePoint, nextTimeCurvePoint, timePositionRelative
    
        return None
    
    
    def getMonitorScreenSize(self):
        return ((self.monitorScreenWidth / 100.) * self.getGridSize() * self.canvasZoom, (self.monitorScreenHeight / 100.) * self.getGridSize() * self.canvasZoom)