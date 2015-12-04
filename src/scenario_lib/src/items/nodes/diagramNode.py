#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from functools import partial
import collections

import rospkg

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.robot import Robot
from _mysql import result

class DiagramNode(object):
    currentNodeId = 0
    
    nodeName = ""
    nodeCategory = ""
    
    maxInputs = 0 # 0 for infinity
    minInputs = 0
    hasOutput = 0
    
    try:
        ui_file = os.path.join(rospkg.RosPack().get_path('execution_diagram'), 'resource', 'diagram_node.ui')
        input_button_ui_file = os.path.join(rospkg.RosPack().get_path('execution_diagram'), 'resource', 'input_button.ui')
    except Exception:
        ui_file = os.path.expanduser("~") + "/catkin_ws/src/gui_execution_diagram/resource/diagram_node.ui"
        input_button_ui_file = os.path.expanduser("~") + "/catkin_ws/src/gui_execution_diagram/resource/input_button.ui"
    
    
    def __init__(self, robotId, parent, canvas, position):
        # vars
        self.id = DiagramNode.currentNodeId
        self.masterId = -1
        DiagramNode.currentNodeId += 1
        
        self.updateCallback = None
        
        self.executing = False
        self.executingInputWidget = None
        
        self.robotId = robotId
        
        # ui
        self.canvas = canvas
        
        self.isMasterMultiRobotsDisplay = False
        
        self.dragging = False
        self.draggingOrigin = None
        self.currentInputButtonPressed = None
        self.widget = uic.loadUi(DiagramNode.ui_file)
        self.outputWidget = None
        
        self.setColors()
        
        self.widget.nodeInstance = self
        self.widget.setParent(parent)
        self.widget.show()
        
        # set opacity
        opacityEffect = QGraphicsOpacityEffect(self.widget)
        opacityEffect.setOpacity(.7)
        self.widget.setGraphicsEffect(opacityEffect)
        self.widget.setAutoFillBackground(False)
        
        # set button color styles
        colorSet = self.getColorSet(Robot.ROBOT_ID_LIST.index(robotId))
        self.originInOutputWidgetButtonCSS = "QPushButton { background: " + str(colorSet[0].name()) + "; border-radius: 7px; }"
        self.executingOutputWidgetButtonCSS = "QPushButton { background: " + str(colorSet[1].name()) + "; border-radius: 7px; }"
        
        self.widget.move(position.x(), position.y())
        self.widget.title_label.mousePressEvent = self.titleMousePressEvent
        self.widget.title_label.mouseMoveEvent = self.titleMouseMoveEvent
        self.widget.title_label.mouseReleaseEvent = self.titleMouseReleaseEvent
        
        self.widget.title_label.setText(self.__class__.nodeName)
        
        self.setTimelineValue(0)
        
        for i in range(self.__class__.minInputs):
            self.addEmptyInput()
        
        if self.__class__.hasOutput:
            self.outputWidget = uic.loadUi(DiagramNode.input_button_ui_file)
            self.outputWidget.setParent(self.widget.container_widget)
            self.outputWidget.setMaximumSize(self.outputWidget.button.width(), self.outputWidget.button.height())
            self.outputWidget.setMinimumSize(self.outputWidget.button.width(), self.outputWidget.button.height())
            self.outputWidget.show()
            
            xPosOutputWidget = self.widget.central_widget.width()
            yPosOutputWidget = self.widget.central_widget.height() / 2
            self.outputWidget.move(xPosOutputWidget, yPosOutputWidget)
            self.outputWidget.button.setStyleSheet(self.originInOutputWidgetButtonCSS)
    
    
    def getInputs(self):
        # get only enabled inputs
        inputs = [inputNode for inputNode in self.getInputsInstances() if inputNode is not None]
        
        if len(inputs) < self.__class__.minInputs:
            raise NodeException(self, u"le nombre de noeud connectés est insuffisant")
        return inputs
    
    
    def getInputWidgetIndexFromInputIndex(self, inputIndex):
        # for the difference if there are gaps between input widgets
        inputInstances = self.getInputsInstances()
        j = 0
        for i in range(len(inputInstances)):
            if inputInstances[i] is not None:
                if j == inputIndex:
                    return i
                j += 1
        
        return -1
   
   
    def refreshUI(self, args):
        # refresh self
        pass
        
        # refresh others
        for inputInstance in self.getInputsInstances():
            if inputInstance is not None:
                inputInstance.refreshUI(args)
       
    
    def setMultiRobotsDisplay(self, offsetIndex, originalNodeInstance):
        self.setColors(offsetIndex)
        
        self.widget.move(self.widget.pos().x(), self.widget.pos().y() + (offsetIndex + 1) * 3)
        
        self.widget.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.widget.setEnabled(False)
        self.widget.central_widget.setVisible(False)
        self.widget.frame.setLineWidth(0)
        self.widget.frame.setFrameStyle(QFrame.NoFrame)
        self.widget.central_widget.setVisible(False)
        self.widget.title_label.setStyleSheet("")
        self.widget.title_label.setText("")
        self.widget.frame.layout().addSpacerItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))
        
        for inputWidget in self.getInputsWidgets():
            if inputWidget.connectedToInstance is None:
                inputWidget.setVisible(False)
        
    
    def setMasterMultiRobotsDisplay(self):
        self.isMasterMultiRobotsDisplay = True
        if type(self) != PlayNode:
            self.widget.central_widget.setEnabled(False)
        self.widget.title_label.setText(self.widget.title_label.text())
        
        
    def stop(self):
        self.stopExecution()
    
    
    def startExecution(self, inputIndexExecuted):
        # set var
        self.executing = True
        
        # mark output button in yellow
        if self.outputWidget is not None:
            self.outputWidget.button.objectName = "executing"
            self.outputWidget.button.setStyleSheet(self.executingOutputWidgetButtonCSS)
            self.outputWidget.button.setEnabled(False)
        
        # mark corresponding input button in yellow
        inputWidgets = self.getInputsWidgets()
        if len(inputWidgets) > 0:
            self.executingInputWidget = inputWidgets[inputIndexExecuted]
            self.executingInputWidget.button.setStyleSheet(self.executingOutputWidgetButtonCSS)
            self.executingInputWidget.button.objectName = "executing"
            self.executingInputWidget.button.setEnabled(False)

        self.canvas.update()
    
    
    def stopExecution(self):
        # reset everything
        self.executing = False
        self.setTimelineValue(0)
        
        if self.outputWidget is not None:
            self.outputWidget.button.objectName = ""
            self.outputWidget.button.setStyleSheet(self.originInOutputWidgetButtonCSS)
            self.outputWidget.button.setEnabled(True)
        
        if self.executingInputWidget is not None and self.originInOutputWidgetButtonCSS is not None:
            self.executingInputWidget.button.objectName = ""
            self.executingInputWidget.button.setStyleSheet(self.originInOutputWidgetButtonCSS)
            self.executingInputWidget.button.setEnabled(True)
            self.executingInputWidget = None
        
        self.canvas.update()
    
    
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass

    
    def setArg(self, key, value):
        for instance in self.getInputsInstances():
            instance.args[key] = value
        
        
    def getInputsWidgets(self):
        return [child for child in self.widget.container_widget.children() if child.objectName == "input"]

    
    def getInputsInstances(self):
        return [inputWidget.connectedToInstance for inputWidget in self.getInputsWidgets()]
        
        
    def addEmptyInput(self):
        topMargin = 10
        
        inputWidget = uic.loadUi(DiagramNode.input_button_ui_file)
        inputWidget.objectName = "input"
        inputWidget.setParent(self.widget.container_widget)
        inputWidget.setMaximumSize(inputWidget.button.width(), inputWidget.button.height())
        inputWidget.setMinimumSize(inputWidget.button.width(), inputWidget.button.height())
        inputWidget.show()
        
        yPosInputWidget = self.widget.central_widget.y() + topMargin + (len(self.getInputsWidgets()) - 1) * (topMargin + inputWidget.height())
        inputWidget.move(0, yPosInputWidget)
        
        # increase height if too small
        minWidgetHeight = yPosInputWidget + inputWidget.height() + topMargin
        if minWidgetHeight > self.widget.height():
            self.widget.resize(self.widget.width(), minWidgetHeight)
        
        inputWidget.button.mousePressEvent = partial(self.inputButtonMousePressEvent, inputWidget.button)
        inputWidget.button.mouseMoveEvent = self.inputButtonMouseMoveEvent
        inputWidget.button.mouseReleaseEvent = self.inputButtonMouseReleaseEvent
        
        # set data
        inputWidget.connectedToInstance = None
    
    
    def setTimelineValue(self, value):
        if value < 0:
            value = 0
        width = value * self.widget.timelineContainer.width()
        self.widget.timeline.setMinimumWidth(width)
        self.widget.timeline.setMaximumWidth(width)
        
    
    # dragging
    def titleMousePressEvent(self, event):
        if self.isMasterMultiRobotsDisplay:
            return
        
        self.dragging = True
        self.draggingOrigin = (event.globalX() - self.widget.x(), event.globalY() - self.widget.y())
        
        
    def titleMouseMoveEvent(self, event):
        if self.isMasterMultiRobotsDisplay:
            return
        
        if self.dragging:
            moveToX, moveToY = event.globalX() - self.draggingOrigin[0], event.globalY() - self.draggingOrigin[1]
            # set limits
            if moveToX < 0: moveToX = 0
            if moveToY < 0: moveToY = 0
            
            # move it
            self.widget.move(moveToX, moveToY)
            self.canvas.update()
            self.canvas.updateBounds()
            
            
    def titleMouseReleaseEvent(self, event):
        if self.isMasterMultiRobotsDisplay:
            return
        
        self.dragging = False
        
    
    # linking
    def inputButtonMousePressEvent(self, target, event):
        if self.isMasterMultiRobotsDisplay:
            return
        
        self.currentInputButtonPressed = target
        self.currentInputButtonPressed.setDown(True)
        self.currentInputButtonPressed.parent().connectedToInstance = None
        
        self.canvas.update()
    
    
    def inputButtonMouseMoveEvent(self, event):
        if self.isMasterMultiRobotsDisplay:
            return
        
        if self.currentInputButtonPressed is not None:
            self.canvas.setCurrrentLink(self.currentInputButtonPressed, QPoint(event.globalX(), event.globalY()))
    
    
    def inputButtonMouseReleaseEvent(self, event):
        if self.isMasterMultiRobotsDisplay:
            return
        
        if self.canvas.nodeWidgetUnderMouse is not None:
            # add link
            self.currentInputButtonPressed.parent().connectedToInstance = self.canvas.nodeWidgetUnderMouse.nodeInstance
            
            # change input
            if len(self.getInputsWidgets()) < self.__class__.maxInputs:
                self.addEmptyInput()
        else:
            # cancel
            self.currentInputButtonPressed.setDown(False)
            #TODO: remove unused input
        
        # remove current link
        self.canvas.setCurrrentLink(None, None)
        self.currentInputButtonPressed = None
        
        self.canvas.update()
    
    
    def destroy(self):
        del self.widget
    
    
    def getDataFromInstance(self):
        nodeData = {}
        nodeData["id"] = self.id
        nodeData["robotId"] = Robot.DEFAULT_ROBOT_ID
        nodeData["class"] = self.__class__.__name__
        position = self.getWidgetAbsolutePosition()
        nodeData["position"] = (position.x(), position.y())
        nodeData["specificsData"] = self.getSpecificsData()
        nodeData["links"] = []
        for nodeInstanceInput in self.getInputsInstances():
            nodeData["links"].append(nodeInstanceInput.id if nodeInstanceInput is not None else None)
        
        return nodeData
                
    
    def getWidgetAbsolutePosition(self):
        return QPoint(self.widget.x(), self.widget.y()) - (self.canvas.mapToGlobal(QPoint(self.canvas.pos())) - self.canvas.mapToGlobal(QPoint()))
    
    
    def getRobotIndex(self):
        result = 0
        isResult = False
        for robotId in Robot.ROBOT_ID_LIST:
            if self.robotId == robotId:
                isResult = True
                break
            
            result += 1
        
        return result if isResult else -1
    
    
    def setColors(self, inputIndex = -1):
        inputIndex += 1
        
        colorSet = self.getColorSet(inputIndex)
        
        if inputIndex == -1:
            self.linkColor = colorSet[0]
            self.executingLinkColor = colorSet[1]
            self.linkPen = QPen(self.linkColor)
        else:
            self.linkColor = colorSet[0]
            self.executingLinkColor = colorSet[1]
            self.linkPen = QPen(self.linkColor)
        
        self.linkPen.setCapStyle(Qt.SquareCap);
        self.linkPen.setWidth(2);
        
        self.widget.timeline.setStyleSheet("background: rgb(" + str(colorSet[0].red()) + ", " + str(colorSet[0].green()) + ", " + str(colorSet[0].blue()) + ")")
    
    
    def getColorSet(self, inputIndex):
        return [Robot.getColor(inputIndex), Robot.getColor(inputIndex, 180)]
        
    
    def getNoDefaultRobotId(self):
        result = None
        if False:#self.robotId == Robot.DEFAULT_ROBOT_ID:
            if self.canvas.ui.defaultRobot_comboBox.currentIndex() >= 0:
                result = self.canvas.ui.defaultRobot_comboBox.currentText()
        else:
            result = self.robotId
        
        return result
    
    
    def changeDefaultRobot(self):
        pass
        
        
    @staticmethod
    def createInstanceFromData(canvas, nodeData):
        nodeClass = eval(nodeData["class"])
        position = QPoint(nodeData["position"][0], nodeData["position"][1]) + (canvas.mapToGlobal(QPoint(canvas.pos())) - canvas.mapToGlobal(QPoint()))
        #robotId = nodeData["robotId"]
        robotId = Robot.DEFAULT_ROBOT_ID# bypass, to take on static
        nodeInstance = nodeClass(robotId, canvas.ui.canvasContainer, canvas, position)
        nodeInstance.id = nodeData["id"]
        while len(nodeInstance.getInputsWidgets()) < len(nodeData["links"]):
            nodeInstance.addEmptyInput()
        if nodeInstance.id >= DiagramNode.currentNodeId:
            DiagramNode.currentNodeId = nodeInstance.id + 1
        nodeInstance.setSpecificsData(nodeData["specificsData"])
        
        return nodeInstance
        
        
    @staticmethod
    def linkInstanceFromData(canvas, nodeData):
        nodeInstanceToLink = canvas.getNodeInstanceById(nodeData["id"])
        linkIndex = 0
        for linkData in nodeData["links"]:
            nodeInstanceLinked = canvas.getNodeInstanceById(linkData)
            inputWidget = nodeInstanceToLink.getInputsWidgets()[linkIndex]
            DiagramNode.connectToInstance(inputWidget, nodeInstanceLinked)
            linkIndex += 1
        
    
    @staticmethod
    def connectToInstance(inputWidget, outputInstance):
        inputWidget.connectedToInstance = outputInstance
        if outputInstance is not None:
            inputWidget.button.setStyleSheet(outputInstance.originInOutputWidgetButtonCSS)


# voluntary on bottom
from src.scenario_lib.src.items.nodes.playNode import PlayNode
from src.scenario_lib.src.items.nodes.completeScenarioNode import CompleteScenarioNode
from src.scenario_lib.src.items.nodes.choregraphicScenarioNode import ChoregraphicScenarioNode
from src.scenario_lib.src.items.nodes.travelScenarioNode import TravelScenarioNode
from src.scenario_lib.src.items.nodes.sequenceNode import SequenceNode
from src.scenario_lib.src.items.nodes.randomChoiceNode import RandomChoiceNode
from src.scenario_lib.src.items.nodes.visitorNode import VisitorNode
from src.scenario_lib.src.items.nodes.obstacleNode import ObstacleNode
from src.scenario_lib.src.items.nodes.masteringNode import MasteringNode
from src.scenario_lib.src.items.nodes.batteryStateNode import BatteryStateNode
