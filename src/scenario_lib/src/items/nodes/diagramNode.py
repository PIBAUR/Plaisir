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

class DiagramNode(object):
    currentNodeId = 0
    
    nodeName = ""
    nodeCategory = ""
    
    maxInputs = 0 # 0 for infinity
    minInputs = 0
    hasOutput = 0
    
    executingOutputWidgetButtonCSS = "QPushButton { background: #cccc00; border-radius: 7px; }"
    
    try:
        ui_file = os.path.join(rospkg.RosPack().get_path('execution_diagram'), 'resource', 'diagram_node.ui')
        input_button_ui_file = os.path.join(rospkg.RosPack().get_path('execution_diagram'), 'resource', 'input_button.ui')
    except Exception:
        ui_file = os.path.expanduser("~") + "/catkin_ws/src/gui_execution_diagram/resource/diagram_node.ui"
        input_button_ui_file = os.path.expanduser("~") + "/catkin_ws/src/gui_execution_diagram/resource/input_button.ui"
    
    
    def __init__(self, robotId, parent, canvas, position):
        # vars
        self.id = DiagramNode.currentNodeId
        DiagramNode.currentNodeId += 1
        
        self.updateCallback = None
        
        self.executing = False
        self.executingInputWidget = None
        
        self.robotId = robotId
        
        # ui
        self.canvas = canvas
        
        self.enabled = True 
        self.dragging = False
        self.draggingOrigin = None
        self.currentInputButtonPressed = None
        self.widget = uic.loadUi(DiagramNode.ui_file)
        self.outputWidget = None
        self.originInOutputWidgetButtonCSS = None
        
        self.widget.nodeInstance = self
        self.widget.setParent(parent)
        self.widget.show()
        self.widget.move(position.x(), position.y())
        self.widget.title_label.mousePressEvent = self.titleMousePressEvent
        self.widget.title_label.mouseMoveEvent = self.titleMouseMoveEvent
        self.widget.title_label.mouseReleaseEvent = self.titleMouseReleaseEvent
        
        self.widget.enabled_checkBox.stateChanged.connect(self.handleEnabledCheckboxStateChanged)
        
        self.widget.title_label.setText(self.robotId + " - " + self.__class__.nodeName)
        
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
        
            # backup origin style of button for restoring after
            self.originInOutputWidgetButtonCSS = self.outputWidget.button.styleSheet()
    
    
    def getInputs(self):
        # get only enabled inputs
        inputs = [inputNode for inputNode in self.getInputsInstances() if inputNode is not None and inputNode.enabled]
        
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
       
    
    def stop(self):
        self.stopExecution()
    
    
    def startExecution(self, inputIndexExecuted):
        # set var
        self.executing = True
        
        # mark output button in yellow
        if self.outputWidget is not None:
            self.outputWidget.button.objectName = "executing"
            self.outputWidget.button.setStyleSheet(DiagramNode.executingOutputWidgetButtonCSS)
            self.outputWidget.button.setEnabled(False)
        
        # mark corresponding input button in yellow
        inputWidgets = self.getInputsWidgets()
        if len(inputWidgets) > 0:
            self.executingInputWidget = inputWidgets[inputIndexExecuted]
            self.executingInputWidget.button.setStyleSheet(DiagramNode.executingOutputWidgetButtonCSS)
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
        
        # backup origin style of button for restoring after if there is no output
        if self.originInOutputWidgetButtonCSS is None:
            self.originInOutputWidgetButtonCSS = inputWidget.button.styleSheet()
        
    
    def setTimelineValue(self, value):
        if value < 0:
            value = 0
        width = value * self.widget.timelineContainer.width()
        self.widget.timeline.setMinimumWidth(width)
        self.widget.timeline.setMaximumWidth(width)
        
    
    def handleEnabledCheckboxStateChanged(self, state):
        self.enabled = state
        self.widget.title_label.setEnabled(state)
        
    
    # dragging
    def titleMousePressEvent(self, event):
        self.dragging = True
        self.draggingOrigin = (event.globalX() - self.widget.x(), event.globalY() - self.widget.y())
        
        
    def titleMouseMoveEvent(self, event):
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
        self.dragging = False
        
    
    # linking
    def inputButtonMousePressEvent(self, target, event):
        self.currentInputButtonPressed = target
        self.currentInputButtonPressed.setDown(True)
        self.currentInputButtonPressed.parent().connectedToInstance = None
        
        self.canvas.update()
    
    
    def inputButtonMouseMoveEvent(self, event):
        if self.currentInputButtonPressed is not None:
            self.canvas.setCurrrentLink(self.currentInputButtonPressed, QPoint(event.globalX(), event.globalY()))
    
    
    def inputButtonMouseReleaseEvent(self, event):
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
        nodeData["class"] = self.__class__.__name__
        position = QPoint(self.widget.x(), self.widget.y()) - (self.canvas.mapToGlobal(QPoint(self.canvas.pos())) - self.canvas.mapToGlobal(QPoint()))
        nodeData["position"] = (position.x(), position.y())
        nodeData["specificsData"] = self.getSpecificsData()
        nodeData["links"] = []
        for nodeInstanceInput in self.getInputsInstances():
            nodeData["links"].append(nodeInstanceInput.id if nodeInstanceInput is not None else None)
        
        return nodeData
                
    
    @staticmethod
    def createInstanceFromData(canvas, nodeData):
        nodeClass = eval(nodeData["class"])
        position = QPoint(nodeData["position"][0], nodeData["position"][1]) + (canvas.mapToGlobal(QPoint(canvas.pos())) - canvas.mapToGlobal(QPoint()))
        nodeInstance = nodeClass(nodeData["id"], canvas.ui.canvasContainer, canvas, position)
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
            nodeInstanceToLink.getInputsWidgets()[linkIndex].connectedToInstance = nodeInstanceLinked
            linkIndex += 1


# voluntary on bottom
from src.scenario_lib.src.items.nodes.playNode import PlayNode
from src.scenario_lib.src.items.nodes.completeScenarioNode import CompleteScenarioNode
from src.scenario_lib.src.items.nodes.choregraphicScenarioNode import ChoregraphicScenarioNode
from src.scenario_lib.src.items.nodes.travelScenarioNode import TravelScenarioNode
from src.scenario_lib.src.items.nodes.sequenceNode import SequenceNode
from src.scenario_lib.src.items.nodes.randomChoiceNode import RandomChoiceNode
from src.scenario_lib.src.items.nodes.visitorNode import VisitorNode
from src.scenario_lib.src.items.nodes.obstacleNode import ObstacleNode
from src.scenario_lib.src.items.nodes.batteryStateNode import BatteryStateNode
from src.scenario_lib.src.items.nodes.pathCheckerNode import PathCheckerNode