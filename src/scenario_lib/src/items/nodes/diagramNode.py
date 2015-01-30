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
    inputGroup = [] # for example, an if condition has got a group: ["valueToTest", "resultIfIsTrue"]
    
    try:
        ui_file = os.path.join(rospkg.RosPack().get_path('execution_diagram'), 'resource', 'diagram_node.ui')
        input_button_ui_file = os.path.join(rospkg.RosPack().get_path('execution_diagram'), 'resource', 'input_button.ui')
    except Exception:
        ui_file = "/home/artlab/catkin_ws/src/gui_execution_diagram/resource/diagram_node.ui"
        input_button_ui_file = "/home/artlab/catkin_ws/src/gui_execution_diagram/resource/input_button.ui"
    
    def __init__(self, parent, canvas, position):
        # vars
        self.id = DiagramNode.currentNodeId
        DiagramNode.currentNodeId += 1
        
        self.updateCallback = None
        
        # ui
        self.canvas = canvas
        
        self.enabled = True 
        self.dragging = False
        self.draggingOrigin = None
        self.currentInputButtonPressed = None
        self.widget = uic.loadUi(DiagramNode.ui_file)
        self.outputWidget = None
        
        self.widget.nodeInstance = self
        self.widget.setParent(parent)
        self.widget.show()
        self.widget.move(position.x(), position.y())
        self.widget.title_label.mousePressEvent = self.titleMousePressEvent
        self.widget.title_label.mouseMoveEvent = self.titleMouseMoveEvent
        self.widget.title_label.mouseReleaseEvent = self.titleMouseReleaseEvent
        
        self.widget.enabled_checkBox.stateChanged.connect(self.handleEnabledCheckboxStateChanged)
        
        self.widget.title_label.setText(self.__class__.nodeName)
        
        self.setTimelineValue(0)
        
        for i in range(self.__class__.minInputs):
            self.addEmptyInput()
        
        if self.__class__.hasOutput:
            self.outputWidget = uic.loadUi(DiagramNode.input_button_ui_file)
            self.outputWidget.setParent(self.widget.container_widget)
            self.outputWidget.label.setText("")
            self.outputWidget.label.setAttribute(Qt.WA_TransparentForMouseEvents)
            self.outputWidget.show()
            
            xPosOutputWidget = self.widget.central_widget.width()
            yPosOutputWidget = self.widget.central_widget.height() / 2
            self.outputWidget.move(xPosOutputWidget, yPosOutputWidget)
            
    
    
    def output(self):
        # get only enabled inputs
        inputs = [inputNode for inputNode in self.getInputsInstances() if inputNode is not None and inputNode.enabled]
        
        if len(inputs) < self.__class__.minInputs:
            raise NodeException(u"le nombre de noeud connectés est insuffisant")
        return inputs
    
    
    def getSpecificsData(self):
        return None
    
    
    def setSpecificsData(self, data):
        pass

    
    def getInputsWidgets(self):
        return [child for child in self.widget.container_widget.children() if child.objectName == "input"]

    
    def getInputsInstances(self):
        return [inputWidget.connectedToInstance for inputWidget in self.getInputsWidgets()]
        
        
    def addEmptyInput(self):
        topMargin = 10
        
        for inputName in self.__class__.inputGroup:
            inputWidget = uic.loadUi(DiagramNode.input_button_ui_file)
            inputWidget.objectName = "input"
            inputWidget.setParent(self.widget.container_widget)
            inputWidget.label.setText(inputName)
            #inputWidget.setAttribute(Qt.WA_TransparentForMouseEvents)
            #inputWidget.setAttribute(Qt.WA_TranslucentBackground)
            #inputWidget.label.setAttribute(Qt.WA_TransparentForMouseEvents)
            #inputWidget.label.setAttribute(Qt.WA_TranslucentBackground)
            inputWidget.show()
            
            yPosInputWidget = self.widget.central_widget.y() + topMargin + (len(self.getInputsWidgets()) - 1) * (topMargin + inputWidget.height())
            inputWidget.move(0, yPosInputWidget)
            
            inputWidget.button.mousePressEvent = partial(self.inputButtonMousePressEvent, inputWidget.button)
            inputWidget.button.mouseMoveEvent = self.inputButtonMouseMoveEvent
            inputWidget.button.mouseReleaseEvent = self.inputButtonMouseReleaseEvent
            
            # set data
            inputWidget.connectedToInstance = None
    
    
    def setTimelineValue(self, value):
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
            self.widget.move(event.globalX() - self.draggingOrigin[0], event.globalY() - self.draggingOrigin[1])
            self.canvas.update()
            
            
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
            if len(self.getInputsWidgets()) < self.__class__.maxInputs * len(self.__class__.inputGroup):
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
        nodeInstance = nodeClass(canvas.ui, canvas, position)
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


from src.scenario_lib.src.items.nodes.playNode import PlayNode
from src.scenario_lib.src.items.nodes.scenarioNode import ScenarioNode
from src.scenario_lib.src.items.nodes.sequenceNode import SequenceNode