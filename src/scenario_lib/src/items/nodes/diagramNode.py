import os
from functools import partial

import rospkg

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

class DiagramNode(object):
    nodeName = ""
    nodeCategory = ""
    
    maxInputs = 0 # 0 for infinity
    minInputs = 0
    hasOutput = 0
    inputGroup = None # for example, an if condition has got a group: ["valueToTest", "resultIfIsTrue"]
    
    try:
        ui_file = os.path.join(rospkg.RosPack().get_path('execution_diagram'), 'resource', 'diagram_node.ui')
        input_button_ui_file = os.path.join(rospkg.RosPack().get_path('execution_diagram'), 'resource', 'input_button.ui')
    except Exception:
        ui_file = "/home/artlab/catkin_ws/src/gui_execution_diagram/resource/diagram_node.ui"
        input_button_ui_file = "/home/artlab/catkin_ws/src/gui_execution_diagram/resource/input_button.ui"
    
    def __init__(self, parent, canvas, position):
        # vars
        self.inputs = {}
        
        # ui
        self.canvas = canvas
        
        self.dragging = False
        self.draggingOrigin = None
        self.currentInputButtonPressed = None
        self.widget = uic.loadUi(DiagramNode.ui_file)
        self.outputWidget = None
        
        self.widget.nodeInstance = self
        self.widget.setParent(parent)
        self.widget.show()
        self.widget.move(position.x() - self.widget.width() / 2, position.y() - 10)
        self.widget.title_label.mousePressEvent = self.titleMousePressEvent
        self.widget.title_label.mouseMoveEvent = self.titleMouseMoveEvent
        self.widget.title_label.mouseReleaseEvent = self.titleMouseReleaseEvent
        
        self.widget.title_label.setText(self.__class__.nodeName)
        
        numInputs = self.__class__.minInputs
        topMargin = 10
        for inputIndex in range(numInputs):
            inputWidget = uic.loadUi(DiagramNode.input_button_ui_file)
            inputWidget.setParent(self.widget.container_widget)
            inputWidget.show()
            
            yPosInputWidget = self.widget.central_widget.y() + topMargin + inputIndex * (topMargin + inputWidget.height())
            inputWidget.move(0, yPosInputWidget)
            
            inputWidget.button.mousePressEvent = partial(self.inputButtonMousePressEvent, inputWidget.button)
            inputWidget.button.mouseMoveEvent = self.inputButtonMouseMoveEvent
            inputWidget.button.mouseReleaseEvent = self.inputButtonMouseReleaseEvent
            
            # set data
            self.inputs[inputWidget] = None
        
        if self.__class__.hasOutput:
            self.outputWidget = uic.loadUi(DiagramNode.input_button_ui_file)
            self.outputWidget.setParent(self.widget.container_widget)
            self.outputWidget.label.setText("")
            self.outputWidget.show()
            
            xPosOutputWidget = self.widget.central_widget.width()
            yPosOutputWidget = self.widget.central_widget.height() / 2
            self.outputWidget.move(xPosOutputWidget, yPosOutputWidget)
    
    
    def output(self):
        #TODO: check for basics error
        
        return None
    
    
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
        self.canvas.deleteDefinitiveLinkFromInput(self.currentInputButtonPressed)
    
    
    def inputButtonMouseMoveEvent(self, event):
        if self.currentInputButtonPressed is not None:
            self.canvas.setCurrrentLink(self.currentInputButtonPressed, QPoint(event.globalX(), event.globalY()))
    
    
    def inputButtonMouseReleaseEvent(self, event):
        if self.canvas.nodeWidgetUnderMouse is not None:
            self.inputs[self.currentInputButtonPressed.parent()] = self.canvas.nodeWidgetUnderMouse.nodeInstance
            self.canvas.addDefinitiveLink(self.currentInputButtonPressed, self.canvas.nodeWidgetUnderMouse.nodeInstance)
        else:
            self.currentInputButtonPressed.setDown(False)
        
        self.canvas.setCurrrentLink(None, None)
        self.currentInputButtonPressed = None
    
    