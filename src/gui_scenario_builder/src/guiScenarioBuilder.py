import os
from functools import partial

import rospkg

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

from canvas import Canvas
        
class GuiScenarioBuilder(QMainWindow):
    def __init__(self):
        super(QMainWindow, self).__init__()
        
        # load ui
        #ui_file = os.path.join(rospkg.RosPack().get_path('gui_scenario_builder'), 'resource', 'gui_scenario_builder.ui')
        ui_file = "/home/artlab/catkin_ws/src/gui_scenario_builder/resource/gui_scenario_builder.ui"
        
        self.ui = uic.loadUi(ui_file)
        self.ui.show()
        
        # canvas
        absoluteCoords = self.ui.canvasContainer.mapToGlobal(self.ui.pos())
        self.canvas = Canvas(absoluteCoords.x(), absoluteCoords.y(), self.ui.canvasContainer.width(), self.ui.canvasContainer.height())
        self.ui.layout().addWidget(self.canvas)
        
        # toggle points for editing
        self.actionButtons = {self.ui.addPoint_button: Canvas.ADD_ACTION,
                         self.ui.removePoint_button: Canvas.REMOVE_ACTION,
                         }
        
        self.toggledActionButton = self.ui.addPoint_button
        self.canvas.currentAction = self.actionButtons[self.toggledActionButton]
        self.toggledActionButton.setChecked(True)
        
        for actionButton in self.actionButtons.keys():
            actionButton.clicked.connect(partial(self.handleActionButtonClicked, actionButton))
            
        # other buttons
        self.ui.showControls_button.clicked.connect(self.handleShowControlsButtonClicked)
        self.handleShowControlsButtonClicked(False)
        self.ui.breakTangent_button.clicked.connect(self.handleBreakTangentButtonClicked)
        self.handleBreakTangentButtonClicked(False)
        
    
    def handleActionButtonClicked(self, button):
        if button.isChecked():
            for actionButton in self.actionButtons.keys():
                if actionButton != button:
                    actionButton.setChecked(False)
        
            self.canvas.currentAction = self.actionButtons[button]
        else:
            self.canvas.currentAction = -1
        
    
    def handleShowControlsButtonClicked(self, event):
        self.canvas.showControls = self.ui.showControls_button.isChecked()
        self.canvas.update()
        
        
    def handleBreakTangentButtonClicked(self, event):
        self.canvas.breakTangent = self.ui.breakTangent_button.isChecked()
