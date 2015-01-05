import os
from functools import partial

import rospkg
import rospy

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

from canvas import Canvas
from robot import Robot
        
class GuiScenarioBuilder(QMainWindow):
    def __init__(self):
        super(QMainWindow, self).__init__()
        
        # load ui
        #TODO: ui_file = os.path.join(rospkg.RosPack().get_path('gui_scenario_builder'), 'resource', 'gui_scenario_builder.ui')
        ui_file = "/home/artlab/catkin_ws/src/gui_scenario_builder/resource/gui_scenario_builder.ui"
        
        self.ui = uic.loadUi(ui_file)
        self.ui.show()
        
        # robots
        self.robots = [Robot()]
        self.currentRobot = None
        self.ui.robots_list.currentItemChanged.connect(self.handleRobotsListSelectionChanged)
        
        # canvas
        absoluteCoords = self.ui.canvasContainer.mapToGlobal(self.ui.pos())
        self.canvas = Canvas(self.save, absoluteCoords.x(), absoluteCoords.y(), self.ui.canvasContainer.width(), self.ui.canvasContainer.height())
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
        
        self.ui.addRobot_button.clicked.connect(self.handleAddRobotButtonClicked)
        self.updateRobots()
        
    
    def save(self):
        self.currentRobot.points = self.canvas.points
    
    
    def updateRobots(self):
        # get previous row
        previousSelection = self.ui.robots_list.selectedItems()
        previousSelectedRow = 0
        if len(previousSelection) > 0:
            previousSelectedRow = self.ui.robots_list.indexFromItem(previousSelection[0]).row()
        
        # limit number of robots
        self.ui.addRobot_button.setEnabled(len(self.robots) < 7)#TODO: rospy.get_param("num_robots"))
        # clear and rebuild everything
        self.ui.robots_list.clear()
        for i in range(len(self.robots)):
            itemWidget = QWidget()
            itemWidget.setLayout(QHBoxLayout())
            
            label = QLabel("Robot " + str(i + 1))
            deleteButton = QPushButton("X")
            deleteButton.setMaximumSize(27, 27)
            deleteButton.clicked.connect(partial(self.handleRemoveRobotButton, self.robots[i]))
            itemWidget.layout().addWidget(label)
            itemWidget.layout().addWidget(deleteButton)
            
            item = QListWidgetItem()
            item.setSizeHint(itemWidget.sizeHint())
            
            
            self.ui.robots_list.addItem(item)
            self.ui.robots_list.setItemWidget(item, itemWidget)
        
        # if not, select first item
        self.ui.robots_list.setCurrentRow(previousSelectedRow)
    
    
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
    
    
    def handleRobotsListSelectionChanged(self, item):
        index = self.ui.robots_list.indexFromItem(item).row()
        if index >= 0:
            self.currentRobot = self.robots[index]
            self.canvas.points = self.currentRobot.points
            self.canvas.update()
        
    
    def handleAddRobotButtonClicked(self, event):
        self.robots.append(Robot())
        
        self.updateRobots()
    
    
    def handleRemoveRobotButton(self, robotToRemove):
        self.robots.remove(robotToRemove)
        
        self.updateRobots()