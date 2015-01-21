#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from functools import partial
import math

import rospkg
import rospy

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

from data.robot import Robot
from data.media import Media
from data.scenario import Scenario
from src.bezier_curve.src import bezier_interpolate

from ui.scenario_edition.canvas import Canvas
from ui.scenario_edition.robotMediaPlayer import RobotMediaPlayer
from ui.scenario_edition.temporalization import Temporalization


class ScenarioEdition():
    def __init__(self):
        self.currentScenario = None
        self.currentFilePath = None
        
        # load ui
        #TODO: ui_file = os.path.join(rospkg.RosPack().get_path('gui_scenario_builder'), 'resource', 'scenario_edition.ui')
        ui_file = "/home/artlab/catkin_ws/src/gui_scenario_builder/resource/scenario_edition.ui"
        self.ui = uic.loadUi(ui_file)
        
        # robots
        self.ui.robots_list.currentItemChanged.connect(self.handleRobotsListSelectionChanged)
        
        # canvas
        self.canvas = Canvas(self.ui)
        self.ui.layout().addWidget(self.canvas)
        
        # toggle points for editing
        self.actionButtons = {self.ui.addPoint_button: Canvas.ADD_ACTION, self.ui.removePoint_button: Canvas.REMOVE_ACTION}
        
        self.toggledActionButton = self.ui.addPoint_button
        self.canvas.currentAction = self.actionButtons[self.toggledActionButton]
        self.toggledActionButton.setChecked(True)
        
        for actionButton in self.actionButtons.keys():
            actionButton.clicked.connect(partial(self.handleActionButtonClicked, actionButton))
        
        # media player
        self.robotMediaPlayer = RobotMediaPlayer(self.ui, self.canvas)
        
        # temporalization
        self.temporalization = Temporalization(self.ui, self.canvas, self.robotMediaPlayer)
        self.robotMediaPlayer.temporalization = self.temporalization
        
        # other buttons
        self.ui.showControls_button.clicked.connect(self.handleShowControlsButtonClicked)
        self.handleShowControlsButtonClicked(False)
        self.ui.breakTangent_button.clicked.connect(self.handleBreakTangentButtonClicked)
        self.handleBreakTangentButtonClicked(False)
        self.ui.mediaPositions_button.clicked.connect(self.handleMediaPositionsButtonClicked)
        self.handleMediaPositionsButtonClicked(False)
        self.ui.showMedia_button.clicked.connect(self.handleShowMediaButtonClicked)
        self.handleShowMediaButtonClicked(False)
        self.ui.addRobot_button.clicked.connect(self.handleAddRobotButtonClicked)
        
        # menu
        self.ui.actionNew.triggered.connect(self.newScenario)
        self.ui.actionOpen.triggered.connect(self.openScenario)
        self.ui.actionSave.triggered.connect(self.saveScenario)
        self.ui.actionSaveAs.triggered.connect(self.saveAsScenario)
        self.ui.actionNew.setShortcut('Ctrl+N')
        self.ui.actionOpen.setShortcut('Ctrl+O')
        self.ui.actionSave.setShortcut('Ctrl+S')
        self.ui.actionSaveAs.setShortcut('Ctrl+Shift+S')
        
        # load datas
        self.newScenario()
        
        self.ui.resizeEvent = self.resizeEvent
        self.ui.show()
        self.resizeEvent()
        
    
    def newScenario(self):
        self.loadScenario(Scenario())
        
    
    def openScenario(self):
        filePathToOpen = QFileDialog.getOpenFileName(self.ui, u"Ajouter un média", "", u"Scénario: *.sce (*.sce)")
        
        if filePathToOpen != "":
            self.currentFilePath = filePathToOpen 
            scenarioToOpen = Scenario.loadFile(self.currentFilePath)
            self.loadScenario(scenarioToOpen)
        
    
    def saveScenario(self):
        if self.currentFilePath is None:
            self.saveAsScenario()
        else:
            self.currentScenario.save(self.currentFilePath)
        
    
    def saveAsScenario(self):
        filePathToOpen = QFileDialog.getSaveFileName(self.ui, u"Sauvegarder le scénario", "", u"Scénario: *.sce (*.sce)")
        filePathToOpen = str(filePathToOpen)
            
        if filePathToOpen != "":
            if not filePathToOpen.endswith(".sce"):
                filePathToOpen += ".sce"
            self.currentFilePath = filePathToOpen
            self.saveScenario()
        
    
    def loadScenario(self, scenario):
        del self.currentScenario
        
        self.currentScenario = scenario
        self.canvas.currentRobot = self.currentScenario.robots[0]
        
        # update
        self.updateRobots()
        self.temporalization.update()
        self.canvas.update()
    
    
    def updateRobots(self):
        # get previous row
        previousSelection = self.ui.robots_list.selectedItems()
        previousSelectedRow = 0
        if len(previousSelection) > 0:
            previousSelectedRow = self.ui.robots_list.indexFromItem(previousSelection[0]).row()
        
        # limit number of robots
        self.ui.addRobot_button.setEnabled(len(self.currentScenario.robots) < 7)#TODO: rospy.get_param("num_robots"))
        # clear and rebuild everything
        self.ui.robots_list.clear()
        for i in range(len(self.currentScenario.robots)):
            robot = self.currentScenario.robots[i]
            
            itemWidget = QWidget()
            itemWidget.setLayout(QHBoxLayout())
            
            colorWidget = QWidget()
            colorWidget.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            colorWidget.setMinimumSize(15, 15)
            colorWidget.setStyleSheet("background: " + robot.color.name() + ";")
            label = QLabel("Robot " + str(i + 1))
            visibleCheckbox = QCheckBox()
            visibleCheckbox.setChecked(True)
            visibleCheckbox.stateChanged.connect(partial(self.handleVisibleCheckboxChanged, visibleCheckbox, robot))
            deleteButton = QPushButton("X")
            if len(self.currentScenario.robots) <= 1:
                deleteButton.setEnabled(False)
            deleteButton.setMaximumSize(27, 27)
            deleteButton.clicked.connect(partial(self.handleRemoveRobotButton, robot))
            itemWidget.layout().addWidget(colorWidget)
            itemWidget.layout().addWidget(label)
            itemWidget.layout().addWidget(visibleCheckbox)
            itemWidget.layout().addWidget(deleteButton)
            
            item = QListWidgetItem()
            item.setSizeHint(itemWidget.sizeHint())
            
            self.ui.robots_list.addItem(item)
            self.ui.robots_list.setItemWidget(item, itemWidget)
        
        # if not, select first item
        self.ui.robots_list.setCurrentRow(previousSelectedRow)
        
    
    def resizeEvent(self, event = None):
        # canvas
        absoluteCoords = self.ui.canvasContainer.mapToGlobal(QPoint(0, 0))
        absoluteCoords -= self.ui.mapToGlobal(QPoint(0, 0))
        self.canvas.setGeometry(absoluteCoords.x(), absoluteCoords.y(), self.ui.canvasContainer.width(), self.ui.canvasContainer.height())
        self.canvas.update()
        # temporalization
        self.ui.temporalization_widget.setMaximumWidth(self.ui.timeline_groupBox.width() - 76)
        addMediaButtonSize = self.ui.temporalization_widget.height()
        self.ui.addMedia_button.setMinimumSize(addMediaButtonSize, addMediaButtonSize)
        self.ui.addMedia_button.setMaximumSize(addMediaButtonSize, addMediaButtonSize)
        # media player ratio
        self.ui.mediaContainer_widget.setMaximumHeight(self.ui.mediaContainer_widget.width() * (16 / 10))#TODO: rospy.get_param("monitor_screen_width_ratio") / rospy.get_param("monitor_screen_height_ratio"))
        
    
    # menu
    def handleNewActionTriggered(self):
        self.newScenario()
    
    # buttons
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
            self.canvas.currentRobot = self.currentScenario.robots[index]
            self.canvas.otherRobots = [robot for robot in self.currentScenario.robots if robot.visible]
            self.canvas.update()
            
            self.temporalization.update()
        
    
    def handleAddRobotButtonClicked(self, event):
        self.currentScenario.robots.append(Robot())
        
        self.updateRobots()
    
    
    def handleVisibleCheckboxChanged(self, checkbox, robot):
        robot.visible = checkbox.isChecked()
        self.canvas.otherRobots = [robot for robot in self.currentScenario.robots if robot.visible]
        self.canvas.update()
        
        
    def handleRemoveRobotButton(self, robotToRemove):
        self.currentScenario.robots.remove(robotToRemove)
        
        self.updateRobots()
    
    
    def handleMediaPositionsButtonClicked(self, event):
        self.canvas.showTemporalization = self.ui.mediaPositions_button.isChecked()
        self.canvas.update()
        
    
    def handleShowMediaButtonClicked(self, event):
        self.canvas.showMedia = self.ui.showMedia_button.isChecked()