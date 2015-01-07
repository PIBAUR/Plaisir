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

from canvas import Canvas
from robot import Robot
from video import Video
from src.bezier_curve.src import bezier_interpolate
        
class GuiScenarioBuilder(QMainWindow):
    def __init__(self):
        super(QMainWindow, self).__init__()
        
        # load ui
        #TODO: ui_file = os.path.join(rospkg.RosPack().get_path('gui_scenario_builder'), 'resource', 'gui_scenario_builder.ui')
        ui_file = "/home/artlab/catkin_ws/src/gui_scenario_builder/resource/gui_scenario_builder.ui"
        
        self.ui = uic.loadUi(ui_file)
        self.ui.show()
        
        self.lastVideoDirectory = ""
        
        # robots
        self.robots = [Robot()]
        self.ui.robots_list.currentItemChanged.connect(self.handleRobotsListSelectionChanged)
        
        # canvas
        absoluteCoords = self.ui.canvasContainer.mapToGlobal(self.ui.pos())
        self.canvas = Canvas(self.save, absoluteCoords.x(), absoluteCoords.y(), self.ui.canvasContainer.width(), self.ui.canvasContainer.height())
        self.canvas.currentRobot = self.robots[0]
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
        
        # temporalization
        self.temporalizationSplitter = None
        self.updateTemporalization()
        
        # other buttons
        self.ui.showControls_button.clicked.connect(self.handleShowControlsButtonClicked)
        self.handleShowControlsButtonClicked(False)
        self.ui.breakTangent_button.clicked.connect(self.handleBreakTangentButtonClicked)
        self.handleBreakTangentButtonClicked(False)
        self.ui.videoPositions_button.clicked.connect(self.handleVideoPositionsButtonClicked)
        self.handleVideoPositionsButtonClicked(False)
        self.ui.addVideo_button.clicked.connect(self.handleAddVideoButtonClicked)
        self.ui.addRobot_button.clicked.connect(self.handleAddRobotButtonClicked)
        self.updateRobots()
        
        
    def save(self):
        pass
    
    
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
            robot = self.robots[i]
            
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
            if len(self.robots) <= 1:
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
    
    
    def updateTemporalization(self):
        # delete previous one
        if self.temporalizationSplitter is not None:
            for child in self.temporalizationSplitter.children():
                child.hide()
                del child
            
            
            self.ui.temporalization_widget.layout().removeWidget(self.temporalizationSplitter)
            del self.temporalizationSplitter
        
        # init a new one
        self.temporalizationSplitter = QSplitter(Qt.Horizontal)
        self.temporalizationSplitter.splitterMoved.connect(self.handleTemporalizationSplitterMoved)
        self.temporalizationSplitter.setChildrenCollapsible(False)
        self.ui.temporalization_widget.layout().addWidget(self.temporalizationSplitter)
        
        # add buttons
        videoSizes = []
        handleWidth = self.temporalizationSplitter.handleWidth()
        splitterWidth = self.temporalizationSplitter.width() - handleWidth * (len(self.canvas.currentRobot.videos) - 1)
        i = 0
        for video in self.canvas.currentRobot.videos:
            videoButton = QPushButton(video.niceName)
            videoButton.setStyleSheet("background: " + video.color.name() + "; text-align: left;")
            videoButton.setMinimumWidth(1)
            
            self.temporalizationSplitter.addWidget(videoButton)
            
            # icon
            videoButton.setIcon(video.thumbnailIcon);
            iconHeight = self.ui.temporalization_widget.height() * 1.55
            videoButton.setIconSize(QSize(iconHeight / video.thumbnailRatio, iconHeight))
            
            # get size for the end
            videoSize = (video.endTime - video.startTime) * splitterWidth
            if len(self.canvas.currentRobot.videos) > 1:
                videoSize -= handleWidth / (2 if (i == 0 or i == len(videoSizes) - 1) else 1)
            
            videoSizes.append(videoSize)
            
            i += 1
        
        # set good videoSizes
        self.temporalizationSplitter.setSizes(videoSizes)
    
    
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
            self.canvas.currentRobot = self.robots[index]
            self.canvas.otherRobots = [robot for robot in self.robots if robot.visible]
            self.canvas.update()
            
            self.updateTemporalization()
        
    
    def handleAddRobotButtonClicked(self, event):
        self.robots.append(Robot())
        
        self.updateRobots()
    
    
    def handleVisibleCheckboxChanged(self, checkbox, robot):
        robot.visible = checkbox.isChecked()
        self.canvas.otherRobots = [robot for robot in self.robots if robot.visible]
        self.canvas.update()
        
        
    def handleRemoveRobotButton(self, robotToRemove):
        self.robots.remove(robotToRemove)
        
        self.updateRobots()
    
    
    def handleVideoPositionsButtonClicked(self, event):
        self.canvas.showTemporization = self.ui.videoPositions_button.isChecked()
        self.canvas.update()
    
    
    def handleAddVideoButtonClicked(self, event):
        # get a file
        filePaths = QFileDialog.getOpenFileNames(self, u"Ajouter une vidéo", self.lastVideoDirectory, u"Vidéos: *.mp4, *.mov (*.mov *.mp4)")
        
        for filePath in filePaths:
            filePath = str(filePath)
            self.lastVideoDirectory = os.path.dirname(filePath)
            
            # create video
            newVideo = Video(filePath)
            
            # set time to the middle of the last one
            newVideo.endTime = 1.0
            if len(self.canvas.currentRobot.videos) == 0:
                newVideo.startTime = 0.0
            else:
                lastVideo = self.canvas.currentRobot.videos[-1]
                lastVideo.endTime = float(lastVideo.startTime + (lastVideo.endTime - lastVideo.startTime) / 2)
                newVideo.startTime = float(lastVideo.endTime)
                
            self.canvas.currentRobot.videos.append(newVideo)
        
        self.updateTemporalization()
        
        self.temporalizationSplitter.refresh()
        self.handleTemporalizationSplitterMoved(-1, -1)
        
    
    def handleTemporalizationSplitterMoved(self, position, index):
        sizes = self.temporalizationSplitter.sizes()
        splitterWidth = self.temporalizationSplitter.width()
        handleWidth = self.temporalizationSplitter.handleWidth()
        previousPosition = 0
        i = 0
        for size in sizes:
            withHandleSize = size
            # add the handle width
            if len(sizes) > 1:
                withHandleSize += handleWidth / (2 if (i == 0 or i == len(sizes) - 1) else 1)
            
            startPosition = previousPosition
            endPosition = startPosition + withHandleSize
            
            startTime = float(startPosition) / float(splitterWidth)
            endTime = float(endPosition) / float(splitterWidth)
            
            self.canvas.currentRobot.videos[i].startTime = startTime
            self.canvas.currentRobot.videos[i].endTime = endTime
            
            previousPosition += withHandleSize
            i += 1
        
        # display tooltip
        if position >= 0:
            percent = math.floor(self.canvas.currentRobot.videos[index].startTime * 1000) / 10
            QToolTip.showText(QCursor.pos(), str(percent) + " %")
        
        self.canvas.update()