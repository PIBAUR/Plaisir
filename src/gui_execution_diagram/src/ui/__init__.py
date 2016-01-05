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

from src.scenario_lib.src.items.nodes.playNode import PlayNode
from src.scenario_lib.src.items.robot import Robot
from canvas import Canvas


class ExecutionDiagram():
    def __init__(self, fileToOpen = None, switchToMultiRobots = False, start = False):
        self.currentFilePath = None
        self.lastChangesSaved = True
        
        # load ui
        try:
            ui_file = os.path.join(rospkg.RosPack().get_path('execution_diagram'), 'resource', 'execution_diagram.ui')
        except Exception:
            ui_file = os.path.expanduser("~") + "/catkin_ws/src/gui_execution_diagram/resource/execution_diagram.ui"
        
        self.ui = uic.loadUi(ui_file)
        
        # canvas
        self.canvas = Canvas(self.ui, self.changeCallback)
        self.ui.canvasContainer.layout().addWidget(self.canvas)
        
        self.ui.switchToMultiRobots_button.clicked.connect(partial(self.handleSwitchToMultiRobotsButtonClicked))
        self.ui.defaultRobot_comboBox.currentIndexChanged.connect(partial(self.handleDefaultRobotComboBoxChanged))
        # menu
        self.ui.actionNew.triggered.connect(self.newDiagram)
        self.ui.actionOpen.triggered.connect(self.openDiagram)
        self.ui.actionSave.triggered.connect(self.saveDiagram)
        self.ui.actionSaveAs.triggered.connect(self.saveAsDiagram)
        self.ui.actionNew.setShortcut('Ctrl+N')
        self.ui.actionOpen.setShortcut('Ctrl+O')
        self.ui.actionSave.setShortcut('Ctrl+S')
        self.ui.actionSaveAs.setShortcut('Ctrl+Shift+S')
        
        # load datas
        self.newDiagram()
        
        self.ui.resizeEvent = self.resizeEvent
        self.ui.show()
        self.resizeEvent()
        
        if fileToOpen is not None:
            self.currentFilePath = fileToOpen
            if os.path.exists(self.currentFilePath):
                diagramToOpen = self.canvas.load(self.currentFilePath)
                self.loadDiagram(diagramToOpen)
                self.lastChangesSaved = True
                self.updateWindowTitle()
            
        """nodeInstanceToPlay = None
        if start:
            for nodeInstance in self.canvas.nodesInstances:
                if type(nodeInstance) == PlayNode:
                    nodeInstanceToPlay = nodeInstance"""
        
        if switchToMultiRobots:
            self.canvas.switchToMultiRobots()
        
        """if nodeInstanceToPlay is not None:
            nodeInstanceToPlay.handlePlayButtonClicked(None)"""
            
        #self.handleDefaultRobotComboBoxChanged()
        
    
    # menu actions
    def newDiagram(self):
        #TODO: warning & stop playing
        self.canvas.load(None)
        self.lastChangesSaved = True
        self.updateWindowTitle()
        
    
    def openDiagram(self):
        # hide and show because of a bug which shows a blank qfiledialog
        self.canvas.hide()
        #TODO: warning & stop playing
        filePathToOpen = QFileDialog.getOpenFileName(self.ui, u"Ouvrir un diagramme", "", u"Diagramme d'exécution: *.dge (*.dge)")
        self.canvas.show()
        
        if filePathToOpen != "":
            self.currentFilePath = filePathToOpen 
            diagramToOpen = self.canvas.load(self.currentFilePath)
            self.loadDiagram(diagramToOpen)
            self.lastChangesSaved = True
            self.updateWindowTitle()
        
    
    def saveDiagram(self):
        if self.currentFilePath is None:
            self.saveAsDiagram()
        else:
            self.canvas.save(self.currentFilePath)
            self.lastChangesSaved = True
            self.updateWindowTitle()
        
    
    def saveAsDiagram(self):
        # hide and show because of a bug which shows a blank qfiledialog
        self.canvas.hide()
        filePathToOpen = QFileDialog.getSaveFileName(self.ui, u"Sauvegarder le scénario", "", u"Scénario: *.dge (*.dge)")
        self.canvas.show()
        
        filePathToOpen = str(filePathToOpen)
        
        if filePathToOpen != "":
            if not filePathToOpen.endswith(".dge"):
                filePathToOpen += ".dge"
            self.currentFilePath = filePathToOpen
            self.saveDiagram()
            self.lastChangesSaved = True
        
    
    def loadDiagram(self, diagram):
        self.lastChangesSaved = True
    
    
    def changeCallback(self):
        self.lastChangesSaved = False
        self.updateWindowTitle()
    
    
    def updateWindowTitle(self):
        self.ui.setWindowTitle(u"Diagramme d'exécution - " + ("*" if not self.lastChangesSaved else "") + (self.currentFilePath if self.currentFilePath is not None else u"nouveau diagramme"))
        
    
    def handleSwitchToMultiRobotsButtonClicked(self):
        self.canvas.switchToMultiRobots()
        
        
    def handleDefaultRobotComboBoxChanged(self):
        pass#for nodeInstance in self.canvas.nodesInstances:
        #    nodeInstance.changeDefaultRobot()
        
    def resizeEvent(self, event = None):
        # canvas
        absoluteCoords = self.ui.canvasContainer.mapToGlobal(QPoint(0, 0))
        absoluteCoords -= self.ui.mapToGlobal(QPoint(0, 0))
        self.canvas.setGeometry(absoluteCoords.x(), absoluteCoords.y(), self.ui.canvasContainer.width(), self.ui.canvasContainer.height())
        self.canvas.update()
        self.canvas.updateBounds()