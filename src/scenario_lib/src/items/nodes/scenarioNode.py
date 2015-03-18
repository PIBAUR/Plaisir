#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import math
import time

from PyQt4.QtCore import *
from PyQt4.QtGui import *

import rospy

from scenario_msgs.msg import PathFeedback as PathFeedbackMsg

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.scenario import Scenario
from src.gui_scenario_db.src.ui import ScenarioDataBase


class ScenarioNode(DiagramNode):
    nodeName = "Scenario"
    nodeCategory = ""
    
    maxInputs = 0
    minInputs = 0
    hasOutput = 1
    
    uid = 0
    
    def __init__(self, parent, canvas, position):
        super(ScenarioNode, self).__init__(parent, canvas, position)
        
        self.scenarioRunningOnRobotUid = -1
        self.currentScenario = None
        self.pathFeedbackValue = 0.
        self.pathFeedbackSubscriber = None
        
        # ui
        self.browse_button = QPushButton(u"non défini")
        self.browse_button.clicked.connect(self.handleBrowseButtonClicked)
        self.widget.central_widget.layout().addWidget(self.browse_button)
        
        # set uiTimer for update graphics, because they must not be executed by the ROS callback thread
        self.uiTimer = QTimer(self.widget)
        self.uiTimer.timeout.connect(self.handleUITimer)
        self.uiTimer.setInterval(10)
        self.uiTimer.start()
        
        # set uiTimer to simulate
        self.simulationTimer = QTimer(self.widget)
        self.simulationTimer.timeout.connect(self.handleSimulationTimer)
        self.simulationTimer.setInterval(100)
    
    
    def output(self, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        if self.currentScenario is None:
            raise NodeException(self, u"aucun scénario n'a été chargé")
        elif not os.path.exists(self.currentScenario.filePath):
            raise NodeException(self, u"le scénario '" + self.currentScenario.niceName() + u"' n'existe plus")
        
        self.currentScenario.uid = ScenarioNode.uid
        self.scenarioRunningOnRobotUid = self.currentScenario.uid
        ScenarioNode.uid += 1
        self.startExecution(0)
        
        # init subscription on the path feedback
        if False:
            self.pathFeedbackSubscriber = rospy.Subscriber("/robot01/path_feedback", PathFeedbackMsg, self.handlePathFeedbackReceived)
            self.handlePathFeedbackReceived(None)
        else:
            self.simulationTimer.start()
            self.handleSimulationTimer()
        
        return self.currentScenario
    
    
    def stop(self):
        super(ScenarioNode, self).stop()
        
        if False:
            if self.pathFeedbackSubscriber is not None:
                self.pathFeedbackSubscriber.unregister()
        else:
            self.simulationTimer.stop()
        
    
    def getSpecificsData(self):
        if self.currentScenario is not None:
            return self.currentScenario.filePath
        else:
            return None
    
    
    def setSpecificsData(self, data):
        if data is not None:
            self.openScenario(data)
    
    
    def handleUITimer(self, firstTime = False):
        self.browse_button.setEnabled(not self.executing)
        
        if self.executing:
            if self.pathFeedbackValue >= 1:
                self.stopExecution()
                self.updateCallback(1, True)
                self.pathFeedbackValue = 0
            else:
                if self.updateCallback is not None:
                    self.updateCallback(self.pathFeedbackValue, False)
                self.setTimelineValue(self.pathFeedbackValue)
        
        
    def handleSimulationTimer(self):
        print self
        if self.pathFeedbackValue >= 1:
            self.simulationTimer.stop()
            print "prout"
        else:
            self.pathFeedbackValue += .05
    
    
    def handlePathFeedbackReceived(self, msg):
        if msg is None or math.isnan(msg.ratio):
            self.pathFeedbackValue = 0
        else:
            # check that the feedback comes from the right node
            if self.scenarioRunningOnRobotUid == msg.uid:
                self.pathFeedbackValue = msg.ratio
                
                if self.pathFeedbackValue >= 1:
                    self.pathFeedbackSubscriber.unregister()

    
    def handleBrowseButtonClicked(self, event):
        ScenarioDataBase(self.handleBrowseScenarioEnded)
    
    
    def handleBrowseScenarioEnded(self, filePathToOpen):
        if filePathToOpen is not None:
            self.openScenario(filePathToOpen)
        
        
    def openScenario(self, filePath):
        # set scenario
        self.currentScenario = Scenario.loadFile(filePath, False)
        self.currentScenario.filePath = filePath
        
        # change display
        self.browse_button.setText(self.currentScenario.name.decode("utf-8"))
        