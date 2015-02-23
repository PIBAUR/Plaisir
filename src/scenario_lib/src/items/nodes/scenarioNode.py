#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *

import rospy

from std_msgs.msg import Float64
from scenario_msgs.msg import Scenario as ScenarioMsg

from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode
from src.scenario_lib.src.items.nodes.nodeException import NodeException
from src.scenario_lib.src.items.scenario import Scenario
from src.gui_scenario_db.src.ui import ScenarioDataBase


class ScenarioNode(DiagramNode):
    uid = 0
    
    nodeName = "Scenario"
    nodeCategory = ""
    
    maxInputs = 0
    minInputs = 0
    hasOutput = 1
    inputGroup = []
    
    def __init__(self, parent, canvas, position):
        super(ScenarioNode, self).__init__(parent, canvas, position)
        
        self.uid = ScenarioNode.uid
        self.scenarioRunningOnRobotUid = -1
        self.currentScenario = None
        self.pathFeedbackValue = 0.
        self.pathFeedbackSubscriber = None
        
        # ui
        self.browse_button = QPushButton(u"non défini")
        self.browse_button.clicked.connect(self.handleBrowseButtonClicked)
        self.widget.central_widget.layout().addWidget(self.browse_button)
        
        # set timer for update graphics, because they must not be executed by the ROS callback thread
        self.timer = QTimer(self.widget)
        self.timer.timeout.connect(self.handleTimer)
        self.timer.setInterval(10)
        self.timer.start()
        
        # init subscription on the current running scenario
        self.scenarioSubscriber = rospy.Subscriber("/robot01/scenario", ScenarioMsg, self.handleScenarioReceived)
        
        ScenarioNode.uid += 1
    
    
    def output(self, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        if self.currentScenario is None:
            raise NodeException(self, u"aucun scénario n'a été chargé")
        elif not os.path.exists(self.currentScenario.filePath):
            raise NodeException(self, u"le scénario '" + self.currentScenario.niceName() + u"' n'existe plus")
        
        self.currentScenario.uid = self.uid
        self.startExecution(0)
        
        # init subscription on the path feedback
        self.pathFeedbackSubscriber = rospy.Subscriber("/robot01/path_feedback", Float64, self.handlePathFeedbackReceived)
        self.handlePathFeedbackReceived(None)
        
        return self.currentScenario
    
    
    def stop(self):
        super(ScenarioNode, self).stop()
        
        if self.pathFeedbackSubscriber is not None:
            self.pathFeedbackSubscriber.unregister()
        
    
    def getSpecificsData(self):
        if self.currentScenario is not None:
            return self.currentScenario.filePath
        else:
            return None
    
    
    def setSpecificsData(self, data):
        if data is not None:
            self.openScenario(data)
    
    
    def handleTimer(self, firstTime = False):
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
        
    
    def handleScenarioReceived(self, msg):
        if msg is not None:
            self.scenarioRunningOnRobotUid = msg.uid
        
        
    def handlePathFeedbackReceived(self, msg):
        if msg is None or math.isnan(msg.data):
            self.pathFeedbackValue = 0
        else:
            # check that the feedback comes from the right node
            if self.scenarioRunningOnRobotUid == self.uid:
                self.pathFeedbackValue = msg.data
                
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
        