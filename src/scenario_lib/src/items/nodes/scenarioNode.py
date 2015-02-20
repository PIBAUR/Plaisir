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
    nodeName = "Scenario"
    nodeCategory = ""
    
    maxInputs = 0
    minInputs = 0
    hasOutput = 1
    inputGroup = []
    
    def __init__(self, parent, canvas, position):
        super(ScenarioNode, self).__init__(parent, canvas, position)
        
        self.currentScenario = None
        
        # ui
        self.browse_button = QPushButton(u"non défini")
        self.browse_button.clicked.connect(self.handleBrowseButtonClicked)
        self.widget.central_widget.layout().addWidget(self.browse_button)
        
        # fake playing
        self.timer = QTimer(self.widget)
        #self.timer.timeout.connect(self.handleTimer)
        #self.timer.setInterval(50)
    
    
    def output(self, updateRatioCallback):
        self.updateCallback = updateRatioCallback
        
        if self.currentScenario is None:
            raise NodeException(self, u"aucun scénario n'a été chargé")
        elif not os.path.exists(self.currentScenario.filePath):
            raise NodeException(self, u"le scénario '" + self.currentScenario.niceName() + u"' n'existe plus")
        
        self.startExecution(0)
        
        # fake init
        self.currentTime = float(0)
        #self.handleTimer(True)
        #self.timer.start()
        
        # true init
        self.subscriber = rospy.Subscriber("/robot01/path_feedback", Float64, self.handlePathFeedbackReceived)
        self.handlePathFeedbackReceived(None)
        
        return self.currentScenario
    
    
    def stop(self):
        super(ScenarioNode, self).stop()
        
        self.timer.stop()
        
    
    def getSpecificsData(self):
        if self.currentScenario is not None:
            return self.currentScenario.filePath
        else:
            return None
    
    
    def setSpecificsData(self, data):
        if data is not None:
            self.openScenario(data)
    
    
    def handleTimer(self, firstTime = False):
        if not firstTime:
            self.currentTime += .0051
        if self.currentTime >= 1:
            self.currentTime = 1
            self.timer.stop()
        
        if self.currentTime >= 1:
            self.stopExecution()
            self.updateCallback(1, True)
        else:
            self.updateCallback(self.currentTime, False)
            self.setTimelineValue(self.currentTime)
        
    
    def handlePathFeedbackReceived(self, msg):
        if msg is None or math.isnan(msg.data):
            value = 0
        else:
            value = msg.data
        
        if value >= 1:
            self.subscriber.unregister()
            self.stopExecution()
            self.updateCallback(1, True)
        else:
            self.updateCallback(value, False)
            self.setTimelineValue(value)

    
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
            