#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import math
from functools import partial

import rospkg
import rospy

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

from src.scenario_lib.src.items.scenario import Scenario

from src.gui_scenario_builder.src.ui import ScenarioEdition
from matplotlib.testing.jpl_units.Duration import Duration

class ScenarioDataBase():
    def __init__(self):
        self.currentScenario = None
        self.currentFilePath = None
        self.lastChangesSaved = True
        
        self.acceptTableItemChanged = False
        
        try:
            ui_file = os.path.join(rospkg.RosPack().get_path('gui_scenario_db'), 'resource', 'scenario_db.ui')
            self.scenario_db_path = rospy.get_param("scenario_db_path")
        except Exception:
            ui_file = "/home/artlab/catkin_ws/src/gui_scenario_db/resource/scenario_db.ui"
            self.scenario_db_path = "/home/artlab/.notrebonplaisir/scenarios"
            
        # load ui
        self.ui = uic.loadUi(ui_file)
        
        # path
        if not os.path.exists(self.scenario_db_path):
            os.makedirs(self.scenario_db_path)
        
        # ui
        self.ui.scenario_db_table.itemChanged.connect(self.handleTableItemChanged)
        self.ui.scenario_db_table.verticalHeader().setVisible(False)
        columns = ["Nom", "Robots", u"Durée", "Comportement", u"Définitif", "Action"]
        for i in range(len(columns)):
            self.ui.scenario_db_table.insertColumn(i)
        self.ui.scenario_db_table.setHorizontalHeaderLabels(columns)
        
        self.ui.newScenario_button.clicked.connect(self.handleNewScenarioClicked)
        
        self.initTable()
        self.acceptTableItemChanged = True
        
        self.ui.show()
        
    
    
    def initTable(self):
        # get all scenarios
        scenarioFiles = os.listdir(self.scenario_db_path)
        i = 0
        for scenarioFile in scenarioFiles:
            if scenarioFile.endswith(".sce"):
                self.insertRow(i, scenarioFile)
            
            i += 1
    
    
    def insertRow(self, index, scenarioPath):
        scenarioFilePath = os.path.join(self.scenario_db_path, scenarioPath)
        scenario = Scenario.loadFile(scenarioFilePath, False)
        
        # populate table
        self.ui.scenario_db_table.insertRow(index)
        nameItem = QTableWidgetItem(str(scenario.niceName()).decode("utf-8"))
        nameItem.filePath = scenarioFilePath
        nameItem.setFlags(nameItem.flags() | Qt.ItemIsEditable)
        self.ui.scenario_db_table.setItem(index, 0, nameItem)
        
        # set them not editable
        robotsItem = QTableWidgetItem()
        robotsItem.setTextAlignment(Qt.AlignCenter)
        robotsItem.setFlags(robotsItem.flags() ^ Qt.ItemIsEditable)
        durationItem = QTableWidgetItem()
        durationItem.setFlags(durationItem.flags() ^ Qt.ItemIsEditable)
        comportementItem = QTableWidgetItem()
        comportementItem.setFlags(comportementItem.flags() ^ Qt.ItemIsEditable)
        definitifItem = QTableWidgetItem()
        definitifItem.setTextAlignment(Qt.AlignCenter)
        definitifItem.setFlags(definitifItem.flags() ^ Qt.ItemIsEditable)
        self.ui.scenario_db_table.setItem(index, 1, robotsItem)
        self.ui.scenario_db_table.setItem(index, 2, durationItem)
        self.ui.scenario_db_table.setItem(index, 3, comportementItem)
        self.ui.scenario_db_table.setItem(index, 4, definitifItem)
        
        self.setCellsForScenario(scenario, index)
        
        # add buttons for action
        actionButtonsContainer = QWidget()
        actionButtonsContainer.setLayout(QHBoxLayout())
        actionButtonsContainer.setContentsMargins(0, -10, 0, -10)
        editButton = QPushButton(u"Editer")
        editButton.clicked.connect(partial(self.handleEditButtonClicked, scenarioFilePath))
        executeButton = QPushButton(u"Exécuter")
        actionButtonsContainer.layout().addWidget(editButton)
        actionButtonsContainer.layout().addWidget(executeButton)
        self.ui.scenario_db_table.setCellWidget(index, 5, actionButtonsContainer)
    
    
    def setCellsForScenario(self, scenario, rowIndex):
        self.ui.scenario_db_table.item(rowIndex, 1).setText(str(len(scenario.robots)))
        self.ui.scenario_db_table.item(rowIndex, 2).setText(str(float(math.floor(100 * scenario.getDuration())) / 100))
        
        comportement = "-"
        if "comportement" in scenario.attributes.keys():
            try:
                comportement = scenario.attributes["comportement"]#.encode("utf-8")
            except UnicodeEncodeError:
                comportement = scenario.attributes["comportement"].encode("utf-8")
        definitif = "-"
        if "definitif" in scenario.attributes.keys():
            definitif = "X" if scenario.attributes["definitif"] else "O"
        
        try:
            self.ui.scenario_db_table.item(rowIndex, 3).setText(comportement.decode("utf-8"))
        except UnicodeEncodeError:
            self.ui.scenario_db_table.item(rowIndex, 3).setText(comportement)
            
        self.ui.scenario_db_table.item(rowIndex, 4).setText(definitif)
        
    
    def handleNewScenarioClicked(self, event):
        newName = str(self.ui.newScenario_lineEdit.text().toUtf8())
        newPath = os.path.join(self.scenario_db_path, newName + ".sce")
        if not os.path.exists(newPath) and newName != "":
            newScenario = Scenario()
            try:
                newScenario.save(newPath)
                self.acceptTableItemChanged = False
                self.insertRow(self.ui.scenario_db_table.rowCount(), newPath)
                self.acceptTableItemChanged = True
            except (OSError, IOError):
                pass
        
    
    def handleTableItemChanged(self, item):
        if self.acceptTableItemChanged:
            self.acceptTableItemChanged = False
            oldName = os.path.basename(item.filePath)[:-4]
            try:
                newName = str(item.text().toUtf8())
                newPath = item.filePath.replace(oldName, newName)
                if not os.path.exists(newPath) and newName != "":
                    os.rename(item.filePath, newPath)
                    item.filePath = newPath
                else:
                    item.setText(oldName.decode("utf-8"))
            except OSError, IOError:
                item.setText(oldName.decode("utf-8"))
            
            self.acceptTableItemChanged = True
                        
        
        
    def handleEditButtonClicked(self, scenarioFilePath):
        scenarioEdition = ScenarioEdition(scenarioFilePath, self.handleScenarioEditionSaved)
        scenarioEdition.ui.closeEvent = self.handleScenarioEditionCloseEvent
        #TODO: disable rename of opened scenario
    
    
    def handleScenarioEditionCloseEvent(self):
        #TODO: enable rename of opened scenario
        pass
        
    
    def handleScenarioEditionSaved(self, updatedFilePath, scenario):
        self.acceptTableItemChanged = False
        
        for row in range(self.ui.scenario_db_table.rowCount()):
            if updatedFilePath == self.ui.scenario_db_table.item(row, 0).filePath:
                self.setCellsForScenario(scenario, row)

        self.acceptTableItemChanged = True