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
        self.editingScenarios = []
        
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
        columns = [("Nom", 160), ("Robots", 80), (u"Durée", 80), ("Comportement", 160), (u"Définitif", 80), ("Action", 80)]
        for i in range(len(columns)):
            self.ui.scenario_db_table.insertColumn(i)
            self.ui.scenario_db_table.setColumnWidth(i, columns[i][1])
        self.ui.scenario_db_table.setHorizontalHeaderLabels([column[0] for column in columns])
        self.ui.scenario_db_table.horizontalHeader().setStretchLastSection(True)
        
        self.ui.newScenario_button.clicked.connect(self.handleNewScenarioClicked)
        
        self.initTable()
        self.acceptTableItemChanged = True
        
        self.ui.show()
    
    
    def initTable(self):
        # insert the first row for filtering
        self.ui.scenario_db_table.insertRow(0)
        self.nameFilterLineEdit = QLineEdit()
        self.comportementFilterComboBox = QComboBox()
        self.comportementFilterComboBox.addItems(["Tous", "Calme", "Furieux", u"Enervé"])
        self.definitifFilterCheckBox = QCheckBox()
        self.definitifFilterCheckBox.setTristate(True)
        
        definitifCheckBoxContainer = QWidget()
        definitifCheckBoxContainer.setContentsMargins(0, -10, 0, -10)
        definitifCheckBoxLayout = QHBoxLayout()
        definitifCheckBoxLayout.setAlignment(Qt.AlignCenter)
        definitifCheckBoxLayout.addWidget(self.definitifFilterCheckBox)
        definitifCheckBoxContainer.setLayout(definitifCheckBoxLayout)
        
        self.ui.scenario_db_table.setCellWidget(0, 0, self.nameFilterLineEdit)
        self.ui.scenario_db_table.setCellWidget(0, 3, self.comportementFilterComboBox)
        self.ui.scenario_db_table.setCellWidget(0, 4, definitifCheckBoxContainer)
        
        self.nameFilterLineEdit.textChanged.connect(self.populateTable)
        self.comportementFilterComboBox.currentIndexChanged.connect(self.populateTable)
        self.definitifFilterCheckBox.stateChanged.connect(self.populateTable)
        
        self.populateTable()
    
    
    def populateTable(self, event = None):
        self.acceptTableItemChanged = False
        
        # clear rows except first one with filter
        for i in range(1, self.ui.scenario_db_table.rowCount()):
            self.ui.scenario_db_table.removeRow(1)
        
        # get all scenarios
        scenarioFiles = os.listdir(self.scenario_db_path)
        scenarioFiles.sort()
        
        # get value for filtering after
        nameFilterValue = self.nameFilterLineEdit.text()
        comportementFilterValue = str(self.comportementFilterComboBox.currentText().toUtf8())
        definitifFilterValue = self.definitifFilterCheckBox.checkState()
        
        for scenarioFile in scenarioFiles:
            if scenarioFile.endswith(".sce"):
                # filter name
                if scenarioFile.startswith(nameFilterValue):
                    # filter attributes
                    scenarioFilePath = os.path.join(self.scenario_db_path, scenarioFile)
                    scenario = Scenario.loadFile(scenarioFilePath, False)
                    
                    if comportementFilterValue == "Tous" or ("comportement" in scenario.attributes.keys() and str(scenario.attributes["comportement"].encode("utf-8")) == comportementFilterValue):
                        if definitifFilterValue == 0 or ("definitif" in scenario.attributes.keys() and scenario.attributes["definitif"] == (definitifFilterValue == 2)):
                            # inser
                            self.insertRow(self.ui.scenario_db_table.rowCount(), scenarioFilePath, scenario)
            
        self.acceptTableItemChanged = True
        
            
    def insertRow(self, index, scenarioFilePath, scenario):
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
        deleteButton = QPushButton(u"Supprimer")
        deleteButton.clicked.connect(partial(self.handleDeleteButtonClicked, scenarioFilePath))
        executeButton = QPushButton(u"Exécuter")
        actionButtonsContainer.layout().addWidget(editButton)
        actionButtonsContainer.layout().addWidget(deleteButton)
        actionButtonsContainer.layout().addWidget(executeButton)
        self.ui.scenario_db_table.setCellWidget(index, 5, actionButtonsContainer)
        
        if scenarioFilePath in self.editingScenarios:
            self.setRowEnabled(index, False)
        self.acceptTableItemChanged = False
    
    
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
    
    
    def getRowFromScenarioFilePath(self, scenarioFilePath):
        for row in range(1, self.ui.scenario_db_table.rowCount()):
            if scenarioFilePath == self.ui.scenario_db_table.item(row, 0).filePath:
                return row
        
        return None
    
    
    def setRowEnabled(self, row, enabled):
        self.acceptTableItemChanged = False
        
        for column in range(self.ui.scenario_db_table.columnCount()):
            item = self.ui.scenario_db_table.item(row, column)
            if item is not None:
                if enabled:
                    item.setFlags(item.flags() | Qt.ItemIsEnabled)
                else:
                    item.setFlags(item.flags() ^ Qt.ItemIsEnabled)
            else:
                self.ui.scenario_db_table.cellWidget(row, column).setEnabled(enabled)
        
        self.acceptTableItemChanged = True
    
    def handleNewScenarioClicked(self, event):
        newName = str(self.ui.newScenario_lineEdit.text().toUtf8())
        newPath = os.path.join(self.scenario_db_path, newName + ".sce")
        if not os.path.exists(newPath) and newName != "":
            newScenario = Scenario()
            try:
                newScenario.save(newPath)
                
                self.populateTable()
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
        scenarioEdition = ScenarioEdition(scenarioFilePath, self.handleScenarioEditionSaved, self.handleScenarioEditionCloseEvent)
        
        # disable row of opened scenario
        row = self.getRowFromScenarioFilePath(scenarioFilePath)
        self.editingScenarios.append(scenarioFilePath)
        self.setRowEnabled(row, False)
    
    
    def handleDeleteButtonClicked(self, scenarioFilePath):
        # remove file
        os.remove(scenarioFilePath)
        
        self.populateTable()
        
    
    def handleScenarioEditionCloseEvent(self, scenarioFilePath):
        # enable row of closed scenario
        row = self.getRowFromScenarioFilePath(scenarioFilePath)
        self.editingScenarios.remove(scenarioFilePath)
        self.setRowEnabled(row, True)
        
    
    def handleScenarioEditionSaved(self, updatedFilePath, scenario):
        self.populateTable()