#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
from functools import partial
import math

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from src.scenario_lib.src.items.sequence import Sequence

class Sequences():
    def __init__(self, ui, canvas, temporalization):
        self.ui = ui
        self.canvas = canvas
        self.temporalization = temporalization
        
        self.canvas.sequences = self
        
        self.listClearing = True
        self.ui.sequences_list.currentItemChanged.connect(self.handleSequencesListCurrentItemChanged)
        self.ui.addSequence_button.clicked.connect(self.handleAddSequenceButtonClicked)
        self.ui.removeSequence_button.clicked.connect(self.handleRemoveSequenceButtonClicked)
        
        self.handleSequencesListCurrentItemChanged()
        
        self.listClearing = False
    
    
    def update(self):
        self.listClearing = True
        self.ui.sequences_list.clear()
        
        for sequence in self.canvas.currentRobot.sequences:
            self.addSequenceToList(float(sequence.timePosition), float(sequence.position), float(sequence.backward))
            
        self.listClearing = False
        
        
    def updateData(self):
        self.canvas.currentRobot.sequences = []
        for i in range(self.ui.sequences_list.count()):
            item = self.ui.sequences_list.itemWidget(self.ui.sequences_list.item(i))
            if item is not None:
                spinBoxesLayout = item.layout()
                backwardValue = spinBoxesLayout.itemAt(0).widget().isChecked()
                timeValue = spinBoxesLayout.itemAt(1).widget().value()
                positionValue = spinBoxesLayout.itemAt(2).widget().value()
                newSequence = Sequence(timeValue, positionValue, backwardValue, self.ui.sequences_list.currentRow() == i)
                self.canvas.currentRobot.sequences.append(newSequence)
        
    
    def offsetTimesAfter(self, pointIndex):
        for sequence in self.canvas.currentRobot.sequences:
            if sequence.position >= pointIndex:
                sequence.position += 1
        
        self.update()
        
        
    def handleSequencesListCurrentItemChanged(self, *args):
        self.ui.removeSequence_button.setEnabled(self.ui.sequences_list.currentItem() is not None)
        
        if not self.listClearing:
            self.updateData()
        
        
    def handleAddSequenceButtonClicked(self, *args):
        self.addSequenceToList((math.floor((self.canvas.currentTimelinePosition * self.temporalization.fullDuration) * 10)) / 10., self.canvas.currentRobot.sequences[-1].position if len(self.canvas.currentRobot.sequences) > 0 else 0)
        
        
    def handleRemoveSequenceButtonClicked(self, *args):
        self.ui.sequences_list.takeItem(self.ui.sequences_list.currentRow())
        self.updateData()
        
    
    def handleValueChanged(self, sequenceIndex):
        self.ui.sequences_list.setCurrentRow(sequenceIndex)
        self.updateData()
        
    
    def addSequenceToList(self, timePosition, position, backward = False):
        sequenceIndex = self.ui.sequences_list.count()
        self.ui.sequences_list.addItem("")
        sequenceWidget = QWidget()
        sequenceWidget.setLayout(QHBoxLayout())
        
        backwardCheckbox = QCheckBox()
        backwardCheckbox.setChecked(backward)
        backwardCheckbox.toggled.connect(partial(self.handleValueChanged, sequenceIndex))
        sequenceWidget.layout().addWidget(backwardCheckbox)
        
        timeSpinBox = QDoubleSpinBox()
        timeSpinBox.valueChanged.connect(partial(self.handleValueChanged, sequenceIndex))
        timeSpinBox.setDecimals(1)
        timeSpinBox.setMinimum(0)
        timeSpinBox.setMaximum(99999999)
        timeSpinBox.setValue(timePosition)
        sequenceWidget.layout().addWidget(timeSpinBox)
        
        positionSpinBox = QSpinBox()
        positionSpinBox.valueChanged.connect(partial(self.handleValueChanged, sequenceIndex))
        positionSpinBox.setMinimum(0)
        positionSpinBox.setMaximum(99999999)
        positionSpinBox.setValue(position)
        sequenceWidget.layout().addWidget(positionSpinBox)
        
        listWidget = self.ui.sequences_list.item(sequenceIndex)
        listWidget.setSizeHint(QSize(0, 40))
        self.ui.sequences_list.setItemWidget(listWidget, sequenceWidget)
        
        self.updateData()
        
        timeSpinBox.valueChanged.connect(partial(self.updateData))
        positionSpinBox.valueChanged.connect(partial(self.updateData))