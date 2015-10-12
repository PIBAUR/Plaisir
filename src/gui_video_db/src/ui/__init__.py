#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from functools import partial
import math
import time
import json
import shutil
import subprocess

import rospkg
import rospy

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

from src.scenario_lib.src.items.media import Media
    

class VideoDatabase():
    dataColumns = ["name", "category", "startPosition", "endPosition"]
    
    def __init__(self, importCallback = None):
        self.importCallback = importCallback
        
        try:
            ui_file = os.path.join(rospkg.RosPack().get_path('gui_video_db'), 'resource', 'video_db.ui')
            self.videosBasePath = rospy.get_param("video_db_path")
            self.monitorScreenWidth = rospy.get_param("monitor_screen_width")
            self.monitorScreenHeight = rospy.get_param("monitor_screen_height")
        except Exception:
            ui_file = os.path.expanduser("~") + "/catkin_ws/src/gui_video_db/resource/video_db.ui"
            self.videosBasePath = "/home/artlab/Bureau/films_notre_bon_plaisir"
            self.monitorScreenWidth = 16
            self.monitorScreenHeight = 10
        
        # path
        if not os.path.exists(self.videosBasePath):
            os.makedirs(self.videosBasePath)
            
        # load ui
        self.ui = uic.loadUi(ui_file)
        
        # ui
        if self.importCallback is not None:
            self.ui.setWindowTitle(u"Importer une vidéo")
        
        self.ui.video_db_table.verticalHeader().setVisible(False)
        columns = [("Nom", 160), (u"Catégorie", 160), (u"Position de début", 160), (u"Position de fin", 160), ("Action", 80)]
        for i in range(len(columns)):
            self.ui.video_db_table.insertColumn(i)
            self.ui.video_db_table.setColumnWidth(i, columns[i][1])
        self.ui.video_db_table.setHorizontalHeaderLabels([column[0] for column in columns])
        self.ui.video_db_table.horizontalHeader().setStretchLastSection(True)
        self.ui.video_db_table.horizontalHeader().sectionResized.connect(self.handleColumnResized)
        self.ui.video_db_table.verticalHeader().setMovable(True)
        
        self.initTable()
        self.acceptTableItemChanged = True
        
        self.ui.show()
        self.ui.closeEvent = self.closeEvent

    
    def initTable(self):
        # insert the first row for filtering
        self.ui.video_db_table.insertRow(0)
        self.nameFilterLineEdit = QLineEdit()
        self.categoryFilterLineEdit = QLineEdit()
        self.startPositionFilterLineEdit = QLineEdit()
        self.endPositionFilterLineEdit = QLineEdit()
        
        # init auto completion
        categoryDelegate = CompleterDelegate(self.ui.video_db_table, self.completerSetupFunction, self.getCompleterStringList, ["category"])
        self.ui.video_db_table.setItemDelegateForColumn(1, categoryDelegate)
        startPositionDelegate = CompleterDelegate(self.ui.video_db_table, self.completerSetupFunction, self.getCompleterStringList, ["startPosition", "endPosition"])
        self.ui.video_db_table.setItemDelegateForColumn(2, startPositionDelegate)
        endPositionDelegate = CompleterDelegate(self.ui.video_db_table, self.completerSetupFunction, self.getCompleterStringList, ["startPosition", "endPosition"])
        self.ui.video_db_table.setItemDelegateForColumn(3, endPositionDelegate)
            
        # init filtering
        self.ui.video_db_table.setCellWidget(0, 0, self.nameFilterLineEdit)
        self.ui.video_db_table.setCellWidget(0, 1, self.categoryFilterLineEdit)
        self.ui.video_db_table.setCellWidget(0, 2, self.startPositionFilterLineEdit)
        self.ui.video_db_table.setCellWidget(0, 3, self.endPositionFilterLineEdit)
        self.nameFilterLineEdit.textChanged.connect(self.populateTable)
        self.categoryFilterLineEdit.textChanged.connect(self.populateTable)
        self.startPositionFilterLineEdit.textChanged.connect(self.populateTable)
        self.endPositionFilterLineEdit.textChanged.connect(self.populateTable)
        
        self.populateTable()
        
    
    def populateTable(self, event = None):
        self.acceptTableItemChanged = False
        
        # clear rows except first one with filter
        for i in range(1, self.ui.video_db_table.rowCount()):  # @UnusedVariable
            self.ui.video_db_table.removeRow(1)
        
        # get value for filtering after
        nameFilterValue = self.nameFilterLineEdit.text()
        categoryFilterValue = self.categoryFilterLineEdit.text()
        startPositionFilterValue = self.startPositionFilterLineEdit.text()
        endPositionFilterValue = self.endPositionFilterLineEdit.text()
        
        # get videos
        self.dbVideo = []
        for videosDir in sorted(os.listdir(self.videosBasePath)):
            videosDirPath = os.path.join(self.videosBasePath, videosDir)
            if os.path.isdir(videosDirPath):
                for videoFile in sorted(os.listdir(os.path.join(self.videosBasePath, videosDir))):
                    videoFilePath = os.path.join(self.videosBasePath, videoFile)
                    if not os.path.isdir(videoFilePath) and videoFile.split(".")[-1] == "mp4":
                        videoFileName = ".".join(videoFile.split(".")[:-1])
                        splittedVideoFileName = videoFileName.split("-")
                        if len(splittedVideoFileName) == 5:
                            video = {}
                            video["path"] = (videoFilePath).decode("utf-8")
                            video["category"] = (splittedVideoFileName[-2]).decode("utf-8")
                            video["name"] = ("-".join(splittedVideoFileName[:-3])).decode("utf-8")
                            video["startPosition"] = (splittedVideoFileName[-3]).decode("utf-8")
                            video["endPosition"] = (splittedVideoFileName[-1]).decode("utf-8")
                        # eventually create thumbs
                        """startThumbFile = videoFileName + "_start_thumb.png"
                        endThumbFile = videoFileName + "_end_thumb.png"
                        startThumbPath = os.path.join(videosDirPath, startThumbFile)
                        endThumbPath = os.path.join(videosDirPath, endThumbFile)
                        if not os.path.exists(startThumbPath):
                            os.system("ffmpeg -i " + videoFilePath + " -v quiet -vf \"select='eq(n, " + str(0) + ")'\" -vframes 1 " + startThumbPath)
                        video["startThumbFile"] = startThumbPath
                        if not os.path.exists(endThumbPath):
                            numberFrames = Media.getNumberFramesOfVideo(videoFilePath)
                            os.system("ffmpeg -i " + videoFilePath + " -v quiet -vf \"select='eq(n, " + str(numberFrames - 1) + ")'\" -vframes 1 " + endThumbPath)
                        video["endThumbFile"] = endThumbPath
                        """
                        self.dbVideo.append(video)
        
        for video in self.dbVideo:
            # filter attributes
            try:
                if video["name"].startswith(str(nameFilterValue.toUtf8()).decode("utf-8")) and video["category"].startswith(str(categoryFilterValue.toUtf8()).decode("utf-8")) and video["startPosition"].startswith(str(startPositionFilterValue.toUtf8()).decode("utf-8")) and video["endPosition"].startswith(str(endPositionFilterValue.toUtf8()).decode("utf-8")):
                    # inser
                    self.insertRow(self.ui.video_db_table.rowCount(), video)
            except:
                pass    
        self.acceptTableItemChanged = True
        
        self.handleColumnResized()
        
            
    def insertRow(self, index, video):
        # populate table
        self.ui.video_db_table.insertRow(index)
        
        i = 0
        for dataColumn in VideoDatabase.dataColumns:
            item = QTableWidgetItem(video[dataColumn])
            if i == 0:
                item.videoPath = video["path"]
            item.setFlags(item.flags() | Qt.ItemIsEditable)
            self.ui.video_db_table.setItem(index, i, item)
            i += 1
        
        """thumbsWidget = QWidget()
        thumbsWidget.setLayout(QHBoxLayout())
        thumbsWidget.setContentsMargins(0, -10, 0, -10)
        startImageWidget = QWidget()
        startImageWidget.setStyleSheet("border-image: url('" + video["startThumbFile"] + "') 0 0 0 0 stretch stretch;")
        thumbsWidget.layout().addWidget(startImageWidget)
        endImageWidget = QWidget()
        endImageWidget.setStyleSheet("border-image: url('" + video["endThumbFile"] + "') 0 0 0 0 stretch stretch;")
        thumbsWidget.layout().addWidget(endImageWidget)
        self.ui.video_db_table.setCellWidget(index, 4, thumbsWidget)
        """
        
        # add buttons for action
        actionButtonsContainer = QWidget()
        actionButtonsContainer.setLayout(QHBoxLayout())
        actionButtonsContainer.setContentsMargins(0, -10, 0, -10)
        if self.importCallback is not None:
            importButton = QPushButton(u"Sélectionner")
            importButton.clicked.connect(partial(self.handleImportButtonClicked, index))
            actionButtonsContainer.layout().addWidget(importButton)
        else:
            playButton = QPushButton(u"Lire")
            playButton.clicked.connect(partial(self.handlePlayButtonClicked, index))
            actionButtonsContainer.layout().addWidget(playButton)
            
        self.ui.video_db_table.setCellWidget(index, 4, actionButtonsContainer)
        
        self.acceptTableItemChanged = False


    def completerSetupFunction(self, editor, index, completerList):
        completer = QCompleter(completerList, editor)
        completer.setCompletionColumn(0)
        completer.setCompletionRole(Qt.EditRole)
        completer.setCaseSensitivity(Qt.CaseInsensitive)
        completer.setCompletionMode(QCompleter.PopupCompletion)
        editor.setCompleter(completer)
    
    
    def getItemByVideoPath(self, videoPath):
        for i in range(self.ui.video_db_table.rowCount()):
            item = self.ui.video_db_table.item(i, 0)
            if item.videoPath == videoPath:
                return item
        
        return None
    
    
    def getVideoByVideoPath(self, videoPath):
        results = [video for video in self.dbVideo if video['path'] == videoPath]
        if len(results) > 0:
            return results[0]
        else:
            return None
    
    
    def getCompleterStringList(self, itemTypes):
        results = []
        for itemType in itemTypes:
            results.extend([dbVideoItem[itemType] for dbVideoItem in self.dbVideo])
        return list(set(results))
        
    
    def dumpDbVideo(self):
        json.dump(self.dbVideo, open(self.dbPath, "w"))
    
    
    def handleColumnResized(self):
        # set row height to have the good ratio for thumbs
        rowHeight = 30#(float((self.ui.video_db_table.columnWidth(4) - 24)) / float(self.monitorScreenWidth) / 2.0) * float(self.monitorScreenHeight)
        for i in range(1, self.ui.video_db_table.rowCount()):
            self.ui.video_db_table.setRowHeight(i, rowHeight)
        
        
    def handleImportButtonClicked(self, row):
        videoPath = self.ui.video_db_table.item(row, 0).videoPath
        
        self.importCallback(videoPath)
        
        self.ui.close()
                        
        
    def handlePlayButtonClicked(self, row):
        videoPath = self.ui.video_db_table.item(row, 0).videoPath
        subprocess.Popen(['/usr/bin/totem', videoPath])
        
        
    def closeEvent(self, event):
        if self.importCallback is not None:
            self.importCallback(None)


# delegate to have auto completion in QTableWidget
class CompleterDelegate(QStyledItemDelegate):
    def __init__(self, parent, completerSetupFunction, completerListCaller, itemTypes):
        super(CompleterDelegate, self).__init__(parent)
        self.completerSetupFunction = completerSetupFunction
        self.completerListCaller = completerListCaller
        self.itemTypes = itemTypes
        
        
    def createEditor(self, parent, option, index):
        editor = QLineEdit(parent)
        self.completerSetupFunction(editor, index, self.completerListCaller(self.itemTypes))
        
        return editor
