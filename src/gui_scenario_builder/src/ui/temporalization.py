#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from functools import partial
import math

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from src.scenario_lib.src.items.media import Media

class Temporalization():
    def __init__(self, ui, canvas, robotMediaPlayer, changeCallback):
        self.ui = ui
        self.canvas = canvas
        self.robotMediaPlayer = robotMediaPlayer
        self.changeCallback = changeCallback
        
        self.lastMediaDirectory = ""
        self.timelineValueIsSetByCode = False
        self.temporalizationSplitter = None
        
        self.ui.timeline_slider.valueChanged.connect(self.handleTimelineSliderValueChanged)
        self.ui.addMedia_button.clicked.connect(self.handleAddMediaButtonClicked)
        self.ui.deleteMedia_button.clicked.connect(self.handleDeleteMediaButtonClicked)
    
    
    def update(self):
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
        mediaSizes = []
        handleWidth = self.temporalizationSplitter.handleWidth()
        splitterWidth = self.temporalizationSplitter.width() - handleWidth * (len(self.canvas.currentRobot.medias) - 1)
        i = 0
        for media in self.canvas.currentRobot.medias:
            mediaButton = QPushButton(media.niceName + "\n" + str(float(int((media.duration) * 100) / 100)) + " s")
            mediaButton.setStyleSheet("background: " + media.color.name() + "; text-align: left;")
            mediaButton.setMinimumWidth(1)
            mediaButton.setCheckable(True)
            mediaButton.clicked.connect(partial(self.updateMedia, mediaButton))
            
            self.temporalizationSplitter.addWidget(mediaButton)
            
            # icon
            mediaButton.setIcon(media.thumbnailIcon);
            iconHeight = self.ui.temporalization_widget.height() * 1.55
            mediaButton.setIconSize(QSize(iconHeight / media.thumbnailRatio, iconHeight))
            
            # get size for the end
            mediaSize = (media.endTime - media.startTime) * splitterWidth
            if len(self.canvas.currentRobot.medias) > 1:
                mediaSize -= handleWidth / (2 if (i == 0 or i == len(mediaSizes) - 1) else 1)
            
            mediaSizes.append(mediaSize)
            
            i += 1
        
        # set good mediaSizes
        self.temporalizationSplitter.setSizes(mediaSizes)
        self.handleTimelineSliderValueChanged()
        
        self.changeCallback()
    

    def updateMedia(self, checkedMediaButton = None):
        previousMedia = self.robotMediaPlayer.currentVideoMediaSource
        
        mediaToPlay = None
        # check each timeline buttons
        for i in range(self.temporalizationSplitter.count()):
            mediaButton = self.temporalizationSplitter.widget(i)
            # uncheck others
            if mediaButton != checkedMediaButton and checkedMediaButton is not None:
                mediaButton.setChecked(False)
            # check if they have focus to display media
            if mediaButton.isChecked():
                # put the media on the player
                mediaToPlay = self.canvas.currentRobot.medias[i]
        
        if mediaToPlay is None or mediaToPlay.media != previousMedia:
            self.robotMediaPlayer.stop()
            self.robotMediaPlayer.setCurrentMedia(mediaToPlay)
    
    
    def setTimelineValueCurrentMedia(self, value):
        self.timelineValueIsSetByCode = True
        if self.robotMediaPlayer.currentMedia is not None:
            newTimelineValue = self.robotMediaPlayer.currentMedia.startTime + value * (self.robotMediaPlayer.currentMedia.endTime - self.robotMediaPlayer.currentMedia.startTime)
            newTimelineValue *= self.ui.timeline_slider.maximum()
            self.ui.timeline_slider.setValue(newTimelineValue)
        self.timelineValueIsSetByCode = False
        
    
    def playNextMedia(self):
        #TODO: conditions depending on the type of media
        currentMediaIndex = self.canvas.currentRobot.medias.index(self.robotMediaPlayer.currentMedia)
        if currentMediaIndex < len(self.canvas.currentRobot.medias) - 1:
            mediaButton = self.temporalizationSplitter.widget(currentMediaIndex + 1)
            mediaButton.setChecked(True)
            self.updateMedia(mediaButton)
            self.robotMediaPlayer.videoPlayer.seek(0)
            self.robotMediaPlayer.play()
        else:
            mediaButton = self.temporalizationSplitter.widget(0)
            mediaButton.setChecked(True)
            self.updateMedia(mediaButton)
            self.robotMediaPlayer.stop()
    
    
    def handleAddMediaButtonClicked(self, event):
        # get a file
        #TODO: conditions depending on the type of media
        
        # hide and show because of a bug which shows a blank qfiledialog
        self.canvas.hide()
        filePaths = QFileDialog.getOpenFileNames(self.ui, u"Ajouter un média", self.lastMediaDirectory, u"Vidéos: *.mp4, *.mov (*.mov *.mp4)")
        self.canvas.show()
        
        for filePath in filePaths:
            filePath = str(filePath)
            self.lastMediaDirectory = os.path.dirname(filePath)
            
            # create media
            newMedia = Media(filePath)
            
            # set time to the middle of the last one
            newMedia.endTime = 1.0
            if len(self.canvas.currentRobot.medias) == 0:
                newMedia.startTime = 0.0
            else:
                lastMedia = self.canvas.currentRobot.medias[-1]
                lastMedia.endTime = float(lastMedia.startTime + (lastMedia.endTime - lastMedia.startTime) / 2)
                newMedia.startTime = float(lastMedia.endTime)
                
            self.canvas.currentRobot.medias.append(newMedia)
        
        self.update()
        
        self.temporalizationSplitter.refresh()
        self.handleTemporalizationSplitterMoved(-1, -1)
    
    
    def handleDeleteMediaButtonClicked(self, event):
        mediaToDelete = self.robotMediaPlayer.currentMedia
        # replace times to fill the gap 
        if len(self.canvas.currentRobot.medias) > 1:
            mediaToDeleteIndex = self.canvas.currentRobot.medias.index(mediaToDelete)
            if mediaToDeleteIndex == len(self.canvas.currentRobot.medias) - 1:
                self.canvas.currentRobot.medias[mediaToDeleteIndex - 1].endTime = mediaToDelete.endTime
            else:
                self.canvas.currentRobot.medias[mediaToDeleteIndex + 1].startTime = mediaToDelete.startTime
            
        # remove it
        self.canvas.currentRobot.medias.remove(mediaToDelete)
        mediaToDelete.destroy()
        mediaToDelete = None
        
        self.update()
        
    
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
            
            self.canvas.currentRobot.medias[i].startTime = startTime
            self.canvas.currentRobot.medias[i].endTime = endTime
            
            previousPosition += withHandleSize
            i += 1
        
        # display tooltip
        if position >= 0:
            percent = math.floor(self.canvas.currentRobot.medias[index].startTime * 1000) / 10
            QToolTip.showText(QCursor.pos(), str(percent) + " %")
        
        self.handleTimelineSliderValueChanged()
        self.canvas.update()
        
        self.changeCallback()
    
    
    def handleTimelineSliderValueChanged(self, value = -1):
        if value == -1:
            value = self.ui.timeline_slider.value()
        
        value = float(value) / (self.ui.timeline_slider.maximum() + 1)
        if not self.timelineValueIsSetByCode:
            # set current media
            if len(self.canvas.currentRobot.medias) > 0:
                i = 0
                for media in self.canvas.currentRobot.medias:
                    if value >= media.startTime and value < media.endTime:
                        break
                    i += 1
                
                media = self.canvas.currentRobot.medias[i]
                mediaSeek = (value - media.startTime) / (media.endTime - media.startTime)
                self.robotMediaPlayer.seek(mediaSeek)
                mediaButton = self.temporalizationSplitter.widget(i)
                mediaButton.setChecked(True)
                self.updateMedia(mediaButton)
            else:
                self.updateMedia()
            
        # update canvas
        self.canvas.currentTimelinePosition = value
        self.canvas.update()
