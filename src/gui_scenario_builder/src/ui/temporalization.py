#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from functools import partial
import math

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from src.scenario_lib.src.items.media import Media
from src.gui_video_db.src.ui import VideoDatabase

class Temporalization():
    def __init__(self, ui, canvas, robotMediaPlayer, changeCallback):
        self.ui = ui
        self.canvas = canvas
        self.robotMediaPlayer = robotMediaPlayer
        self.changeCallback = changeCallback
        
        self.lastMediaDirectory = ""
        self.timelineValueIsSetByCode = False
        self.temporalizationSplitter = None
        self.fullDuration = 0.
        
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
        self.temporalizationSplitter.setChildrenCollapsible(False)
        self.ui.temporalization_widget.layout().addWidget(self.temporalizationSplitter)
        
        # add buttons
        mediaSizes = []
        self.temporalizationSplitter.setHandleWidth(1)
        handleWidth = self.temporalizationSplitter.handleWidth()
        splitterWidth = self.temporalizationSplitter.width() - handleWidth * (len(self.canvas.currentRobot.medias) - 1)
        i = 0
        self.fullDuration = 0.
        for media in self.canvas.currentRobot.medias:
            self.fullDuration += media.duration
            
        for media in self.canvas.currentRobot.medias:
            mediaButton = QPushButton(media.niceName + "\n" + str(float(int((media.duration) * 100) / 100)) + " s")
            mediaButton.setStyleSheet("background: " + media.color.name() + "; text-align: left;")
            mediaButton.setMinimumWidth(1)
            mediaButton.setCheckable(True)
            mediaButton.clicked.connect(partial(self.updateMedia, mediaButton))
            
            self.temporalizationSplitter.addWidget(mediaButton)
            
            # icon
            mediaButton.setIcon(media.thumbnailIcon)
            iconHeight = self.ui.temporalization_widget.height() * 1.55
            mediaButton.setIconSize(QSize(iconHeight / media.thumbnailRatio, iconHeight))
            
            # get size for the end
            mediaSize = (float(media.duration) / float(self.fullDuration)) * splitterWidth
            if len(self.canvas.currentRobot.medias) > 1:
                mediaSize -= handleWidth / (2 if (i == 0 or i == len(mediaSizes) - 1) else 1)
            
            mediaSizes.append(mediaSize)
            
            i += 1
        
        for i in range(self.temporalizationSplitter.count()):
            self.temporalizationSplitter.handle(i).setEnabled(False)
        
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
            
            currentMediaStartTime = 0
            for media in self.canvas.currentRobot.medias:
                if media == self.robotMediaPlayer.currentMedia:
                    break
                else:
                    currentMediaStartTime += media.duration
            
            newTimelineValue = currentMediaStartTime / self.fullDuration + value * (self.robotMediaPlayer.currentMedia.duration / self.fullDuration)
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
        VideoDatabase(self.handleVideoSelected)
        
        
    def handleVideoSelected(self, filePath):
        if filePath is not None:
            filePath = str(filePath)
            self.lastMediaDirectory = os.path.dirname(filePath)
            
            # create media
            self.canvas.currentRobot.medias.append(Media(filePath))
        
        self.update()
        
        self.temporalizationSplitter.refresh()
    
    
    def handleDeleteMediaButtonClicked(self, event):
        mediaToDelete = self.robotMediaPlayer.currentMedia
        # replace times to fill the gap 
        if len(self.canvas.currentRobot.medias) > 1:
            mediaToDeleteIndex = self.canvas.currentRobot.medias.index(mediaToDelete)
            
        # remove it
        self.canvas.currentRobot.medias.remove(mediaToDelete)
        mediaToDelete.destroy()
        mediaToDelete = None
        
        self.update()
        
    
    def handleTimelineSliderValueChanged(self, value = -1):
        if value == -1:
            value = self.ui.timeline_slider.value()
        
        value = float(value) / (self.ui.timeline_slider.maximum() + 1)
        """
        if not self.timelineValueIsSetByCode:
            if len(self.canvas.currentRobot.medias) > 0:
                # set current media
                currentDuration = 0
                for i in range(len(self.canvas.currentRobot.medias)):
                    media = self.canvas.currentRobot.medias[i]
                    if value <= currentDuration:
                        mediaSeek = 0
                        self.robotMediaPlayer.seek(mediaSeek)
                        mediaButton = self.temporalizationSplitter.widget(i)
                        mediaButton.setChecked(True)
                        self.updateMedia(mediaButton)
                        break
                    currentDuration += media.duration
            else:
                self.updateMedia()
            """
        
        # update canvas
        self.canvas.currentTimelinePosition = value
        self.ui.timelinePosition_label.setText(str((float(math.floor(value * self.fullDuration * 100)) / 100)) + " s")
        self.canvas.update()