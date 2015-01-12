#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from functools import partial
import math

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from video import Video

class Temporalization():
    def __init__(self, uiMain, ui, canvas, robotVideoPlayer):
        self.uiMain = uiMain
        self.ui = ui
        self.canvas = canvas
        self.robotVideoPlayer = robotVideoPlayer
        
        self.lastVideoDirectory = ""
        self.timelineValueIsSetByCode = False
        self.temporalizationSplitter = None
        
        self.update()
        self.ui.timeline_slider.valueChanged.connect(self.handleTimelineSliderValueChanged)
        self.ui.addVideo_button.clicked.connect(self.handleAddVideoButtonClicked)
    
    
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
        videoSizes = []
        handleWidth = self.temporalizationSplitter.handleWidth()
        splitterWidth = self.temporalizationSplitter.width() - handleWidth * (len(self.canvas.currentRobot.videos) - 1)
        i = 0
        for video in self.canvas.currentRobot.videos:
            videoButton = QPushButton(video.niceName)
            videoButton.setStyleSheet("background: " + video.color.name() + "; text-align: left;")
            videoButton.setMinimumWidth(1)
            videoButton.setCheckable(True)
            videoButton.clicked.connect(partial(self.updateVideo, videoButton))
            
            self.temporalizationSplitter.addWidget(videoButton)
            
            # icon
            videoButton.setIcon(video.thumbnailIcon);
            iconHeight = self.ui.temporalization_widget.height() * 1.55
            videoButton.setIconSize(QSize(iconHeight / video.thumbnailRatio, iconHeight))
            
            # get size for the end
            videoSize = (video.endTime - video.startTime) * splitterWidth
            if len(self.canvas.currentRobot.videos) > 1:
                videoSize -= handleWidth / (2 if (i == 0 or i == len(videoSizes) - 1) else 1)
            
            videoSizes.append(videoSize)
            
            i += 1
        
        # set good videoSizes
        self.temporalizationSplitter.setSizes(videoSizes)
        self.handleTimelineSliderValueChanged()
    

    def updateVideo(self, checkedVideoButton = None):
        previousMedia = self.robotVideoPlayer.currentVideoMediaSource
        
        videoToPlay = None
        # check each timeline buttons
        for i in range(self.temporalizationSplitter.count()):
            videoButton = self.temporalizationSplitter.widget(i)
            # uncheck others
            if videoButton != checkedVideoButton and checkedVideoButton is not None:
                videoButton.setChecked(False)
            # check if they have focus to display video
            if videoButton.isChecked():
                # put the video on the player
                videoToPlay = self.canvas.currentRobot.videos[i]
        
        if videoToPlay is None or videoToPlay.media != previousMedia:
            self.robotVideoPlayer.stop()
            self.robotVideoPlayer.setCurrentVideo(videoToPlay)
    
    
    def setTimelineValueCurrentVideo(self, value):
        self.timelineValueIsSetByCode = True
        if self.robotVideoPlayer.currentVideo is not None:
            newTimelineValue = self.robotVideoPlayer.currentVideo.startTime + value * (self.robotVideoPlayer.currentVideo.endTime - self.robotVideoPlayer.currentVideo.startTime)
            newTimelineValue *= self.ui.timeline_slider.maximum()
            self.ui.timeline_slider.setValue(newTimelineValue)
        self.timelineValueIsSetByCode = False
        
    
    def playNextVideo(self):
        currentVideoIndex = self.canvas.currentRobot.videos.index(self.robotVideoPlayer.currentVideo)
        if currentVideoIndex < len(self.canvas.currentRobot.videos) - 1:
            videoButton = self.temporalizationSplitter.widget(currentVideoIndex + 1)
            videoButton.setChecked(True)
            self.updateVideo(videoButton)
            self.robotVideoPlayer.videoPlayer.seek(0)
            self.robotVideoPlayer.play()
        else:
            videoButton = self.temporalizationSplitter.widget(0)
            videoButton.setChecked(True)
            self.updateVideo(videoButton)
            self.robotVideoPlayer.stop()
    
    
    def handleAddVideoButtonClicked(self, event):
        # get a file
        filePaths = QFileDialog.getOpenFileNames(self.uiMain, u"Ajouter une vidéo", self.lastVideoDirectory, u"Vidéos: *.mp4, *.mov (*.mov *.mp4)")
        
        for filePath in filePaths:
            filePath = str(filePath)
            self.lastVideoDirectory = os.path.dirname(filePath)
            
            # create video
            newVideo = Video(filePath)
            
            # set time to the middle of the last one
            newVideo.endTime = 1.0
            if len(self.canvas.currentRobot.videos) == 0:
                newVideo.startTime = 0.0
            else:
                lastVideo = self.canvas.currentRobot.videos[-1]
                lastVideo.endTime = float(lastVideo.startTime + (lastVideo.endTime - lastVideo.startTime) / 2)
                newVideo.startTime = float(lastVideo.endTime)
                
            self.canvas.currentRobot.videos.append(newVideo)
        
        self.update()
        
        self.temporalizationSplitter.refresh()
        self.handleTemporalizationSplitterMoved(-1, -1)
        
    
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
            
            self.canvas.currentRobot.videos[i].startTime = startTime
            self.canvas.currentRobot.videos[i].endTime = endTime
            
            previousPosition += withHandleSize
            i += 1
        
        # display tooltip
        if position >= 0:
            percent = math.floor(self.canvas.currentRobot.videos[index].startTime * 1000) / 10
            QToolTip.showText(QCursor.pos(), str(percent) + " %")
        
        self.handleTimelineSliderValueChanged()
        self.canvas.update()
    
    
    def handleTimelineSliderValueChanged(self, value = -1):
        if value == -1:
            value = self.ui.timeline_slider.value()
        
        value = float(value) / (self.ui.timeline_slider.maximum() + 1)
        if not self.timelineValueIsSetByCode:
            # set current video
            if len(self.canvas.currentRobot.videos) > 0:
                i = 0
                for video in self.canvas.currentRobot.videos:
                    if value >= video.startTime and value < video.endTime:
                        break
                    i += 1
                
                video = self.canvas.currentRobot.videos[i]
                videoSeek = (value - video.startTime) / (video.endTime - video.startTime)
                self.robotVideoPlayer.seek(videoSeek)
                videoButton = self.temporalizationSplitter.widget(i)
                videoButton.setChecked(True)
                self.updateVideo(videoButton)
            else:
                self.updateVideo()
            
        # update canvas
        self.canvas.currentTimelinePosition = value
        self.canvas.update()