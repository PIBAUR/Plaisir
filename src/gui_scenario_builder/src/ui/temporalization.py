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
        self.isPlaying = False
        
        self.ui.timeline_slider.valueChanged.connect(self.handleTimelineSliderValueChanged)
        self.ui.addMediaBefore_button.clicked.connect(self.handleAddMediaBeforeButtonClicked)
        self.ui.addMediaAfter_button.clicked.connect(self.handleAddMediaAfterButtonClicked)
        self.ui.deleteMedia_button.clicked.connect(self.handleDeleteMediaButtonClicked)
        self.ui.playPause_button.clicked.connect(self.handlePlayPauseMediaButtonClicked)
        
        self.playingTimer = QTimer()
        self.playingTimer.setInterval(1000. / 25)
        self.playingTimer.timeout.connect(self.handlePlaying)
        self.playingTimer.start()
        
        # disable focus on all widgets and add event for key press
        self.ui.keyPressEvent = self.handleKeyPressEvent
        for child in self.ui.findChildren(QWidget):
            if callable(getattr(child, "setFocusPolicy", None)):
                child.setFocusPolicy(Qt.NoFocus)
    
    
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
            mediaButton = QPushButton(media.niceName + "\n" + str(float(int((media.duration) * 10) / 10)) + " s")
            mediaButton.setStyleSheet("background: " + media.color.name() + "; text-align: left;")
            mediaButton.setMinimumWidth(1)
            mediaButton.setCheckable(True)
            mediaButton.setFocusPolicy(Qt.NoFocus)
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
            
            if self.fullDuration != 0:
                newTimelineValue = currentMediaStartTime / self.fullDuration + value * (self.robotMediaPlayer.currentMedia.duration / self.fullDuration)
                newTimelineValue *= self.ui.timeline_slider.maximum()
                self.ui.timeline_slider.setValue(newTimelineValue)
                
        self.timelineValueIsSetByCode = False
        
    
    def playNextMedia(self):
        #TODO: conditions depending on the type of media
        if self.robotMediaPlayer.currentMedia is not None:
            currentMediaIndex = self.canvas.currentRobot.medias.index(self.robotMediaPlayer.currentMedia)
            if currentMediaIndex < len(self.canvas.currentRobot.medias) - 1:
                mediaButton = self.temporalizationSplitter.widget(currentMediaIndex + 1)
                mediaButton.setChecked(True)
                self.updateMedia(mediaButton)
                self.robotMediaPlayer.seek(0)
                #self.play()
            else:
                mediaButton = self.temporalizationSplitter.widget(0)
                mediaButton.setChecked(True)
                self.updateMedia(mediaButton)
                self.robotMediaPlayer.stop()
    
    
    def setTimelineTime(self, timelineTime):
        if self.fullDuration > 0:
            self.handleTimelineSliderValueChanged(int(((float(timelineTime) / 1000) / self.fullDuration) * self.ui.timeline_slider.maximum()))
    
    
    def getTimelineTime(self):
        return ((float(self.ui.timeline_slider.value()) / self.ui.timeline_slider.maximum()) * self.fullDuration) * 1000
        
    
    def handleAddMediaBeforeButtonClicked(self, event):
        VideoDatabase(self.handleVideoSelected, True)
    
    
    def handleAddMediaAfterButtonClicked(self, event):
        VideoDatabase(self.handleVideoSelected, False)
        
        
    def handleVideoSelected(self, filePath, before = False):
        if filePath is not None:
            filePath = str(filePath.encode("utf-8"))
            self.lastMediaDirectory = os.path.dirname(filePath)
            
            # create media
            if self.robotMediaPlayer.currentMedia is not None:
                currentMediaIndex = 0
                if len(self.canvas.currentRobot.medias) > 0:
                    currentMediaIndex = 0
                    for media in self.canvas.currentRobot.medias:
                        if media == self.robotMediaPlayer.currentMedia:
                            break
                        currentMediaIndex += 1
                    if not before:
                        currentMediaIndex += 1
                self.canvas.currentRobot.medias.insert(currentMediaIndex, Media(filePath))
            else:
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
        
        if not self.timelineValueIsSetByCode:
            timelineValue = value * self.fullDuration
            
            if len(self.canvas.currentRobot.medias) > 0:
                # set current media
                currentDuration = 0
                mediaIndex = 0
                for media in self.canvas.currentRobot.medias:
                    currentDuration += media.duration
                    
                    if timelineValue <= currentDuration:
                        mediaSeek = int((timelineValue - (currentDuration - media.duration)) * 1000)
                        self.robotMediaPlayer.seek(mediaSeek)
                        mediaButton = self.temporalizationSplitter.widget(mediaIndex)
                        mediaButton.setChecked(True)
                        self.updateMedia(mediaButton)
                        break
                    
                    mediaIndex += 1
            else:
                self.updateMedia()
        
        # update canvas
        self.canvas.currentTimelinePosition = value
        self.ui.timeline_groupBox.setTitle("Timeline - " + str((float(math.floor(self.canvas.currentTimelinePosition * self.fullDuration * 100)) / 100)) + " s")
        self.canvas.update()
        
    
    def handlePlaying(self):
        if self.isPlaying:
            toSeek = self.robotMediaPlayer.currentTime() + self.playingTimer.interval()
            if toSeek < self.robotMediaPlayer.totalTime():
                self.robotMediaPlayer.seek(int(toSeek))
            else:
                self.robotMediaPlayer.handleVideoPlayerFinished()
        
        
    def handlePlayPauseMediaButtonClicked(self):
        if self.isPlaying:
            self.pause()
        else:
            self.play()
            
            
    def play(self):
        if self.robotMediaPlayer.currentMedia is not None:
            self.ui.playPause_button.setText("||")
            self.isPlaying = True
    
    
    def pause(self):
        self.ui.playPause_button.setText(">")
        self.isPlaying = False
    
    
    def handleKeyPressEvent(self, event):
        # play / pause
        if event.key() == Qt.Key_Space:
            self.handlePlayPauseMediaButtonClicked()
        
        # move forward / backward
        smallInterval = 2000. / 25
        bigInterval = smallInterval * 25
        currentTime = (self.canvas.currentTimelinePosition * 1000.) * self.fullDuration
        if event.key() == Qt.Key_Left:
            toSeek = int(currentTime - smallInterval)
            if toSeek > 0:
                self.robotMediaPlayer.seek(toSeek)
        if event.key() == Qt.Key_Right:
            toSeek = int(currentTime + smallInterval)
            if toSeek < self.fullDuration * 1000:
                self.robotMediaPlayer.seek(toSeek)
        if event.key() == Qt.Key_PageDown:
            toSeek = int(currentTime - bigInterval)
            if toSeek > 0:
                self.setTimelineTime(toSeek)
        if event.key() == Qt.Key_PageUp:
            toSeek = int(currentTime + bigInterval)
            if toSeek < self.fullDuration * 1000:
                self.setTimelineTime(toSeek)
