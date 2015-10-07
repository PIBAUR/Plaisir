#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from functools import partial
import math

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4.phonon import Phonon

class RobotMediaPlayer():
    def __init__(self, ui, canvas):
        self.ui = ui
        self.canvas = canvas
        self.temporalization = None
        
        self.currentVideoMediaSource = None
        self.currentMedia = None
        self.tryToPause = False
        self.mediaSeekValueIsSetByCode = False
        
        #TODO: conditions depending on the type of media
        self.videoPlayer = Phonon.VideoPlayer()
        self.ui.mediaContainer_widget.layout().addWidget(self.videoPlayer.videoWidget())
        self.videoPlayer.finished.connect(self.handleVideoPlayerFinished)
        
        self.ui.playPauseMedia_button.clicked.connect(self.handlePlayPauseMediaButtonClicked)
        self.ui.mediaSeek_slider.valueChanged.connect(self.handleMediaSeekSliderValueChanged)
        
        self.ui.media_groupBox.setEnabled(False)
        
        self.mediaPlayingTimer = QTimer()
        self.mediaPlayingTimer.setInterval(30)
        self.mediaPlayingTimer.timeout.connect(self.handleMediaPlaying)
        self.mediaPlayingTimer.start()
    
    
    def play(self):
        self.videoPlayer.play(self.currentVideoMediaSource)
        self.ui.playPauseMedia_button.setText("||")
    
    
    def pause(self):
        self.videoPlayer.pause()
        self.ui.playPauseMedia_button.setText(">")
        
    
    def stop(self):
        self.videoPlayer.stop()
        self.ui.mediaSeek_slider.setValue(0)
        self.ui.playPauseMedia_button.setText(">")
    
    
    def seek(self, value):
        self.ui.mediaSeek_slider.setValue(value * self.ui.mediaSeek_slider.maximum())
    
    
    def setCurrentMedia(self, media):
        #TODO: conditions depending on the type of media
        if media is not None:
            self.currentVideoMediaSource = media.media
            self.currentMedia = media
            self.videoPlayer.load(media.media)
            self.play()
            self.tryToPause = True
            self.ui.media_groupBox.setEnabled(True)
            self.ui.media_groupBox.setTitle(u"Vidéo (" + os.path.basename(media.niceName) + ")")
        else:
            self.currentVideoMediaSource = None
            self.currentMedia = None
            self.ui.media_groupBox.setEnabled(False)
            self.ui.media_groupBox.setTitle(u"Vidéo")
        
        if self.temporalization is not None:
            self.temporalization.setTimelineValueCurrentMedia(0)
            
    
    def sendMediaToCanvas(self):
        #TODO: conditions depending on the type of media
        videoWidget = self.videoPlayer.videoWidget()
        videoWidgetAbsoluteCoords = videoWidget.mapToGlobal(QPoint(0, 0))
        snapshotRect = QRect(videoWidgetAbsoluteCoords.x(), videoWidgetAbsoluteCoords.y(), videoWidget.width(), videoWidget.height())
        snapshotPixmap = QPixmap.grabWindow(self.videoPlayer.winId(), snapshotRect.x(), snapshotRect.y(), snapshotRect.width(), snapshotRect.height())
        
        self.canvas.mediaPixmap = snapshotPixmap
        self.canvas.update()
        del snapshotPixmap
        
        
    def handleMediaPlaying(self):
        value = 0
        if self.currentVideoMediaSource is None:
            self.ui.mediaSeek_slider.setValue(value)
        else:
            value = float(self.videoPlayer.currentTime()) / self.videoPlayer.totalTime()
            
            if self.videoPlayer.isPlaying():
                
                self.mediaSeekValueIsSetByCode = True
                self.ui.mediaSeek_slider.setValue(value * self.ui.mediaSeek_slider.maximum())
                self.temporalization.setTimelineValueCurrentMedia(value)
                self.mediaSeekValueIsSetByCode = False
            
        # update canvas
        self.sendMediaToCanvas()
        
    
    def handlePlayPauseMediaButtonClicked(self):
        if self.videoPlayer.isPlaying():
            self.pause()
        else:
            self.play()
    
    
    def handleVideoPlayerFinished(self):
        if self.ui.continuousPlaying_checkbox.isChecked():
            self.temporalization.playNextMedia()
        else:
            self.stop()
        
    
    def handleMediaSeekSliderValueChanged(self, value):
        if not self.mediaSeekValueIsSetByCode:
            sliderValue = float(value) / self.ui.mediaSeek_slider.maximum()
            self.videoPlayer.pause()
            self.videoPlayer.seek(sliderValue * self.videoPlayer.totalTime())
            self.temporalization.setTimelineValueCurrentMedia(sliderValue)