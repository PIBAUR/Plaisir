#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from functools import partial

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4.phonon import Phonon

class RobotVideoPlayer():
    def __init__(self, ui, canvas):
        self.ui = ui
        self.canvas = canvas
        self.temporalization = None
        
        self.currentVideoMediaSource = None
        self.currentVideo = None
        self.tryToPause = False
        self.videoSeekValueIsSetByCode = False
        
        self.videoPlayer = Phonon.VideoPlayer()
        self.ui.videoContainer_widget.layout().addWidget(self.videoPlayer.videoWidget())
        self.videoPlayer.finished.connect(self.handleVideoPlayerFinished)
        
        self.ui.playPauseVideo_button.clicked.connect(self.handlePlayPauseVideoButtonClicked)
        self.ui.videoSeek_slider.valueChanged.connect(self.handleVideoSeekSliderValueChanged)
        
        self.videoPlayingTimer = QTimer()
        self.videoPlayingTimer.setInterval(30)
        self.videoPlayingTimer.timeout.connect(self.handleVideoPlaying)
        self.videoPlayingTimer.start()
    
    
    def play(self):
        self.videoPlayer.play()
        self.ui.playPauseVideo_button.setText("||")
    
    
    def pause(self):
        self.videoPlayer.pause()
        self.ui.playPauseVideo_button.setText(">")
        
    
    def stop(self):
        self.videoPlayer.stop()
        self.ui.videoSeek_slider.setValue(0)
        self.ui.playPauseVideo_button.setText(">")
    
    
    def seek(self, value):
        self.ui.videoSeek_slider.setValue(value * self.ui.videoSeek_slider.maximum())
    
    
    def setCurrentVideo(self, video):
        if video is not None:
            self.currentVideoMediaSource = video.media
            self.currentVideo = video
            self.videoPlayer.load(video.media)
            self.play()
            self.tryToPause = True
            self.ui.video_groupBox.setEnabled(True)
            self.ui.video_groupBox.setTitle(u"Vidéo (" + os.path.basename(video.niceName) + ")")
        else:
            self.currentVideoMediaSource = None
            self.currentVideo = None
            self.ui.video_groupBox.setEnabled(False)
            self.ui.video_groupBox.setTitle(u"Vidéo")
        
        if self.temporalization is not None:
            self.temporalization.setTimelineValueCurrentVideo(0)
            
    
    def sendVideoToCanvas(self):
        videoWidget = self.videoPlayer.videoWidget()
        videoWidgetAbsoluteCoords = videoWidget.mapToGlobal(QPoint(0, 0))
        snapshotRect = QRect(videoWidgetAbsoluteCoords.x(), videoWidgetAbsoluteCoords.y(), videoWidget.width(), videoWidget.height())
        snapshotPixmap = QPixmap.grabWindow(self.videoPlayer.winId(), snapshotRect.x(), snapshotRect.y(), snapshotRect.width(), snapshotRect.height())
        
        self.canvas.videoPixmap = snapshotPixmap
        self.canvas.update()
        del snapshotPixmap
        
        
    def handleVideoPlaying(self):
        if self.currentVideoMediaSource is None:
            self.ui.videoSeek_slider.setValue(0)
            
        if self.videoPlayer.isPlaying():
            value = float(self.videoPlayer.currentTime()) / self.videoPlayer.totalTime()
            
            self.videoSeekValueIsSetByCode = True
            self.ui.videoSeek_slider.setValue(value * self.ui.videoSeek_slider.maximum())
            self.temporalization.setTimelineValueCurrentVideo(value)
            self.videoSeekValueIsSetByCode = False
            
        # update canvas
        self.sendVideoToCanvas()
        
    
    def handlePlayPauseVideoButtonClicked(self):
        if self.videoPlayer.isPlaying():
            self.pause()
        else:
            self.play()
    
    
    def handleVideoPlayerFinished(self):
        if self.ui.continuousPlaying_checkbox.isChecked():
            self.temporalization.playNextVideo()
        else:
            self.stop()
        
    
    def handleVideoSeekSliderValueChanged(self, value):
        if not self.videoSeekValueIsSetByCode:
            sliderValue = float(value) / self.ui.videoSeek_slider.maximum()
            self.videoPlayer.seek(sliderValue * self.videoPlayer.totalTime())
            self.temporalization.setTimelineValueCurrentVideo(sliderValue)