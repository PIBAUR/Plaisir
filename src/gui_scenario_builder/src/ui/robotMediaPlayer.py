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
        
        #TODO: conditions depending on the type of media
        self.videoPlayer = Phonon.VideoPlayer()
        self.ui.mediaContainer_widget.layout().addWidget(self.videoPlayer.videoWidget())
        self.videoPlayer.finished.connect(self.handleVideoPlayerFinished)
        
        self.ui.media_groupBox.setEnabled(False)
        
        self.mediaPlayingTimer = QTimer()
        self.mediaPlayingTimer.setInterval(30)
        self.mediaPlayingTimer.timeout.connect(self.handleMediaPlaying)
        self.mediaPlayingTimer.start()
    
    
    def play(self):
        if self.currentVideoMediaSource is not None:
            self.videoPlayer.play()
    
    
    def pause(self):
        self.videoPlayer.pause()
        
    
    def stop(self):
        self.videoPlayer.stop()
    
    
    def seek(self, value):
        self.videoPlayer.seek(value)
    
    
    def currentTime(self):
        return self.videoPlayer.currentTime()
    
    
    def totalTime(self):
        return self.videoPlayer.totalTime()
    
    
    def setCurrentMedia(self, media, tryToPause = True):
        #TODO: conditions depending on the type of media
        if media is not None:
            self.currentVideoMediaSource = media.media
            self.currentMedia = media
            self.videoPlayer.load(media.media)
            self.play()
            # if was playing, don't try to pause
            if tryToPause:
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
        value = float(self.videoPlayer.currentTime()) / self.videoPlayer.totalTime()
        
        if self.videoPlayer.isPlaying():
            self.temporalization.setTimelineValueCurrentMedia(value)
            self.pause()
        
        if self.tryToPause:
            self.pause()
            self.tryToPause = False
        
        # update canvas
        self.sendMediaToCanvas()
        
    
    def handleVideoPlayerFinished(self):
        print "play next media"
        self.temporalization.playNextMedia()
