import os
import random

import gst

from PyQt4.QtGui import *
from PyQt4.phonon import Phonon

class Media():
    currentHue = 50
    currentLuminosity = 100
    
    def __init__(self, filePath, loadWithVideos = True):
        self.filePath = filePath
        self.duration = Media.getDurationOfVideo(filePath)
        self.startTime = -1
        self.endTime = -1
        
        # only for display
        if loadWithVideos:
            self.loadVideo()
        
        self.niceName = (".").join(os.path.basename(self.filePath).split(".")[0:-1])
        self.color = self.getColor()
    
    
    def save(self):
        result = {}
        result["filePath"] = self.filePath
        self.duration = Media.getDurationOfVideo(self.filePath)
        result["startTime"] = self.startTime
        result["endTime"] = self.endTime
        result["color"] = str(self.color.name())
        
        return result
    
    
    def load(self, data):
        self.startTime = data["startTime"]
        self.endTime = data["endTime"]
        self.color = QColor(data["color"])
    
    
    def loadVideo(self):
        self.media = Phonon.MediaSource(self.filePath)
        imageBuffer = self.getFrameFromMedia(self.filePath)
        image = QImage.fromData(imageBuffer)
        pixmap = QPixmap.fromImage(image)
        self.thumbnailIcon = QIcon(pixmap)
        self.thumbnailRatio = pixmap.width() / pixmap.height() 
        
        
    def destroy(self):
        del self.media
    
    
    def getColor(self):
        color = QColor()
        color.setHsl(Media.currentHue, 127, Media.currentLuminosity)
        Media.currentHue += 25
        if Media.currentHue > 255:
            Media.currentHue = 0
            Media.currentLuminosity -= 25
            if Media.currentLuminosity <= 25:
                Media.currentLuminosity = 100
        
        return color
        
    
    def getFrameFromMedia(self, path, offset = 0):
        #TODO: conditions depending on the type of media
        pipeline = gst.parse_launch('playbin2')
        pipeline.props.uri = 'file://' + os.path.abspath(path)
        pipeline.props.audio_sink = gst.element_factory_make('fakesink')
        pipeline.props.video_sink = gst.element_factory_make('fakesink')
        pipeline.set_state(gst.STATE_PAUSED)
        
        # wait for state change to finish.
        pipeline.get_state()
        assert pipeline.seek_simple(gst.FORMAT_TIME, gst.SEEK_FLAG_FLUSH, offset * gst.SECOND)
        
        # wait for seek to finish.
        pipeline.get_state()
        imageBuffer = pipeline.emit('convert-frame', gst.Caps('image/png'))
        pipeline.set_state(gst.STATE_NULL)
        
        return imageBuffer
    
    
    @staticmethod
    def getDurationOfVideo(filePath):
        return float(os.popen('ffprobe ' + filePath + ' -show_format -v quiet | sed -n \'s/duration=//p\'').read())
    
    
    @staticmethod
    def reinit():
        Media.currentHue = 50
        Media.currentLuminosity = 100
        