import os
import random

import gst

from PyQt4.QtGui import *
from PyQt4.phonon import Phonon

class Video():
    currentHue = 50
    currentLuminosity = 100
    
    def __init__(self, filePath):
        self.filePath = filePath
        
        # only for display
        self.media = Phonon.MediaSource(filePath)
        imageBuffer = self.getFrameFromVideo(filePath)
        image = QImage.fromData(imageBuffer)
        pixmap = QPixmap.fromImage(image)
        self.thumbnailIcon = QIcon(pixmap)
        self.thumbnailRatio = pixmap.width() / pixmap.height() 
        
        self.niceName = (".").join(os.path.basename(self.filePath).split(".")[0:-1])
        self.color = self.getColor()
        
    
    def getColor(self):
        color = QColor()
        color.setHsl(Video.currentHue, 127, Video.currentLuminosity)
        Video.currentHue += 25
        if Video.currentHue > 255:
            Video.currentHue = 0
            Video.currentLuminosity -= 25
            if Video.currentLuminosity <= 25:
                Video.currentLuminosity = 100
        
        return color
        
    
    def getFrameFromVideo(self, path, offset = 0):
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