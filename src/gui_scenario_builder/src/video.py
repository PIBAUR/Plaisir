import random
from PyQt4.QtGui import *

class Video():
    currentHue = 50
    currentLuminosity = 100
    
    def __init__(self, filePath):
        self.filePath = filePath
        
        # only for display
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