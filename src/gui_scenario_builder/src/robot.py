import random
from PyQt4.QtGui import *

from video import Video

class Robot():
    currentHue = 0
    currentLuminosity = 100
    
    def __init__(self):
        self.points = []
        self.videos = []
        
        # only for display
        self.color = self.getColor()
        self.visible = True
        
    
    def getColor(self):
        color = QColor()
        color.setHsl(Robot.currentHue, 255, Robot.currentLuminosity)
        Robot.currentHue += 51
        if Robot.currentHue > 255:
            Robot.currentHue = 0
            Robot.currentLuminosity -= 25
            if Robot.currentLuminosity <= 25:
                Robot.currentLuminosity = 100
        
        return color