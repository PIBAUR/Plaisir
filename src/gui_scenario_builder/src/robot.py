import random
from PyQt4.QtGui import *

from media import Media

class Robot():
    currentHue = 0
    currentLuminosity = 100
    
    def __init__(self):
        self.points = []
        self.medias = []
        
        # only for display
        self.color = self.getColor()
        self.visible = True
    
    
    def save(self):
        result = {}
        result["points"] = [point.save() for point in self.points]
        result["medias"] = [media.save() for media in self.medias]
        result["color"] = self.color.name()
                
        return result    
    
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
    