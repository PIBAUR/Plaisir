import random

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from media import Media
from curvePoint import CurvePoint
from src.bezier_curve.src import bezier_interpolate

class Robot():
    currentHue = 0
    currentLuminosity = 100
    
    def __init__(self, loadWithVideos = True):
        self.points = []
        self.medias = []
        
        # only for display
        self.loadWithVideos = loadWithVideos
        self.color = self.getColor()
        self.visible = True
    
    
    def save(self, scale = 1):
        result = {}
        result["points"] = [point.save() for point in self.points]
        result["pathLength"] = self.getPathLength(scale)
        result["medias"] = [media.save() for media in self.medias]
        result["color"] = str(self.color.name())
                
        return result    
    
    
    def load(self, data):
        self.points = []
        self.medias = []
        
        for pointData in data["points"]:
            pointToAppend = CurvePoint(QPoint(pointData["anchor"][0], pointData["anchor"][1]), QPoint(pointData["control1"][0], pointData["control1"][1]), QPoint(pointData["control2"][0], pointData["control2"][1]))
            self.points.append(pointToAppend)
        
        for mediaData in data["medias"]:
            mediaToAppend = Media(mediaData["filePath"], self.loadWithVideos)
            mediaToAppend.load(mediaData)
            self.medias.append(mediaToAppend)
        
        self.color = QColor(data["color"])
    
    
    def getPathLength(self, scale):
        result = 0
        
        for i in range(len(self.points)):
            point = self.points[i]
            
            if i + 1 < len(self.points):
                nextPoint = self.points[i + 1]
                bezierCurve = point.getBezierCurveWithNextPoint(nextPoint, scale)
                result += bezier_interpolate.getBezierCurveLength(bezierCurve)
                
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
    
    
    @staticmethod
    def reinit():
        Robot.currentHue = 0
        Robot.currentLuminosity = 100