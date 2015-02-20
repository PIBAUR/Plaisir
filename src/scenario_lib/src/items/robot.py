import random

from PyQt4.QtGui import *
from PyQt4.QtCore import *

import rospy

from scenario_msgs.msg import Scenario as ScenarioMsg
from scenario_msgs.msg import BezierPath as BezierPathMsg
from scenario_msgs.msg import BezierCurve as BezierCurveMsg
from std_msgs.msg import Header as HeaderMsg

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
    
    
    def save(self):
        result = {}
        result["points"] = [point.save() for point in self.points]
        result["pathLength"] = self.getPathLength()
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
    
    
    def loadVideos(self):
        for media in self.medias:
            media.loadVideo()
        
    
    def getScenarioMsg(self, gridSize):
        gridSize = float(gridSize)
        
        scenarioMsg = ScenarioMsg()
        headerMsg = HeaderMsg()
        scenarioMsg.bezier_paths = BezierPathMsg()
        
        #TODO: convert video_player to media_player
        #self.scenarioMsg.video_player = VideoPlayerMsg()
        #self.scenarioMsg.video_player.video_paths = ["test_video.mp4"]
        headerMsg.frame_id = "/map"
        headerMsg.stamp = rospy.Time.now()
        scenarioMsg.bezier_paths.header = headerMsg
        
        scenarioMsg.bezier_paths.curves = []
        firstAnchor = None
        for i in range(len(self.points)):
            point = self.points[i]
            
            if i == 0:
                firstAnchor = point.anchor
            
            if i + 1 < len(self.points):
                nextPoint = self.points[i + 1]
                bezierCurve = point.getBezierCurveWithNextPoint(nextPoint, -1, firstAnchor)
                bezierCurve.anchor_1.x /= gridSize
                bezierCurve.anchor_1.y /= gridSize
                bezierCurve.anchor_2.x /= gridSize
                bezierCurve.anchor_2.y /= gridSize
                bezierCurve.control_1.x /= gridSize
                bezierCurve.control_1.y /= gridSize
                bezierCurve.control_2.x /= gridSize
                bezierCurve.control_2.y /= gridSize
                scenarioMsg.bezier_paths.curves.append(bezierCurve)
        
        return scenarioMsg
    
    def getPathLength(self):
        result = 0
        
        for i in range(len(self.points)):
            point = self.points[i]
            
            if i + 1 < len(self.points):
                nextPoint = self.points[i + 1]
                bezierCurve = point.getBezierCurveWithNextPoint(nextPoint)
                result += bezier_interpolate.getBezierCurveLength(bezierCurve)
                
        return result
        
    
    def getDuration(self):
        result = 0.
        for media in self.medias:
            result += media.duration
        
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