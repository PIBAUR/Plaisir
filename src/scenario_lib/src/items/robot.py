import os
import random

from PyQt4.QtGui import *
from PyQt4.QtCore import *

import rospy
try:
    import tf
except:
    print "import tf failed"
import numpy

from scenario_msgs.msg import Scenario as ScenarioMsg
from scenario_msgs.msg import Media as MediaMsg
from scenario_msgs.msg import MediaArray as MediaArrayMsg
from scenario_msgs.msg import BezierPath as BezierPathMsg
from scenario_msgs.msg import BezierCurve as BezierCurveMsg
from std_msgs.msg import Header as HeaderMsg

from media import Media
from curvePoint import CurvePoint
from src.bezier_curve.src import bezier_interpolate

class Robot():
    currentHue = 0
    currentLuminosity = 100
    server_videos_path = None
    robot_videos_path = None
    
    def __init__(self, loadWithVideos = True):
        try:
            Robot.server_videos_path = rospy.get_param("server_videos_path")
            Robot.robot_videos_path = rospy.get_param("robot_videos_path")
        except:
            Robot.server_videos_path = os.path.expanduser("~") + "/.notrebonplaisir/videos"
            Robot.robot_videos_path = os.path.expanduser("~") + "/.notrebonplaisir/videos"
        
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
            if os.path.exists(mediaData["filePath"]):
                mediaToAppend = Media(mediaData["filePath"], self.loadWithVideos)
                mediaToAppend.load(mediaData)
                self.medias.append(mediaToAppend)
            else:
                #TODO: display error
                pass
        
        self.color = QColor(data["color"])
    
    
    def loadVideos(self):
        for media in self.medias:
            media.loadVideo()
        
    
    def getScenarioMsg(self, transformPosition, scale, transformOrientation):
        scenarioMsg = ScenarioMsg()
        
        self.setHeaderAndVideosForScenarioMsg(scenarioMsg)
        
        transformationMatrix = self.getTransformationMatrix(scale, transformOrientation)
        
        scenarioMsg.bezier_paths.curves = []
        firstAnchor = None
        for i in range(len(self.points)):
            point = self.points[i]
            
            if i == 0:
                firstAnchor = point.anchor
            
            if i + 1 < len(self.points):
                nextPoint = self.points[i + 1]
                
                bezierCurve = point.getBezierCurveWithNextPoint(nextPoint, -1, firstAnchor)
                
                bezierCurve.anchor_1.x, bezierCurve.anchor_1.y, z, w = self.getTransformedPoint(bezierCurve.anchor_1.x, bezierCurve.anchor_1.y, transformationMatrix, transformPosition)
                bezierCurve.anchor_2.x, bezierCurve.anchor_2.y, z, w = self.getTransformedPoint(bezierCurve.anchor_2.x, bezierCurve.anchor_2.y, transformationMatrix, transformPosition)
                bezierCurve.control_1.x, bezierCurve.control_1.y, z, w = self.getTransformedPoint(bezierCurve.control_1.x, bezierCurve.control_1.y, transformationMatrix, transformPosition)
                bezierCurve.control_2.x, bezierCurve.control_2.y, z, w = self.getTransformedPoint(bezierCurve.control_2.x, bezierCurve.control_2.y, transformationMatrix, transformPosition)
                
                scenarioMsg.bezier_paths.curves.append(bezierCurve)
        
        return scenarioMsg
    
    
    def setHeaderAndVideosForScenarioMsg(self, scenarioMsg):
        headerMsg = HeaderMsg()
        scenarioMsg.bezier_paths = BezierPathMsg()
        scenarioMsg.type = "choregraphic"
        
        # put medias into the message
        scenarioMsg.medias = MediaArrayMsg()
        scenarioMsg.medias.medias = []
        
        for media in self.medias :
            mediaMsg = MediaMsg()
            mediaMsg.type = "video"
            # replace file from server to robot 
            mediaMsg.path = media.filePath.replace(Robot.server_videos_path, Robot.robot_videos_path)
            mediaMsg.duration = media.duration
            mediaMsg.start_time = media.startTime
            mediaMsg.end_time = media.endTime
            scenarioMsg.medias.medias.append(mediaMsg)
       
        headerMsg.frame_id = "/map"
        headerMsg.stamp = rospy.Time.now()
        scenarioMsg.bezier_paths.header = headerMsg
    
    
    def getTransformationMatrix(self, scale, transformOrientation):
        origin = (0, 0, 0)
        quaternionMatrix = tf.transformations.quaternion_matrix(transformOrientation)
        scaleMatrix = tf.transformations.scale_matrix(scale, origin)
        
        return tf.transformations.concatenate_matrices(scaleMatrix, quaternionMatrix)
    
    
    def getTransformedPoint(self, x, y, transformationMatrix, transformPosition):
        result = numpy.dot(transformationMatrix, (x, -y, 0, 0))
        result[0] = result[0] + transformPosition[0]
        result[1] = result[1] + transformPosition[1]
        
        return result
    
    
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