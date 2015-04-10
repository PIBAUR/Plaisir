import os
import random
import math

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
from geometry_msgs.msg import Point as PointMsg
from std_msgs.msg import Header as HeaderMsg

from media import Media
from curvePoint import CurvePoint
from point import Point
from src.bezier_curve.src import bezier_interpolate

class Robot():
    currentHue = 0
    currentLuminosity = 100
    server_videos_path = None
    robot_videos_path = None
    
    def __init__(self, scenario):
        self.scenario = scenario
        
        try:
            Robot.server_videos_path = rospy.get_param("server_videos_path")
            Robot.robot_videos_path = rospy.get_param("robot_videos_path")
        except:
            Robot.server_videos_path = os.path.expanduser("~") + "/.notrebonplaisir/videos"
            Robot.robot_videos_path = os.path.expanduser("~") + "/.notrebonplaisir/videos"
        
        self.points = []
        self.medias = []
        
        # only for display
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
            pointToAppend = CurvePoint(Point(pointData["anchor"][0], pointData["anchor"][1]), Point(pointData["control1"][0], pointData["control1"][1]), Point(pointData["control2"][0], pointData["control2"][1]))
            self.points.append(pointToAppend)
        
        for mediaData in data["medias"]:
            if os.path.exists(mediaData["filePath"]):
                mediaToAppend = Media(mediaData["filePath"], self.scenario.loadWithVideos)
                mediaToAppend.load(mediaData)
                self.medias.append(mediaToAppend)
            else:
                #TODO: display error
                pass
        
        self.color = QColor(data["color"])
    
    
    def loadVideos(self):
        for media in self.medias:
            media.loadVideo()
    
    
    def getScenarioMsgWithParams(self, transformPosition, scale, transformOrientation, interpolation, rotateFromTarget):
        scenarioMsg = ScenarioMsg()
        
        self.setHeaderAndVideosForScenarioMsg(scenarioMsg)
        
        transformationMatrix = self.getTransformationMatrix(scale, transformOrientation)
        
        scenarioMsg.bezier_paths.curves = []
        firstAnchor = None
        if rotateFromTarget:
            # get pivot for future rotation
            pivot = [self.scenario.targetPosition[0], self.scenario.targetPosition[1]]
            if len(self.points) > 0:
                pivot[0] -= self.points[0].anchor._x
                pivot[1] -= self.points[0].anchor._y
        else:
            pivot = [0, 0]
        
        for i in range(len(self.points)):
            point = self.points[i]
            
            if interpolation:
                if i == 0:
                    firstAnchor = point.anchor
                
                if i + 1 < len(self.points):
                    nextPoint = self.points[i + 1]
                    
                    bezierCurve = point.getBezierCurveWithNextPoint(nextPoint, -1, firstAnchor)
                    bezierCurve.anchor_1.x, bezierCurve.anchor_1.y, z, w = self.getTransformedPoint(bezierCurve.anchor_1.x, bezierCurve.anchor_1.y, transformationMatrix, transformPosition, pivot, scale)
                    bezierCurve.anchor_2.x, bezierCurve.anchor_2.y, z, w = self.getTransformedPoint(bezierCurve.anchor_2.x, bezierCurve.anchor_2.y, transformationMatrix, transformPosition, pivot, scale)
                    bezierCurve.control_1.x, bezierCurve.control_1.y, z, w = self.getTransformedPoint(bezierCurve.control_1.x, bezierCurve.control_1.y, transformationMatrix, transformPosition, pivot, scale)
                    bezierCurve.control_2.x, bezierCurve.control_2.y, z, w = self.getTransformedPoint(bezierCurve.control_2.x, bezierCurve.control_2.y, transformationMatrix, transformPosition, pivot, scale)
            else:
                bezierCurve = BezierCurveMsg(anchor_1 = PointMsg(x = point.anchor._x, y = point.anchor._y, z = point.anchor._theta))
                
            scenarioMsg.bezier_paths.curves.append(bezierCurve)
        
        return scenarioMsg
        
        
    def setHeaderAndVideosForScenarioMsg(self, scenarioMsg):
        headerMsg = HeaderMsg()
        scenarioMsg.bezier_paths = BezierPathMsg()
        
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
        scaleMatrix = tf.transformations.scale_matrix(scale, origin)
        quaternionMatrix = tf.transformations.quaternion_matrix(transformOrientation)
        
        return tf.transformations.concatenate_matrices(scaleMatrix, quaternionMatrix)
    
    
    def getTransformedPoint(self, x, y, transformationMatrix, transformPosition, pivot, scale):
        result = numpy.dot(transformationMatrix, (x - pivot[0], y + pivot[1], 0, 0))
        result[0] = result[0] + transformPosition[0] + pivot[0] * scale
        result[1] = result[1] + transformPosition[1] - pivot[1] * scale
        
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