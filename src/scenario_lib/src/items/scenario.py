#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time
import json

from robot import Robot
from media import Media

class Scenario():
    def __init__(self, loadWithVideos = True):
        Robot.reinit()
        Media.reinit()
        
        self.creationTime = time.time()
        self.modificationTime = time.time()
        self.loadWithVideos = loadWithVideos
        
        self.robots = [Robot(loadWithVideos)]
        
        self.name = None
        self.type = None
        self.target = None
        self.behaviour = None
    
    
    def getDataDict(self):
        data = {}
        data["creationTime"] = self.creationTime
        data["modificationTime"] = time.time()
        data["robots"] = [robot.save() for robot in self.robots]
        data["type"] = self.type
        data["target"] = self.target
        data["behaviour"] = self.behaviour
        
        return data
    
    
    def setDataDict(self, data):
        self.creationTime = data["creationTime"]
        self.modificationTime = data["modificationTime"]
        self.robots = []
        for robotData in data["robots"]:
            robotToAppend = Robot(self.loadWithVideos)
            robotToAppend.load(robotData)
            self.robots.append(robotToAppend)
        self.type = data["type"]
        self.target = data["target"]
        self.behaviour = data["behaviour"]
    
    
    def save(self, filePath):
        self.name = os.path.basename(str(filePath))
        
        with open(filePath, 'w') as outFile:
            json.dump(self.getDataDict(), outFile)
    
    
    def toScenarioMsg(self):
        return None
    
    
    @staticmethod
    def loadFile(filePath, loadWithVideos = True):
        data = json.loads(open(filePath).read())
        scenario = Scenario(loadWithVideos)
        scenario.name = os.path.basename(str(filePath))
        scenario.setDataDict(data)
        
        return scenario
