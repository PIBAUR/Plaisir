#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time
import json

from data.robot import Robot
from data.media import Media

class Scenario():
    def __init__(self):
        Robot.reinit()
        Media.reinit()
        
        self.creationTime = time.time()
        self.modificationTime = time.time()
        
        self.robots = [Robot()]
        
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
            robotToAppend = Robot()
            robotToAppend.load(robotData)
            self.robots.append(robotToAppend)
        self.type = data["type"]
        self.target = data["target"]
        self.behaviour = data["behaviour"]
    
    
    def save(self, filePath):
        with open(filePath, 'w') as outFile:
            json.dump(self.getDataDict(), outFile)
    
    
    @staticmethod
    def loadFile(filePath):
        data = json.loads(open(filePath).read())
        scenario = Scenario()
        scenario.setDataDict(data)
        
        return scenario
