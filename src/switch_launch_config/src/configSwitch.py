#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
from lxml import etree

import rospkg
import dynamic_reconfigure.client

class ConfigSwitch():
    def __init__(self, nodeName, delay, package, launchFile):
        self.client = dynamic_reconfigure.client.Client(nodeName)
        
        time.sleep(delay)
        self.client.update_configuration({"update_min_d": .1})
        
        packagePath = rospkg.RosPack().get_path(package)
        
        tree = etree.parse(os.path.join(packagePath, launchFile))
        nodes = tree.xpath("/launch/node[@name='" + nodeName + "']/param")
        for node in nodes:
            paramName = node.get("name")
            paramValue = node.get("value")
            if paramName in self.client.config.keys():
                self.client.update_configuration({paramName: paramValue})