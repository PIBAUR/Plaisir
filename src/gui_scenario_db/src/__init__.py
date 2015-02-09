#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import signal

import rospkg
from rospkg.common import ResourceNotFound
try:
    packagePath = rospkg.RosPack().get_path('gui_scenario_db')
    pathToAdd = packagePath.split(os.path.sep)[0:-2]
    sys.path.append(os.path.sep.join(pathToAdd))
except ResourceNotFound:
    pass

import rospy
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from ui import ScenarioDataBase

DEBUG_WITH_ROS = False

def sigintHandler(*args):
    """ Handler for the SIGINT signal. """
    sys.stderr.write('\r')
    QApplication.quit()
    
    
if __name__ == '__main__':
    # debug
    if DEBUG_WITH_ROS:
        if "-eclipse-debug" in sys.argv:
            import src.launch_utils.src as launch_utils
            launch_utils.launchRosNode("gui_scenario_db", "eclipse.launch")
            launch_utils.launchDebug()
    
    # run
    signal.signal(signal.SIGINT, sigintHandler)
    
    try:
        if DEBUG_WITH_ROS:
            rospy.init_node('gui_scenario_db', anonymous = True)
        
        app = QApplication(sys.argv)
        scenarioEdition = ScenarioDataBase()
        
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
