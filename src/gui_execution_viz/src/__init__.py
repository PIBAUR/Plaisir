#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" --------------------------------------------------------------------
    TO CHANGE DEPENDING ON THE PACKAGE:
    DON'T FORGET TO CREATE AN launch/eclipse.launch WITH THE CORRECT PACKAGE
"""
NODE_NAME = "gui_execution_viz"
""" -------------------------------------------------------------------- """

import sys
import os
import signal

import rospkg
from rospkg.common import ResourceNotFound
try:
    packagePath = rospkg.RosPack().get_path(NODE_NAME)
    pathToAdd = packagePath.split(os.path.sep)[0:-2]
    sys.path.append(os.path.sep.join(pathToAdd))
except ResourceNotFound:
    pass

import rospy
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *


DEBUG_WITH_ROS = True
DEBUG_SERVER = "-sever-debug" in sys.argv

def sigintHandler(*args):
    """ Handler for the SIGINT signal. """
    sys.stderr.write('\r')
    QApplication.quit()
    
    
if __name__ == '__main__':
    # debug
    if DEBUG_WITH_ROS:
        import src.launch_utils.src as launch_utils
        if "-eclipse-debug" in sys.argv:
            launch_utils.launchRosNode(NODE_NAME, "eclipse.launch")
        else:
            if DEBUG_SERVER:
                launch_utils.launchDebug()
    
    # run
    signal.signal(signal.SIGINT, sigintHandler)
    
    try:
        if DEBUG_WITH_ROS:
            rospy.init_node(NODE_NAME, anonymous = True)
        
        app = QApplication(sys.argv)  # @UndefinedVariable
        
        """ -----------------------------------
        TO CHANGE DEPENDING ON THE PACKAGE:
        """
        from ui import ExecutionViz
        main = ExecutionViz()
        """ ----------------------------------- """

        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
