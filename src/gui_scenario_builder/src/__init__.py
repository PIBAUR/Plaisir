#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import signal

import rospy
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from data.scenario import Scenario
from ui.scenario_edition import ScenarioEdition

def sigintHandler(*args):
    """ Handler for the SIGINT signal. """
    sys.stderr.write('\r')
    QApplication.quit()
    
    
if __name__ == '__main__':
    # debug
    """
    if "-eclipse-debug" in sys.argv:
        import src.launch_utils.src as launch_utils
        launch_utils.launchRosNode("gui_scenario_builder", "eclipse.launch")
        launch_utils.launchDebug()
    """
    # run
    signal.signal(signal.SIGINT, sigintHandler)
    
    try:
        #rospy.init_node('gui_scenario_builder', anonymous = True)
        
        app = QApplication(sys.argv)
        scenarioEdition = ScenarioEdition()
        
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
