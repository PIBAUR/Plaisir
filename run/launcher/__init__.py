#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import signal
import subprocess
from functools import partial

from PyQt4 import uic
from PyQt4.QtCore import *
from PyQt4.QtGui import *


class Launcher(QMainWindow):
    def __init__(self):
        super(QMainWindow, self).__init__()
        
        ui_file = os.path.expanduser("~") + "/catkin_ws/run/launcher/resource/launcher.ui"
        
        self.ui = uic.loadUi(ui_file)
        
        self.ui.controller_button.clicked.connect(partial(self.handleCommandButtonClicked, "gui/gui_controller"))
        self.ui.scenarioDb_button.clicked.connect(partial(self.handleCommandButtonClicked, "gui/gui_scenario_db"))
        self.ui.scenarioExecutionDiagram_button.clicked.connect(partial(self.handleCommandButtonClicked, "gui/gui_execution_diagram"))
        
        self.ui.show()
        
        
    def handleCommandButtonClicked(self, packageName):
        subprocess.Popen(['/bin/bash', os.path.expanduser("~") + "/catkin_ws/run/" + packageName + ".sh"])
    
        
def sigintHandler(*args):
    """ Handler for the SIGINT signal. """
    sys.stderr.write('\r')
    QApplication.quit()
    
    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigintHandler)
    
    app = QApplication(sys.argv)
    main = Launcher()
    
    sys.exit(app.exec_())
