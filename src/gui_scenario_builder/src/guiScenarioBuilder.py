import os
from functools import partial

import rospkg

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

from canvas import Canvas
        
class GuiScenarioBuilder(QMainWindow):
    def __init__(self):
        super(QMainWindow, self).__init__()
        
        #ui_file = os.path.join(rospkg.RosPack().get_path('gui_scenario_builder'), 'resource', 'gui_scenario_builder.ui')
        ui_file = "/home/artlab/catkin_ws/src/gui_scenario_builder/resource/gui_scenario_builder.ui"
        
        self.ui = uic.loadUi(ui_file)
        self.ui.show()
        
        absoluteCoords = self.ui.canvasContainer.mapToGlobal(self.ui.pos())
        canvas = Canvas(absoluteCoords.x(), absoluteCoords.y(), self.ui.canvasContainer.width(), self.ui.canvasContainer.height())
        self.ui.layout().addWidget(canvas)
        