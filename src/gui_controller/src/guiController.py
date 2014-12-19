import os
from functools import partial

import rospkg

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

from twistController import TwistController
from scenarioController import ScenarioController
from canvas import Canvas
        
class GuiController(QMainWindow):
    def __init__(self):
        super(QMainWindow, self).__init__()
        self.twistController = TwistController()
        self.scenarioController = ScenarioController()
        
        ui_file = os.path.join(rospkg.RosPack().get_path('gui_controller'), 'resource', 'gui_controller.ui')
        
        self.ui = uic.loadUi(ui_file)
        self.ui.show()
        
        self.currentLinearSpeed = 0
        self.currentAngularSpeed = 0
        
        self.ui.goForward_button.pressed.connect(partial(self.handleButtonChanged, 1, 0))
        self.ui.goForward_button.released.connect(partial(self.handleButtonChanged, -1, 0))
        self.ui.goBackward_button.pressed.connect(partial(self.handleButtonChanged, -1, 0))
        self.ui.goBackward_button.released.connect(partial(self.handleButtonChanged, 1, 0))
        self.ui.turnLeft_button.pressed.connect(partial(self.handleButtonChanged, 0, 1))
        self.ui.turnLeft_button.released.connect(partial(self.handleButtonChanged, 0, -1))
        self.ui.turnRight_button.pressed.connect(partial(self.handleButtonChanged, 0, -1))
        self.ui.turnRight_button.released.connect(partial(self.handleButtonChanged, 0, 1))
        
        self.ui.addPoint_button.clicked.connect(self.handleAddPointClicked)
        self.ui.removePoint_button.clicked.connect(self.handleRemovePointClicked)
        self.ui.bezier_treeWidget.itemSelectionChanged.connect(self.handleBezierTreeWidgetChanged)
        self.ui.bezier_treeWidget.itemDoubleClicked.connect(self.handlePointDoubleClicked)
        self.ui.bezier_treeWidget.itemChanged.connect(self.handlePointEnteredClicked)
        self.ui.sendBezier_button.clicked.connect(self.handleSendBezierClicked)
        self.handleBezierTreeWidgetChanged()
        
        absoluteCoords = self.ui.canvasContainer.mapToGlobal(self.ui.pos())
        canvas = Canvas(self.handleCanvasCursorMoved, absoluteCoords.x(), absoluteCoords.y(), self.ui.canvasContainer.width(), self.ui.canvasContainer.height())
        self.ui.layout().addWidget(canvas)
        
        self.ui.keyPressEvent = self.keyPressEvent
        self.ui.keyReleaseEvent = self.keyReleaseEvent
    
    # keyboard
    def handleButtonChanged(self, linearSpeed, angularSpeed):
        inputLinearSpeed = float(self.ui.linearSpeed_slider.value()) / 10.
        inputAngularSpeed = 2 * float(self.ui.angularSpeed_slider.value()) / 10.
        self.currentLinearSpeed += linearSpeed
        self.currentAngularSpeed += angularSpeed
        self.twistController.move(self.currentLinearSpeed * inputLinearSpeed, self.currentAngularSpeed * inputAngularSpeed)
        
    
    def keyPressEvent(self, event):
        self.manageKey(event.key, True)
    
    
    def keyReleaseEvent(self, event):
        self.manageKey(event.key, False)
    
    
    def manageKey(self, key, pressed):
        button = None
        
        if key() == Qt.Key_Z:
            button = self.ui.goForward_button
        if key() == Qt.Key_S:
            button = self.ui.goBackward_button
        if key() == Qt.Key_Q:
            button = self.ui.turnLeft_button
        if key() == Qt.Key_D:
            button = self.ui.turnRight_button
        
        if button is not None:
            button.setDown(pressed)
            if pressed:
                button.pressed.emit()
            else:
                button.released.emit()
    
    
    # trackpad
    def handleCanvasCursorMoved(self, x, y):
        self.twistController.move(-y, -x * 2)
    
    
    # bezier
    def handleAddPointClicked(self, event):
        item = QTreeWidgetItem(self.ui.bezier_treeWidget)
        item.setText(0, "0");
        item.setText(1, "1");
        item.setText(2, "2");
        self.ui.bezier_treeWidget.addTopLevelItem(item)
    
    
    def handlePointDoubleClicked(self, item, column):
        self.ui.bezier_treeWidget.openPersistentEditor(item, column)
        
        
    def handlePointEnteredClicked(self, item, column):
        if str(item.data(column, 0).toString()).isdigit(): 
            self.ui.bezier_treeWidget.closePersistentEditor(item, column)
    
    
    def handleRemovePointClicked(self, event):
        self.ui.bezier_treeWidget.takeTopLevelItem(self.ui.bezier_treeWidget.indexOfTopLevelItem(self.ui.bezier_treeWidget.currentItem()))
    

    def handleBezierTreeWidgetChanged(self):
        self.ui.removePoint_button.setEnabled(self.ui.bezier_treeWidget.currentItem() is not None)
    
    
    def handleSendBezierClicked(self, event):
        self.scenarioController.send()