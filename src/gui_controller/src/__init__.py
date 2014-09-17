#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
from functools import partial
"""
# debug
try:
    sys.path.append("/opt/eclipse/plugins/org.python.pydev_3.7.1.201409021729/pysrc")
    import pydevd
    pydevd.settrace(stdoutToServer = True, stderrToServer = True)
except:
    print "debug failed"
"""
import rospy
import rospkg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic

class GuiController(QMainWindow):
    def __init__(self):
        super(QMainWindow, self).__init__()
        
        self.twistController = TwistController()
        
        ui_file = os.path.join(rospkg.RosPack().get_path('gui_controller'), 'resource', 'gui_controller.ui')
        
        self.ui = uic.loadUi(ui_file)
        self.ui.show()
        
        linearSpeed = .5
        angularSpeed = 2
        
        self.ui.goForward_button.pressed.connect(partial(self.twistController.move, linearSpeed, 0))
        self.ui.goForward_button.released.connect(partial(self.twistController.move, 0, 0))
        self.ui.goBackward_button.pressed.connect(partial(self.twistController.move, -linearSpeed, 0))
        self.ui.goBackward_button.released.connect(partial(self.twistController.move, 0, 0))
        self.ui.turnLeft_button.pressed.connect(partial(self.twistController.move, 0, angularSpeed))
        self.ui.turnLeft_button.released.connect(partial(self.twistController.move, 0, 0))
        self.ui.turnRight_button.pressed.connect(partial(self.twistController.move, 0, -angularSpeed))
        self.ui.turnRight_button.released.connect(partial(self.twistController.move, 0, 0))
        
        canvas = Canvas(self.handleCanvasCursorMoved, 0, 0, 200, 200)
        self.ui.layout().addWidget(canvas)
    
    def handleCanvasCursorMoved(self, x, y):
        self.twistController.move(-y, -x * 2)
    

class Canvas(QWidget):
    def __init__(self, callback, x, y, widht, height):
        super(QWidget, self).__init__()
        self.callback = callback
        
        self.setGeometry(x, y, widht, height)
        
        self.points = []
        self.pressed = False
    
    
    def paintEvent(self, e):
        painter = QPainter(self)
        painter.begin(self)
        self.drawBackground(painter)
        self.drawPoints(painter)
        painter.end()
        
        
    def mousePressEvent(self, event):
        self.pressed = True
        self.updateMouse(event)
        
        
    def mouseReleaseEvent(self, event):
        self.pressed = False
        self.update()
        self.callback(0, 0)
        
        
    def mouseMoveEvent(self, event):
        self.updateMouse(event)
    
    
    def updateMouse(self, event):
        if self.pressed:
            self.points.append((event.x(), event.y()))
            self.update()
            self.callback((float(event.x()) / self.width()) * 2 - 1, (float(event.y()) / self.height()) * 2 - 1)
    
    
    def drawBackground(self, painter):
        painter.fillRect(QRectF(self.x(), self.y(), self.width(), self.height()), Qt.blue)
    
    
    def drawPoints(self, painter):
        linePen = QPen(Qt.red);
        linePen.setCapStyle(Qt.RoundCap);
        linePen.setWidth(1);
        painter.setRenderHint(QPainter.Antialiasing, True);
        painter.setPen(linePen);
        
        for point in self.points:
            painter.drawPoint(point[0], point[1])
        
        linePen.setWidth(30);
        painter.setPen(linePen);
        if self.pressed:
            painter.drawPoint(self.points[-1][0], self.points[-1][1])


class TwistController():
    def __init__(self):
        self.publisher = rospy.Publisher('cmd_vel', Twist)
        self.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    
    
    def move(self, linear, angular):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        rospy.loginfo(str(self.twist))
        self.publisher.publish(self.twist)


if __name__ == '__main__':
    try:
        rospy.init_node('gui_controller', anonymous = True)
        
        app = QApplication(sys.argv)
        guiController = GuiController()
        sys.exit(app.exec_())
    
    except rospy.ROSInterruptException:
        pass
