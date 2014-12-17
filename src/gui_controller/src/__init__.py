#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
from functools import partial
import signal

if "-eclipse-debug" in sys.argv:
    os.environ["PYTHONPATH"] = "/home/artlab/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/groovy/lib/python2.7/dist-packages"
    os.environ["ROS_DISTRO"] = "groovy"
    os.environ["ROS_ETC_DIR"] = "/opt/ros/groovy/etc/ros"
    os.environ["ROS_HOME"] = "/home/artlab/.ros"
    os.environ["ROS_HOSTNAME"] = "192.168.150.1"
    os.environ["ROS_IP"] = "192.168.150.1"
    os.environ["ROS_LOG_DIR"] = "/home/artlab/.ros/log"
    os.environ["ROS_MASTER_URI"] = "http://192.168.150.1:11311"
    os.environ["ROS_PACKAGE_PATH"] = "/home/artlab/catkin_ws/src:/opt/ros/groovy/share:/opt/ros/groovy/stacks"
    os.environ["ROS_ROOT"] = "/opt/ros/groovy/share/ros"
    os.environ["ROS_TEST_RESULTS_DIR"] = "/home/artlab/catkin_ws/build/test_results"
    os.environ["ROSLISP_PACKAGE_DIRECTORIES"] = "/home/artlab/catkin_ws/devel/share/common-lisp"
    
    command = "/opt/ros/groovy/bin/roslaunch gui_controller eclipse.launch"
    sys.argv = ["roslaunch", "gui_controller", "eclipse.launch"]
    
    sys.path.append("/opt/ros/groovy/lib/python2.7/dist-packages")
    import roslaunch
    roslaunch.main()
    sys.exit()

# debug
if False:
    try:
        sys.path.append("/opt/eclipse/plugins/org.python.pydev_3.7.1.201409021729/pysrc")
        import pydevd
        pydevd.settrace(stdoutToServer = True, stderrToServer = True)
    except:
        print "debug failed"


import rospy
import rospkg
from std_msgs.msg import *
from geometry_msgs.msg import *
from bezier_curve.msg import BezierCurve
from scenario.msg import Scenario

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic
        
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
    
    
    # bézier
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
    
    
class Canvas(QWidget):
    def __init__(self, callback, x, y, widht, height):
        super(QWidget, self).__init__()
        self.callback = callback
        
        self.setGeometry(x, y, widht, height)
        
        self.point = None
    
    
    def paintEvent(self, e):
        painter = QPainter(self)
        painter.begin(self)
        self.drawBackground(painter)
        self.drawPoints(painter)
        painter.end()
        
        
    def mousePressEvent(self, event):
        self.point = (event.x(), event.y())
        self.updateMouse(event)
    
        
    def mouseReleaseEvent(self, event):
        self.point = None
        self.update()
        self.callback(0, 0)
        
        
    def mouseMoveEvent(self, event):
        self.updateMouse(event)
    
    
    def updateMouse(self, event):
        if self.point is not None:
            self.point = (event.x(), event.y())
            self.update()
            self.callback((float(event.x()) / self.width()) * 2 - 1, (float(event.y()) / self.height()) * 2 - 1)
    
    
    def drawBackground(self, painter):
        painter.fillRect(QRectF(0, 0, self.width(), self.height()), QColor(200, 200, 200))
    
    
    def drawPoints(self, painter):
        linePen = QPen(QColor(150, 150 ,150));
        linePen.setCapStyle(Qt.RoundCap);
        painter.setRenderHint(QPainter.Antialiasing, True);
        painter.setPen(linePen);
        linePen.setWidth(30);
        painter.setPen(linePen);
        
        if self.point is not None:
            painter.drawPoint(self.point[0], self.point[1])


class TwistController():
    def __init__(self):
        self.publisher = rospy.Publisher('cmd_vel', Twist)
        self.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    
    
    def move(self, linear, angular):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        rospy.loginfo(str(self.twist))
        self.publisher.publish(self.twist)


class ScenarioController():
    def __init__(self):
        self.publisher = rospy.Publisher('scenario', Scenario)
        self.scenario = Scenario()
    
    
    def send(self):
        self.scenario.video = "test_video.mp4"
        header = Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.now()
        startPoint = Point(0, 0, 0)
        endPoint = Point(1, 0, 0)
        self.scenario.header = header
        self.scenario.curves = [BezierCurve(startPoint, endPoint, startPoint, endPoint),
                                BezierCurve(endPoint, startPoint, endPoint, startPoint)]
        rospy.loginfo(str(self.scenario))
        self.publisher.publish(self.scenario)


def sigintHandler(*args):
    """ Handler for the SIGINT signal. """
    sys.stderr.write('\r')
    QApplication.quit()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigintHandler)
    
    try:
        rospy.init_node('gui_controller', anonymous = True)
        
        app = QApplication(sys.argv)
        guiController = GuiController()
        
        timer = QTimer()
        timer.start(500)  # You may change this if you wish.
        timer.timeout.connect(lambda: None)
        
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
