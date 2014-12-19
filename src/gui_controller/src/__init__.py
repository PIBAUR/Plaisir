#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
from functools import partial
import signal
from mpl_toolkits.axisartist.axis_artist import BezierPath
from scenario_msgs.msg._VideoPlayer import VideoPlayer

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
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from guiController import GuiController

    
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
