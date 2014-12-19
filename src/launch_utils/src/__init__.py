import sys
import os
from functools import partial
from PyQt4.QtGui import *
from mpl_toolkits.axisartist.axis_artist import BezierPath
from scenario_msgs.msg._VideoPlayer import VideoPlayer


def launchRosNode(nodeName, launchFile):
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
    
    command = "/opt/ros/groovy/bin/roslaunch gui_controller " + launchFile
    sys.argv = ["roslaunch", nodeName, launchFile]
    
    sys.path.append("/opt/ros/groovy/lib/python2.7/dist-packages")
    import roslaunch
    roslaunch.main()
    sys.exit()

# debug
def launchDebug():
    try:
        sys.path.append("/opt/eclipse/plugins/org.python.pydev_3.7.1.201409021729/pysrc")
        import pydevd
        pydevd.settrace(stdoutToServer = True, stderrToServer = True)
    except:
        print "debug failed"
    print "lala"
    