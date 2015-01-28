import sys
import os

import roslaunch

def launchRosNode(nodeName, launchFile):
    baseDir = os.path.expanduser('~')
    
    os.environ["PYTHONPATH"] = baseDir + "/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/groovy/lib/python2.7/dist-packages"
    os.environ["ROS_DISTRO"] = "groovy"
    os.environ["ROS_ETC_DIR"] = "/opt/ros/groovy/etc/ros"
    os.environ["ROS_HOME"] = baseDir + "/.ros"
    os.environ["ROS_HOSTNAME"] = "192.168.150.1"
    os.environ["ROS_IP"] = "192.168.150.1"
    os.environ["ROS_LOG_DIR"] = baseDir + "/.ros/log"
    os.environ["ROS_MASTER_URI"] = "http://192.168.150.1:11311"
    os.environ["ROS_PACKAGE_PATH"] = baseDir + "/catkin_ws/src:/opt/ros/groovy/share:/opt/ros/groovy/stacks"
    os.environ["ROS_ROOT"] = "/opt/ros/groovy/share/ros"
    os.environ["ROS_TEST_RESULTS_DIR"] = baseDir + "/catkin_ws/build/test_results"
    os.environ["ROSLISP_PACKAGE_DIRECTORIES"] = baseDir + "/catkin_ws/devel/share/common-lisp"
    
    sys.path.append("/opt/ros/groovy/lib/python2.7/dist-packages")
    roslaunch.main(["roslaunch", nodeName, launchFile])
    sys.exit()

# debug
def launchDebug():
    try:
        sys.path.append("/opt/eclipse/plugins/org.python.pydev_3.7.1.201409021729/pysrc")
        import pydevd
        pydevd.settrace(stdoutToServer = True, stderrToServer = True)
    except:
        print "debug failed"
    