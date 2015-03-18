import sys
import os

import roslaunch


def launchRosNode(nodeName, launchFile):
    baseDir = os.path.expanduser('~')
    
    if os.path.exists(os.path.join(baseDir, ".SET_FAKE_ROS_ENV")):
        IP = "127.0.0.1"
    else:
        IP = "192.168.150.1"
    
    os.environ["PYTHONPATH"] = baseDir + "/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/groovy/lib/python2.7/dist-packages"
    os.environ["ROS_DISTRO"] = "groovy"
    os.environ["ROS_ETC_DIR"] = "/opt/ros/groovy/etc/ros"
    os.environ["ROS_HOME"] = baseDir + "/.ros"
    os.environ["ROS_HOSTNAME"] = IP
    os.environ["ROS_IP"] = IP
    os.environ["ROS_LOG_DIR"] = baseDir + "/.ros/log"
    os.environ["ROS_MASTER_URI"] = "http://" + IP + ":11311"
    os.environ["ROS_PACKAGE_PATH"] = baseDir + "/catkin_ws/src:/opt/ros/groovy/share:/opt/ros/groovy/stacks"
    os.environ["ROS_ROOT"] = "/opt/ros/groovy/share/ros"
    os.environ["ROS_TEST_RESULTS_DIR"] = baseDir + "/catkin_ws/build/test_results"
    os.environ["ROSLISP_PACKAGE_DIRECTORIES"] = baseDir + "/catkin_ws/devel/share/common-lisp"
    os.environ["LD_LIBRARY_PATH"] += ":" + baseDir + "/catkin_ws/devel/lib:/opt/ros/groovy/lib"
    os.environ["PKG_CONFIG_PATH"] = baseDir + "/catkin_ws/devel/lib/pkgconfig:/opt/ros/groovy/lib/pkgconfig"
    os.environ["CPATH"] = baseDir + "/catkin_ws/devel/include:/opt/ros/groovy/include"
    os.environ["PATH"] = baseDir + "/catkin_ws/devel/bin:/opt/ros/groovy/bin:/usr/lib/lightdm/lightdm:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/lib/jvm/java-7-oracle/bin:/usr/lib/jvm/java-7-oracle/db/bin:/usr/lib/jvm/java-7-oracle/jre/bin"
    os.environ["CMAKE_PREFIX_PATH"] = baseDir + "/catkin_ws/devel:/opt/ros/groovy"
    os.environ["_"] = "/opt/ros/groovy/bin/roslaunch"
    os.environ["CATKIN_TEST_RESULTS_DIR"] = baseDir + "/catkin_ws/build/test_results"
    
    sys.path.append("/opt/ros/groovy/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/groovy/lib/python2.7/dist-packages/rviz/librviz_sip.so")
    sys.path.append("/opt/ros/groovy/share/rviz")
    roslaunch.main(["roslaunch", nodeName, launchFile])
    sys.exit()

# debug
def launchDebug():
    try:
        sys.path.append("/usr/lib/eclipse/plugins/org.python.pydev_3.7.1.201409021729/pysrc")
        sys.path.append("/opt/eclipse/plugins/org.python.pydev_3.7.1.201409021729/pysrc")
        import pydevd
        print "init debug"
        pydevd.settrace(stdoutToServer = True, stderrToServer = True)
    except:
        print "debug failed"
    