#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

def setROSEnv():
    baseDir = os.path.expanduser('~')
    
    ip = os.popen("bash ~/catkin_ws/params/echo_this_ip.sh").read().replace("\n", "").replace("\\n", "")
    distro = "hydro"
    
    os.environ["PYTHONPATH"] = baseDir + "/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/" + distro + "/lib/python2.7/dist-packages"
    os.environ["ROS_DISTRO"] = distro
    os.environ["ROS_ETC_DIR"] = "/opt/ros/" + distro + "/etc/ros"
    os.environ["ROS_HOME"] = baseDir + "/.ros"
    os.environ["ROS_HOSTNAME"] = ip
    os.environ["ROS_IP"] = ip
    os.environ["ROS_LOG_DIR"] = baseDir + "/.ros/log"
    os.environ["ROS_MASTER_URI"] = "http://" + ip + ":11311"
    os.environ["ROS_PACKAGE_PATH"] = baseDir + "/catkin_ws/src:/opt/ros/" + distro + "/share:/opt/ros/" + distro + "/stacks"
    os.environ["ROS_ROOT"] = "/opt/ros/" + distro + "/share/ros"
    os.environ["ROS_TEST_RESULTS_DIR"] = baseDir + "/catkin_ws/build/test_results"
    os.environ["ROSLISP_PACKAGE_DIRECTORIES"] = baseDir + "/catkin_ws/devel/share/common-lisp"
    os.environ["LD_LIBRARY_PATH"] += ":" + baseDir + "/catkin_ws/devel/lib:/opt/ros/" + distro + "/lib"
    os.environ["PKG_CONFIG_PATH"] = baseDir + "/catkin_ws/devel/lib/pkgconfig:/opt/ros/" + distro + "/lib/pkgconfig"
    os.environ["CPATH"] = baseDir + "/catkin_ws/devel/include:/opt/ros/" + distro + "/include"
    os.environ["PATH"] = baseDir + "/catkin_ws/devel/bin:/opt/ros/" + distro + "/bin:/usr/lib/lightdm/lightdm:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/lib/jvm/java-7-oracle/bin:/usr/lib/jvm/java-7-oracle/db/bin:/usr/lib/jvm/java-7-oracle/jre/bin"
    os.environ["CMAKE_PREFIX_PATH"] = baseDir + "/catkin_ws/devel:/opt/ros/" + distro + ""
    os.environ["_"] = "/opt/ros/" + distro + "/bin/roslaunch"
    os.environ["CATKIN_TEST_RESULTS_DIR"] = baseDir + "/catkin_ws/build/test_results"
    
    sys.path.append("/opt/ros/" + distro + "/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/" + distro + "/lib/python2.7/dist-packages/rviz/librviz_sip.so")
    sys.path.append("/opt/ros/" + distro + "/share/rviz")


def getLaunchedRobots():
    result = []
    
    setROSEnv()
    
    launchedNodes = os.popen("/opt/ros/hydro/bin/rosnode list").read().split("\n")
    for launchedNode in launchedNodes:
        splittedLaunchedNode = launchedNode.split("/")
        if launchedNode.startswith("/robot") and launchedNode.endswith("/MD25") and len(splittedLaunchedNode) > 2:
            robotId = splittedLaunchedNode[1]
            if robotId not in result:
                result.append(robotId)
    
    return result


if __name__ == "__main__":
    print "\n".join(getLaunchedRobots())