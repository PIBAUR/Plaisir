import sys
import os

import roslaunch


def launchRosNode(nodeName, launchFile):
    # set environment vars
    envVars = os.popen('/bin/bash ' + os.path.expanduser("~") + '/catkin_ws/run/tools/print_env.sh').read()
    envVars = envVars.split("\n")
    for envVar in envVars:
        try:
            os.environ[envVar.split(";;;")[0]] = envVar.split(";;;")[1]
        except:
            pass
        
    sys.path.append("/opt/ros/groovy/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/groovy/lib/python2.7/dist-packages/rviz/librviz_sip.so")
    sys.path.append("/opt/ros/groovy/share/rviz")
    roslaunch.main(["roslaunch", nodeName, launchFile])
    sys.exit()


def launchDebug():
    try:
        sys.path.append("/usr/lib/eclipse/plugins/org.python.pydev_3.7.1.201409021729/pysrc")
        sys.path.append("/opt/eclipse/plugins/org.python.pydev_3.7.1.201409021729/pysrc")
        import pydevd
        print "init debug"
        pydevd.settrace(stdoutToServer = True, stderrToServer = True)
    except:
        print "debug failed"
    