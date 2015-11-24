import sys
import os

import roslaunch

from src.robot.src import utils

def launchRosNode(nodeName, launchFile):
    utils.setROSEnv()
    
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
    
