#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import time

import rospy

from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float64 as Float64Msg
from std_msgs.msg import Bool as BoolMsg
from sensor_msgs.msg import LaserScan as LaserScanMsg
from scenario_msgs.msg import Scenario as ScenarioMsg

lastUpdate = None

def execute(tfPrefix):
    if lastUpdate is not None:
        if (time.time() - lastUpdate) > .5:
            rospy.loginfo("rplidar timeout, kill /" + tfPrefix + "/rplidar")
            os.system("/opt/ros/hydro/bin/rosnode kill /" + tfPrefix + "/rplidar")
            time.sleep(10)
    
    
def laserScanCB(self):
    global lastUpdate
    
    lastUpdate = time.time()
    
    
if __name__ == '__main__':
    rospy.init_node("rplidar_watch_dog_node", log_level = rospy.INFO)
    rospy.Subscriber("scan", LaserScanMsg, laserScanCB)
    
    tfPrefix = rospy.get_param("tf_prefix")
    
    while not rospy.is_shutdown():
        execute(tfPrefix)
        rospy.sleep(.5)
