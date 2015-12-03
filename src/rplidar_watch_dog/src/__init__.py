#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import time
from threading import Thread

import rospy

from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float64 as Float64Msg
from std_msgs.msg import Bool as BoolMsg
from geometry_msgs.msg import Twist as TwistMsg
from geometry_msgs.msg import Vector3 as Vector3Msg
from sensor_msgs.msg import LaserScan as LaserScanMsg
from scenario_msgs.msg import Scenario as ScenarioMsg

lastUpdate = None

class FloodCmdVelThread(Thread):
    def __init__(self):
        super(FloodCmdVelThread, self).__init__()
        self.flooding = True
        self.floodMsg = TwistMsg(Vector3Msg(0, 0, 0), Vector3Msg(0, 0, 0))
        
        self.kill = False
        self.cmdVelPublisher = rospy.Publisher("cmd_vel", TwistMsg)
        
        
    def run(self):
        while True:
            if self.kill:
                break
            
            if self.flooding:
                self.cmdVelPublisher.publish(self.floodMsg)
            else:
                time.sleep(.005)


def execute(tfPrefix):
    if lastUpdate is not None:
        if (time.time() - lastUpdate) > .5:
            rospy.loginfo("flood cmd_vel at 0 while lidar isn't alive")
            floodCmdVelThread.flooding = True
            rospy.loginfo("rplidar timeout, kill /" + tfPrefix + "/rplidar")
            os.system("/opt/ros/hydro/bin/rosnode kill /" + tfPrefix + "/rplidar")
            time.sleep(10)
        else:
            floodCmdVelThread.flooding = False
    
    
def laserScanCB(self):
    global lastUpdate
    
    lastUpdate = time.time()
    
    
if __name__ == '__main__':
    rospy.init_node("rplidar_watch_dog_node", log_level = rospy.INFO)
    rospy.Subscriber("scan", LaserScanMsg, laserScanCB)
    
    floodCmdVelThread = FloodCmdVelThread()
    floodCmdVelThread.start()
    
    tfPrefix = rospy.get_param("tf_prefix")
    
    while not rospy.is_shutdown():
        execute(tfPrefix)
        rospy.sleep(.5)
    
    # end
    floodCmdVelThread.kill = True
